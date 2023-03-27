/*
 * Copyright 2014, General Dynamics C4 Systems
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <config.h>
#include <fastpath/fastpath.h>
#include <model/smp.h>

#ifdef CONFIG_BENCHMARK_TRACK_KERNEL_ENTRIES
#include <benchmark/benchmark_track.h>
#endif
#include <benchmark/benchmark_utilisation.h>

#ifdef CONFIG_ARCH_ARM
static inline
FORCE_INLINE
#endif
void NORETURN fastpath_signal(word_t cptr, word_t msgInfo)
{
    word_t fault_type;
    sched_context_t *sc = NULL;
    bool_t schedulable = false;
    bool_t crossnode = false;

    /* Get fault type. */
    fault_type = seL4_Fault_get_seL4_FaultType(NODE_STATE(ksCurThread)->tcbFault);

    /* Check there's no saved fault. Can be removed if the current thread can't
     * have a fault while invoking the fastpath */
    if (unlikely(fault_type != seL4_Fault_NullFault)) {
        slowpathShared(SysSend);
    }

    /* Lookup the cap */
    cap_t cap = lookup_fp(TCB_PTR_CTE_PTR(NODE_STATE(ksCurThread), tcbCTable)->cap, cptr);

    /* Check it's a notification */
    if (unlikely(!cap_capType_equals(cap, cap_notification_cap))) {
        slowpathShared(SysSend);
    }

    /* Check that we are allowed to send to this cap */
    if (unlikely(!cap_notification_cap_get_capNtfnCanSend(cap))) {
        slowpathShared(SysSend);
    }

    /* Get the notification address */
    notification_t *ntfnPtr = NTFN_PTR(cap_notification_cap_get_capNtfnPtr(cap));
    ntfn_lock_acquire(ntfnPtr);

    /* Get the notification state */
    uint32_t ntfnState = notification_ptr_get_state(ntfnPtr);

    /* Get the notification badge */
    word_t badge = cap_notification_cap_get_capNtfnBadge(cap);
    switch (ntfnState) {
    case NtfnState_Active: {
#ifdef CONFIG_BENCHMARK_TRACK_KERNEL_ENTRIES
        ksKernelEntry.is_fastpath = true;
#endif
        ntfn_set_active(ntfnPtr, badge);
        ntfn_lock_release(ntfnPtr);
        restore_user_context();
        UNREACHABLE();
    }
    case NtfnState_Idle: {
        tcb_t *dest = (tcb_t *) notification_ptr_get_ntfnBoundTCB(ntfnPtr);

        /* Check if we are bound and that thread is waiting for a message */
        if (dest && thread_state_ptr_get_tsType(&dest->tcbState) == ThreadState_BlockedOnReceive) {
            endpoint_t *ep_ptr = EP_PTR(thread_state_get_blockingObject(dest->tcbState));
            ep_lock_acquire(ep_ptr);
            if (thread_state_ptr_get_tsType(&dest->tcbState) != ThreadState_BlockedOnReceive) {
                ep_lock_release(ep_ptr);
                ntfn_lock_release(ntfnPtr);
                slowpathShared(SysSend);
            }
            /* Basically equivalent to maybeDonateSchedContext. Check whether the thread already has
             * a SC or if one can be donated from the notification. If neither is true, go to
             * slowpath */
            if (!dest->tcbSchedContext) {
                sc = SC_PTR(notification_ptr_get_ntfnSchedContext(ntfnPtr));
                if (sc == NULL || sc->scTcb != NULL) {
                    ep_lock_release(ep_ptr);
                    ntfn_lock_release(ntfnPtr);
                    slowpathShared(SysSend);
                    UNREACHABLE();
                }

                /* Slowpath FPU saving case. We check that dest either isn't currently using the FPU
                 * so that we do not have to save its state for migration (lazy FPU) or that it is the
                 * the thread that is currently using the FPU, but the CPU is the same as that of the
                 * scheduling context so that tcb migration is not needed. Not exactly the same as the
                 * slowpath as this saves FPU state in the second case but there should be no reason
                 * to touch the FPU if migration is not required */
#ifdef ENABLE_SMP_SUPPORT
#ifdef CONFIG_HAVE_FPU
                if (nativeThreadUsingFPU(dest)) {
                    ep_lock_release(ep_ptr);
                    ntfn_lock_release(ntfnPtr);
                    slowpathShared(SysSend);
                    UNREACHABLE();
                }
#endif
#endif
            } else {
                sc = dest->tcbSchedContext;
            }

            /* Signal to higher prio thread is NOT fastpathed. Not sure if this handles idle thread */
            if (NODE_STATE_ON_CORE(ksCurThread, sc->scCore)->tcbPriority < dest->tcbPriority) {
                ep_lock_release(ep_ptr);
                ntfn_lock_release(ntfnPtr);
                slowpathShared(SysSend);
            }

            /* Simplified schedContext_resume that does not change state and reverts to the
            * slowpath in cases where the SC does not have sufficient budget, as this case
            * adds extra scheduler logic. Normally, this is done after the sched_context donate
            * but after tweaking it, I don't immediately see anything executed in schedContext_donate
            * that will affect the conditions of this check */

            if (sc->scRefillMax > 0 && !thread_state_get_tcbInReleaseQueue(dest->tcbState)) {
                if (!(refill_ready(sc) && refill_sufficient(sc, 0))) {
                    ep_lock_release(ep_ptr);
                    ntfn_lock_release(ntfnPtr);
                    slowpathShared(SysSend);
                    UNREACHABLE();
                }
                schedulable = true;
            }

            if (ksCurDomain != dest->tcbDomain SMP_COND_STATEMENT( || sc->scCore != getCurrentCPUIndex())) {
                crossnode = true;
            }

            reply_t *reply = REPLY_PTR(thread_state_get_replyObject(dest->tcbState));
            if (reply && !reply_object_lock_try_acquire(reply)) {
                ep_lock_release(ep_ptr);
                ntfn_lock_release(ntfnPtr);
                slowpathShared(SysSend);
            }

            /*  Point of no return */
#ifdef CONFIG_BENCHMARK_TRACK_KERNEL_ENTRIES
            ksKernelEntry.is_fastpath = true;
#endif

            /* Equivalent to cancel_ipc */
            tcb_queue_t queue;

            queue = ep_ptr_get_queue(ep_ptr);
            queue = tcbEPDequeue(dest, queue);
            ep_ptr_set_queue(ep_ptr, queue);

            if (!queue.head) {
                endpoint_ptr_set_state(ep_ptr, EPState_Idle);
            }

            if (reply != NULL) {
                reply_unlink(reply, dest);
            }

            /* Now we donate the SC. The checks required for this were
             * already done before the point of no return. */
            setRegister(dest, badgeRegister, badge);
            thread_state_ptr_set_tsType_np(&dest->tcbState, ThreadState_Running);

            if (!dest->tcbSchedContext) {
                /* Equivalent to schedContext_donate without migrateTCB() */
                sc->scTcb = dest;
                dest->tcbSchedContext = sc;
            }
            assert(dest->tcbSchedContext);

#ifdef ENABLE_SMP_SUPPORT
            /* The part of migrateTCB() that doesn't involve the slowpathed FPU save */
#ifdef CONFIG_DEBUG_BUILD
            tcbDebugRemove(dest);
#endif
            dest->tcbAffinity = sc->scCore;
#ifdef CONFIG_DEBUG_BUILD
            tcbDebugAppend(dest);
#endif
#endif

            /* Left this in the same form as the slowpath. Not sure if optimal */
            if (sc_sporadic(dest->tcbSchedContext)) {
                assert(dest->tcbSchedContext != NODE_STATE(ksCurSC));
                if (dest->tcbSchedContext != NODE_STATE(ksCurSC)) {
                    refill_unblock_check(dest->tcbSchedContext);
                }
            }

            /* If dest was already not schedulable prior to the budget check
             * the slowpath doesn't seem to do anything special besides just not
             * not scheduling the dest thread. */
            if (schedulable) {
                scheduler_lock_acquire(dest->tcbAffinity);
                if (NODE_STATE(ksCurThread)->tcbPriority < dest->tcbPriority || crossnode) {
                    SCHED_ENQUEUE(dest);
                } else {
                    SCHED_APPEND(dest);
                }
                scheduler_lock_release(dest->tcbAffinity);
            }

            if (reply) {
                reply_object_lock_release(reply);
            }
            ep_lock_release(ep_ptr);
            ntfn_lock_release(ntfnPtr);

            restore_user_context();
            UNREACHABLE();
        } else {
#ifdef CONFIG_BENCHMARK_TRACK_KERNEL_ENTRIES
            ksKernelEntry.is_fastpath = true;
#endif
            ntfn_set_active(ntfnPtr, badge);
            ntfn_lock_release(ntfnPtr);
            restore_user_context();
            UNREACHABLE();
        }
    }
    case NtfnState_Waiting: {
        tcb_t *dest = TCB_PTR(notification_ptr_get_ntfnQueue_head(ntfnPtr));

        /* Basically equivalent to maybeDonateSchedContext. Check whether the thread already has
         * a SC or if one can be donated from the notification. If neither is true, go to
         * slowpath */
        if (!dest->tcbSchedContext) {
            sc = SC_PTR(notification_ptr_get_ntfnSchedContext(ntfnPtr));
            if (sc == NULL || sc->scTcb != NULL) {
                ntfn_lock_release(ntfnPtr);
                slowpathShared(SysSend);
            }

            /* Slowpath FPU saving case. We check that dest either isn't currently using the FPU
             * so that we do not have to save its state for migration (lazy FPU) or that it is the
             * the thread that is currently using the FPU, but the CPU is the same as that of the
             * scheduling context so that tcb migration is not needed. Not exactly the same as the
             * slowpath as this saves FPU state in the second case but there should be no reason
             * to touch the FPU if migration is not required */
#ifdef ENABLE_SMP_SUPPORT
#ifdef CONFIG_HAVE_FPU
            if (nativeThreadUsingFPU(dest)) {
                ntfn_lock_release(ntfnPtr);
                slowpathShared(SysSend);
                UNREACHABLE();
            }
#endif
#endif
        } else {
            sc = dest->tcbSchedContext;
        }

        /* Signal to higher prio thread is NOT fastpathed. Not sure if this handles idle thread */
        if (NODE_STATE_ON_CORE(ksCurThread, sc->scCore)->tcbPriority < dest->tcbPriority) {
            ntfn_lock_release(ntfnPtr);
            slowpathShared(SysSend);
        }

        /* Simplified schedContext_resume that does not change state and does not depend
         * on state that would otherwise be changed by schedContext_donate in the slowpath.
         * Reverts to theslowpath in cases where the SC does not have sufficient budget, as this case
         * adds extra scheduler logic. Normally, this is done after the sched_context donate
         * but after tweaking it, I don't immediately see anything executed in schedContext_donate
         * that will affect the conditions of this check */

        if (sc->scRefillMax > 0 && !thread_state_get_tcbInReleaseQueue(dest->tcbState)) {
            if (!(refill_ready(sc) && refill_sufficient(sc, 0))) {
                ntfn_lock_release(ntfnPtr);
                slowpathShared(SysSend);
                UNREACHABLE();
            }
            schedulable = true;
        }

        if (ksCurDomain != dest->tcbDomain SMP_COND_STATEMENT( || sc->scCore != getCurrentCPUIndex())) {
            crossnode = true;
        }

        /*  Point of no return */
#ifdef CONFIG_BENCHMARK_TRACK_KERNEL_ENTRIES
        ksKernelEntry.is_fastpath = true;
#endif

        tcb_queue_t ntfn_queue;
        ntfn_queue.head = (tcb_t *)notification_ptr_get_ntfnQueue_head(ntfnPtr);
        ntfn_queue.end = (tcb_t *)notification_ptr_get_ntfnQueue_tail(ntfnPtr);

        ntfn_queue = tcbEPDequeue(dest, ntfn_queue);

        notification_ptr_set_ntfnQueue_head(ntfnPtr, (word_t)ntfn_queue.head);
        notification_ptr_set_ntfnQueue_tail(ntfnPtr, (word_t)ntfn_queue.end);

        if (!ntfn_queue.head) {
            notification_ptr_set_state(ntfnPtr, NtfnState_Idle);
        }

        /* Now we donate the SC. The checks required for this were
         * already done before the point of no return. */
        setRegister(dest, badgeRegister, badge);
        thread_state_ptr_set_tsType_np(&dest->tcbState, ThreadState_Running);

        if (!dest->tcbSchedContext) {
            /* Equivalent to schedContext_donate without migrateTCB() */
            sc->scTcb = dest;
            dest->tcbSchedContext = sc;
        }
        assert(dest->tcbSchedContext);

#ifdef ENABLE_SMP_SUPPORT
        /* The part of migrateTCB() that doesn't involve the slowpathed FPU save */
#ifdef CONFIG_DEBUG_BUILD
        tcbDebugRemove(dest);
#endif
        dest->tcbAffinity = sc->scCore;
#ifdef CONFIG_DEBUG_BUILD
        tcbDebugAppend(dest);
#endif
#endif


        /* Left this in the same form as the slowpath. Not sure if optimal */
        if (sc_sporadic(dest->tcbSchedContext)) {
            assert(dest->tcbSchedContext != NODE_STATE(ksCurSC));
            if (dest->tcbSchedContext != NODE_STATE(ksCurSC)) {
                refill_unblock_check(dest->tcbSchedContext);
            }
        }

        /* If dest was already not schedulable prior to the budget check
         * the slowpath doesn't seem to do anything special besides just not
         * not scheduling the dest thread. */
        if (schedulable) {
            scheduler_lock_acquire(dest->tcbAffinity);
            if (NODE_STATE(ksCurThread)->tcbPriority < dest->tcbPriority || crossnode) {
                SCHED_ENQUEUE(dest);
            } else {
                /* This behaviour isn't quite matching with the slowpath,
                 * as a thread on a different CPU is always enqueued regardless
                 * of priority */
                SCHED_APPEND(dest);
            }
            scheduler_lock_release(dest->tcbAffinity);
        }

        ntfn_lock_release(ntfnPtr);

        restore_user_context();
        UNREACHABLE();
    }
    default:
        fail("Invalid notification state");
        UNREACHABLE();
    }
    UNREACHABLE();
}

#ifdef CONFIG_ARCH_ARM
static inline
FORCE_INLINE
#endif
void NORETURN fastpath_call(word_t cptr, word_t msgInfo)
{
    seL4_MessageInfo_t info;
    cap_t ep_cap;
    endpoint_t *ep_ptr;
    word_t length;
    tcb_t *dest;
    word_t badge;
    cap_t newVTable;
    vspace_root_t *cap_pd;
    pde_t stored_hw_asid;
    word_t fault_type;
    dom_t dom;

    /* Get message info, length, and fault type. */
    info = messageInfoFromWord_raw(msgInfo);
    length = seL4_MessageInfo_get_length(info);
    fault_type = seL4_Fault_get_seL4_FaultType(NODE_STATE(ksCurThread)->tcbFault);

    /* Check there's no extra caps, the length is ok and there's no
     * saved fault. */
    if (unlikely(fastpath_mi_check(msgInfo) ||
                 fault_type != seL4_Fault_NullFault)) {
        slowpathExclusive(SysCall);
    }

    /* Lookup the cap */
    ep_cap = lookup_fp(TCB_PTR_CTE_PTR(NODE_STATE(ksCurThread), tcbCTable)->cap, cptr);

    /* Check it's an endpoint */
    if (unlikely(!cap_capType_equals(ep_cap, cap_endpoint_cap) ||
                 !cap_endpoint_cap_get_capCanSend(ep_cap))) {
        slowpathExclusive(SysCall);
    }

    /* Get the endpoint address */
    ep_ptr = EP_PTR(cap_endpoint_cap_get_capEPPtr(ep_cap));
    ep_lock_acquire(ep_ptr);

    /* Get the destination thread, which is only going to be valid
     * if the endpoint is valid. */
    dest = TCB_PTR(endpoint_ptr_get_epQueue_head(ep_ptr));

    /* Check that there's a thread waiting to receive */
    if (unlikely(endpoint_ptr_get_state(ep_ptr) != EPState_Recv)) {
        ep_lock_release(ep_ptr);
        slowpathShared(SysCall);
    }

    /* ensure we are not single stepping the destination in ia32 */
#if defined(CONFIG_HARDWARE_DEBUG_API) && defined(CONFIG_ARCH_IA32)
    if (unlikely(dest->tcbArch.tcbContext.breakpointState.single_step_enabled)) {
        ep_lock_release(ep_ptr);
        slowpathShared(SysCall);
    }
#endif

    /* Get destination thread.*/
    newVTable = TCB_PTR_CTE_PTR(dest, tcbVTable)->cap;

    /* Get vspace root. */
    cap_pd = cap_vtable_cap_get_vspace_root_fp(newVTable);

    /* Ensure that the destination has a valid VTable. */
    if (unlikely(! isValidVTableRoot_fp(newVTable))) {
        ep_lock_release(ep_ptr);
        slowpathExclusive(SysCall);
    }

#ifdef CONFIG_ARCH_AARCH32
    /* Get HW ASID */
    stored_hw_asid = cap_pd[PD_ASID_SLOT];
#endif

#ifdef CONFIG_ARCH_X86_64
    /* borrow the stored_hw_asid for PCID */
    stored_hw_asid.words[0] = cap_pml4_cap_get_capPML4MappedASID_fp(newVTable);
#endif

#ifdef CONFIG_ARCH_IA32
    /* stored_hw_asid is unused on ia32 fastpath, but gets passed into a function below. */
    stored_hw_asid.words[0] = 0;
#endif
#ifdef CONFIG_ARCH_AARCH64
    /* Need to test that the ASID is still valid */
    asid_t asid = cap_vtable_root_get_mappedASID(newVTable);
    asid_map_t asid_map = findMapForASID(asid);
    if (unlikely(asid_map_get_type(asid_map) != asid_map_asid_map_vspace ||
                 VSPACE_PTR(asid_map_asid_map_vspace_get_vspace_root(asid_map)) != cap_pd)) {
        ep_lock_release(ep_ptr);
        slowpathExclusive(SysCall);
    }
#ifdef CONFIG_ARM_HYPERVISOR_SUPPORT
    /* Ensure the vmid is valid. */
    if (unlikely(!asid_map_asid_map_vspace_get_stored_vmid_valid(asid_map))) {
        ep_lock_release(ep_ptr);
        slowpathShared(SysCall);
    }
    /* vmids are the tags used instead of hw_asids in hyp mode */
    stored_hw_asid.words[0] = asid_map_asid_map_vspace_get_stored_hw_vmid(asid_map);
#else
    stored_hw_asid.words[0] = asid;
#endif
#endif

#ifdef CONFIG_ARCH_RISCV
    /* Get HW ASID */
    stored_hw_asid.words[0] = cap_page_table_cap_get_capPTMappedASID(newVTable);
#endif

    /* let gcc optimise this out for 1 domain */
    dom = maxDom ? ksCurDomain : 0;
    /* ensure only the idle thread or lower prio threads are present in the scheduler */
    if (unlikely(dest->tcbPriority < NODE_STATE(ksCurThread->tcbPriority) &&
                 !isHighestPrio(dom, dest->tcbPriority))) {
        ep_lock_release(ep_ptr);
        slowpathShared(SysCall);
    }

    /* Ensure that the endpoint has has grant or grant-reply rights so that we can
     * create the reply cap */
    if (unlikely(!cap_endpoint_cap_get_capCanGrant(ep_cap) &&
                 !cap_endpoint_cap_get_capCanGrantReply(ep_cap))) {
        ep_lock_release(ep_ptr);
        slowpathExclusive(SysCall);
    }

#ifdef CONFIG_ARCH_AARCH32
    if (unlikely(!pde_pde_invalid_get_stored_asid_valid(stored_hw_asid))) {
        ep_lock_release(ep_ptr);
        slowpathShared(SysCall);
    }
#endif

    /* Ensure the original caller is in the current domain and can be scheduled directly. */
    if (unlikely(dest->tcbDomain != ksCurDomain && 0 < maxDom)) {
        ep_lock_release(ep_ptr);
        slowpathShared(SysCall);
    }

#ifdef CONFIG_KERNEL_MCS
    if (unlikely(dest->tcbSchedContext != NULL)) {
        ep_lock_release(ep_ptr);
        slowpathShared(SysCall);
    }

    reply_t *reply = thread_state_get_replyObject_np(dest->tcbState);
    if (unlikely(reply == NULL)) {
        ep_lock_release(ep_ptr);
        slowpathShared(SysCall);
    }
#endif

#ifdef ENABLE_SMP_SUPPORT
    /* Ensure both threads have the same affinity */
    if (unlikely(NODE_STATE(ksCurThread)->tcbAffinity != dest->tcbAffinity)) {
        ep_lock_release(ep_ptr);
        slowpathShared(SysCall);
    }
#endif /* ENABLE_SMP_SUPPORT */

    /*
     * --- POINT OF NO RETURN ---
     *
     * At this stage, we have committed to performing the IPC.
     */

#ifdef CONFIG_BENCHMARK_TRACK_KERNEL_ENTRIES
    ksKernelEntry.is_fastpath = true;
#endif

    /* Dequeue the destination. */
    endpoint_ptr_set_epQueue_head_np(ep_ptr, TCB_REF(dest->tcbEPNext));
    if (unlikely(dest->tcbEPNext)) {
        dest->tcbEPNext->tcbEPPrev = NULL;
    } else {
        endpoint_ptr_mset_epQueue_tail_state(ep_ptr, 0, EPState_Idle);
    }

    badge = cap_endpoint_cap_get_capEPBadge(ep_cap);

    /* Unlink dest <-> reply, link src (cur thread) <-> reply */
    thread_state_ptr_set_tsType_np(&NODE_STATE(ksCurThread)->tcbState,
                                   ThreadState_BlockedOnReply);
#ifdef CONFIG_KERNEL_MCS
    thread_state_ptr_set_replyObject_np(&dest->tcbState, 0);
    thread_state_ptr_set_replyObject_np(&NODE_STATE(ksCurThread)->tcbState, REPLY_REF(reply));
    reply->replyTCB = NODE_STATE(ksCurThread);

    sched_context_t *sc = NODE_STATE(ksCurThread)->tcbSchedContext;
    sc->scTcb = dest;
    dest->tcbSchedContext = sc;
    NODE_STATE(ksCurThread)->tcbSchedContext = NULL;

    reply_t *old_caller = sc->scReply;
    reply->replyPrev = call_stack_new(REPLY_REF(sc->scReply), false);
    if (unlikely(old_caller)) {
        old_caller->replyNext = call_stack_new(REPLY_REF(reply), false);
    }
    reply->replyNext = call_stack_new(SC_REF(sc), true);
    sc->scReply = reply;
#else
    /* Get sender reply slot */
    cte_t *replySlot = TCB_PTR_CTE_PTR(NODE_STATE(ksCurThread), tcbReply);

    /* Get dest caller slot */
    cte_t *callerSlot = TCB_PTR_CTE_PTR(dest, tcbCaller);

    /* Insert reply cap */
    word_t replyCanGrant = thread_state_ptr_get_blockingIPCCanGrant(&dest->tcbState);;
    cap_reply_cap_ptr_new_np(&callerSlot->cap, replyCanGrant, 0,
                             TCB_REF(NODE_STATE(ksCurThread)));
    mdb_node_ptr_set_mdbPrev_np(&callerSlot->cteMDBNode, CTE_REF(replySlot));
    mdb_node_ptr_mset_mdbNext_mdbRevocable_mdbFirstBadged(
        &replySlot->cteMDBNode, CTE_REF(callerSlot), 1, 1);
#endif

    fastpath_copy_mrs(length, NODE_STATE(ksCurThread), dest);

    /* Dest thread is set Running, but not queued. */
    thread_state_ptr_set_tsType_np(&dest->tcbState,
                                   ThreadState_Running);
    switchToThread_fp(dest, cap_pd, stored_hw_asid);

    msgInfo = wordFromMessageInfo(seL4_MessageInfo_set_capsUnwrapped(info, 0));


    ep_lock_release(ep_ptr);

    NODE_READ_UNLOCK;
    fastpath_restore(badge, msgInfo, NODE_STATE(ksCurThread));
}

#ifdef CONFIG_ARCH_ARM
static inline
FORCE_INLINE
#endif
#ifdef CONFIG_KERNEL_MCS
void NORETURN fastpath_reply_recv(word_t cptr, word_t msgInfo, word_t reply)
#else
void NORETURN fastpath_reply_recv(word_t cptr, word_t msgInfo)
#endif
{
    seL4_MessageInfo_t info;
    cap_t ep_cap;
    endpoint_t *ep_ptr;
    word_t length;
    tcb_t *caller;
    word_t badge;
    tcb_t *endpointTail;
    word_t fault_type;

    cap_t newVTable;
    vspace_root_t *cap_pd;
    pde_t stored_hw_asid;
    dom_t dom;

    /* Get message info and length */
    info = messageInfoFromWord_raw(msgInfo);
    length = seL4_MessageInfo_get_length(info);
    fault_type = seL4_Fault_get_seL4_FaultType(NODE_STATE(ksCurThread)->tcbFault);

    /* Check there's no extra caps, the length is ok and there's no
     * saved fault. */
    if (unlikely(fastpath_mi_check(msgInfo) ||
                 fault_type != seL4_Fault_NullFault)) {
        slowpathShared(SysReplyRecv);
    }



#ifdef CONFIG_KERNEL_MCS
    /* lookup the reply object */
    cap_t reply_cap = lookup_fp(TCB_PTR_CTE_PTR(NODE_STATE(ksCurThread), tcbCTable)->cap, reply);

    /* check it's a reply object */
    if (unlikely(!cap_capType_equals(reply_cap, cap_reply_cap))) {
        slowpathShared(SysReplyRecv);
    }
#endif
#ifdef CONFIG_KERNEL_MCS
    /* Get the reply address */
    reply_t *reply_ptr = REPLY_PTR(cap_reply_cap_get_capReplyPtr(reply_cap));
    if (!reply_object_lock_try_acquire(reply_ptr)) {
        slowpathShared(SysReplyRecv);
    }
    /* check that its valid and at the head of the call chain
       and that the current thread's SC is going to be donated. */
    if (unlikely(reply_ptr->replyTCB == NULL ||
                 call_stack_get_isHead(reply_ptr->replyNext) == 0 ||
                 SC_PTR(call_stack_get_callStackPtr(reply_ptr->replyNext)) != NODE_STATE(ksCurThread)->tcbSchedContext)) {
        reply_object_lock_release(reply_ptr);
        slowpathShared(SysReplyRecv);
    }

    /* Determine who the caller is. */
    caller = reply_ptr->replyTCB;
#else
    /* Only reply if the reply cap is valid. */
    cte_t *callerSlot = TCB_PTR_CTE_PTR(NODE_STATE(ksCurThread), tcbCaller);
    cap_t callerCap = callerSlot->cap;
    if (unlikely(!fastpath_reply_cap_check(callerCap))) {
        slowpathShared(SysReplyRecv);
    }

    /* Determine who the caller is. */
    caller = TCB_PTR(cap_reply_cap_get_capTCBPtr(callerCap));
#endif

        /* Lookup the cap */
    ep_cap = lookup_fp(TCB_PTR_CTE_PTR(NODE_STATE(ksCurThread), tcbCTable)->cap,
                       cptr);

    /* Check it's an endpoint */
    if (unlikely(!cap_capType_equals(ep_cap, cap_endpoint_cap) ||
                 !cap_endpoint_cap_get_capCanReceive(ep_cap))) {
        reply_object_lock_release(reply_ptr);
        slowpathShared(SysReplyRecv);
    }

    /* Check there is nothing waiting on the notification */
    if (unlikely(NODE_STATE(ksCurThread)->tcbBoundNotification &&
                 notification_ptr_get_state(NODE_STATE(ksCurThread)->tcbBoundNotification) == NtfnState_Active)) {
        reply_object_lock_release(reply_ptr);
        slowpathShared(SysReplyRecv);
    }

    /* Get the endpoint address */
    ep_ptr = EP_PTR(cap_endpoint_cap_get_capEPPtr(ep_cap));
    ep_lock_acquire(ep_ptr);

    /* Check that there's not a thread waiting to send */
    if (unlikely(endpoint_ptr_get_state(ep_ptr) == EPState_Send)) {
        ep_lock_release(ep_ptr);
        reply_object_lock_release(reply_ptr);
        slowpathShared(SysReplyRecv);
    }

    /* ensure we are not single stepping the caller in ia32 */
#if defined(CONFIG_HARDWARE_DEBUG_API) && defined(CONFIG_ARCH_IA32)
    if (unlikely(caller->tcbArch.tcbContext.breakpointState.single_step_enabled)) {
        ep_lock_release(ep_ptr);
        reply_object_lock_release(reply_ptr);
        slowpathShared(SysReplyRecv);
    }
#endif

    /* Check that the caller has not faulted, in which case a fault
       reply is generated instead. */
    fault_type = seL4_Fault_get_seL4_FaultType(caller->tcbFault);
    if (unlikely(fault_type != seL4_Fault_NullFault)) {
        ep_lock_release(ep_ptr);
        reply_object_lock_release(reply_ptr);
        slowpathShared(SysReplyRecv);
    }

    /* Get destination thread.*/
    newVTable = TCB_PTR_CTE_PTR(caller, tcbVTable)->cap;

    /* Get vspace root. */
    cap_pd = cap_vtable_cap_get_vspace_root_fp(newVTable);

    /* Ensure that the destination has a valid MMU. */
    if (unlikely(! isValidVTableRoot_fp(newVTable))) {
        ep_lock_release(ep_ptr);
        reply_object_lock_release(reply_ptr);
        slowpathShared(SysReplyRecv);
    }

#ifdef CONFIG_ARCH_AARCH32
    /* Get HWASID. */
    stored_hw_asid = cap_pd[PD_ASID_SLOT];
#endif

#ifdef CONFIG_ARCH_X86_64
    stored_hw_asid.words[0] = cap_pml4_cap_get_capPML4MappedASID(newVTable);
#endif
#ifdef CONFIG_ARCH_IA32
    /* stored_hw_asid is unused on ia32 fastpath, but gets passed into a function below. */
    stored_hw_asid.words[0] = 0;
#endif
#ifdef CONFIG_ARCH_AARCH64
    /* Need to test that the ASID is still valid */
    asid_t asid = cap_vtable_root_get_mappedASID(newVTable);
    asid_map_t asid_map = findMapForASID(asid);
    if (unlikely(asid_map_get_type(asid_map) != asid_map_asid_map_vspace ||
                 VSPACE_PTR(asid_map_asid_map_vspace_get_vspace_root(asid_map)) != cap_pd)) {
        ep_lock_release(ep_ptr);
        reply_object_lock_release(reply_ptr);
        slowpathShared(SysReplyRecv);
    }
#ifdef CONFIG_ARM_HYPERVISOR_SUPPORT
    /* Ensure the vmid is valid. */
    if (unlikely(!asid_map_asid_map_vspace_get_stored_vmid_valid(asid_map))) {
        ep_lock_release(ep_ptr);
        reply_object_lock_release(reply_ptr);
        slowpathShared(SysReplyRecv);
    }

    /* vmids are the tags used instead of hw_asids in hyp mode */
    stored_hw_asid.words[0] = asid_map_asid_map_vspace_get_stored_hw_vmid(asid_map);
#else
    stored_hw_asid.words[0] = asid;
#endif
#endif

#ifdef CONFIG_ARCH_RISCV
    stored_hw_asid.words[0] = cap_page_table_cap_get_capPTMappedASID(newVTable);
#endif

    /* Ensure the original caller can be scheduled directly. */
    dom = maxDom ? ksCurDomain : 0;
    if (unlikely(!isHighestPrio(dom, caller->tcbPriority))) {
        ep_lock_release(ep_ptr);
        reply_object_lock_release(reply_ptr);
        slowpathShared(SysReplyRecv);
    }

#ifdef CONFIG_ARCH_AARCH32
    /* Ensure the HWASID is valid. */
    if (unlikely(!pde_pde_invalid_get_stored_asid_valid(stored_hw_asid))) {
        ep_lock_release(ep_ptr);
        reply_object_lock_release(reply_ptr);
        slowpathShared(SysReplyRecv);
    }
#endif

    /* Ensure the original caller is in the current domain and can be scheduled directly. */
    if (unlikely(caller->tcbDomain != ksCurDomain && 0 < maxDom)) {
        ep_lock_release(ep_ptr);
        reply_object_lock_release(reply_ptr);
        slowpathShared(SysReplyRecv);
    }

#ifdef CONFIG_KERNEL_MCS
    if (unlikely(caller->tcbSchedContext != NULL)) {
        ep_lock_release(ep_ptr);
        reply_object_lock_release(reply_ptr);
        slowpathShared(SysReplyRecv);
    }
#endif

#ifdef ENABLE_SMP_SUPPORT
    /* Ensure both threads have the same affinity */
    if (unlikely(NODE_STATE(ksCurThread)->tcbAffinity != caller->tcbAffinity)) {
        ep_lock_release(ep_ptr);
        reply_object_lock_release(reply_ptr);
        slowpathShared(SysReplyRecv);
    }
#endif /* ENABLE_SMP_SUPPORT */

#ifdef CONFIG_KERNEL_MCS
    /* not possible to set reply object and not be blocked */
    assert(thread_state_get_replyObject(NODE_STATE(ksCurThread)->tcbState) == 0);
#endif

    /*
     * --- POINT OF NO RETURN ---
     *
     * At this stage, we have committed to performing the IPC.
     */

#ifdef CONFIG_BENCHMARK_TRACK_KERNEL_ENTRIES
    ksKernelEntry.is_fastpath = true;
#endif

    /* Set thread state to BlockedOnReceive */
    thread_state_ptr_mset_blockingObject_tsType(
        &NODE_STATE(ksCurThread)->tcbState, (word_t)ep_ptr, ThreadState_BlockedOnReceive);
#ifdef CONFIG_KERNEL_MCS
    /* unlink reply object from caller */
    thread_state_ptr_set_replyObject_np(&caller->tcbState, 0);
    /* set the reply object */
    thread_state_ptr_set_replyObject_np(&NODE_STATE(ksCurThread)->tcbState, REPLY_REF(reply_ptr));
    reply_ptr->replyTCB = NODE_STATE(ksCurThread);
#else
    thread_state_ptr_set_blockingIPCCanGrant(&NODE_STATE(ksCurThread)->tcbState,
                                             cap_endpoint_cap_get_capCanGrant(ep_cap));;
#endif

    /* Place the thread in the endpoint queue */
    endpointTail = endpoint_ptr_get_epQueue_tail_fp(ep_ptr);
    if (likely(!endpointTail)) {
        NODE_STATE(ksCurThread)->tcbEPPrev = NULL;
        NODE_STATE(ksCurThread)->tcbEPNext = NULL;

        /* Set head/tail of queue and endpoint state. */
        endpoint_ptr_set_epQueue_head_np(ep_ptr, TCB_REF(NODE_STATE(ksCurThread)));
        endpoint_ptr_mset_epQueue_tail_state(ep_ptr, TCB_REF(NODE_STATE(ksCurThread)),
                                             EPState_Recv);
    } else {
#ifdef CONFIG_KERNEL_MCS
        /* Update queue. */
        tcb_queue_t queue = tcbEPAppend(NODE_STATE(ksCurThread), ep_ptr_get_queue(ep_ptr));
        endpoint_ptr_set_epQueue_head_np(ep_ptr, TCB_REF(queue.head));
        endpoint_ptr_mset_epQueue_tail_state(ep_ptr, TCB_REF(queue.end), EPState_Recv);
#else
        /* Append current thread onto the queue. */
        endpointTail->tcbEPNext = NODE_STATE(ksCurThread);
        NODE_STATE(ksCurThread)->tcbEPPrev = endpointTail;
        NODE_STATE(ksCurThread)->tcbEPNext = NULL;

        /* Update tail of queue. */
        endpoint_ptr_mset_epQueue_tail_state(ep_ptr, TCB_REF(NODE_STATE(ksCurThread)),
                                             EPState_Recv);
#endif
    }

#ifdef CONFIG_KERNEL_MCS
    /* update call stack */
    word_t prev_ptr = call_stack_get_callStackPtr(reply_ptr->replyPrev);
    sched_context_t *sc = NODE_STATE(ksCurThread)->tcbSchedContext;
    NODE_STATE(ksCurThread)->tcbSchedContext = NULL;
    caller->tcbSchedContext = sc;
    sc->scTcb = caller;

    sc->scReply = REPLY_PTR(prev_ptr);
    if (unlikely(REPLY_PTR(prev_ptr) != NULL)) {
        sc->scReply->replyNext = reply_ptr->replyNext;
    }

    /* TODO neccessary? */
    reply_ptr->replyPrev.words[0] = 0;
    reply_ptr->replyNext.words[0] = 0;
#else
    /* Delete the reply cap. */
    mdb_node_ptr_mset_mdbNext_mdbRevocable_mdbFirstBadged(
        &CTE_PTR(mdb_node_get_mdbPrev(callerSlot->cteMDBNode))->cteMDBNode,
        0, 1, 1);
    callerSlot->cap = cap_null_cap_new();
    callerSlot->cteMDBNode = nullMDBNode;
#endif

    /* I know there's no fault, so straight to the transfer. */

    /* Replies don't have a badge. */
    badge = 0;

    fastpath_copy_mrs(length, NODE_STATE(ksCurThread), caller);

    /* Dest thread is set Running, but not queued. */
    thread_state_ptr_set_tsType_np(&caller->tcbState,
                                   ThreadState_Running);
    switchToThread_fp(caller, cap_pd, stored_hw_asid);

    msgInfo = wordFromMessageInfo(seL4_MessageInfo_set_capsUnwrapped(info, 0));

    ep_lock_release(ep_ptr);
    reply_object_lock_release(reply_ptr);

    NODE_READ_UNLOCK;

    fastpath_restore(badge, msgInfo, NODE_STATE(ksCurThread));
}
