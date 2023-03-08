/*
 * Copyright 2014, General Dynamics C4 Systems
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#include <config.h>
#include <types.h>
#include <benchmark/benchmark.h>
#include <arch/benchmark.h>
#include <benchmark/benchmark_track.h>
#include <benchmark/benchmark_utilisation.h>
#include <api/syscall.h>
#include <api/failures.h>
#include <api/faults.h>
#include <kernel/cspace.h>
#include <kernel/faulthandler.h>
#include <kernel/thread.h>
#include <kernel/vspace.h>
#include <machine/io.h>
#include <plat/machine/hardware.h>
#include <object/interrupt.h>
#include <model/statedata.h>
#include <string.h>
#include <kernel/traps.h>
#include <arch/machine.h>
#ifdef ENABLE_SMP_SUPPORT
#include <smp/ipi.h>
#endif
#ifdef CONFIG_DEBUG_BUILD
#include <arch/machine/capdl.h>
#endif

/* The haskell function 'handleEvent' is split into 'handleXXX' variants
 * for each event causing a kernel entry */

exception_t handleInterruptEntry(void)
{
    irq_t irq;

    irq = getActiveIRQ();
#ifdef CONFIG_KERNEL_MCS
    if (SMP_TERNARY(clh_is_self_in_queue(), 1)) {
        updateTimestamp();
        checkBudget();
    }
#endif

    if (IRQT_TO_IRQ(irq) != IRQT_TO_IRQ(irqInvalid)) {
        handleInterrupt(irq);
    } else {
#ifdef CONFIG_IRQ_REPORTING
        userError("Spurious interrupt!");
#endif
        handleSpuriousIRQ();
    }

#ifdef CONFIG_KERNEL_MCS
    if (SMP_TERNARY(clh_is_self_in_queue(), 1)) {
#endif
        schedule();
        activateThread();
#ifdef CONFIG_KERNEL_MCS
    }
#endif

    return EXCEPTION_NONE;
}

exception_t handleUnknownSyscall(word_t w)
{
#ifdef CONFIG_PRINTING
    if (w == SysDebugPutChar) {
        kernel_putchar(getRegister(NODE_STATE(ksCurThread), capRegister));
        return EXCEPTION_NONE;
    }
    if (w == SysDebugDumpScheduler) {
#ifdef CONFIG_DEBUG_BUILD
        debug_dumpScheduler();
#endif
        return EXCEPTION_NONE;
    }
#endif
#ifdef CONFIG_DEBUG_BUILD
    if (w == SysDebugHalt) {
        tcb_t *UNUSED tptr = NODE_STATE(ksCurThread);
        printf("Debug halt syscall from user thread %p \"%s\"\n", tptr, TCB_PTR_DEBUG_PTR(tptr)->tcbName);
        halt();
    }
    if (w == SysDebugSnapshot) {
        tcb_t *UNUSED tptr = NODE_STATE(ksCurThread);
        printf("Debug snapshot syscall from user thread %p \"%s\"\n",
               tptr, TCB_PTR_DEBUG_PTR(tptr)->tcbName);
        debug_capDL();
        return EXCEPTION_NONE;
    }
    if (w == SysDebugCapIdentify) {
        word_t cptr = getRegister(NODE_STATE(ksCurThread), capRegister);
        lookupCapAndSlot_ret_t lu_ret = lookupCapAndSlot(NODE_STATE(ksCurThread), cptr);
        word_t cap_type = cap_get_capType(lu_ret.cap);
        setRegister(NODE_STATE(ksCurThread), capRegister, cap_type);
        return EXCEPTION_NONE;
    }

    if (w == SysDebugNameThread) {
        /* This is a syscall meant to aid debugging, so if anything goes wrong
         * then assume the system is completely misconfigured and halt */
        const char *name;
        word_t len;
        word_t cptr = getRegister(NODE_STATE(ksCurThread), capRegister);
        lookupCapAndSlot_ret_t lu_ret = lookupCapAndSlot(NODE_STATE(ksCurThread), cptr);
        /* ensure we got a TCB cap */
        word_t cap_type = cap_get_capType(lu_ret.cap);
        if (cap_type != cap_thread_cap) {
            userError("SysDebugNameThread: cap is not a TCB, halting");
            halt();
        }
        /* Add 1 to the IPC buffer to skip the message info word */
        name = (const char *)(lookupIPCBuffer(true, NODE_STATE(ksCurThread)) + 1);
        if (!name) {
            userError("SysDebugNameThread: Failed to lookup IPC buffer, halting");
            halt();
        }
        /* ensure the name isn't too long */
        len = strnlen(name, seL4_MsgMaxLength * sizeof(word_t));
        if (len == seL4_MsgMaxLength * sizeof(word_t)) {
            userError("SysDebugNameThread: Name too long, halting");
            halt();
        }
        setThreadName(TCB_PTR(cap_thread_cap_get_capTCBPtr(lu_ret.cap)), name);
        return EXCEPTION_NONE;
    }
#ifdef ENABLE_SMP_SUPPORT
    if (w == SysDebugSendIPI) {
        return handle_SysDebugSendIPI();
    }
#endif /* ENABLE_SMP_SUPPORT */
#endif /* CONFIG_DEBUG_BUILD */

#ifdef CONFIG_DANGEROUS_CODE_INJECTION
    if (w == SysDebugRun) {
        ((void (*)(void *))getRegister(NODE_STATE(ksCurThread), capRegister))((void *)getRegister(NODE_STATE(ksCurThread),
                                                                                                  msgInfoRegister));
        return EXCEPTION_NONE;
    }
#endif

#ifdef CONFIG_KERNEL_X86_DANGEROUS_MSR
    if (w == SysX86DangerousWRMSR) {
        uint64_t val;
        uint32_t reg = getRegister(NODE_STATE(ksCurThread), capRegister);
        if (CONFIG_WORD_SIZE == 32) {
            val = (uint64_t)getSyscallArg(0, NULL) | ((uint64_t)getSyscallArg(1, NULL) << 32);
        } else {
            val = getSyscallArg(0, NULL);
        }
        x86_wrmsr(reg, val);
        return EXCEPTION_NONE;
    } else if (w == SysX86DangerousRDMSR) {
        uint64_t val;
        uint32_t reg = getRegister(NODE_STATE(ksCurThread), capRegister);
        val = x86_rdmsr(reg);
        int num = 1;
        if (CONFIG_WORD_SIZE == 32) {
            setMR(NODE_STATE(ksCurThread), NULL, 0, val & 0xffffffff);
            setMR(NODE_STATE(ksCurThread), NULL, 1, val >> 32);
            num++;
        } else {
            setMR(NODE_STATE(ksCurThread), NULL, 0, val);
        }
        setRegister(NODE_STATE(ksCurThread), msgInfoRegister, wordFromMessageInfo(seL4_MessageInfo_new(0, 0, 0, num)));
        return EXCEPTION_NONE;
    }
#endif

#ifdef CONFIG_ENABLE_BENCHMARKS
    switch (w) {
    case SysBenchmarkFlushCaches:
        return handle_SysBenchmarkFlushCaches();
    case SysBenchmarkResetLog:
        return handle_SysBenchmarkResetLog();
    case SysBenchmarkFinalizeLog:
        return handle_SysBenchmarkFinalizeLog();
#ifdef CONFIG_KERNEL_LOG_BUFFER
    case SysBenchmarkSetLogBuffer:
        return handle_SysBenchmarkSetLogBuffer();
#endif /* CONFIG_KERNEL_LOG_BUFFER */
#ifdef CONFIG_BENCHMARK_TRACK_UTILISATION
    case SysBenchmarkGetThreadUtilisation:
        return handle_SysBenchmarkGetThreadUtilisation();
    case SysBenchmarkResetThreadUtilisation:
        return handle_SysBenchmarkResetThreadUtilisation();
#ifdef CONFIG_DEBUG_BUILD
    case SysBenchmarkDumpAllThreadsUtilisation:
        return handle_SysBenchmarkDumpAllThreadsUtilisation();
    case SysBenchmarkResetAllThreadsUtilisation:
        return handle_SysBenchmarkResetAllThreadsUtilisation();
#endif /* CONFIG_DEBUG_BUILD */
#endif /* CONFIG_BENCHMARK_TRACK_UTILISATION */
    case SysBenchmarkNullSyscall:
        return EXCEPTION_NONE;
    default:
        break; /* syscall is not for benchmarking */
    } /* end switch(w) */
#endif /* CONFIG_ENABLE_BENCHMARKS */

    MCS_DO_IF_BUDGET({
#ifdef CONFIG_SET_TLS_BASE_SELF
        if (w == SysSetTLSBase)
        {
            word_t tls_base = getRegister(NODE_STATE(ksCurThread), capRegister);
            /*
             * This updates the real register as opposed to the thread state
             * value. For many architectures, the TLS variables only get
             * updated on a thread switch.
             */
            return Arch_setTLSRegister(tls_base);
        }
#endif
        current_fault = seL4_Fault_UnknownSyscall_new(w);
        handleFault(NODE_STATE(ksCurThread));
    })

    schedule();
    activateThread();

    return EXCEPTION_NONE;
}

exception_t handleUserLevelFault(word_t w_a, word_t w_b)
{
    MCS_DO_IF_BUDGET({
        current_fault = seL4_Fault_UserException_new(w_a, w_b);
        handleFault(NODE_STATE(ksCurThread));
    })
    schedule();
    activateThread();

    return EXCEPTION_NONE;
}

exception_t handleVMFaultEvent(vm_fault_type_t vm_faultType)
{
    MCS_DO_IF_BUDGET({

        exception_t status = handleVMFault(NODE_STATE(ksCurThread), vm_faultType);
        if (status != EXCEPTION_NONE)
        {
            handleFault(NODE_STATE(ksCurThread));
        }
    })

    schedule();
    activateThread();

    return EXCEPTION_NONE;
}

#ifdef CONFIG_KERNEL_MCS
struct lbo_ret {
    bool_t success;
    endpoint_t *blocking_ep;
    reply_t *reply;
};
typedef struct lbo_ret lbo_ret_t;

lbo_ret_t lock_blocking_objects(tcb_t *tcb)
{
    assert(tcb);

    thread_state_t *state = &tcb->tcbState;
    if (thread_state_ptr_get_tsType(state) != ThreadState_BlockedOnReceive) {
        return (lbo_ret_t){ .success = true, 0 };
    }

    endpoint_t *blocking_ep = EP_PTR(thread_state_ptr_get_blockingObject(state));
    if (!blocking_ep) {
        return (lbo_ret_t){ .success = true, 0 };
    }
    ep_lock_acquire(blocking_ep);

    state = &tcb->tcbState;
    if (EP_PTR(thread_state_ptr_get_blockingObject(state)) != blocking_ep) {
        ep_lock_release(blocking_ep);
        return (lbo_ret_t){ .success = false, 0 };
    }

    reply_t *reply = REPLY_PTR(thread_state_get_replyObject(*state));
    if (reply) {
        reply_object_lock_acquire(reply);
    }

    return (lbo_ret_t){
        .success = true,
        .blocking_ep = blocking_ep,
        .reply = reply,
    };
}

void unlock_blocking_objects(lbo_ret_t bo)
{
    if (bo.reply) {
        reply_object_lock_release(bo.reply);
    }
    if (bo.blocking_ep) {
        ep_lock_release(bo.blocking_ep);
    }
}
#endif

#ifdef CONFIG_KERNEL_MCS
static exception_t handleInvocation(bool_t isCall, bool_t isBlocking, bool_t canDonate, bool_t firstPhase, cptr_t cptr)
#else
static exception_t handleInvocation(bool_t isCall, bool_t isBlocking)
#endif
{
    seL4_MessageInfo_t info;
    lookupCapAndSlot_ret_t lu_ret;
    word_t *buffer;
    exception_t status;
    word_t length;
    tcb_t *thread;

    thread = NODE_STATE(ksCurThread);

    info = messageInfoFromWord(getRegister(thread, msgInfoRegister));
#ifndef CONFIG_KERNEL_MCS
    cptr_t cptr = getRegister(thread, capRegister);
#endif

    /* faulting section */
    lu_ret = lookupCapAndSlot(thread, cptr);

    if (unlikely(lu_ret.status != EXCEPTION_NONE)) {
        userError("Invocation of invalid cap #%lu.", cptr);
        current_fault = seL4_Fault_CapFault_new(cptr, false);

        if (isBlocking) {
            handleFault(thread);
        }

        return EXCEPTION_NONE;
    }

    buffer = lookupIPCBuffer(false, thread);

    status = lookupExtraCaps(thread, buffer, info);

    if (unlikely(status != EXCEPTION_NONE)) {
        userError("Lookup of extra caps failed.");
        if (isBlocking) {
            handleFault(thread);
        }
        return EXCEPTION_NONE;
    }

    /* Syscall error/Preemptible section */
    length = seL4_MessageInfo_get_length(info);
    if (unlikely(length > n_msgRegisters && !buffer)) {
        length = n_msgRegisters;
    }
#ifdef CONFIG_KERNEL_MCS
    status = decodeInvocation(seL4_MessageInfo_get_label(info), length,
                              cptr, lu_ret.slot, lu_ret.cap,
                              isBlocking, isCall,
                              canDonate, firstPhase, buffer);
#else
    status = decodeInvocation(seL4_MessageInfo_get_label(info), length,
                              cptr, lu_ret.slot, lu_ret.cap,
                              isBlocking, isCall, buffer);
#endif

    if (unlikely(status == EXCEPTION_PREEMPTED)) {
        return status;
    }

    if (unlikely(status == EXCEPTION_SYSCALL_ERROR)) {
        if (isCall) {
            replyFromKernel_error(thread);
        }
        return EXCEPTION_NONE;
    }

    if (unlikely(
            thread_state_get_tsType(thread->tcbState) == ThreadState_Restart)) {
        if (isCall) {
            replyFromKernel_success_empty(thread);
        }
        setThreadState(thread, ThreadState_Running);
    }

    return EXCEPTION_NONE;
}

#ifdef CONFIG_KERNEL_MCS
static inline lookupCap_ret_t lookupReply(void)
{
    word_t replyCPtr = getRegister(NODE_STATE(ksCurThread), replyRegister);
    lookupCap_ret_t lu_ret = lookupCap(NODE_STATE(ksCurThread), replyCPtr);
    if (unlikely(lu_ret.status != EXCEPTION_NONE)) {
        userError("Reply cap lookup failed");
        current_fault = seL4_Fault_CapFault_new(replyCPtr, true);
        handleFault(NODE_STATE(ksCurThread));
        return lu_ret;
    }

    if (unlikely(cap_get_capType(lu_ret.cap) != cap_reply_cap)) {
        userError("Cap in reply slot is not a reply");
        current_fault = seL4_Fault_CapFault_new(replyCPtr, true);
        handleFault(NODE_STATE(ksCurThread));
        lu_ret.status = EXCEPTION_FAULT;
        return lu_ret;
    }

    return lu_ret;
}

#ifdef CONFIG_KERNEL_MCS
static exception_t handleSend(bool_t call, bool_t isBlocking, bool_t canDonate)
{
    lookupCap_ret_t lu_ret;
    word_t cptr;

    cptr = getRegister(NODE_STATE(ksCurThread), capRegister);

    /* faulting section */
    lu_ret = lookupCap(NODE_STATE(ksCurThread), cptr);
    if (unlikely(lu_ret.status != EXCEPTION_NONE)) {
        retry_syscall_exclusive();
    }

    seL4_MessageInfo_t info = messageInfoFromWord(getRegister(NODE_STATE(ksCurThread), msgInfoRegister));
    if (seL4_MessageInfo_get_extraCaps(info) != 0) {
        retry_syscall_exclusive();
    }

    switch (cap_get_capType(lu_ret.cap)) {
    case cap_endpoint_cap: {
        if (unlikely(!cap_endpoint_cap_get_capCanSend(lu_ret.cap))) {
            retry_syscall_exclusive();
        }

        endpoint_t *epptr = EP_PTR(cap_endpoint_cap_get_capEPPtr(lu_ret.cap));
        word_t badge = cap_endpoint_cap_get_capEPBadge(lu_ret.cap);
        bool_t canGrant = cap_endpoint_cap_get_capCanGrant(lu_ret.cap);
        bool_t canGrantReply = cap_endpoint_cap_get_capCanGrantReply(lu_ret.cap);

        ep_lock_acquire(epptr);

        reply_t *reply = NULL;
        if (endpoint_ptr_get_state(epptr) == EPState_Recv) {
            tcb_queue_t queue = ep_ptr_get_queue(epptr);
            tcb_t *dest = queue.head;
            reply = REPLY_PTR(thread_state_get_replyObject(dest->tcbState));
            if (reply) {
                reply_object_lock_acquire(reply);
            } else {
                ep_lock_release(epptr);
                retry_syscall_exclusive();
            }
        }

        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
        sendIPCShared(isBlocking, call, badge, canGrant, canGrantReply, canDonate, NODE_STATE(ksCurThread), epptr);
        if (unlikely(thread_state_get_tsType(NODE_STATE(ksCurThread)->tcbState) == ThreadState_Restart)) {
            setThreadState(NODE_STATE(ksCurThread), ThreadState_Running);
        }

        if (reply) {
            reply_object_lock_release(reply);
        }
        ep_lock_release(epptr);
        break;
    }
    case cap_notification_cap: {
        if (unlikely(!cap_notification_cap_get_capNtfnCanSend(lu_ret.cap))) {
            retry_syscall_exclusive();
        }

        notification_t *ntfnptr = NTFN_PTR(cap_notification_cap_get_capNtfnPtr(lu_ret.cap));

        tcb_t *bound_tcb = (tcb_t *)notification_ptr_get_ntfnBoundTCB(ntfnptr);

        ntfn_lock_acquire(ntfnptr);

        word_t badge = cap_notification_cap_get_capNtfnBadge(lu_ret.cap);

        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);

        bool_t ntfn_idle = notification_ptr_get_state(ntfnptr) == NtfnState_Idle;
        if (bound_tcb && ntfn_idle) {
            lbo_ret_t lbo_ret = {0};
            while (!lbo_ret.success) {
                lbo_ret = lock_blocking_objects(bound_tcb);
            }
            if (lbo_ret.blocking_ep) {
                sendSignalBlockedBoundTCB(ntfnptr, bound_tcb, badge);
            } else {
                sendSignalShared(ntfnptr, badge);
            }
            unlock_blocking_objects(lbo_ret);
        } else {
            sendSignalShared(ntfnptr, badge);
        }


    if (unlikely(thread_state_get_tsType(NODE_STATE(ksCurThread)->tcbState) == ThreadState_Restart)) {
        setThreadState(NODE_STATE(ksCurThread), ThreadState_Running);
    }

        ntfn_lock_release(ntfnptr);
        break;
    }
    default:
        retry_syscall_exclusive();
    }

    return EXCEPTION_NONE;
}
#endif

static inline lookupCap_ret_t lookupReplyShared(void)
{
    word_t replyCPtr = getRegister(NODE_STATE(ksCurThread), replyRegister);
    lookupCap_ret_t lu_ret = lookupCap(NODE_STATE(ksCurThread), replyCPtr);
    if (unlikely(lu_ret.status != EXCEPTION_NONE)) {
        return lu_ret;
    }

    if (unlikely(cap_get_capType(lu_ret.cap) != cap_reply_cap)) {
        lu_ret.status = EXCEPTION_FAULT;
    }

    return lu_ret;
}

static void handleReply(bool_t canDonate, bool_t canGrant)
{
    seL4_MessageInfo_t info;
    word_t *buffer;
    word_t length;
    tcb_t *thread;
    lookupCap_ret_t lu_ret;
    reply_t *replyptr;

    thread = NODE_STATE(ksCurThread);
    buffer = lookupIPCBuffer(false, thread);
    info = messageInfoFromWord(getRegister(thread, msgInfoRegister));

    if (seL4_MessageInfo_get_extraCaps(info) != 0) {
        retry_syscall_exclusive();
    }

    length = seL4_MessageInfo_get_length(info);
    if (unlikely(length > n_msgRegisters && !buffer)) {
        length = n_msgRegisters;
    }

    lu_ret = lookupReplyShared();
    if (unlikely(lu_ret.status != EXCEPTION_NONE)) {
        retry_syscall_exclusive();
    }
    replyptr = REPLY_PTR(cap_reply_cap_get_capReplyPtr(lu_ret.cap));

    reply_object_lock_acquire(replyptr);

    setThreadState(thread, ThreadState_Restart);
    doReplyTransferShared(thread, replyptr);

    reply_object_lock_release(replyptr);
}

#else
static void handleReply(void)
{
    cte_t *callerSlot;
    cap_t callerCap;

    callerSlot = TCB_PTR_CTE_PTR(NODE_STATE(ksCurThread), tcbCaller);
    callerCap = callerSlot->cap;

    switch (cap_get_capType(callerCap)) {
    case cap_reply_cap: {
        tcb_t *caller;

        if (cap_reply_cap_get_capReplyMaster(callerCap)) {
            break;
        }
        caller = TCB_PTR(cap_reply_cap_get_capTCBPtr(callerCap));
        /* Haskell error:
         * "handleReply: caller must not be the current thread" */
        assert(caller != NODE_STATE(ksCurThread));
        doReplyTransfer(NODE_STATE(ksCurThread), caller, callerSlot,
                        cap_reply_cap_get_capReplyCanGrant(callerCap));
        return;
    }

    case cap_null_cap:
        /* Do nothing when no caller is pending */
        return;

    default:
        break;
    }

    fail("handleReply: invalid caller cap");
}
#endif

#ifdef CONFIG_KERNEL_MCS
static void handleReplyRecvShared(void)
{
    seL4_MessageInfo_t info;
    word_t *buffer;
    word_t length;
    lookupCap_ret_t lu_ret;
    reply_t *replyptr;
    word_t cptr;
    cap_t recv_cap;

    buffer = lookupIPCBuffer(false, NODE_STATE(ksCurThread));
    info = messageInfoFromWord(getRegister(NODE_STATE(ksCurThread), msgInfoRegister));
    if (seL4_MessageInfo_get_extraCaps(info) != 0) {
        retry_syscall_exclusive();
    }

    length = seL4_MessageInfo_get_length(info);
    if (unlikely(length > n_msgRegisters && !buffer)) {
        length = n_msgRegisters;
    }

    cptr = getRegister(NODE_STATE(ksCurThread), capRegister);
    lu_ret = lookupCap(NODE_STATE(ksCurThread), cptr);
    if (unlikely(lu_ret.status != EXCEPTION_NONE)) {
        retry_syscall_exclusive();
    }
    recv_cap = lu_ret.cap;

    lu_ret = lookupReplyShared();
    if (unlikely(lu_ret.status != EXCEPTION_NONE)) {
        retry_syscall_exclusive();
    }
    cap_t reply_cap = lu_ret.cap;
    replyptr = REPLY_PTR(cap_reply_cap_get_capReplyPtr(lu_ret.cap));

    switch (cap_get_capType(recv_cap)) {
    case cap_endpoint_cap: {
        if (unlikely(!cap_endpoint_cap_get_capCanReceive(recv_cap))) {
            retry_syscall_exclusive();
            break;
        }

        notification_t *bound_ntfn = NODE_STATE(ksCurThread)->tcbBoundNotification;
        if (bound_ntfn && notification_ptr_get_state(bound_ntfn) == NtfnState_Active) {
            ntfn_lock_acquire(bound_ntfn);
            if (notification_ptr_get_state(bound_ntfn) == NtfnState_Active) {
                completeSignal(bound_ntfn, NODE_STATE(ksCurThread));
                ntfn_lock_release(bound_ntfn);
                break;
            } else {
                ntfn_lock_release(bound_ntfn);
            }
        }


        bool_t receiver_has_cap_slot = false;
        bool_t sender_is_transferring_cap = false;

        word_t *receiver_buffer = lookupIPCBuffer(true, NODE_STATE(ksCurThread));
        receiver_has_cap_slot = getReceiveSlots(NODE_STATE(ksCurThread), receiver_buffer) != NULL;

        cap_t ep_cap = recv_cap;
        endpoint_t *epptr = EP_PTR(cap_endpoint_cap_get_capEPPtr(ep_cap));

        ep_lock_acquire(epptr);

        if (endpoint_ptr_get_state(epptr) == EPState_Send) {
            tcb_queue_t queue;
            tcb_t *sender;
            queue = ep_ptr_get_queue(epptr);
            sender = queue.head;
            seL4_MessageInfo_t info = messageInfoFromWord(getRegister(sender, msgInfoRegister));
            sender_is_transferring_cap = (seL4_MessageInfo_get_extraCaps(info) != 0);
        }

        if (receiver_has_cap_slot && sender_is_transferring_cap) {
            ep_lock_release(epptr);
            retry_syscall_exclusive();
            break;
        }

        reply_object_lock_acquire(replyptr);

        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
        doReplyTransferShared(NODE_STATE(ksCurThread), replyptr);

        if (unlikely(
                thread_state_get_tsType(NODE_STATE(ksCurThread)->tcbState) == ThreadState_Restart)) {
            setThreadState(NODE_STATE(ksCurThread), ThreadState_Running);
        }

        receiveIPCShared(NODE_STATE(ksCurThread), epptr, true, replyptr);

        reply_object_lock_release(replyptr);
        ep_lock_release(epptr);

        break;
    }
    case cap_notification_cap: {
        notification_t *ntfn_ptr;
        tcb_t *bound_tcb;
        ntfn_ptr = NTFN_PTR(cap_notification_cap_get_capNtfnPtr(recv_cap));
        bound_tcb = (tcb_t *)notification_ptr_get_ntfnBoundTCB(ntfn_ptr);
        if (unlikely(!cap_notification_cap_get_capNtfnCanReceive(recv_cap)
                     || (bound_tcb && bound_tcb != NODE_STATE(ksCurThread)))) {
            retry_syscall_exclusive();
            break;
        }

        ntfn_lock_acquire(ntfn_ptr);
        reply_object_lock_acquire(replyptr);

        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
        doReplyTransferShared(NODE_STATE(ksCurThread), replyptr);

        reply_object_lock_release(replyptr);

        receiveSignalShared(NODE_STATE(ksCurThread), ntfn_ptr, true);

        ntfn_lock_release(ntfn_ptr);

        break;
    }
    default:
        retry_syscall_exclusive();
        break;
    }
}
#endif

#ifdef CONFIG_KERNEL_MCS
static void handleRecvShared(bool_t isBlocking, bool_t canReply)
{
    word_t cptr;
    lookupCap_ret_t lu_ret;

    cptr = getRegister(NODE_STATE(ksCurThread), capRegister);
    lu_ret = lookupCap(NODE_STATE(ksCurThread), cptr);
    if (unlikely(lu_ret.status != EXCEPTION_NONE)) {
        retry_syscall_exclusive();
    }

    switch (cap_get_capType(lu_ret.cap)) {
    case cap_endpoint_cap: {
        if (unlikely(!cap_endpoint_cap_get_capCanReceive(lu_ret.cap))) {
            retry_syscall_exclusive();
            break;
        }

        reply_t *replyptr = NULL;
        if (canReply) {
            lookupCap_ret_t lu_ret = lookupReplyShared();
            if (unlikely(lu_ret.status != EXCEPTION_NONE)) {
                return;
            }
            replyptr = REPLY_PTR(cap_reply_cap_get_capReplyPtr(lu_ret.cap));
        }

        notification_t *bound_ntfn = NODE_STATE(ksCurThread)->tcbBoundNotification;
        if (bound_ntfn && notification_ptr_get_state(bound_ntfn) == NtfnState_Active) {
            ntfn_lock_acquire(bound_ntfn);
            if (notification_ptr_get_state(bound_ntfn) == NtfnState_Active) {
                completeSignal(bound_ntfn, NODE_STATE(ksCurThread));
                ntfn_lock_release(bound_ntfn);
                break;
            } else {
                ntfn_lock_release(bound_ntfn);
            }
        }

        bool_t receiver_has_cap_slot = false;
        bool_t sender_is_transferring_cap = false;

        word_t *receiver_buffer = lookupIPCBuffer(true, NODE_STATE(ksCurThread));
        receiver_has_cap_slot = getReceiveSlots(NODE_STATE(ksCurThread), receiver_buffer) != NULL;

        cap_t ep_cap = lu_ret.cap;
        endpoint_t *epptr = EP_PTR(cap_endpoint_cap_get_capEPPtr(ep_cap));

        ep_lock_acquire(epptr);

        if (endpoint_ptr_get_state(epptr) == EPState_Send) {
            tcb_queue_t queue;
            tcb_t *sender;
            queue = ep_ptr_get_queue(epptr);
            sender = queue.head;
            seL4_MessageInfo_t info = messageInfoFromWord(getRegister(sender, msgInfoRegister));
            sender_is_transferring_cap = (seL4_MessageInfo_get_extraCaps(info) != 0);
        }

        if (receiver_has_cap_slot && sender_is_transferring_cap) {
            ep_lock_release(epptr);
            retry_syscall_exclusive();
            break;
        }

        if (replyptr) {
            reply_object_lock_acquire(replyptr);
        }

        receiveIPCShared(NODE_STATE(ksCurThread), epptr, isBlocking, replyptr);

        if (replyptr) {
            reply_object_lock_release(replyptr);
        }
        ep_lock_release(epptr);

        break;
    }
    case cap_notification_cap: {
        notification_t *ntfn_ptr;
        tcb_t *bound_tcb;
        ntfn_ptr = NTFN_PTR(cap_notification_cap_get_capNtfnPtr(lu_ret.cap));
        bound_tcb = (tcb_t *)notification_ptr_get_ntfnBoundTCB(ntfn_ptr);
        if (unlikely(!cap_notification_cap_get_capNtfnCanReceive(lu_ret.cap)
                     || (bound_tcb && bound_tcb != NODE_STATE(ksCurThread)))) {
            retry_syscall_exclusive();
            break;
        }

        ntfn_lock_acquire(ntfn_ptr);
        receiveSignalShared(NODE_STATE(ksCurThread), ntfn_ptr, isBlocking);
        ntfn_lock_release(ntfn_ptr);

        break;
    }
    default:
        retry_syscall_exclusive();
        break;
    }
}
#endif

#ifdef CONFIG_KERNEL_MCS
static void handleRecv(bool_t isBlocking, bool_t canReply)
#else
static void handleRecv(bool_t isBlocking)
#endif
{
    word_t epCPtr;
    lookupCap_ret_t lu_ret;

    epCPtr = getRegister(NODE_STATE(ksCurThread), capRegister);

    lu_ret = lookupCap(NODE_STATE(ksCurThread), epCPtr);

    if (unlikely(lu_ret.status != EXCEPTION_NONE)) {
        /* current_lookup_fault has been set by lookupCap */
        current_fault = seL4_Fault_CapFault_new(epCPtr, true);
        handleFault(NODE_STATE(ksCurThread));
        return;
    }

    switch (cap_get_capType(lu_ret.cap)) {
    case cap_endpoint_cap:
        if (unlikely(!cap_endpoint_cap_get_capCanReceive(lu_ret.cap))) {
            current_lookup_fault = lookup_fault_missing_capability_new(0);
            current_fault = seL4_Fault_CapFault_new(epCPtr, true);
            handleFault(NODE_STATE(ksCurThread));
            break;
        }

#ifdef CONFIG_KERNEL_MCS
        cap_t ep_cap = lu_ret.cap;
        cap_t reply_cap = cap_null_cap_new();
        if (canReply) {
            lu_ret = lookupReply();
            if (lu_ret.status != EXCEPTION_NONE) {
                return;
            } else {
                reply_cap = lu_ret.cap;
            }
        }
        receiveIPC(NODE_STATE(ksCurThread), ep_cap, isBlocking, reply_cap);
#else
        deleteCallerCap(NODE_STATE(ksCurThread));
        receiveIPC(NODE_STATE(ksCurThread), lu_ret.cap, isBlocking);
#endif
        break;

    case cap_notification_cap: {
        notification_t *ntfnPtr;
        tcb_t *boundTCB;
        ntfnPtr = NTFN_PTR(cap_notification_cap_get_capNtfnPtr(lu_ret.cap));
        boundTCB = (tcb_t *)notification_ptr_get_ntfnBoundTCB(ntfnPtr);
        if (unlikely(!cap_notification_cap_get_capNtfnCanReceive(lu_ret.cap)
                     || (boundTCB && boundTCB != NODE_STATE(ksCurThread)))) {
            current_lookup_fault = lookup_fault_missing_capability_new(0);
            current_fault = seL4_Fault_CapFault_new(epCPtr, true);
            handleFault(NODE_STATE(ksCurThread));
            break;
        }

        receiveSignal(NODE_STATE(ksCurThread), lu_ret.cap, isBlocking);
        break;
    }
    default:
        current_lookup_fault = lookup_fault_missing_capability_new(0);
        current_fault = seL4_Fault_CapFault_new(epCPtr, true);
        handleFault(NODE_STATE(ksCurThread));
        break;
    }
}

#ifdef CONFIG_KERNEL_MCS
void handleNBSendRecv(bool_t canReply)
{
    lookupCap_ret_t lu_ret;

    seL4_MessageInfo_t info = messageInfoFromWord(getRegister(NODE_STATE(ksCurThread), msgInfoRegister));
    if (seL4_MessageInfo_get_extraCaps(info) != 0) {
        retry_syscall_exclusive();
    }

    word_t *buffer = lookupIPCBuffer(false, NODE_STATE(ksCurThread));
    info = messageInfoFromWord(getRegister(NODE_STATE(ksCurThread), msgInfoRegister));
    if (seL4_MessageInfo_get_extraCaps(info) != 0) {
        retry_syscall_exclusive();
    }
    if (getReceiveSlots(NODE_STATE(ksCurThread), buffer) != NULL) {
        retry_syscall_exclusive();
    }

    cptr_t dest_cptr = getNBSendRecvDest();
    lu_ret = lookupCap(NODE_STATE(ksCurThread), dest_cptr);
    if (unlikely(lu_ret.status != EXCEPTION_NONE)) {
        retry_syscall_exclusive();
    }
    cap_t dest_cap = lu_ret.cap;

    cptr_t src_cptr = getRegister(NODE_STATE(ksCurThread), capRegister);
    lu_ret = lookupCap(NODE_STATE(ksCurThread), src_cptr);
    if (unlikely(lu_ret.status != EXCEPTION_NONE)) {
        retry_syscall_exclusive();
    }
    cap_t src_cap = lu_ret.cap;

    reply_t *replyptr = NULL;

    endpoint_t *src_epptr = NULL;
    notification_t *src_ntfnptr = NULL;

    notification_t *bound_ntfn = NODE_STATE(ksCurThread)->tcbBoundNotification;

    switch (cap_get_capType(src_cap)) {
    case cap_endpoint_cap: {
        if (unlikely(!cap_endpoint_cap_get_capCanReceive(src_cap))) {
            retry_syscall_exclusive();
        }
        src_epptr = EP_PTR(cap_endpoint_cap_get_capEPPtr(src_cap));

        if (canReply) {
            lu_ret = lookupReplyShared();
            if (unlikely(lu_ret.status != EXCEPTION_NONE)) {
                retry_syscall_exclusive();
            }
            replyptr = REPLY_PTR(cap_reply_cap_get_capReplyPtr(lu_ret.cap));
        }
        break;
    }
    case cap_notification_cap: {
        if (unlikely(!cap_notification_cap_get_capNtfnCanSend(src_cap))) {
            retry_syscall_exclusive();
        }
        src_ntfnptr = NTFN_PTR(cap_notification_cap_get_capNtfnPtr(src_cap));
        tcb_t *bound_tcb = (tcb_t *)notification_ptr_get_ntfnBoundTCB(src_ntfnptr);
        if (unlikely(!cap_notification_cap_get_capNtfnCanReceive(src_cap)
                     || (bound_tcb && bound_tcb != NODE_STATE(ksCurThread)))) {
            retry_syscall_exclusive();
            break;
        }
        break;
    }
    default:
        retry_syscall_exclusive();
    }

    endpoint_t *dest_epptr = NULL;
    notification_t *dest_ntfnptr = NULL;
    tcb_t *dest_bound_tcb = NULL;
    reply_t *dest_replyptr = NULL;

    switch (cap_get_capType(dest_cap)) {
    case cap_endpoint_cap: {
        if (unlikely(!cap_endpoint_cap_get_capCanSend(dest_cap))) {
            retry_syscall_exclusive();
        }
        dest_epptr = EP_PTR(cap_endpoint_cap_get_capEPPtr(dest_cap));

        word_t badge = cap_endpoint_cap_get_capEPBadge(dest_cap);
        bool_t canGrant = cap_endpoint_cap_get_capCanGrant(dest_cap);
        bool_t canGrantReply = cap_endpoint_cap_get_capCanGrantReply(dest_cap);

        ep_lock_acquire(dest_epptr);

        if (endpoint_ptr_get_state(dest_epptr) == EPState_Recv) {
            tcb_queue_t queue = ep_ptr_get_queue(dest_epptr);
            tcb_t *dest = queue.head;
            dest_replyptr = REPLY_PTR(thread_state_get_replyObject(dest->tcbState));
            if (dest_replyptr) {
                reply_object_lock_acquire(dest_replyptr);
            } else {
                ep_lock_release(dest_epptr);
                retry_syscall_exclusive();
            }
        }

        setThreadState(NODE_STATE(ksCurThread), ThreadState_Restart);
        sendIPCShared(false, false, badge, canGrant, canGrantReply, false, NODE_STATE(ksCurThread), dest_epptr);
        if (unlikely(thread_state_get_tsType(NODE_STATE(ksCurThread)->tcbState) == ThreadState_Restart)) {
            setThreadState(NODE_STATE(ksCurThread), ThreadState_Running);
        }

        if (dest_replyptr) {
            reply_object_lock_release(dest_replyptr);
        }
        ep_lock_release(dest_epptr);
        break;
    }
    case cap_notification_cap: {
        if (unlikely(!cap_notification_cap_get_capNtfnCanSend(dest_cap))) {
            retry_syscall_exclusive();
        }
        dest_ntfnptr = NTFN_PTR(cap_notification_cap_get_capNtfnPtr(dest_cap));
        tcb_t *dest_bound_tcb = (tcb_t *)notification_ptr_get_ntfnBoundTCB(dest_ntfnptr);
        word_t badge = cap_notification_cap_get_capNtfnBadge(dest_cap);

        ntfn_lock_acquire(dest_ntfnptr);

        bool_t ntfn_idle = notification_ptr_get_state(dest_ntfnptr) == NtfnState_Idle;
        if (dest_bound_tcb && ntfn_idle) {
            lbo_ret_t lbo_ret = {0};
            while (!lbo_ret.success) {
                lbo_ret = lock_blocking_objects(dest_bound_tcb);
            }
            if (lbo_ret.blocking_ep) {
                sendSignalBlockedBoundTCB(dest_ntfnptr, dest_bound_tcb, badge);
            } else {
                sendSignalShared(dest_ntfnptr, badge);
            }
            unlock_blocking_objects(lbo_ret);
        } else {
            sendSignalShared(dest_ntfnptr, badge);
        }

        if (unlikely(thread_state_get_tsType(NODE_STATE(ksCurThread)->tcbState) == ThreadState_Restart)) {
            setThreadState(NODE_STATE(ksCurThread), ThreadState_Running);
        }

        ntfn_lock_release(dest_ntfnptr);
        break;
    }
    default:
        retry_syscall_exclusive();
    }

    switch (cap_get_capType(src_cap)) {
    case cap_endpoint_cap: {
        if (bound_ntfn && notification_ptr_get_state(bound_ntfn) == NtfnState_Active) {
            ntfn_lock_acquire(bound_ntfn);
            if (notification_ptr_get_state(bound_ntfn) == NtfnState_Active) {
                completeSignal(bound_ntfn, NODE_STATE(ksCurThread));
                ntfn_lock_release(bound_ntfn);
                break;
            } else {
                ntfn_lock_release(bound_ntfn);
            }
        }

        ep_lock_acquire(src_epptr);

        if (replyptr) {
            reply_object_lock_acquire(replyptr);
        }

        receiveIPCShared(NODE_STATE(ksCurThread), src_epptr, true, replyptr);

        if (replyptr) {
            reply_object_lock_release(replyptr);
        }
        ep_lock_release(src_epptr);

        break;
    }
    case cap_notification_cap: {
        ntfn_lock_acquire(src_ntfnptr);
        receiveSignalShared(NODE_STATE(ksCurThread), src_ntfnptr, true);
        ntfn_lock_release(src_ntfnptr);

        break;
    }
    }
}
#endif

#ifdef CONFIG_KERNEL_MCS
static inline void mcsPreemptionPoint(irq_t irq)
{
    /* at this point we could be handling a timer interrupt which actually ends the current
     * threads timeslice. However, preemption is possible on revoke, which could have deleted
     * the current thread and/or the current scheduling context, rendering them invalid. */
    if (isSchedulable(NODE_STATE(ksCurThread))) {
        /* if the thread is schedulable, the tcb and scheduling context are still valid */
        checkBudget();
    } else if (NODE_STATE(ksCurSC)->scRefillMax) {
        /* otherwise, if the thread is not schedulable, the SC could be valid - charge it if so */
        chargeBudget(NODE_STATE(ksConsumed), false);
    } else {
        /* If the current SC is no longer configured the time can no
         * longer be charged to it. Simply dropping the consumed time
         * here is equivalent to having charged the consumed time and
         * then having cleared the SC. */
        NODE_STATE(ksConsumed) = 0;
    }
}
#else
#define handleRecv(isBlocking, canReply) handleRecv(isBlocking)
#define mcsPreemptionPoint(irq)
#define handleInvocation(isCall, isBlocking, canDonate, firstPhase, cptr) handleInvocation(isCall, isBlocking)
#endif

static void handleYield(void)
{
#ifdef CONFIG_KERNEL_MCS
    /* Yield the current remaining budget */
    ticks_t consumed = NODE_STATE(ksCurSC)->scConsumed + NODE_STATE(ksConsumed);
    chargeBudget(refill_head(NODE_STATE(ksCurSC))->rAmount, false);
    /* Manually updated the scConsumed so that the full timeslice isn't added, just what was consumed */
    NODE_STATE(ksCurSC)->scConsumed = consumed;
#else
    tcbSchedDequeue(NODE_STATE(ksCurThread));
    SCHED_APPEND_CURRENT_TCB;
    rescheduleRequired();
#endif
}

#ifdef CONFIG_KERNEL_MCS
exception_t handleSyscallShared(syscall_t syscall)
{
    scheduler_lock_acquire(getCurrentCPUIndex());
    updateTimestamp();
    bool_t budget_sufficient = checkBudgetRestart();
    scheduler_lock_release(getCurrentCPUIndex());
    if (budget_sufficient) {
        switch (syscall)
        {
        case SysSend: {
            handleSend(false, true, false);
            break;
        }
        case SysNBSend: {
            handleSend(false, false, false);
            break;
        }
        case SysCall: {
            handleSend(true, true, true);
            break;
        }
        case SysRecv: {
            handleRecvShared(true, true);
            break;
        }
        case SysWait: {
            handleRecvShared(true, false);
            break;
        }
        case SysNBWait: {
            handleRecvShared(false, false);
            break;
        }
        case SysReplyRecv: {
            handleReplyRecvShared();
            break;
        }
        case SysNBSendRecv: {
            handleNBSendRecv(true);
            break;
        }
        case SysNBSendWait: {
            handleNBSendRecv(false);
            break;
        }
        case SysNBRecv: {
            handleRecvShared(false, true);
            break;
        }
        case SysYield: {
            handleYield();;
            break;
        }
        default: {
            fail("Invalid syscall");
        }
        }
    }

    word_t cpu = getCurrentCPUIndex();
    word_t cur_thread_cpu = NODE_STATE(ksCurThread)->tcbAffinity;
    if (cpu < cur_thread_cpu) {
        scheduler_lock_acquire(cpu);
        scheduler_lock_acquire(cur_thread_cpu);
        schedule();
        scheduler_lock_release(cur_thread_cpu);
        scheduler_lock_release(cpu);
    } else if (cpu > cur_thread_cpu) {
        scheduler_lock_acquire(cur_thread_cpu);
        scheduler_lock_acquire(cpu);
        schedule();
        scheduler_lock_release(cpu);
        scheduler_lock_release(cur_thread_cpu);
    } else {
        scheduler_lock_acquire(cpu);
        schedule();
        scheduler_lock_release(cpu);
    }

    activateThread();
    return EXCEPTION_NONE;
}
#endif

exception_t handleSyscall(syscall_t syscall)
{
    exception_t ret;
    irq_t irq;
    MCS_DO_IF_BUDGET({
        switch (syscall)
        {
        case SysSend:
            ret = handleInvocation(false, true, false, false, getRegister(NODE_STATE(ksCurThread), capRegister));
            if (unlikely(ret != EXCEPTION_NONE)) {
                irq = getActiveIRQ();
                mcsPreemptionPoint(irq);
                if (IRQT_TO_IRQ(irq) != IRQT_TO_IRQ(irqInvalid)) {
                    handleInterrupt(irq);
                }
            }

            break;

        case SysNBSend:
            ret = handleInvocation(false, false, false, false, getRegister(NODE_STATE(ksCurThread), capRegister));
            if (unlikely(ret != EXCEPTION_NONE)) {
                irq = getActiveIRQ();
                mcsPreemptionPoint(irq);
                if (IRQT_TO_IRQ(irq) != IRQT_TO_IRQ(irqInvalid)) {
                    handleInterrupt(irq);
                }
            }
            break;

        case SysCall:
            ret = handleInvocation(true, true, true, false, getRegister(NODE_STATE(ksCurThread), capRegister));
            if (unlikely(ret != EXCEPTION_NONE)) {
                irq = getActiveIRQ();
                mcsPreemptionPoint(irq);
                if (IRQT_TO_IRQ(irq) != IRQT_TO_IRQ(irqInvalid)) {
                    handleInterrupt(irq);
                }
            }
            break;

        case SysRecv:
            handleRecv(true, true);
            break;
#ifndef CONFIG_KERNEL_MCS
        case SysReply:
            handleReply();
            break;

        case SysReplyRecv:
            handleReply();
            handleRecv(true, true);
            break;

#else /* CONFIG_KERNEL_MCS */
        case SysWait:
            handleRecv(true, false);
            break;

        case SysNBWait:
            handleRecv(false, false);
            break;
        case SysReplyRecv: {
            cptr_t reply = getRegister(NODE_STATE(ksCurThread), replyRegister);
            ret = handleInvocation(false, false, true, true, reply);
            /* reply cannot error and is not preemptible */
            assert(ret == EXCEPTION_NONE);
            handleRecv(true, true);
            break;
        }

        case SysNBSendRecv: {
            cptr_t dest = getNBSendRecvDest();
            ret = handleInvocation(false, false, true, true, dest);
            if (unlikely(ret != EXCEPTION_NONE)) {
                irq = getActiveIRQ();
                mcsPreemptionPoint(irq);
                if (IRQT_TO_IRQ(irq) != IRQT_TO_IRQ(irqInvalid)) {
                    handleInterrupt(irq);
                }
                break;
            }
            handleRecv(true, true);
            break;
        }

        case SysNBSendWait:
            ret = handleInvocation(false, false, true, true, getRegister(NODE_STATE(ksCurThread), replyRegister));
            if (unlikely(ret != EXCEPTION_NONE)) {
                irq = getActiveIRQ();
                mcsPreemptionPoint(irq);
                if (IRQT_TO_IRQ(irq) != IRQT_TO_IRQ(irqInvalid)) {
                    handleInterrupt(irq);
                }
                break;
            }
            handleRecv(true, false);
            break;
#endif
        case SysNBRecv:
            handleRecv(false, true);
            break;

        case SysYield:
            handleYield();
            break;

        default:
            fail("Invalid syscall");
        }

    })

    schedule();
    activateThread();

    return EXCEPTION_NONE;
}

#ifdef CONFIG_KERNEL_MCS
void retry_syscall_exclusive(void)
{
    NODE_TAKE_WRITE_IF_READ_HELD;
    handleSyscall(NODE_STATE(ksSyscallNumber));
    restore_user_context();
    UNREACHABLE();
}
#endif
