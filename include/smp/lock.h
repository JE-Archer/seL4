/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <config.h>
#include <types.h>
#include <util.h>
#include <mode/machine.h>
#include <arch/model/statedata.h>
#include <smp/ipi.h>
#include <util.h>

#ifdef ENABLE_SMP_SUPPORT

/* CLH lock is FIFO lock for machines with coherent caches (coherent-FIFO lock).
 * See ftp://ftp.cs.washington.edu/tr/1993/02/UW-CSE-93-02-02.pdf */

typedef enum {
    CLHState_Granted = 0,
    CLHState_Pending
} clh_qnode_state_t;

typedef struct clh_qnode {
    clh_qnode_state_t value;

    PAD_TO_NEXT_CACHE_LN(sizeof(clh_qnode_state_t));
} clh_qnode_t;

typedef struct clh_qnode_p {
    clh_qnode_t *node;
    clh_qnode_t *next;
    /* This is the software IPI flag */
    word_t ipi;

    PAD_TO_NEXT_CACHE_LN(sizeof(clh_qnode_t *) +
                         sizeof(clh_qnode_t *) +
                         sizeof(word_t));
} clh_qnode_p_t;

#define W ((word_t)0)
#define R ((word_t)1)

typedef struct clh_lock {
    clh_qnode_t nodes[CONFIG_MAX_NUM_NODES + 1];
    clh_qnode_p_t node_owners[CONFIG_MAX_NUM_NODES];

    clh_qnode_t *head;
    PAD_TO_NEXT_CACHE_LN(sizeof(clh_qnode_t *));

    struct node_read_state {
        word_t w_flag, r_flag, turn;
        bool_t own_read_lock;
        bool_t waiting_on_read_lock;
        PAD_TO_NEXT_CACHE_LN(3 * sizeof(word_t) + 2 * sizeof (bool_t));
    } node_read_state[CONFIG_MAX_NUM_NODES];
} clh_lock_t;

extern clh_lock_t big_kernel_lock;
BOOT_CODE void clh_lock_init(void);

extern word_t scheduler_locks[CONFIG_MAX_NUM_NODES];
extern word_t scheduler_locks_held_by_node[CONFIG_MAX_NUM_NODES][CONFIG_MAX_NUM_NODES];

static inline bool_t FORCE_INLINE clh_is_ipi_pending(word_t cpu)
{
    return big_kernel_lock.node_owners[cpu].ipi == 1;
}

static inline void *sel4_atomic_exchange(void *ptr, bool_t
                                         irqPath, word_t cpu, int memorder)
{
    clh_qnode_t *prev;

    if (memorder == __ATOMIC_RELEASE || memorder == __ATOMIC_ACQ_REL) {
        __atomic_thread_fence(__ATOMIC_RELEASE);
    } else if (memorder == __ATOMIC_SEQ_CST) {
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
    }

    while (!try_arch_atomic_exchange_rlx(&big_kernel_lock.head,
                                         (void *) big_kernel_lock.node_owners[cpu].node,
                                         (void **) &prev)) {
        if (clh_is_ipi_pending(cpu)) {
            /* we only handle irq_remote_call_ipi here as other type of IPIs
             * are async and could be delayed. 'handleIPI' may not return
             * based on value of the 'irqPath'. */
            handleIPI(CORE_IRQ_TO_IRQT(cpu, irq_remote_call_ipi), irqPath);
        }

        arch_pause();
    }

    if (memorder == __ATOMIC_ACQUIRE || memorder == __ATOMIC_ACQ_REL) {
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
    } else if (memorder == __ATOMIC_SEQ_CST) {
        __atomic_thread_fence(__ATOMIC_SEQ_CST);
    }

    return prev;
}

static inline void FORCE_INLINE clh_lock_acquire(word_t cpu, bool_t irqPath)
{
    clh_qnode_t *prev;
    big_kernel_lock.node_owners[cpu].node->value = CLHState_Pending;

    prev = sel4_atomic_exchange(&big_kernel_lock.head, irqPath, cpu, __ATOMIC_ACQ_REL);

    big_kernel_lock.node_owners[cpu].next = prev;

    /* We do not have an __atomic_thread_fence here as this is already handled by the
     * atomic_exchange just above */
    while (big_kernel_lock.node_owners[cpu].next->value != CLHState_Granted) {
        /* As we are in a loop we need to ensure that any loads of future iterations of the
         * loop are performed after this one */
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        if (clh_is_ipi_pending(cpu)) {
            /* we only handle irq_remote_call_ipi here as other type of IPIs
             * are async and could be delayed. 'handleIPI' may not return
             * based on value of the 'irqPath'. */
            handleIPI(CORE_IRQ_TO_IRQT(cpu, irq_remote_call_ipi), irqPath);
            /* We do not need to perform a memory release here as we would have only modified
             * local state that we do not need to make visible */
        }
        arch_pause();
    }

    __atomic_thread_fence(__ATOMIC_ACQUIRE);

    for (word_t node = 0; node < CONFIG_MAX_NUM_NODES; node++) {
        struct node_read_state *s = &big_kernel_lock.node_read_state[node];
        s->w_flag = true;
        s->turn = R;
    }

    __atomic_thread_fence(__ATOMIC_ACQ_REL);

    for (word_t node = 0; node < CONFIG_MAX_NUM_NODES; node++) {
        struct node_read_state *s = &big_kernel_lock.node_read_state[node];
        while (s->r_flag && s->turn == R) {
            __atomic_thread_fence(__ATOMIC_ACQUIRE);
        }
    }

    /* make sure no resource access passes from this point */
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
}

static inline void FORCE_INLINE clh_lock_release(word_t cpu)
{
    /* make sure no resource access passes from this point */
    __atomic_thread_fence(__ATOMIC_RELEASE);

    for (word_t node = 0; node < CONFIG_MAX_NUM_NODES; node++) {
        struct node_read_state *s = &big_kernel_lock.node_read_state[node];
        s->w_flag = false;
    }

    big_kernel_lock.node_owners[cpu].node->value = CLHState_Granted;
    big_kernel_lock.node_owners[cpu].node =
        big_kernel_lock.node_owners[cpu].next;
}

static inline bool_t FORCE_INLINE is_self_waiting_on_read_lock(void)
{
    return big_kernel_lock.node_read_state[getCurrentCPUIndex()].waiting_on_read_lock;
}

static inline bool_t FORCE_INLINE does_self_own_read_lock(void)
{
    return big_kernel_lock.node_read_state[getCurrentCPUIndex()].own_read_lock;
}

static inline void FORCE_INLINE clh_lock_read_acquire(void)
{
    struct node_read_state *s = &big_kernel_lock.node_read_state[getCurrentCPUIndex()];

    s->r_flag = true;
    s->turn = W;

    __atomic_thread_fence(__ATOMIC_ACQ_REL);

    big_kernel_lock.node_read_state[getCurrentCPUIndex()].waiting_on_read_lock = true;
    while (s->w_flag && s->turn == W) {
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        if (clh_is_ipi_pending(getCurrentCPUIndex())) {
            /* we only handle irq_remote_call_ipi here as other type of IPIs
             * are async and could be delayed. 'handleIPI' may not return
             * based on value of the 'irqPath'. */
            handleIPI(CORE_IRQ_TO_IRQT(getCurrentCPUIndex(), irq_remote_call_ipi), false);
            /* We do not need to perform a memory release here as we would have only modified
             * local state that we do not need to make visible */
        }
        arch_pause();
    }

    s->waiting_on_read_lock = false;
    s->own_read_lock = true;

    /* make sure no resource access passes from this point */
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
}

static inline void FORCE_INLINE clh_lock_read_release(void)
{
    __atomic_thread_fence(__ATOMIC_RELEASE);
    struct node_read_state *s = &big_kernel_lock.node_read_state[getCurrentCPUIndex()];
    s->r_flag = false;
    s->own_read_lock = false;
}

static inline bool_t FORCE_INLINE clh_is_self_in_queue(void)
{
    return big_kernel_lock.node_owners[getCurrentCPUIndex()].node->value == CLHState_Pending;
}

#define NODE_LOCK(_irqPath) do {                         \
    clh_lock_acquire(getCurrentCPUIndex(), _irqPath);    \
} while(0)

#define NODE_UNLOCK do {                                 \
    clh_lock_release(getCurrentCPUIndex());              \
} while(0)

#define NODE_READ_LOCK_ do {                             \
    clh_lock_read_acquire();         \
} while (0)

#define NODE_READ_UNLOCK_ do {                           \
    clh_lock_read_release();         \
} while (0)

#define NODE_LOCK_IF(_cond, _irqPath) do {               \
    if((_cond)) {                                        \
        NODE_LOCK(_irqPath);                             \
    }                                                    \
} while(0)

#define NODE_UNLOCK_IF_HELD do {                         \
    if(clh_is_self_in_queue()) {                         \
        NODE_UNLOCK;                                     \
    }                                                    \
    else if (does_self_own_read_lock()) { \
        NODE_READ_UNLOCK_;                               \
    }                                                    \
} while(0)

#define NODE_TAKE_WRITE_IF_READ_HELD_ do {               \
    if (does_self_own_read_lock()) { \
        NODE_READ_UNLOCK_;                               \
        NODE_LOCK_SYS;                                   \
    }                                                    \
} while(0)

#else
#define NODE_LOCK(_irq) do {} while (0)
#define NODE_UNLOCK do {} while (0)
#define NODE_LOCK_IF(_cond, _irq) do {} while (0)
#define NODE_UNLOCK_IF_HELD do {} while (0)
#define NODE_READ_LOCK_ do {} while (0)
#define NODE_READ_UNLOCK_ do {} while (0)
#define NODE_TAKE_WRITE_IF_READ_HELD_ do {} while (0)
#endif /* ENABLE_SMP_SUPPORT */

#define NODE_LOCK_SYS NODE_LOCK(false)
#define NODE_LOCK_IRQ NODE_LOCK(true)
#define NODE_LOCK_SYS_IF(_cond) NODE_LOCK_IF(_cond, false)
#define NODE_LOCK_IRQ_IF(_cond) NODE_LOCK_IF(_cond, true)
#define NODE_READ_LOCK NODE_READ_LOCK_
#define NODE_READ_UNLOCK NODE_READ_UNLOCK_
#define NODE_TAKE_WRITE_IF_READ_HELD NODE_TAKE_WRITE_IF_READ_HELD_

static bool_t clh_is_self_in_queue(void);

static inline
FORCE_INLINE
void spinlock_acquire(uint8_t *lock)
{
    if (clh_is_self_in_queue())
        return;
    uint8_t expected = 0;
    while (!__atomic_compare_exchange_n(lock, &expected, (uint8_t)(getCurrentCPUIndex()+1), true, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)) {
        expected = 0;
    }
//    __atomic_thread_fence(__ATOMIC_SEQ_CST);
}

static inline
FORCE_INLINE
int spinlock_try_acquire(uint8_t *lock)
{
    if (clh_is_self_in_queue())
        return 1;
    uint8_t expected = 0;
    if (!__atomic_compare_exchange_n(lock, &expected, (uint8_t)(getCurrentCPUIndex()+1), false, __ATOMIC_ACQUIRE, __ATOMIC_RELAXED)) {
        return 0;
    }
//    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    return 1;
}

static inline
FORCE_INLINE
void spinlock_release(uint8_t *lock)
{
    if (clh_is_self_in_queue())
        return;
//    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    __atomic_store_n(lock, (uint8_t)0, __ATOMIC_RELEASE);
}

//#define DISABLE_SCHEDULER_LOCKS
//#define DISABLE_ENDPOINT_LOCKS
//#define DISABLE_NOTIFICATION_LOCKS
//#define DISABLE_REPLY_OBJECT_LOCKS

#ifndef DISABLE_SCHEDULER_LOCKS
#define scheduler_lock_get(node) ((uint8_t *)&scheduler_locks[node])
#define scheduler_lock_acquire(node) do { \
    spinlock_acquire(scheduler_lock_get(node)); \
} while (0);
#define scheduler_lock_try_acquire(node) (spinlock_try_acquire(scheduler_lock_get(node)))
#define scheduler_lock_release(node) do { spinlock_release(scheduler_lock_get(node)); } while (0);
#else
void scheduler_lock_acquire(seL4_Word core) {}
void scheduler_lock_release(seL4_Word core) {}
#endif

#ifndef DISABLE_ENDPOINT_LOCKS
#define ep_lock_get(ep_ptr) (&((uint8_t *)&(ep_ptr)->words[0])[0])
#define ep_lock_acquire(ep_ptr) do { \
    spinlock_acquire(ep_lock_get(ep_ptr)); \
} while (0);
#define ep_lock_try_acquire(ep_ptr) (spinlock_try_acquire(ep_lock_get(ep_ptr)))
#define ep_lock_release(ep_ptr) do { spinlock_release(ep_lock_get(ep_ptr)); } while (0);
#else
#define ep_lock_acquire(ep_ptr) {}
#define ep_lock_try_acquire(ep_ptr) (1)
#define ep_lock_release(ep_ptr) {}
#endif

#ifndef DISABLE_NOTIFICATION_LOCKS
#define ntfn_lock_get(ntfn_ptr) (&((uint8_t *)&ntfn_ptr->words[0])[0])
#define ntfn_lock_acquire(ntfn_ptr) do { \
    spinlock_acquire(ntfn_lock_get(ntfn_ptr)); \
} while (0);
#define ntfn_lock_try_acquire(ntfn_ptr) (spinlock_try_acquire(ntfn_lock_get(ntfn_ptr)))
#define ntfn_lock_release(ntfn_ptr) do { spinlock_release(ntfn_lock_get(ntfn_ptr)); } while (0);
#else
#define ntfn_lock_acquire(ntfn_ptr) {}
#define ntfn_lock_try_acquire(ntfn_ptr) (1)
#define ntfn_lock_release(ntfn_ptr) {}
#endif

#ifndef DISABLE_REPLY_OBJECT_LOCKS
#define reply_object_lock_get(reply_ptr) ((uint8_t *)&reply_ptr->lock)
#define reply_object_lock_acquire(reply_ptr) do { \
    spinlock_acquire(reply_object_lock_get(reply_ptr)); \
} while (0);
#define reply_object_lock_try_acquire(reply_ptr) (spinlock_try_acquire(reply_object_lock_get(reply_ptr)))
#define reply_object_lock_release(reply_ptr) do { spinlock_release(reply_object_lock_get(reply_ptr)); } while (0);
#else
#define reply_object_lock_acquire(reply_ptr) {}
#define reply_object_lock_try_acquire(reply_ptr) (1)
#define reply_object_lock_release(reply_ptr) {}
#endif

