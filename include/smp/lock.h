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

typedef struct clh_lock {
    clh_qnode_t nodes[CONFIG_MAX_NUM_NODES + 1];
    clh_qnode_p_t node_owners[CONFIG_MAX_NUM_NODES];

    clh_qnode_t *head;
    PAD_TO_NEXT_CACHE_LN(sizeof(clh_qnode_t *));

    struct {
        word_t turn;
        PAD_TO_NEXT_CACHE_LN(sizeof(word_t));
    } current_writer_turn, completed_writer_turn;

    struct {
        word_t count;
        PAD_TO_NEXT_CACHE_LN(sizeof(word_t));
    } reader_cohorts[2];

    struct {
        word_t observed_writer_turn;
        word_t own_read_lock;
        word_t waiting_on_read_lock;
        PAD_TO_NEXT_CACHE_LN(sizeof(word_t) +
                             sizeof(word_t));
    } node_read_state[CONFIG_MAX_NUM_NODES];
} clh_lock_t;

extern clh_lock_t big_kernel_lock;
BOOT_CODE void clh_lock_init(void);

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

    big_kernel_lock.current_writer_turn.turn = !big_kernel_lock.current_writer_turn.turn;
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    while (big_kernel_lock.reader_cohorts[!big_kernel_lock.current_writer_turn.turn].count != 0) {
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

    /* make sure no resource access passes from this point */
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
}

static inline void FORCE_INLINE clh_lock_release(word_t cpu)
{
    /* make sure no resource access passes from this point */
    __atomic_thread_fence(__ATOMIC_RELEASE);

    big_kernel_lock.completed_writer_turn.turn = !big_kernel_lock.completed_writer_turn.turn;
    __atomic_thread_fence(__ATOMIC_RELEASE);

    big_kernel_lock.node_owners[cpu].node->value = CLHState_Granted;
    big_kernel_lock.node_owners[cpu].node =
        big_kernel_lock.node_owners[cpu].next;
}

static inline void FORCE_INLINE clh_lock_read_acquire(word_t cpu)
{
    big_kernel_lock.node_read_state[cpu].waiting_on_read_lock = true;
    __atomic_fetch_add(&big_kernel_lock.reader_cohorts[0].count, 1, __ATOMIC_ACQUIRE);
    __atomic_fetch_add(&big_kernel_lock.reader_cohorts[1].count, 1, __ATOMIC_ACQUIRE);
    big_kernel_lock.node_read_state[cpu].observed_writer_turn = big_kernel_lock.current_writer_turn.turn;
    __atomic_fetch_sub(&big_kernel_lock.reader_cohorts[!big_kernel_lock.node_read_state[cpu].observed_writer_turn].count, 1, __ATOMIC_ACQUIRE);

    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    while (big_kernel_lock.completed_writer_turn.turn != big_kernel_lock.node_read_state[cpu].observed_writer_turn) {
        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        if (clh_is_ipi_pending(cpu)) {
            /* we only handle irq_remote_call_ipi here as other type of IPIs
             * are async and could be delayed. 'handleIPI' may not return
             * based on value of the 'irqPath'. */
            handleIPI(CORE_IRQ_TO_IRQT(cpu, irq_remote_call_ipi), false);
            /* We do not need to perform a memory release here as we would have only modified
             * local state that we do not need to make visible */
        }
        arch_pause();
    }

    big_kernel_lock.node_read_state[cpu].own_read_lock = true;
    big_kernel_lock.node_read_state[cpu].waiting_on_read_lock = false;
}

static inline void FORCE_INLINE clh_lock_read_release(word_t cpu)
{
    __atomic_fetch_sub(&big_kernel_lock.reader_cohorts[big_kernel_lock.node_read_state[cpu].observed_writer_turn].count, 1, __ATOMIC_RELEASE);
    big_kernel_lock.node_read_state[cpu].own_read_lock = false;
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
    clh_lock_read_acquire(getCurrentCPUIndex());         \
} while (0)

#define NODE_READ_UNLOCK_ do {                           \
    clh_lock_read_release(getCurrentCPUIndex());         \
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
    else if (big_kernel_lock.node_read_state[getCurrentCPUIndex()].own_read_lock) { \
        NODE_READ_UNLOCK_;                               \
    }                                                    \
} while(0)

#define NODE_TAKE_WRITE_IF_READ_HELD_ do {               \
    if (big_kernel_lock.node_read_state[getCurrentCPUIndex()].own_read_lock) { \
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
