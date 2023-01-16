/*
 * Copyright 2020, Data61, CSIRO (ABN 41 687 119 230)
 *
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <config.h>
#include <arch/types.h>
#include <arch/model/statedata.h>
#include <model/statedata.h>

#ifdef ENABLE_SMP_SUPPORT

typedef struct smpStatedata {
    archNodeState_t cpu;
    nodeState_t system;
    PAD_TO_NEXT_CACHE_LN(sizeof(archNodeState_t) + sizeof(nodeState_t));
} smpStatedata_t;

extern smpStatedata_t ksSMP[CONFIG_MAX_NUM_NODES];

extern word_t scheduler_locks[8];

static inline
FORCE_INLINE
void spinlock_acquire(uint8_t *lock)
{
    while (__atomic_test_and_set(lock, __ATOMIC_SEQ_CST));
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
}

static inline
FORCE_INLINE
int spinlock_try_acquire(uint8_t *lock)
{
    if (__atomic_test_and_set(lock, __ATOMIC_SEQ_CST))
        return 0;
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    return 1;
}

static inline
FORCE_INLINE
void spinlock_release(uint8_t *lock)
{
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    __atomic_clear(lock, __ATOMIC_SEQ_CST);
}

#ifndef DISABLE_SCHEDULER_LOCKS
#define scheduler_lock_get(node) ((uint8_t *)&scheduler_locks[node])
#define scheduler_lock_acquire(node) do { spinlock_acquire(scheduler_lock_get(node)); } while (0);
#define scheduler_lock_release(node) do { spinlock_release(scheduler_lock_get(node)); } while (0);
#else
#define scheduler_lock_acquire(node) {}
#define scheduler_lock_release(node) {}
#endif

#ifndef DISABLE_ENDPOINT_LOCKS
#define ep_lock_get(ep_ptr) (&((uint8_t *)&(ep_ptr)->words[0])[0])
#define ep_lock_acquire(ep_ptr) do { spinlock_acquire(ep_lock_get(ep_ptr)); } while (0);
#define ep_lock_try_acquire(ep_ptr) (spinlock_try_acquire(ep_lock_get(ep_ptr)))
#define ep_lock_release(ep_ptr) do { spinlock_release(ep_lock_get(ep_ptr)); } while (0);
#else
#define ep_lock_acquire(ep_ptr) {}
#define ep_lock_try_acquire(ep_ptr) (1)
#define ep_lock_release(ep_ptr) {}
#endif

#ifndef DISABLE_NOTIFICATION_LOCKS
#define ntfn_lock_get(ntfn_ptr) (&((uint8_t *)&ntfn_ptr->words[0])[0])
#define ntfn_lock_acquire(ntfn_ptr) do { spinlock_acquire(ntfn_lock_get(ntfn_ptr)); } while (0);
#define ntfn_lock_try_acquire(ntfn_ptr) (spinlock_try_acquire(ntfn_lock_get(ntfn_ptr)))
#define ntfn_lock_release(ntfn_ptr) do { spinlock_release(ntfn_lock_get(ntfn_ptr)); } while (0);
#else
#define ntfn_lock_acquire(ntfn_ptr) {}
#define ntfn_lock_try_acquire(ntfn_ptr) (1)
#define ntfn_lock_release(ntfn_ptr) {}
#endif

#ifndef DISABLE_REPLY_OBJECT_LOCKS
#define reply_object_lock_get(reply_ptr) ((uint8_t *)&reply_ptr->lock)
#define reply_object_lock_acquire(reply_ptr) do { spinlock_acquire(reply_object_lock_get(reply_ptr)); } while (0);
#define reply_object_lock_try_acquire(reply_ptr) (spinlock_try_acquire(reply_object_lock_get(reply_ptr)))
#define reply_object_lock_release(reply_ptr) do { spinlock_release(reply_object_lock_get(reply_ptr)); } while (0);
#else
#define reply_object_lock_acquire(reply_ptr) {}
#define reply_object_lock_try_acquire(reply_ptr) (1)
#define reply_object_lock_release(reply_ptr) {}
#endif

void migrateTCB(tcb_t *tcb, word_t new_core);

#endif /* ENABLE_SMP_SUPPORT */

