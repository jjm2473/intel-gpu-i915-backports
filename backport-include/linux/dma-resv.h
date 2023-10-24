#ifndef __BACKPORT_DMA_RESV_H
#define __BACKPORT_DMA_RESV_H

#include_next <linux/dma-resv.h>
#define dma_resv_test_signaled dma_resv_test_signaled_rcu
#define dma_resv_get_fences dma_resv_get_fences_rcu
#define dma_resv_get_excl_unlocked dma_resv_get_excl_rcu

#ifdef BPM_DMA_RESV_RESERVE_SHARED_NOT_PRESENT
#define dma_resv_reserve_shared dma_resv_reserve_fences
#endif

#ifdef BPM_DMA_RESV_TEST_SIGNALED_BOOLEAN_ARG_NOT_PRESENT
bool backport_dma_resv_test_signaled(struct dma_resv *obj, bool test_all);
#define dma_resv_test_signaled backport_dma_resv_test_signaled
#endif

#ifdef DMA_RESV_EXCL_UNLOCKED_NOT_PRESENT

/**
 * dma_resv_get_excl_unlocked - get the reservation object's
 * exclusive fence, without lock held.
 * @obj: the reservation object
 *
 * If there is an exclusive fence, this atomically increments it's
 * reference count and returns it.
 *
 * RETURNS
 * The exclusive fence or NULL if none
 */
#define dma_resv_get_excl_unlocked LINUX_I915_BACKPORT(dma_resv_get_excl_unlocked)
static inline struct dma_fence *
dma_resv_get_excl_unlocked(struct dma_resv *obj)
{
        struct dma_fence *fence;

        if (!rcu_access_pointer(obj->fence_excl))
                return NULL;

        rcu_read_lock();
        fence = dma_fence_get_rcu_safe(&obj->fence_excl);
        rcu_read_unlock();

        return fence;
}
#endif /* DMA_RESV_EXCL_UNLOCKED_NOT_PRESENT */

/**
 * dma_resv_excl_fence - return the object's exclusive fence
 * @obj: the reservation object
 *
 * Returns the exclusive fence (if any). Caller must either hold the objects
 * through dma_resv_lock() or the RCU read side lock through rcu_read_lock(),
 * or one of the variants of each
 *
 * RETURNS
 * The exclusive fence or NULL
 */
static inline struct dma_fence *
dma_resv_excl_fence(struct dma_resv *obj)
{
	return rcu_dereference_check(obj->fence_excl, dma_resv_held(obj));
}

/**
 * dma_resv_shared_list - get the reservation object's shared fence list
 * @obj: the reservation object
 *
 * Returns the shared fence list. Caller must either hold the objects
 * through dma_resv_lock() or the RCU read side lock through rcu_read_lock(),
 * or one of the variants of each
 */
static inline struct dma_resv_list *dma_resv_shared_list(struct dma_resv *obj)
{
	return rcu_dereference_check(obj->fence, dma_resv_held(obj));
}

#endif /* __BACKPORT_DMA_RESV_H */
