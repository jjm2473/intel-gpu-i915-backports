/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2018 Intel Corporation
 */

#include <linux/mutex.h>

#include "i915_drv.h"
#include "i915_globals.h"
#include "i915_request.h"
#include "i915_scheduler.h"

static struct i915_global_scheduler {
	struct i915_global base;
	struct kmem_cache *slab_dependencies;
	struct kmem_cache *slab_priorities;
} global;

static DEFINE_SPINLOCK(schedule_lock);

static struct i915_sched_node *node_get(struct i915_sched_node *node)
{
	i915_request_get(container_of(node, struct i915_request, sched));
	return node;
}

static void node_put(struct i915_sched_node *node)
{
	i915_request_put(container_of(node, struct i915_request, sched));
}

static const struct i915_request *
node_to_request(const struct i915_sched_node *node)
{
	return container_of(node, const struct i915_request, sched);
}

static inline bool node_started(const struct i915_sched_node *node)
{
	return i915_request_started(node_to_request(node));
}

static inline bool node_signaled(const struct i915_sched_node *node)
{
	return i915_request_completed(node_to_request(node));
}

static inline struct i915_priolist *to_priolist(struct rb_node *rb)
{
	return rb_entry(rb, struct i915_priolist, node);
}

static void assert_priolists(struct i915_sched_engine * const sched_engine)
{
	struct rb_node *rb;
	long last_prio;

	if (!IS_ENABLED(CPTCFG_DRM_I915_DEBUG_GEM))
		return;

	GEM_BUG_ON(rb_first_cached(&sched_engine->queue) !=
		   rb_first(&sched_engine->queue.rb_root));

	last_prio = INT_MAX;
	for (rb = rb_first_cached(&sched_engine->queue); rb; rb = rb_next(rb)) {
		const struct i915_priolist *p = to_priolist(rb);

		GEM_BUG_ON(p->priority > last_prio);
		last_prio = p->priority;
	}
}

struct list_head *
i915_sched_lookup_priolist(struct i915_sched_engine *sched_engine, int prio)
{
	struct i915_priolist *p;
	struct rb_node **parent, *rb;
	bool first = true;

	lockdep_assert_held(&sched_engine->lock);
	assert_priolists(sched_engine);

	if (unlikely(sched_engine->no_priolist))
		prio = I915_PRIORITY_NORMAL;

find_priolist:
	/* most positive priority is scheduled first, equal priorities fifo */
	rb = NULL;
	parent = &sched_engine->queue.rb_root.rb_node;
	while (*parent) {
		rb = *parent;
		p = to_priolist(rb);
		if (prio > p->priority) {
			parent = &rb->rb_left;
		} else if (prio < p->priority) {
			parent = &rb->rb_right;
			first = false;
		} else {
			return &p->requests;
		}
	}

	if (prio == I915_PRIORITY_NORMAL) {
		p = &sched_engine->default_priolist;
	} else {
		p = kmem_cache_alloc(global.slab_priorities, GFP_ATOMIC);
		/* Convert an allocation failure to a priority bump */
		if (unlikely(!p)) {
			prio = I915_PRIORITY_NORMAL; /* recurses just once */

			/* To maintain ordering with all rendering, after an
			 * allocation failure we have to disable all scheduling.
			 * Requests will then be executed in fifo, and schedule
			 * will ensure that dependencies are emitted in fifo.
			 * There will be still some reordering with existing
			 * requests, so if userspace lied about their
			 * dependencies that reordering may be visible.
			 */
			sched_engine->no_priolist = true;
			goto find_priolist;
		}
	}

	p->priority = prio;
	INIT_LIST_HEAD(&p->requests);

	rb_link_node(&p->node, rb, parent);
	rb_insert_color_cached(&p->node, &sched_engine->queue, first);

	return &p->requests;
}

void __i915_priolist_free(struct i915_priolist *p)
{
	kmem_cache_free(global.slab_priorities, p);
}

struct sched_cache {
	struct list_head *priolist;
};

static struct i915_sched_engine *
lock_sched_engine(struct i915_sched_node *node,
		  struct i915_sched_engine *locked,
		  struct sched_cache *cache)
{
	const struct i915_request *rq = node_to_request(node);
	struct i915_sched_engine *sched_engine;

	GEM_BUG_ON(!locked);

	/*
	 * Virtual engines complicate acquiring the engine timeline lock,
	 * as their rq->engine pointer is not stable until under that
	 * engine lock. The simple ploy we use is to take the lock then
	 * check that the rq still belongs to the newly locked engine.
	 */
	while (locked != (sched_engine = READ_ONCE(rq->engine)->sched_engine)) {
		spin_unlock(&locked->lock);
		memset(cache, 0, sizeof(*cache));
		spin_lock(&sched_engine->lock);
		locked = sched_engine;
	}

	GEM_BUG_ON(locked != sched_engine);
	return locked;
}

static void __i915_schedule(struct i915_sched_node *node,
			    const struct i915_sched_attr *attr)
{
	const int prio = max(attr->priority, node->attr.priority);
	struct i915_sched_engine *sched_engine;
	struct i915_dependency *dep, *p;
	struct i915_dependency stack;
	struct sched_cache cache;
	LIST_HEAD(dfs);

	/* Needed in order to use the temporary link inside i915_dependency */
	lockdep_assert_held(&schedule_lock);
	GEM_BUG_ON(prio == I915_PRIORITY_INVALID);

	if (node_signaled(node))
		return;

	stack.signaler = node;
	list_add(&stack.dfs_link, &dfs);

	/*
	 * Recursively bump all dependent priorities to match the new request.
	 *
	 * A naive approach would be to use recursion:
	 * static void update_priorities(struct i915_sched_node *node, prio) {
	 *	list_for_each_entry(dep, &node->signalers_list, signal_link)
	 *		update_priorities(dep->signal, prio)
	 *	queue_request(node);
	 * }
	 * but that may have unlimited recursion depth and so runs a very
	 * real risk of overunning the kernel stack. Instead, we build
	 * a flat list of all dependencies starting with the current request.
	 * As we walk the list of dependencies, we add all of its dependencies
	 * to the end of the list (this may include an already visited
	 * request) and continue to walk onwards onto the new dependencies. The
	 * end result is a topological list of requests in reverse order, the
	 * last element in the list is the request we must execute first.
	 */
	list_for_each_entry(dep, &dfs, dfs_link) {
		struct i915_sched_node *node = dep->signaler;

		/* If we are already flying, we know we have no signalers */
		if (node_started(node))
			continue;

		/*
		 * Within an engine, there can be no cycle, but we may
		 * refer to the same dependency chain multiple times
		 * (redundant dependencies are not eliminated) and across
		 * engines.
		 */
		list_for_each_entry(p, &node->signalers_list, signal_link) {
			GEM_BUG_ON(p == dep); /* no cycles! */

			if (node_signaled(p->signaler))
				continue;

			if (prio > READ_ONCE(p->signaler->attr.priority))
				list_move_tail(&p->dfs_link, &dfs);
		}
	}

	/*
	 * If we didn't need to bump any existing priorities, and we haven't
	 * yet submitted this request (i.e. there is no potential race with
	 * execlists_submit_request()), we can set our own priority and skip
	 * acquiring the engine locks.
	 */
	if (node->attr.priority == I915_PRIORITY_INVALID) {
		GEM_BUG_ON(!list_empty(&node->link));
		node->attr = *attr;

		if (stack.dfs_link.next == stack.dfs_link.prev)
			return;

		__list_del_entry(&stack.dfs_link);
	}

	memset(&cache, 0, sizeof(cache));
	sched_engine = node_to_request(node)->engine->sched_engine;
	spin_lock(&sched_engine->lock);

	/* Fifo and depth-first replacement ensure our deps execute before us */
	sched_engine = lock_sched_engine(node, sched_engine, &cache);
	list_for_each_entry_safe_reverse(dep, p, &dfs, dfs_link) {
		struct i915_request *from = container_of(dep->signaler,
							 struct i915_request,
							 sched);
		INIT_LIST_HEAD(&dep->dfs_link);

		node = dep->signaler;
		sched_engine = lock_sched_engine(node, sched_engine, &cache);
		lockdep_assert_held(&sched_engine->lock);

		/* Recheck after acquiring the engine->timeline.lock */
		if (prio <= node->attr.priority || node_signaled(node))
			continue;

		GEM_BUG_ON(node_to_request(node)->engine->sched_engine !=
			   sched_engine);

		/* Must be called before changing the nodes priority */
		if (sched_engine->bump_inflight_request_prio)
			sched_engine->bump_inflight_request_prio(from, prio);

		WRITE_ONCE(node->attr.priority, prio);

		/*
		 * Once the request is ready, it will be placed into the
		 * priority lists and then onto the HW runlist. Before the
		 * request is ready, it does not contribute to our preemption
		 * decisions and we can safely ignore it, as it will, and
		 * any preemption required, be dealt with upon submission.
		 * See engine->submit_request()
		 */
		if (list_empty(&node->link))
			continue;

		if (i915_request_in_priority_queue(node_to_request(node))) {
			if (!cache.priolist)
				cache.priolist =
					i915_sched_lookup_priolist(sched_engine,
								   prio);
			list_move_tail(&node->link, cache.priolist);
		}

		/* Defer (tasklet) submission until after all of our updates. */
		if (sched_engine->kick_backend)
			sched_engine->kick_backend(node_to_request(node), prio);
	}

	spin_unlock(&sched_engine->lock);
}

void i915_schedule(struct i915_request *rq, const struct i915_sched_attr *attr)
{
	spin_lock_irq(&schedule_lock);
	__i915_schedule(&rq->sched, attr);
	spin_unlock_irq(&schedule_lock);
}

void i915_sched_node_init(struct i915_sched_node *node)
{
	spin_lock_init(&node->lock);

	INIT_LIST_HEAD(&node->signalers_list);
	INIT_LIST_HEAD(&node->waiters_list);
	INIT_LIST_HEAD(&node->link);

	i915_sched_node_reinit(node);
}

void i915_sched_node_reinit(struct i915_sched_node *node)
{
	node->attr.priority = I915_PRIORITY_INVALID;
	node->semaphores = 0;
	node->flags = 0;

	GEM_BUG_ON(!list_empty(&node->signalers_list));
	GEM_BUG_ON(!list_empty(&node->waiters_list));
	GEM_BUG_ON(!list_empty(&node->link));
}

static struct i915_dependency *
i915_dependency_alloc(void)
{
	return kmem_cache_alloc(global.slab_dependencies, GFP_KERNEL);
}

static void
rcu_dependency_free(struct rcu_head *rcu)
{
	kmem_cache_free(global.slab_dependencies,
			container_of(rcu, typeof(struct i915_dependency), rcu));
}

static void
i915_dependency_free(struct i915_dependency *dep)
{
	call_rcu(&dep->rcu, rcu_dependency_free);
}

bool __i915_sched_node_add_dependency(struct i915_sched_node *node,
				      struct i915_sched_node *signal,
				      struct i915_dependency *dep,
				      unsigned long flags)
{
	bool ret = false;

	/* The signal->lock is always the outer lock in this double-lock. */
	spin_lock(&signal->lock);

	if (!node_signaled(signal)) {
		INIT_LIST_HEAD(&dep->dfs_link);
		dep->signaler = signal;
		dep->waiter = node_get(node);
		dep->flags = flags;

		/* All set, now publish. Beware the lockless walkers. */
		spin_lock_nested(&node->lock, SINGLE_DEPTH_NESTING);
		list_add_rcu(&dep->signal_link, &node->signalers_list);
		list_add_rcu(&dep->wait_link, &signal->waiters_list);
		spin_unlock(&node->lock);

		/* Propagate the chains */
		node->flags |= signal->flags;
		ret = true;
	}

	spin_unlock(&signal->lock);

	return ret;
}

int i915_sched_node_add_dependency(struct i915_sched_node *node,
				   struct i915_sched_node *signal,
				   unsigned long flags)
{
	struct i915_dependency *dep;

	dep = i915_dependency_alloc();
	if (!dep)
		return -ENOMEM;

	if (!__i915_sched_node_add_dependency(node, signal, dep,
					      flags | I915_DEPENDENCY_ALLOC))
		i915_dependency_free(dep);

	return 0;
}

void i915_sched_node_retire(struct i915_sched_node *node)
{
	struct i915_dependency *dep, *tmp;
	LIST_HEAD(waiters);

	/*
	 * Everyone we depended upon (the fences we wait to be signaled)
	 * should retire before us and remove themselves from our list.
	 * However, retirement is run independently on each timeline and
	 * so we may be called out-of-order. As we need to avoid taking
	 * the signaler's lock, just mark up our completion and be wary
	 * in traversing the signalers->waiters_list.
	 */

	/* Remove ourselves from everyone who depends upon us */
	spin_lock(&node->lock);
	if (!list_empty(&node->waiters_list)) {
		list_replace_rcu(&node->waiters_list, &waiters);
		INIT_LIST_HEAD_RCU(&node->waiters_list);
	}
	spin_unlock(&node->lock);

	list_for_each_entry_safe(dep, tmp, &waiters, wait_link) {
		struct i915_sched_node *w = dep->waiter;

		GEM_BUG_ON(dep->signaler != node);

		spin_lock(&w->lock);
		list_del_rcu(&dep->signal_link);
		spin_unlock(&w->lock);
		node_put(w);

		if (dep->flags & I915_DEPENDENCY_ALLOC)
			i915_dependency_free(dep);
	}
}

void i915_request_show_with_schedule(struct drm_printer *m,
				     const struct i915_request *rq,
				     const char *prefix,
				     int indent)
{
	struct i915_dependency *dep;

	i915_request_show(m, rq, prefix, indent);
	if (i915_request_completed(rq))
		return;

	rcu_read_lock();
	for_each_signaler(dep, rq) {
		const struct i915_request *signaler =
			node_to_request(dep->signaler);

		/* Dependencies along the same timeline are expected. */
		if (signaler->timeline == rq->timeline)
			continue;

		if (__i915_request_is_complete(signaler))
			continue;

		i915_request_show(m, signaler, prefix, indent + 2);
	}
	rcu_read_unlock();
}

static void default_destroy(struct kref *kref)
{
	struct i915_sched_engine *sched_engine =
		container_of(kref, typeof(*sched_engine), ref);

	tasklet_kill(&sched_engine->tasklet); /* flush the callback */
	kfree(sched_engine);
}

static bool default_disabled(struct i915_sched_engine *sched_engine)
{
	return false;
}

struct i915_sched_engine *
i915_sched_engine_create(unsigned int subclass)
{
	struct i915_sched_engine *sched_engine;

	sched_engine = kzalloc(sizeof(*sched_engine), GFP_KERNEL);
	if (!sched_engine)
		return NULL;

	kref_init(&sched_engine->ref);

	sched_engine->queue = RB_ROOT_CACHED;
	sched_engine->queue_priority_hint = INT_MIN;
	sched_engine->destroy = default_destroy;
	sched_engine->disabled = default_disabled;

	INIT_LIST_HEAD(&sched_engine->requests);
	INIT_LIST_HEAD(&sched_engine->hold);

	spin_lock_init(&sched_engine->lock);
	lockdep_set_subclass(&sched_engine->lock, subclass);
	mark_lock_used_irq(&sched_engine->lock);

	/*
	 * Due to an interesting quirk in lockdep's internal debug tracking,
	 * after setting a subclass we must ensure the lock is used. Otherwise,
	 * nr_unused_locks is incremented once too often.
	 */
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	local_irq_disable();
	lock_map_acquire(&sched_engine->lock.dep_map);
	lock_map_release(&sched_engine->lock.dep_map);
	local_irq_enable();
#endif

	return sched_engine;
}

static void i915_global_scheduler_show(struct drm_printer *p)
{
	i915_globals_show_slab(global.slab_dependencies, "i915_dependency", p);
	i915_globals_show_slab(global.slab_priorities, "i915_priolist", p);
}

static void i915_global_scheduler_shrink(void)
{
	kmem_cache_shrink(global.slab_dependencies);
	kmem_cache_shrink(global.slab_priorities);
}

static void i915_global_scheduler_exit(void)
{
	kmem_cache_destroy(global.slab_dependencies);
	kmem_cache_destroy(global.slab_priorities);
}

static struct i915_global_scheduler global = { {
	.show = i915_global_scheduler_show,
	.shrink = i915_global_scheduler_shrink,
	.exit = i915_global_scheduler_exit,
} };

int __init i915_global_scheduler_init(void)
{
	global.slab_dependencies = KMEM_CACHE(i915_dependency,
					      SLAB_HWCACHE_ALIGN);
	if (!global.slab_dependencies)
		return -ENOMEM;

	global.slab_priorities = KMEM_CACHE(i915_priolist, 0);
	if (!global.slab_priorities)
		goto err_priorities;

	i915_global_register(&global.base);
	return 0;

err_priorities:
	kmem_cache_destroy(global.slab_priorities);
	return -ENOMEM;
}