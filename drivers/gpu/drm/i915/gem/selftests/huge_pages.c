/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2017 Intel Corporation
 */

#include <linux/prime_numbers.h>
#include <linux/string_helpers.h>
#include <linux/swap.h>

#include "i915_selftest.h"

#include "gem/i915_gem_internal.h"
#include "gem/i915_gem_lmem.h"
#include "gem/i915_gem_pm.h"
#include "gem/i915_gem_region.h"

#include "gt/intel_gt.h"

#include "igt_gem_utils.h"
#include "mock_context.h"

#include "selftests/mock_drm.h"
#include "selftests/mock_gem_device.h"
#include "selftests/mock_region.h"
#include "selftests/i915_random.h"

static struct i915_gem_context *hugepage_ctx(struct drm_i915_private *i915,
					     struct file *file)
{
	struct i915_gem_context *ctx = live_context(i915, file);
	struct i915_address_space *vm;

	if (IS_ERR(ctx))
		return ctx;

	vm = ctx->vm;
	if (vm)
		WRITE_ONCE(vm->scrub_64K, true);

	return ctx;
}

static const unsigned int page_sizes[] = {
	I915_GTT_PAGE_SIZE_2M,
	I915_GTT_PAGE_SIZE_64K,
	I915_GTT_PAGE_SIZE_4K,
};

static unsigned int get_largest_page_size(struct drm_i915_private *i915,
					  u64 rem)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(page_sizes); ++i) {
		unsigned int page_size = page_sizes[i];

		if (HAS_PAGE_SIZES(i915, page_size) && rem >= page_size)
			return page_size;
	}

	return 0;
}

static void huge_pages_free_pages(struct sg_table *st)
{
	struct scatterlist *sg;

	for (sg = st->sgl; sg; sg = __sg_next(sg)) {
		if (sg_page(sg))
			__free_pages(sg_page(sg), get_order(sg->length));
	}

	sg_free_table(st);
	kfree(st);
}

static int get_huge_pages(struct drm_i915_gem_object *obj)
{
#define GFP (GFP_KERNEL | __GFP_NOWARN | __GFP_NORETRY)
	unsigned int page_mask = obj->mm.page_mask;
	struct sg_table *st;
	struct scatterlist *sg;
	unsigned int sg_page_sizes;
	u64 rem;

	st = kmalloc(sizeof(*st), GFP);
	if (!st)
		return -ENOMEM;

	if (sg_alloc_table(st, obj->base.size >> PAGE_SHIFT, GFP)) {
		kfree(st);
		return -ENOMEM;
	}

	rem = obj->base.size;
	sg = st->sgl;
	st->nents = 0;
	sg_page_sizes = 0;

	/*
	 * Our goal here is simple, we want to greedily fill the object from
	 * largest to smallest page-size, while ensuring that we use *every*
	 * page-size as per the given page-mask.
	 */
	do {
		unsigned int bit = ilog2(page_mask);
		unsigned int page_size = BIT(bit);
		int order = get_order(page_size);

		do {
			struct page *page;

			GEM_BUG_ON(order >= MAX_ORDER);
			page = alloc_pages(GFP | __GFP_ZERO, order);
			if (!page)
				goto err;

			sg_set_page(sg, page, page_size, 0);
			sg_page_sizes |= page_size;
			st->nents++;

			rem -= page_size;
			if (!rem) {
				sg_mark_end(sg);
				break;
			}

			sg = __sg_next(sg);
		} while ((rem - ((page_size-1) & page_mask)) >= page_size);

		page_mask &= (page_size-1);
	} while (page_mask);

	if (i915_gem_gtt_prepare_pages(obj, st))
		goto err;

	GEM_BUG_ON(sg_page_sizes != obj->mm.page_mask);
	__i915_gem_object_set_pages(obj, st, sg_page_sizes);

	return 0;

err:
	sg_set_page(sg, NULL, 0, 0);
	sg_mark_end(sg);
	huge_pages_free_pages(st);

	return -ENOMEM;
}

static int put_huge_pages(struct drm_i915_gem_object *obj,
			   struct sg_table *pages)
{
	i915_gem_gtt_finish_pages(obj, pages);
	huge_pages_free_pages(pages);

	__start_cpu_write(obj);

	return 0;
}

static const struct drm_i915_gem_object_ops huge_page_ops = {
	.name = "huge-gem",
	.flags = I915_GEM_OBJECT_IS_SHRINKABLE,
	.get_pages = get_huge_pages,
	.put_pages = put_huge_pages,
};

static int fake_get_huge_pages(struct drm_i915_gem_object *obj)
{
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	const u64 max_len = rounddown_pow_of_two(UINT_MAX);
	struct sg_table *st;
	struct scatterlist *sg;
	unsigned int sg_page_sizes;
	u64 rem;

	st = kmalloc(sizeof(*st), GFP);
	if (!st)
		return -ENOMEM;

	if (sg_alloc_table(st, obj->base.size >> PAGE_SHIFT, GFP)) {
		kfree(st);
		return -ENOMEM;
	}

	/* Use optimal page sized chunks to fill in the sg table */
	rem = obj->base.size;
	sg = st->sgl;
	st->nents = 0;
	sg_page_sizes = 0;
	do {
		unsigned int page_size = get_largest_page_size(i915, rem);
		unsigned int len = min(page_size * div_u64(rem, page_size),
				       max_len);

		GEM_BUG_ON(!page_size);

		sg->offset = 0;
		sg->length = len;
		sg_dma_len(sg) = len;
		sg_dma_address(sg) = page_size;

		sg_page_sizes |= len;

		st->nents++;

		rem -= len;
		if (!rem) {
			sg_mark_end(sg);
			break;
		}

		sg = sg_next(sg);
	} while (1);

	i915_sg_trim(st);

	__i915_gem_object_set_pages(obj, st, sg_page_sizes);

	return 0;
}

static int fake_get_huge_pages_single(struct drm_i915_gem_object *obj)
{
	struct drm_i915_private *i915 = to_i915(obj->base.dev);
	struct sg_table *st;
	struct scatterlist *sg;
	unsigned int page_size;

	st = kmalloc(sizeof(*st), GFP);
	if (!st)
		return -ENOMEM;

	if (sg_alloc_table(st, 1, GFP)) {
		kfree(st);
		return -ENOMEM;
	}

	sg = st->sgl;
	st->nents = 1;

	page_size = get_largest_page_size(i915, obj->base.size);
	GEM_BUG_ON(!page_size);

	sg->offset = 0;
	sg->length = obj->base.size;
	sg_dma_len(sg) = obj->base.size;
	sg_dma_address(sg) = page_size;

	__i915_gem_object_set_pages(obj, st, sg->length);

	return 0;
#undef GFP
}

static void fake_free_huge_pages(struct drm_i915_gem_object *obj,
				 struct sg_table *pages)
{
	sg_free_table(pages);
	kfree(pages);
}

static int fake_put_huge_pages(struct drm_i915_gem_object *obj,
				struct sg_table *pages)
{
	fake_free_huge_pages(obj, pages);
	return 0;
}

static const struct drm_i915_gem_object_ops fake_ops = {
	.name = "fake-gem",
	.flags = I915_GEM_OBJECT_IS_SHRINKABLE,
	.get_pages = fake_get_huge_pages,
	.put_pages = fake_put_huge_pages,
};

static const struct drm_i915_gem_object_ops fake_ops_single = {
	.name = "fake-gem",
	.flags = I915_GEM_OBJECT_IS_SHRINKABLE,
	.get_pages = fake_get_huge_pages_single,
	.put_pages = fake_put_huge_pages,
};

static struct drm_i915_gem_object *
fake_huge_pages_object(struct drm_i915_private *i915, u64 size, bool single)
{
	static struct lock_class_key lock_class;
	struct drm_i915_gem_object *obj;

	GEM_BUG_ON(!size);
	GEM_BUG_ON(!IS_ALIGNED(size, I915_GTT_PAGE_SIZE));

	if (size >> PAGE_SHIFT > UINT_MAX)
		return ERR_PTR(-E2BIG);

	if (overflows_type(size, obj->base.size))
		return ERR_PTR(-E2BIG);

	obj = i915_gem_object_alloc();
	if (!obj)
		return ERR_PTR(-ENOMEM);

	drm_gem_private_object_init(&i915->drm, &obj->base, size);

	if (single)
		i915_gem_object_init(obj, &fake_ops_single, &lock_class, 0);
	else
		i915_gem_object_init(obj, &fake_ops, &lock_class, 0);

	i915_gem_object_set_volatile(obj);

	obj->write_domain = I915_GEM_DOMAIN_CPU;
	obj->read_domains = I915_GEM_DOMAIN_CPU;
	obj->pat_index = i915_gem_get_pat_index(i915, I915_CACHE_NONE);

	return obj;
}

static int igt_check_page_sizes(struct i915_vma *vma)
{
	struct drm_i915_private *i915 = vma->vm->i915;
	unsigned int supported = INTEL_INFO(i915)->page_sizes;
	int err;

	/* We have to wait for the async bind to complete before our asserts */
	err = i915_vma_sync(vma);
	if (err)
		return err;

	if (!HAS_PAGE_SIZES(i915, vma->page_sizes)) {
		pr_err("unsupported page_sizes=%x, supported=%x\n",
		       vma->page_sizes & ~supported, supported);
		err = -EINVAL;
	}

	return err;
}

static int igt_mock_exhaust_device_supported_pages(void *arg)
{
	struct i915_ppgtt *ppgtt = arg;
	struct drm_i915_private *i915 = ppgtt->vm.i915;
	unsigned int saved_mask = INTEL_INFO(i915)->page_sizes;
	struct drm_i915_gem_object *obj;
	struct i915_vma *vma;
	int i, j, single;
	int err;

	/*
	 * Sanity check creating objects with every valid page support
	 * combination for our mock device.
	 */

	for (i = 1; i < BIT(ARRAY_SIZE(page_sizes)); i++) {
		unsigned int combination = SZ_4K; /* Required for ppGTT */

		for (j = 0; j < ARRAY_SIZE(page_sizes); j++) {
			if (i & BIT(j))
				combination |= page_sizes[j];
		}

		mkwrite_device_info(i915)->page_sizes = combination;

		for (single = 0; single <= 1; ++single) {
			obj = fake_huge_pages_object(i915, combination, !!single);
			if (IS_ERR(obj)) {
				err = PTR_ERR(obj);
				goto out_device;
			}

			if (obj->base.size != combination) {
				pr_err("obj->base.size=%zu, expected=%u\n",
				       obj->base.size, combination);
				err = -EINVAL;
				goto out_put;
			}

			vma = i915_vma_instance(obj, &ppgtt->vm, NULL);
			if (IS_ERR(vma)) {
				err = PTR_ERR(vma);
				goto out_put;
			}

			err = i915_vma_pin(vma, 0, 0, PIN_USER);
			if (err)
				goto out_put;

			err = igt_check_page_sizes(vma);

			if (vma->page_sizes != combination) {
				pr_err("page_sizes.sg=%u, expected=%u\n",
				       vma->page_sizes, combination);
				err = -EINVAL;
			}

			i915_vma_unpin(vma);
			i915_gem_object_put(obj);

			if (err)
				goto out_device;
		}
	}

	goto out_device;

out_put:
	i915_gem_object_put(obj);
out_device:
	mkwrite_device_info(i915)->page_sizes = saved_mask;

	return err;
}

static int igt_mock_memory_region_huge_pages(void *arg)
{
	const unsigned int flags[] = { 0, I915_BO_ALLOC_CONTIGUOUS };
	struct i915_ppgtt *ppgtt = arg;
	struct drm_i915_private *i915 = ppgtt->vm.i915;
	unsigned long supported = INTEL_INFO(i915)->page_sizes;
	struct intel_memory_region *mem;
	struct drm_i915_gem_object *obj;
	struct i915_vma *vma;
	int bit;
	int err = 0;

	mem = mock_region_create(ppgtt->vm.gt, 0, SZ_2G, I915_GTT_PAGE_SIZE_4K, 0, 0);
	if (IS_ERR(mem)) {
		pr_err("%s failed to create memory region\n", __func__);
		return PTR_ERR(mem);
	}

	for_each_set_bit(bit, &supported, ilog2(I915_GTT_MAX_PAGE_SIZE) + 1) {
		unsigned int page_size = BIT(bit);
		resource_size_t phys;
		int i;

		for (i = 0; i < ARRAY_SIZE(flags); ++i) {
			obj = i915_gem_object_create_region(mem, page_size,
							    flags[i]);
			if (IS_ERR(obj)) {
				err = PTR_ERR(obj);
				goto out_region;
			}

			vma = i915_vma_instance(obj, &ppgtt->vm, NULL);
			if (IS_ERR(vma)) {
				err = PTR_ERR(vma);
				goto out_put;
			}

			err = i915_vma_pin(vma, 0, 0, PIN_USER);
			if (err)
				goto out_put;

			err = igt_check_page_sizes(vma);
			if (err)
				goto out_unpin;

			phys = i915_gem_object_get_dma_address(obj, 0);
			if (!IS_ALIGNED(phys, page_size)) {
				pr_err("%s addr misaligned(%pa) page_size=%u\n",
				       __func__, &phys, page_size);
				err = -EINVAL;
				goto out_unpin;
			}

			if (vma->page_sizes != page_size) {
				pr_err("%s page_sizes=%u, expected=%u\n",
				       __func__, vma->page_sizes,
				       page_size);
				err = -EINVAL;
				goto out_unpin;
			}

			i915_vma_unpin(vma);
			__i915_gem_object_put_pages(obj);
			i915_gem_object_put(obj);
		}
	}

	goto out_region;

out_unpin:
	i915_vma_unpin(vma);
out_put:
	i915_gem_object_put(obj);
out_region:
	intel_memory_region_put(mem);
	return err;
}

static void close_object_list(struct list_head *objects)
{
	struct drm_i915_gem_object *obj, *on;

	list_for_each_entry_safe(obj, on, objects, st_link) {
		list_del(&obj->st_link);
		i915_gem_object_lock(obj, NULL);
		i915_gem_object_unpin_pages(obj);
		__i915_gem_object_put_pages(obj);
		i915_gem_object_unlock(obj);
		i915_gem_object_put(obj);
	}
}

static int igt_ppgtt_huge_fill(void *arg)
{
	struct drm_i915_private *i915 = arg;
	unsigned int supported = INTEL_INFO(i915)->page_sizes;
	struct i915_address_space *vm;
	struct i915_gem_context *ctx;
	unsigned long max_pages;
	unsigned long page_num;
	struct file *file;
	bool single = false;
	LIST_HEAD(objects);
	IGT_TIMEOUT(end_time);
	int err = -ENODEV;

	if (supported == I915_GTT_PAGE_SIZE_4K)
		return 0;

	file = mock_file(i915);
	if (IS_ERR(file))
		return PTR_ERR(file);

	ctx = hugepage_ctx(i915, file);
	if (IS_ERR(ctx)) {
		err = PTR_ERR(ctx);
		goto out;
	}
	vm = i915_gem_context_get_eb_vm(ctx);
	max_pages = vm->total >> PAGE_SHIFT;

	for_each_prime_number_from(page_num, 1, max_pages) {
		struct drm_i915_gem_object *obj;
		u64 size = page_num << PAGE_SHIFT;
		struct i915_vma *vma;
		unsigned int expected_gtt = 0;
		int i;

		obj = fake_huge_pages_object(i915, size, single);
		if (IS_ERR(obj)) {
			err = PTR_ERR(obj);
			break;
		}

		if (obj->base.size != size) {
			pr_err("obj->base.size=%zd, expected=%llu\n",
			       obj->base.size, size);
			i915_gem_object_put(obj);
			err = -EINVAL;
			break;
		}

		err = i915_gem_object_pin_pages_unlocked(obj);
		if (err) {
			i915_gem_object_put(obj);
			break;
		}

		list_add(&obj->st_link, &objects);

		vma = i915_vma_instance(obj, vm, NULL);
		if (IS_ERR(vma)) {
			err = PTR_ERR(vma);
			break;
		}

		/* vma start must be aligned to BIT(21) to allow 2M PTEs */
		err = i915_vma_pin(vma, 0, BIT(21), PIN_USER);
		if (err)
			break;

		err = igt_check_page_sizes(vma);
		if (err) {
			i915_vma_unpin(vma);
			break;
		}

		/*
		 * Figure out the expected gtt page size knowing that we go from
		 * largest to smallest page size sg chunks, and that we align to
		 * the largest page size.
		 */
		for (i = 0; i < ARRAY_SIZE(page_sizes); ++i) {
			unsigned int page_size = page_sizes[i];

			if (HAS_PAGE_SIZES(i915, page_size) &&
			    size >= page_size) {
				expected_gtt |= page_size;
				size &= page_size-1;
			}
		}

		GEM_BUG_ON(!expected_gtt);
		GEM_BUG_ON(size);

		i915_vma_unpin(vma);

		if (!HAS_64K_PAGES(vma->vm->i915) &&
		    vma->page_sizes & I915_GTT_PAGE_SIZE_64K) {
			if (!IS_ALIGNED(vma->node.start,
					I915_GTT_PAGE_SIZE_2M)) {
				pr_err("node.start(%llx) not aligned to 2M\n",
				       vma->node.start);
				err = -EINVAL;
				break;
			}

			if (!IS_ALIGNED(vma->node.size,
					I915_GTT_PAGE_SIZE_2M)) {
				pr_err("node.size(%llx) not aligned to 2M\n",
				       vma->node.size);
				err = -EINVAL;
				break;
			}
		}

		if (vma->page_sizes != expected_gtt) {
			pr_err("gtt=%u, expected=%u, size=%zd, single=%s\n",
			       vma->page_sizes, expected_gtt,
			       obj->base.size, str_yes_no(!!single));
			err = -EINVAL;
			break;
		}

		if (igt_timeout(end_time,
				"%s timed out at size %zd\n",
				__func__, obj->base.size))
			break;

		single = !single;
	}

	close_object_list(&objects);

	if (err == -ENOMEM || err == -ENOSPC)
		err = 0;

	i915_vm_put(vm);
out:
	fput(file);
	return err;
}

static int gpu_write(struct intel_context *ce,
		     struct i915_vma *vma,
		     u32 dw,
		     u32 val)
{
	int err;

	i915_gem_object_lock(vma->obj, NULL);
	err = i915_gem_object_set_to_gtt_domain(vma->obj, true);
	i915_gem_object_unlock(vma->obj);
	if (err)
		return err;

	return igt_gpu_fill_dw(ce, vma, dw * sizeof(u32),
			       vma->size >> PAGE_SHIFT, val);
}

static int
__cpu_check_shmem(struct drm_i915_gem_object *obj, u32 dword, u32 val)
{
	unsigned int needs_flush;
	unsigned long n;
	int err;

	i915_gem_object_lock(obj, NULL);
	err = i915_gem_object_prepare_read(obj, &needs_flush);
	if (err)
		goto err_unlock;

	for (n = 0; n < obj->base.size >> PAGE_SHIFT; ++n) {
		u32 *ptr = kmap_atomic(i915_gem_object_get_page(obj, n));

		if (needs_flush & CLFLUSH_BEFORE)
			drm_clflush_virt_range(ptr, PAGE_SIZE);

		if (ptr[dword] != val) {
			pr_err("n=%lu ptr[%u]=%u, val=%u\n",
			       n, dword, ptr[dword], val);
			kunmap_atomic(ptr);
			err = -EINVAL;
			break;
		}

		kunmap_atomic(ptr);
	}

	i915_gem_object_finish_access(obj);
err_unlock:
	i915_gem_object_unlock(obj);

	return err;
}

static int __cpu_check_vmap(struct drm_i915_gem_object *obj, u32 dword, u32 val)
{
	unsigned long n = obj->base.size >> PAGE_SHIFT;
	u32 *ptr;
	int err;

	err = i915_gem_object_wait(obj, 0, MAX_SCHEDULE_TIMEOUT);
	if (err)
		return err;

	ptr = i915_gem_object_pin_map_unlocked(obj, I915_MAP_WC);
	if (IS_ERR(ptr))
		return PTR_ERR(ptr);

	ptr += dword;
	while (n--) {
		if (*ptr != val) {
			pr_err("base[%u]=%08x, val=%08x\n",
			       dword, *ptr, val);
			err = -EINVAL;
			break;
		}

		ptr += PAGE_SIZE / sizeof(*ptr);
	}

	i915_gem_object_unpin_map(obj);
	return err;
}

static int cpu_check(struct drm_i915_gem_object *obj, u32 dword, u32 val)
{
	if (i915_gem_object_has_struct_page(obj))
		return __cpu_check_shmem(obj, dword, val);
	else
		return __cpu_check_vmap(obj, dword, val);
}

static int __igt_write_huge(struct intel_context *ce,
			    struct drm_i915_gem_object *obj,
			    u64 size, u64 offset,
			    u32 dword, u32 val)
{
	unsigned int flags = PIN_USER | PIN_OFFSET_FIXED;
	struct i915_vma *vma;
	int err;

	vma = i915_vma_instance(obj, ce->vm, NULL);
	if (IS_ERR(vma))
		return PTR_ERR(vma);

	err = i915_vma_pin(vma, size, 0, flags | offset);
	if (err) {
		/*
		 * The ggtt may have some pages reserved so
		 * refrain from erroring out.
		 */
		if (err == -ENOSPC && i915_is_ggtt(ce->vm))
			err = 0;

		return err;
	}

	err = igt_check_page_sizes(vma);
	if (err)
		goto out_vma_unpin;

	err = gpu_write(ce, vma, dword, val);
	if (err) {
		pr_err("gpu-write failed at offset=%llx\n", offset);
		goto out_vma_unpin;
	}

	err = cpu_check(obj, dword, val);
	if (err) {
		pr_err("cpu-check failed at offset=%llx\n", offset);
		goto out_vma_unpin;
	}

out_vma_unpin:
	i915_vma_unpin(vma);
	return err;
}

static int igt_write_huge(struct drm_i915_private *i915,
			  struct drm_i915_gem_object *obj)
{
	struct i915_gem_engines *engines;
	struct i915_gem_engines_iter it;
	struct intel_context *ce;
	I915_RND_STATE(prng);
	IGT_TIMEOUT(end_time);
	unsigned int max_page_size;
	unsigned int count;
	struct i915_gem_context *ctx;
	struct file *file;
	u64 max;
	u64 num;
	u64 size;
	int *order;
	int i, n;
	int err = 0;

	file = mock_file(i915);
	if (IS_ERR(file))
		return PTR_ERR(file);

	ctx = hugepage_ctx(i915, file);
	if (IS_ERR(ctx)) {
		err = PTR_ERR(ctx);
		goto out;
	}

	GEM_BUG_ON(!i915_gem_object_has_pinned_pages(obj));

	size = obj->base.size;
	if (obj->mm.page_sizes & I915_GTT_PAGE_SIZE_64K)
		size = round_up(size, I915_GTT_PAGE_SIZE_2M);

	n = 0;
	count = 0;
	max = U64_MAX;
	for_each_gem_engine(ce, i915_gem_context_lock_engines(ctx), it) {
		count++;
		if (!intel_engine_can_store_dword(ce->engine))
			continue;

		max = min(max, ce->vm->total);
		max = min(max, BIT_ULL(ce->engine->ppgtt_size));
		n++;
	}
	i915_gem_context_unlock_engines(ctx);
	if (!n)
		goto out;

	/*
	 * To keep things interesting when alternating between engines in our
	 * randomized order, lets also make feeding to the same engine a few
	 * times in succession a possibility by enlarging the permutation array.
	 */
	order = i915_random_order(count * count, &prng);
	if (!order) {
		err = -ENOMEM;
		goto out;
	}

	max_page_size = rounddown_pow_of_two(obj->mm.page_sizes);
	max = div_u64(max - size, max_page_size);

	/*
	 * Try various offsets in an ascending/descending fashion until we
	 * timeout -- we want to avoid issues hidden by effectively always using
	 * offset = 0.
	 */
	i = 0;
	engines = i915_gem_context_lock_engines(ctx);
	for_each_prime_number_from(num, 0, max) {
		u64 offset_low = num * max_page_size;
		u64 offset_high = (max - num) * max_page_size;
		u32 dword = offset_in_page(num) / 4;
		struct intel_context *ce;

		ce = engines->engines[order[i] % engines->num_engines];
		i = (i + 1) % (count * count);
		if (!ce || !intel_engine_can_store_dword(ce->engine))
			continue;

		/*
		 * In order to utilize 64K pages we need to both pad the vma
		 * size and ensure the vma offset is at the start of the pt
		 * boundary, however to improve coverage we opt for testing both
		 * aligned and unaligned offsets.
		 */
		if (obj->mm.page_sizes & I915_GTT_PAGE_SIZE_64K)
			offset_low = round_down(offset_low,
						I915_GTT_PAGE_SIZE_2M);

		/* Needed for Wa_1409502670:xehpsdv, with vma node starting at 64K */
		if (IS_XEHPSDV(ce->vm->i915))
			offset_low = max(offset_low,  I915_GTT_PAGE_SIZE_64K);

		err = __igt_write_huge(ce, obj, size, offset_low,
				       dword, num + 1);
		if (err)
			break;

		err = __igt_write_huge(ce, obj, size, offset_high,
				       dword, num + 1);
		if (err)
			break;

		if (igt_timeout(end_time,
				"%s timed out on %s, offset_low=%llx offset_high=%llx, max_page_size=%x\n",
				__func__, ce->engine->name, offset_low, offset_high,
				max_page_size))
			break;
	}
	i915_gem_context_unlock_engines(ctx);

	kfree(order);

out:
	fput(file);
	return err;
}

typedef struct drm_i915_gem_object *
(*igt_create_fn)(struct drm_i915_private *i915, u32 size, u32 flags);

static inline bool igt_can_allocate_thp(struct drm_i915_private *i915)
{
	return i915->mm.gemfs && has_transparent_hugepage();
}

static struct drm_i915_gem_object *
igt_create_shmem(struct drm_i915_private *i915, u32 size, u32 flags)
{
	if (!igt_can_allocate_thp(i915)) {
		pr_info("%s missing THP support, skipping\n", __func__);
		return ERR_PTR(-ENODEV);
	}

	return i915_gem_object_create_shmem(i915, size);
}

static struct drm_i915_gem_object *
igt_create_internal(struct drm_i915_private *i915, u32 size, u32 flags)
{
	return i915_gem_object_create_internal(i915, size);
}

static struct drm_i915_gem_object *
igt_create_local(struct drm_i915_private *i915, u32 size, u32 flags)
{
	return i915_gem_object_create_lmem(i915, size, flags);
}

static u32 igt_random_size(struct rnd_state *prng,
			   u32 min_page_size,
			   u32 max_page_size)
{
	u64 mask;
	u32 size;

	GEM_BUG_ON(!is_power_of_2(min_page_size));
	GEM_BUG_ON(!is_power_of_2(max_page_size));
	GEM_BUG_ON(min_page_size < PAGE_SIZE);
	GEM_BUG_ON(min_page_size > max_page_size);

	mask = ((max_page_size << 1ULL) - 1) & PAGE_MASK;
	size = prandom_u32_state(prng) & mask;
	if (size < min_page_size)
		size |= min_page_size;

	return size;
}

static int igt_ppgtt_smoke_huge(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct drm_i915_gem_object *obj;
	I915_RND_STATE(prng);
	struct {
		igt_create_fn fn;
		u32 min;
		u32 max;
	} backends[] = {
		{ igt_create_internal, SZ_64K, SZ_2M,  },
		{ igt_create_shmem,    SZ_64K, SZ_32M, },
		{ igt_create_local,    SZ_64K, SZ_1G,  },
	};
	int err;
	int i;

	/*
	 * Sanity check that the HW uses huge pages correctly through our
	 * various backends -- ensure that our writes land in the right place.
	 */

	for (i = 0; i < ARRAY_SIZE(backends); ++i) {
		u32 min = backends[i].min;
		u32 max = backends[i].max;
		u32 size = max;

try_again:
		size = igt_random_size(&prng, min, rounddown_pow_of_two(size));

		obj = backends[i].fn(i915, size, 0);
		if (IS_ERR(obj)) {
			err = PTR_ERR(obj);
			if (err == -E2BIG) {
				size >>= 1;
				goto try_again;
			} else if (err == -ENODEV) {
				err = 0;
				continue;
			}

			return err;
		}

		err = i915_gem_object_pin_pages_unlocked(obj);
		if (err) {
			if (err == -ENXIO || err == -E2BIG) {
				i915_gem_object_put(obj);
				size >>= 1;
				goto try_again;
			}
			goto out_put;
		}

		if (obj->mm.page_sizes < min) {
			pr_info("%s unable to allocate huge-page(s) with size=%u, i=%d\n",
				__func__, size, i);
			err = -ENOMEM;
			goto out_unpin;
		}

		err = igt_write_huge(i915, obj);
		if (err) {
			pr_err("%s write-huge failed with size=%u, i=%d\n",
			       __func__, size, i);
		}
out_unpin:
		i915_gem_object_lock(obj, NULL);
		i915_gem_object_unpin_pages(obj);
		__i915_gem_object_put_pages(obj);
		i915_gem_object_unlock(obj);
out_put:
		i915_gem_object_put(obj);

		if (err == -ENOMEM || err == -ENXIO)
			err = 0;

		if (err)
			break;

		cond_resched();
	}

	return err;
}


static int igt_tmpfs_fallback(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct i915_address_space *vm;
	struct i915_gem_context *ctx;
	struct vfsmount *gemfs = i915->mm.gemfs;
	struct drm_i915_gem_object *obj;
	struct i915_vma *vma;
	struct file *file;
	u32 *vaddr;
	int err = 0;

	file = mock_file(i915);
	if (IS_ERR(file))
		return PTR_ERR(file);

	ctx = hugepage_ctx(i915, file);
	if (IS_ERR(ctx)) {
		err = PTR_ERR(ctx);
		goto out;
	}
	vm = i915_gem_context_get_eb_vm(ctx);

	/*
	 * Make sure that we don't burst into a ball of flames upon falling back
	 * to tmpfs, which we rely on if on the off-chance we encouter a failure
	 * when setting up gemfs.
	 */

	i915->mm.gemfs = NULL;

	obj = i915_gem_object_create_shmem(i915, PAGE_SIZE);
	if (IS_ERR(obj)) {
		err = PTR_ERR(obj);
		goto out_restore;
	}

	vaddr = i915_gem_object_pin_map_unlocked(obj, I915_MAP_WB);
	if (IS_ERR(vaddr)) {
		err = PTR_ERR(vaddr);
		goto out_put;
	}
	*vaddr = 0xdeadbeaf;

	__i915_gem_object_flush_map(obj, 0, 64);
	i915_gem_object_unpin_map(obj);

	vma = i915_vma_instance(obj, vm, NULL);
	if (IS_ERR(vma)) {
		err = PTR_ERR(vma);
		goto out_put;
	}

	err = i915_vma_pin(vma, 0, 0, PIN_USER);
	if (err)
		goto out_put;

	err = igt_check_page_sizes(vma);

	i915_vma_unpin(vma);
out_put:
	i915_gem_object_put(obj);
out_restore:
	i915->mm.gemfs = gemfs;

	i915_vm_put(vm);
out:
	fput(file);
	return err;
}

static int igt_shrink_thp(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct i915_address_space *vm;
	struct i915_gem_context *ctx;
	struct drm_i915_gem_object *obj;
	struct i915_gem_engines_iter it;
	struct intel_context *ce;
	struct i915_vma *vma;
	struct file *file;
	unsigned int flags = PIN_USER;
	unsigned int n;
	intel_wakeref_t wf;
	bool should_swap;
	int err;

	if (!igt_can_allocate_thp(i915)) {
		pr_info("missing THP support, skipping\n");
		return 0;
	}

	file = mock_file(i915);
	if (IS_ERR(file))
		return PTR_ERR(file);

	ctx = hugepage_ctx(i915, file);
	if (IS_ERR(ctx)) {
		err = PTR_ERR(ctx);
		goto out;
	}
	vm = i915_gem_context_get_eb_vm(ctx);

	/*
	 * Sanity check shrinking huge-paged object -- make sure nothing blows
	 * up.
	 */

	obj = i915_gem_object_create_shmem(i915, SZ_2M);
	if (IS_ERR(obj)) {
		err = PTR_ERR(obj);
		goto out_vm;
	}

	vma = i915_vma_instance(obj, vm, NULL);
	if (IS_ERR(vma)) {
		err = PTR_ERR(vma);
		goto out_put;
	}

	wf = intel_runtime_pm_get(&i915->runtime_pm); /* active shrink */

	err = i915_vma_pin(vma, 0, 0, flags);
	if (err)
		goto out_wf;

	if (obj->mm.page_sizes < I915_GTT_PAGE_SIZE_2M) {
		pr_info("failed to allocate THP, finishing test early\n");
		goto out_unpin;
	}

	err = igt_check_page_sizes(vma);
	if (err)
		goto out_unpin;

	n = 0;

	for_each_gem_engine(ce, i915_gem_context_lock_engines(ctx), it) {
		if (!intel_engine_can_store_dword(ce->engine))
			continue;

		err = gpu_write(ce, vma, n++, 0xdeadbeaf);
		if (err)
			break;
	}
	i915_gem_context_unlock_engines(ctx);
	/*
	 * Nuke everything *before* we unpin the pages so we can be reasonably
	 * sure that when later checking get_nr_swap_pages() that some random
	 * leftover object doesn't steal the remaining swap space.
	 */
	i915_gem_shrink(NULL, i915, -1UL, NULL,
			I915_SHRINK_BOUND |
			I915_SHRINK_UNBOUND |
			I915_SHRINK_ACTIVE);
	i915_vma_unpin(vma);
	if (err)
		goto out_wf;

	/*
	 * Now that the pages are *unpinned* shrinking should invoke
	 * shmem to truncate our pages, if we have available swap.
	 */
	should_swap = get_nr_swap_pages() > 0;
	i915_gem_shrink(NULL, i915, -1UL, NULL,
			I915_SHRINK_BOUND |
			I915_SHRINK_UNBOUND |
			I915_SHRINK_ACTIVE |
			I915_SHRINK_WRITEBACK);
	if (should_swap == i915_gem_object_has_pages(obj)) {
		pr_err("unexpected pages mismatch, should_swap=%s\n",
		       str_yes_no(should_swap));
		err = -EINVAL;
		goto out_wf;
	}

	err = i915_vma_pin(vma, 0, 0, flags);
	if (err)
		goto out_wf;

	while (n--) {
		err = cpu_check(obj, n, 0xdeadbeaf);
		if (err)
			break;
	}

out_unpin:
	i915_vma_unpin(vma);
out_wf:
	intel_runtime_pm_put(&i915->runtime_pm, wf);
out_put:
	i915_gem_object_put(obj);
out_vm:
	i915_vm_put(vm);
out:
	fput(file);
	return err;
}

int i915_gem_huge_page_mock_selftests(void)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(igt_mock_exhaust_device_supported_pages),
		SUBTEST(igt_mock_memory_region_huge_pages),
	};
	struct drm_i915_private *dev_priv;
	struct i915_ppgtt *ppgtt;
	int err;

	dev_priv = mock_gem_device();
	if (!dev_priv)
		return -ENOMEM;

	/* Pretend to be a device which supports the 48b PPGTT */
	mkwrite_device_info(dev_priv)->ppgtt_type = INTEL_PPGTT_FULL;
	mkwrite_device_info(dev_priv)->ppgtt_size = 48;

	ppgtt = i915_ppgtt_create(to_gt(dev_priv), 0);
	if (IS_ERR(ppgtt)) {
		err = PTR_ERR(ppgtt);
		goto out_unlock;
	}

	if (i915_vm_lvl(&ppgtt->vm) < 4) {
		pr_err("failed to create 48b PPGTT\n");
		err = -EINVAL;
		goto out_put;
	}

	/* If we were ever hit this then it's time to mock the 64K scratch */
	if (!i915_vm_has_scratch_64K(&ppgtt->vm)) {
		pr_err("PPGTT missing 64K scratch page\n");
		err = -EINVAL;
		goto out_put;
	}

	err = i915_subtests(tests, ppgtt);

out_put:
	i915_vm_put(&ppgtt->vm);
out_unlock:
	mock_destroy_device(dev_priv);
	return err;
}

int i915_gem_huge_page_live_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(igt_shrink_thp),
		SUBTEST(igt_tmpfs_fallback),
		SUBTEST(igt_ppgtt_smoke_huge),
		SUBTEST(igt_ppgtt_huge_fill),
	};

	if (!HAS_PPGTT(i915)) {
		pr_info("PPGTT not supported, skipping live-selftests\n");
		return 0;
	}

	if (intel_gt_is_wedged(to_gt(i915)))
		return 0;

	return i915_live_subtests(tests, i915);
}
