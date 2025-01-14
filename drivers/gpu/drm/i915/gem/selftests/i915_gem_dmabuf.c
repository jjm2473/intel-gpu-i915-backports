/*
 * SPDX-License-Identifier: MIT
 *
 * Copyright © 2016 Intel Corporation
 */

#include "i915_drv.h"
#include "i915_selftest.h"

#include "mock_dmabuf.h"
#include "selftests/mock_gem_device.h"

static int igt_dmabuf_export(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct drm_i915_gem_object *obj;
	struct dma_buf *dmabuf;

	obj = i915_gem_object_create_shmem(i915, PAGE_SIZE);
	if (IS_ERR(obj))
		return PTR_ERR(obj);

	dmabuf = i915_gem_prime_export(&obj->base, 0);
	i915_gem_object_put(obj);
	if (IS_ERR(dmabuf)) {
		pr_err("i915_gem_prime_export failed with err=%d\n",
		       (int)PTR_ERR(dmabuf));
		return PTR_ERR(dmabuf);
	}

	dma_buf_put(dmabuf);
	return 0;
}

static int igt_dmabuf_import_self(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct drm_i915_gem_object *obj, *import_obj;
	struct drm_gem_object *import;
	struct dma_buf *dmabuf;
	int err;

	obj = i915_gem_object_create_shmem(i915, PAGE_SIZE);
	if (IS_ERR(obj))
		return PTR_ERR(obj);

	dmabuf = i915_gem_prime_export(&obj->base, 0);
	if (IS_ERR(dmabuf)) {
		pr_err("i915_gem_prime_export failed with err=%d\n",
		       (int)PTR_ERR(dmabuf));
		err = PTR_ERR(dmabuf);
		goto out;
	}

	import = i915_gem_prime_import(&i915->drm, dmabuf);
	if (IS_ERR(import)) {
		pr_err("i915_gem_prime_import failed with err=%d\n",
		       (int)PTR_ERR(import));
		err = PTR_ERR(import);
		goto out_dmabuf;
	}
	import_obj = to_intel_bo(import);

	if (import != &obj->base) {
		pr_err("i915_gem_prime_import created a new object!\n");
		err = -EINVAL;
		goto out_import;
	}

	i915_gem_object_lock(import_obj, NULL);
	err = __i915_gem_object_get_pages(import_obj);
	i915_gem_object_unlock(import_obj);
	if (err) {
		pr_err("Same object dma-buf get_pages failed!\n");
		goto out_import;
	}

	err = 0;
out_import:
	i915_gem_object_put(import_obj);
out_dmabuf:
	dma_buf_put(dmabuf);
out:
	i915_gem_object_put(obj);
	return err;
}

static int igt_dmabuf_import_same_driver_lmem(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct intel_memory_region *lmem = to_gt(i915)->lmem;
	struct drm_i915_gem_object *obj;
	struct drm_gem_object *import;
	struct dma_buf *dmabuf;
	int err;

	if (!lmem)
		return 0;

	/*
	 * Asks about the device - if both sides support p2p,
	 * we can use lmem inplace.
	 */
	if (i915_p2p_distance(i915, i915->drm.dev) >= 0)
		return 0; /* skip */

	force_different_devices = true;

	obj = i915_gem_object_create_lmem(i915, PAGE_SIZE, 0);
	if (IS_ERR(obj)) {
		pr_err("__i915_gem_object_create_user failed with err=%ld\n",
		       PTR_ERR(obj));
		err = PTR_ERR(obj);
		goto out_ret;
	}

	dmabuf = i915_gem_prime_export(&obj->base, 0);
	if (IS_ERR(dmabuf)) {
		pr_err("i915_gem_prime_export failed with err=%ld\n",
		       PTR_ERR(dmabuf));
		err = PTR_ERR(dmabuf);
		goto out;
	}

	/*
	 * We expect an import of an LMEM-only object to fail with
	 * -EOPNOTSUPP because it can't be migrated to SMEM. However,
	 * if both sides support peer2peer access, then it can be used
	 * inplace from lmem.
	 */
	import = i915_gem_prime_import(&i915->drm, dmabuf);
	if (!IS_ERR(import)) {
		struct dma_buf_attachment *attach = obj->base.import_attach;

		/*
		 * Asks about the object/attachment -If both sides support p2p,
		 * we can use lmem inplace.
		 */
		if (object_to_attachment_p2p_distance(obj, attach) >= 0) {
			pr_err("this is unexpected!\n");
			err = 0;
		} else {
			pr_err("i915_gem_prime_import succeeded when it shouldn't have\n");
			err = -EINVAL;
		}
		drm_gem_object_put(import);
		pr_err("i915_gem_prime_import succeeded when it shouldn't have\n");
		err = -EINVAL;
	} else if (PTR_ERR(import) != -EOPNOTSUPP) {
		pr_err("i915_gem_prime_import failed with the wrong err=%ld\n",
		       PTR_ERR(import));
		err = PTR_ERR(import);
	} else {
		err = 0;
	}

	dma_buf_put(dmabuf);
out:
	i915_gem_object_put(obj);
out_ret:
	force_different_devices = false;
	return err;
}

static int igt_dmabuf_import_same_driver(struct drm_i915_private *i915,
					 struct intel_memory_region **regions,
					 unsigned int num_regions)
{
	struct drm_i915_gem_object *obj, *import_obj;
	struct drm_gem_object *import;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *import_attach;
	struct sg_table *st;
	long timeout;
	int err;

	force_different_devices = true;

	obj = i915_gem_object_create_user(i915, PAGE_SIZE,
					  regions, num_regions);
	if (IS_ERR(obj)) {
		pr_err("__i915_gem_object_create_user failed with err=%ld\n",
		       PTR_ERR(obj));
		err = PTR_ERR(obj);
		goto out_ret;
	}

	dmabuf = i915_gem_prime_export(&obj->base, 0);
	if (IS_ERR(dmabuf)) {
		pr_err("i915_gem_prime_export failed with err=%ld\n",
		       PTR_ERR(dmabuf));
		err = PTR_ERR(dmabuf);
		goto out;
	}

	import = i915_gem_prime_import(&i915->drm, dmabuf);
	if (IS_ERR(import)) {
		pr_err("i915_gem_prime_import failed with err=%ld\n",
		       PTR_ERR(import));
		err = PTR_ERR(import);
		goto out_dmabuf;
	}
	import_obj = to_intel_bo(import);

	if (import == &obj->base) {
		pr_err("i915_gem_prime_import reused gem object!\n");
		err = -EINVAL;
		goto out_import;
	}

	i915_gem_object_lock(import_obj, NULL);
	err = __i915_gem_object_get_pages(import_obj);
	if (err) {
		pr_err("Different objects dma-buf get_pages failed!\n");
		i915_gem_object_unlock(import_obj);
		goto out_import;
	}

	/*
	 * If the exported object is not in system memory, something
	 * weird is going on. TODO: When p2p is supported, this is no
	 * longer considered weird.
	 */
	if (obj->mm.region.mem != i915->mm.regions[INTEL_REGION_SMEM]) {
		pr_err("Exported dma-buf is not in system memory\n");
		err = -EINVAL;
	}

	i915_gem_object_unlock(import_obj);

	/* Now try a fake an importer */
	import_attach = dma_buf_attach(dmabuf, obj->base.dev->dev);
	if (IS_ERR(import_attach)) {
		err = PTR_ERR(import_attach);
		goto out_import;
	}

	st = dma_buf_map_attachment(import_attach, DMA_BIDIRECTIONAL);
	if (IS_ERR(st)) {
		err = PTR_ERR(st);
		goto out_detach;
	}

	timeout = dma_resv_wait_timeout(dmabuf->resv, false, true, 5 * HZ);
	if (!timeout) {
		pr_err("dmabuf wait for exclusive fence timed out.\n");
		timeout = -ETIME;
	}
	err = timeout > 0 ? 0 : timeout;
	dma_buf_unmap_attachment(import_attach, st, DMA_BIDIRECTIONAL);
out_detach:
	dma_buf_detach(dmabuf, import_attach);
out_import:
	i915_gem_object_put(import_obj);
out_dmabuf:
	dma_buf_put(dmabuf);
out:
	i915_gem_object_put(obj);
out_ret:
	force_different_devices = false;
	return err;
}

static int igt_dmabuf_import_same_driver_smem(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct intel_memory_region *smem = i915->mm.regions[INTEL_REGION_SMEM];

	return igt_dmabuf_import_same_driver(i915, &smem, 1);
}

static int igt_dmabuf_import_same_driver_lmem_smem(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct intel_memory_region *regions[2];

	if (!to_gt(i915)->lmem)
		return 0;

	regions[0] = to_gt(i915)->lmem;
	regions[1] = i915->mm.regions[INTEL_REGION_SMEM];
	return igt_dmabuf_import_same_driver(i915, regions, 2);
}

static int igt_dmabuf_import(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct drm_i915_gem_object *obj;
	struct dma_buf *dmabuf;
	void *obj_map, *dma_map;
	struct iosys_map map;
	u32 pattern[] = { 0, 0xaa, 0xcc, 0x55, 0xff };
	int err, i;

	dmabuf = mock_dmabuf(1);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	obj = to_intel_bo(i915_gem_prime_import(&i915->drm, dmabuf));
	if (IS_ERR(obj)) {
		pr_err("i915_gem_prime_import failed with err=%d\n",
		       (int)PTR_ERR(obj));
		err = PTR_ERR(obj);
		goto out_dmabuf;
	}

	if (obj->base.dev != &i915->drm) {
		pr_err("i915_gem_prime_import created a non-i915 object!\n");
		err = -EINVAL;
		goto out_obj;
	}

	if (obj->base.size != PAGE_SIZE) {
		pr_err("i915_gem_prime_import is wrong size found %lld, expected %ld\n",
		       (long long)obj->base.size, PAGE_SIZE);
		err = -EINVAL;
		goto out_obj;
	}

	err = dma_buf_vmap(dmabuf, &map);
	dma_map = err ? NULL : map.vaddr;
	if (!dma_map) {
		pr_err("dma_buf_vmap failed\n");
		err = -ENOMEM;
		goto out_obj;
	}

	if (0) { /* Can not yet map dmabuf */
		obj_map = i915_gem_object_pin_map(obj, I915_MAP_WB);
		if (IS_ERR(obj_map)) {
			err = PTR_ERR(obj_map);
			pr_err("i915_gem_object_pin_map failed with err=%d\n", err);
			goto out_dma_map;
		}

		for (i = 0; i < ARRAY_SIZE(pattern); i++) {
			memset(dma_map, pattern[i], PAGE_SIZE);
			if (memchr_inv(obj_map, pattern[i], PAGE_SIZE)) {
				err = -EINVAL;
				pr_err("imported vmap not all set to %x!\n", pattern[i]);
				i915_gem_object_unpin_map(obj);
				goto out_dma_map;
			}
		}

		for (i = 0; i < ARRAY_SIZE(pattern); i++) {
			memset(obj_map, pattern[i], PAGE_SIZE);
			if (memchr_inv(dma_map, pattern[i], PAGE_SIZE)) {
				err = -EINVAL;
				pr_err("exported vmap not all set to %x!\n", pattern[i]);
				i915_gem_object_unpin_map(obj);
				goto out_dma_map;
			}
		}

		i915_gem_object_unpin_map(obj);
	}

	err = 0;
out_dma_map:
	dma_buf_vunmap(dmabuf, &map);
out_obj:
	i915_gem_object_put(obj);
out_dmabuf:
	dma_buf_put(dmabuf);
	return err;
}

static int igt_dmabuf_import_ownership(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct drm_i915_gem_object *obj;
	struct dma_buf *dmabuf;
	struct iosys_map map;
	void *ptr;
	int err;

	dmabuf = mock_dmabuf(1);
	if (IS_ERR(dmabuf))
		return PTR_ERR(dmabuf);

	err = dma_buf_vmap(dmabuf, &map);
	ptr = err ? NULL : map.vaddr;
	if (!ptr) {
		pr_err("dma_buf_vmap failed\n");
		err = -ENOMEM;
		goto err_dmabuf;
	}

	memset(ptr, 0xc5, PAGE_SIZE);
	dma_buf_vunmap(dmabuf, &map);

	obj = to_intel_bo(i915_gem_prime_import(&i915->drm, dmabuf));
	if (IS_ERR(obj)) {
		pr_err("i915_gem_prime_import failed with err=%d\n",
		       (int)PTR_ERR(obj));
		err = PTR_ERR(obj);
		goto err_dmabuf;
	}

	dma_buf_put(dmabuf);

	err = i915_gem_object_pin_pages_unlocked(obj);
	if (err) {
		pr_err("i915_gem_object_pin_pages failed with err=%d\n", err);
		goto out_obj;
	}

	err = 0;
	i915_gem_object_unpin_pages(obj);
out_obj:
	i915_gem_object_put(obj);
	return err;

err_dmabuf:
	dma_buf_put(dmabuf);
	return err;
}

static int igt_dmabuf_export_vmap(void *arg)
{
	struct drm_i915_private *i915 = arg;
	struct drm_i915_gem_object *obj;
	struct dma_buf *dmabuf;
	struct iosys_map map;
	void *ptr;
	int err;

	obj = i915_gem_object_create_shmem(i915, PAGE_SIZE);
	if (IS_ERR(obj))
		return PTR_ERR(obj);

	dmabuf = i915_gem_prime_export(&obj->base, 0);
	if (IS_ERR(dmabuf)) {
		pr_err("i915_gem_prime_export failed with err=%d\n",
		       (int)PTR_ERR(dmabuf));
		err = PTR_ERR(dmabuf);
		goto err_obj;
	}
	i915_gem_object_put(obj);

	err = dma_buf_vmap(dmabuf, &map);
	ptr = err ? NULL : map.vaddr;
	if (!ptr) {
		pr_err("dma_buf_vmap failed\n");
		err = -ENOMEM;
		goto out;
	}

	if (memchr_inv(ptr, 0, dmabuf->size)) {
		pr_err("Exported object not initialiased to zero!\n");
		err = -EINVAL;
		goto out;
	}

	memset(ptr, 0xc5, dmabuf->size);

	err = 0;
	dma_buf_vunmap(dmabuf, &map);
out:
	dma_buf_put(dmabuf);
	return err;

err_obj:
	i915_gem_object_put(obj);
	return err;
}

int i915_gem_dmabuf_mock_selftests(void)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(igt_dmabuf_export),
		SUBTEST(igt_dmabuf_import_self),
		SUBTEST(igt_dmabuf_import),
		SUBTEST(igt_dmabuf_import_ownership),
		SUBTEST(igt_dmabuf_export_vmap),
	};
	struct drm_i915_private *i915;
	int err;

	i915 = mock_gem_device();
	if (!i915)
		return -ENOMEM;

	err = i915_subtests(tests, i915);

	mock_destroy_device(i915);
	return err;
}

int i915_gem_dmabuf_live_selftests(struct drm_i915_private *i915)
{
	static const struct i915_subtest tests[] = {
		SUBTEST(igt_dmabuf_export),
		SUBTEST(igt_dmabuf_import_same_driver_lmem),
		SUBTEST(igt_dmabuf_import_same_driver_smem),
		SUBTEST(igt_dmabuf_import_same_driver_lmem_smem),
	};

	return i915_live_subtests(tests, i915);
}
