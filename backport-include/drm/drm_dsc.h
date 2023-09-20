/* SPDX-License-Identifier: MIT
 * Copyright (C) 2018 Intel Corp.
 *
 * Authors:
 * Manasi Navare <manasi.d.navare@intel.com>
 */

#ifndef _BACKPORT_DRM_DSC_H_
#define _BACKPORT_DRM_DSC_H_

#ifdef BPM_DISPLAY_DRM_DSC_PRESENT
#include_next <drm/display/drm_dsc.h>
#include_next <drm/display/drm_dsc_helper.h>
#else
#include_next <drm/drm_dsc.h>
#endif

#define drm_dsc_dp_rc_buffer_size LINUX_I915_BACKPORT(drm_dsc_dp_rc_buffer_size)
int drm_dsc_dp_rc_buffer_size(u8 rc_buffer_block_size, u8 rc_buffer_size);

#endif /* _BACKPORT_DRM_DSC_H_ */
