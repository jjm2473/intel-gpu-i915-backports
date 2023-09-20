#ifndef _BACKPORT_DRM_RECT_H_
#define _BACKPORT_DRM_RECT_H_

#include_next <drm/drm_rect.h>

/**
 * drm_rect_fp_to_int - Convert a rect in 16.16 fixed point form to int form.
 * @dst: rect to be stored the converted value
 * @src: rect in 16.16 fixed point form
 */
static inline void drm_rect_fp_to_int(struct drm_rect *dst,
				      const struct drm_rect *src)
{
	drm_rect_init(dst, src->x1 >> 16, src->y1 >> 16,
		      drm_rect_width(src) >> 16,
		      drm_rect_height(src) >> 16);
}

#endif /* _BACKPORT_DRM_RECT_H_ */
