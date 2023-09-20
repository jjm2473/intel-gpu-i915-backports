#ifndef _BACKPORT_DRM_PLANE_H_
#define _BACKPORT_DRM_PLANE_H_

#include_next <drm/drm_plane.h>

enum drm_scaling_filter {
	DRM_SCALING_FILTER_DEFAULT,
	DRM_SCALING_FILTER_NEAREST_NEIGHBOR,
};

#define drm_plane_create_scaling_filter_property LINUX_I915_BACKPORT(drm_plane_create_scaling_filter_property)
int drm_plane_create_scaling_filter_property(struct drm_plane *plane,
					     unsigned int supported_filters);

#endif /* _BACKPORT_DRM_PLANE_H_ */
