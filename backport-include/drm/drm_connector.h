#ifndef _BACKPORT_DRM_CONNECTOR_H_
#define _BACKPORT_DRM_CONNECTOR_H_

#include_next  <drm/drm_connector.h>

#define drm_connector_atomic_hdr_metadata_equal LINUX_I915_BACKPORT(drm_connector_atomic_hdr_metadata_equal)
bool drm_connector_atomic_hdr_metadata_equal(struct drm_connector_state *old_state,
					     struct drm_connector_state *new_state);

#define drm_connector_attach_colorspace_property LINUX_I915_BACKPORT(drm_connector_attach_colorspace_property)
int drm_connector_attach_colorspace_property(struct drm_connector *connector);
#define drm_connector_attach_hdr_output_metadata_property LINUX_I915_BACKPORT(drm_connector_attach_hdr_output_metadata_property)
int drm_connector_attach_hdr_output_metadata_property(struct drm_connector *connector);

#endif /* _BACKPORT_DRM_CONNECTOR_H_ */
