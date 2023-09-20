/*
 * Copyright (C) 2021 Intel Corporation
 */

#include <linux/delay.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/pci.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc.h>
#include <drm/drm_dsc.h>
#include <drm/drm_plane.h>

#ifdef BPM_VMA_SET_FILE_NOT_PRESENT
/*
 * Change backing file, only valid to use during initial VMA setup.
 */
void vma_set_file(struct vm_area_struct *vma, struct file *file)
{
	/* Changing an anonymous vma with this is illegal */
	get_file(file);
	swap(vma->vm_file, file);
	fput(file);
}
EXPORT_SYMBOL(vma_set_file);
#endif

#ifdef BPM_PCI_REBAR_SIZE_NOT_PRESENT
/**
 * pci_rebar_find_pos - find position of resize ctrl reg for BAR
 * @pdev: PCI device
 * @bar: BAR to find
 *
 * Helper to find the position of the ctrl register for a BAR.
 * Returns -ENOTSUPP if resizable BARs are not supported at all.
 * Returns -ENOENT if no ctrl register for the BAR could be found.
 */
static int pci_rebar_find_pos(struct pci_dev *pdev, int bar)
{
        unsigned int pos, nbars, i;
        u32 ctrl;

        pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_REBAR);
        if (!pos)
                return -ENOTSUPP;

        pci_read_config_dword(pdev, pos + PCI_REBAR_CTRL, &ctrl);
        nbars = (ctrl & PCI_REBAR_CTRL_NBAR_MASK) >>
                    PCI_REBAR_CTRL_NBAR_SHIFT;

        for (i = 0; i < nbars; i++, pos += 8) {
                int bar_idx;

                pci_read_config_dword(pdev, pos + PCI_REBAR_CTRL, &ctrl);
                bar_idx = ctrl & PCI_REBAR_CTRL_BAR_IDX;
                if (bar_idx == bar)
                        return pos;
        }

        return -ENOENT;
}

/**
 * pci_rebar_get_possible_sizes - get possible sizes for BAR
 * @pdev: PCI device
 * @bar: BAR to query
 *
 * Get the possible sizes of a resizable BAR as bitmask defined in the spec
 * (bit 0=1MB, bit 19=512GB). Returns 0 if BAR isn't resizable.
 */
u32 pci_rebar_get_possible_sizes(struct pci_dev *pdev, int bar)
{
        int pos;
        u32 cap;

        pos = pci_rebar_find_pos(pdev, bar);
        if (pos < 0)
                return 0;

        pci_read_config_dword(pdev, pos + PCI_REBAR_CAP, &cap);
        cap &= PCI_REBAR_CAP_SIZES;

        /* Sapphire RX 5600 XT Pulse has an invalid cap dword for BAR 0 */
        if (pdev->vendor == PCI_VENDOR_ID_ATI && pdev->device == 0x731f &&
            bar == 0 && cap == 0x7000)
                cap = 0x3f000;

        return cap >> 4;
}
EXPORT_SYMBOL(pci_rebar_get_possible_sizes);
#endif

u8 dp_link_status(const u8 link_status[DP_LINK_STATUS_SIZE], int r)
{
	return link_status[r - DP_LANE0_1_STATUS];
}
EXPORT_SYMBOL(dp_link_status);

/* DP 2.0 128b/132b */
#ifdef DRM_DP_GET_ADJUST_NOT_PRESENT
u8 drm_dp_get_adjust_tx_ffe_preset(const u8 link_status[DP_LINK_STATUS_SIZE],
				   int lane)
{
	int i = DP_ADJUST_REQUEST_LANE0_1 + (lane >> 1);
	int s = ((lane & 1) ?
		 DP_ADJUST_TX_FFE_PRESET_LANE1_SHIFT :
		 DP_ADJUST_TX_FFE_PRESET_LANE0_SHIFT);
	u8 l = dp_link_status(link_status, i);

	return (l >> s) & 0xf;
}
EXPORT_SYMBOL(drm_dp_get_adjust_tx_ffe_preset);
#endif /* DRM_DP_GET_ADJUST_NOT_PRESENT */

#ifdef DRM_EDP_BACKLIGHT_NOT_PRESENT
static inline int
drm_edp_backlight_probe_level(struct drm_dp_aux *aux, struct drm_edp_backlight_info *bl,
			      u8 *current_mode)
{
	int ret;
	u8 buf[2];
	u8 mode_reg;

	ret = drm_dp_dpcd_readb(aux, DP_EDP_BACKLIGHT_MODE_SET_REGISTER, &mode_reg);
	if (ret != 1) {
		DRM_DEBUG_DP("%s: Failed to read backlight mode: %d\n",
				aux->name, ret);
		return ret < 0 ? ret : -EIO;
	}

	*current_mode = (mode_reg & DP_EDP_BACKLIGHT_CONTROL_MODE_MASK);
	if (*current_mode == DP_EDP_BACKLIGHT_CONTROL_MODE_DPCD) {
		int size = 1 + bl->lsb_reg_used;

		ret = drm_dp_dpcd_read(aux, DP_EDP_BACKLIGHT_BRIGHTNESS_MSB, buf, size);
		if (ret != size) {
			DRM_DEBUG_DP("%s: Failed to read backlight level: %d\n",
					aux->name, ret);
			return ret < 0 ? ret : -EIO;
		}

		if (bl->lsb_reg_used)
			return (buf[0] << 8) | buf[1];
		else
			return buf[0];
	}

	/*
	 * If we're not in DPCD control mode yet, the programmed brightness value is meaningless and
	 * the driver should assume max brightness
	 */
	return bl->max;
}

static inline int
drm_edp_backlight_probe_max(struct drm_dp_aux *aux, struct drm_edp_backlight_info *bl,
			    u16 driver_pwm_freq_hz, const u8 edp_dpcd[EDP_DISPLAY_CTL_CAP_SIZE])
{
	int fxp, fxp_min, fxp_max, fxp_actual, f = 1;
	int ret;
	u8 pn, pn_min, pn_max;

	ret = drm_dp_dpcd_readb(aux, DP_EDP_PWMGEN_BIT_COUNT, &pn);
	if (ret != 1) {
		DRM_DEBUG_DP("%s: Failed to read pwmgen bit count cap: %d\n",
			    aux->name, ret);
		return -ENODEV;
	}

	pn &= DP_EDP_PWMGEN_BIT_COUNT_MASK;
	bl->max = (1 << pn) - 1;
	if (!driver_pwm_freq_hz)
		return 0;

	/*
	 * Set PWM Frequency divider to match desired frequency provided by the driver.
	 * The PWM Frequency is calculated as 27Mhz / (F x P).
	 * - Where F = PWM Frequency Pre-Divider value programmed by field 7:0 of the
	 *	       EDP_BACKLIGHT_FREQ_SET register (DPCD Address 00728h)
	 * - Where P = 2^Pn, where Pn is the value programmed by field 4:0 of the
	 *	       EDP_PWMGEN_BIT_COUNT register (DPCD Address 00724h)
	 */

	/* Find desired value of (F x P)
	 * Note that, if F x P is out of supported range, the maximum value or minimum value will
	 * applied automatically. So no need to check that.
	 */
	fxp = DIV_ROUND_CLOSEST(1000 * DP_EDP_BACKLIGHT_FREQ_BASE_KHZ, driver_pwm_freq_hz);

	/* Use highest possible value of Pn for more granularity of brightness adjustment while
	 * satifying the conditions below.
	 * - Pn is in the range of Pn_min and Pn_max
	 * - F is in the range of 1 and 255
	 * - FxP is within 25% of desired value.
	 *   Note: 25% is arbitrary value and may need some tweak.
	 */
	ret = drm_dp_dpcd_readb(aux, DP_EDP_PWMGEN_BIT_COUNT_CAP_MIN, &pn_min);
	if (ret != 1) {
		DRM_DEBUG_DP("%s: Failed to read pwmgen bit count cap min: %d\n",
			    aux->name, ret);
		return 0;
	}
	ret = drm_dp_dpcd_readb(aux, DP_EDP_PWMGEN_BIT_COUNT_CAP_MAX, &pn_max);
	if (ret != 1) {
		DRM_DEBUG_DP("%s: Failed to read pwmgen bit count cap max: %d\n",
			    aux->name, ret);
		return 0;
	}
	pn_min &= DP_EDP_PWMGEN_BIT_COUNT_MASK;
	pn_max &= DP_EDP_PWMGEN_BIT_COUNT_MASK;

	/* Ensure frequency is within 25% of desired value */
	fxp_min = DIV_ROUND_CLOSEST(fxp * 3, 4);
	fxp_max = DIV_ROUND_CLOSEST(fxp * 5, 4);
	if (fxp_min < (1 << pn_min) || (255 << pn_max) < fxp_max) {
		DRM_DEBUG_DP(
			    "%s: Driver defined backlight frequency (%d) out of range\n",
			    aux->name, driver_pwm_freq_hz);
		return 0;
	}

	for (pn = pn_max; pn >= pn_min; pn--) {
		f = clamp(DIV_ROUND_CLOSEST(fxp, 1 << pn), 1, 255);
		fxp_actual = f << pn;
		if (fxp_min <= fxp_actual && fxp_actual <= fxp_max)
			break;
	}

	ret = drm_dp_dpcd_writeb(aux, DP_EDP_PWMGEN_BIT_COUNT, pn);
	if (ret != 1) {
		DRM_DEBUG_DP("%s: Failed to write aux pwmgen bit count: %d\n",
			    aux->name, ret);
		return 0;
	}
	bl->pwmgen_bit_count = pn;
	bl->max = (1 << pn) - 1;

	if (edp_dpcd[2] & DP_EDP_BACKLIGHT_FREQ_AUX_SET_CAP) {
		bl->pwm_freq_pre_divider = f;
		DRM_DEBUG_DP("%s: Using backlight frequency from driver (%dHz)\n",
			    aux->name, driver_pwm_freq_hz);
	}

	return 0;
}

static int
drm_edp_backlight_set_enable(struct drm_dp_aux *aux, const struct drm_edp_backlight_info *bl,
			     bool enable)
{
	int ret;
	u8 buf;

	/* The panel uses something other then DPCD for enabling its backlight */
	if (!bl->aux_enable)
		return 0;

	ret = drm_dp_dpcd_readb(aux, DP_EDP_DISPLAY_CONTROL_REGISTER, &buf);
	if (ret != 1) {
		DRM_ERROR("%s: Failed to read eDP display control register: %d\n",
			aux->name, ret);
		return ret < 0 ? ret : -EIO;
	}
	if (enable)
		buf |= DP_EDP_BACKLIGHT_ENABLE;
	else
		buf &= ~DP_EDP_BACKLIGHT_ENABLE;

	ret = drm_dp_dpcd_writeb(aux, DP_EDP_DISPLAY_CONTROL_REGISTER, buf);
	if (ret != 1) {
		DRM_ERROR("%s: Failed to write eDP display control register: %d\n",
			aux->name, ret);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}

int drm_edp_backlight_set_level(struct drm_dp_aux *aux, const struct drm_edp_backlight_info *bl,
				u16 level)
{
	int ret;
	u8 buf[2] = { 0 };

	if (bl->lsb_reg_used) {
		buf[0] = (level & 0xff00) >> 8;
		buf[1] = (level & 0x00ff);
	} else {
		buf[0] = level;
	}

	ret = drm_dp_dpcd_write(aux, DP_EDP_BACKLIGHT_BRIGHTNESS_MSB, buf, sizeof(buf));
	if (ret != sizeof(buf)) {
		DRM_ERROR(
			"%s: Failed to write aux backlight level: %d\n",
			aux->name, ret);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}
EXPORT_SYMBOL(drm_edp_backlight_set_level);

int drm_edp_backlight_enable(struct drm_dp_aux *aux, const struct drm_edp_backlight_info *bl,
			     const u16 level)
{
	int ret;
	u8 dpcd_buf = DP_EDP_BACKLIGHT_CONTROL_MODE_DPCD;

	if (bl->pwmgen_bit_count) {
		ret = drm_dp_dpcd_writeb(aux, DP_EDP_PWMGEN_BIT_COUNT, bl->pwmgen_bit_count);
		if (ret != 1)
			DRM_DEBUG_DP("%s: Failed to write aux pwmgen bit count: %d\n",
				    aux->name, ret);
	}

	if (bl->pwm_freq_pre_divider) {
		ret = drm_dp_dpcd_writeb(aux, DP_EDP_BACKLIGHT_FREQ_SET, bl->pwm_freq_pre_divider);
		if (ret != 1)
			DRM_DEBUG_DP(
				    "%s: Failed to write aux backlight frequency: %d\n",
				    aux->name, ret);
		else
			dpcd_buf |= DP_EDP_BACKLIGHT_FREQ_AUX_SET_ENABLE;
	}

	ret = drm_dp_dpcd_writeb(aux, DP_EDP_BACKLIGHT_MODE_SET_REGISTER, dpcd_buf);
	if (ret != 1) {
		DRM_DEBUG_DP("%s: Failed to write aux backlight mode: %d\n",
			    aux->name, ret);
		return ret < 0 ? ret : -EIO;
	}

	ret = drm_edp_backlight_set_level(aux, bl, level);
	if (ret < 0)
		return ret;
	ret = drm_edp_backlight_set_enable(aux, bl, true);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL(drm_edp_backlight_enable);

int drm_edp_backlight_disable(struct drm_dp_aux *aux, const struct drm_edp_backlight_info *bl)
{
	int ret;

	ret = drm_edp_backlight_set_enable(aux, bl, false);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL(drm_edp_backlight_disable);

int
drm_edp_backlight_init(struct drm_dp_aux *aux, struct drm_edp_backlight_info *bl,
		       u16 driver_pwm_freq_hz, const u8 edp_dpcd[EDP_DISPLAY_CTL_CAP_SIZE],
		       u16 *current_level, u8 *current_mode)
{
	int ret;

	if (edp_dpcd[1] & DP_EDP_BACKLIGHT_AUX_ENABLE_CAP)
		bl->aux_enable = true;
	if (edp_dpcd[2] & DP_EDP_BACKLIGHT_BRIGHTNESS_BYTE_COUNT)
		bl->lsb_reg_used = true;

	ret = drm_edp_backlight_probe_max(aux, bl, driver_pwm_freq_hz, edp_dpcd);
	if (ret < 0)
		return ret;

	ret = drm_edp_backlight_probe_level(aux, bl, current_mode);
	if (ret < 0)
		return ret;
	*current_level = ret;

	DRM_DEBUG_DP(
		    "%s: Found backlight level=%d/%d pwm_freq_pre_divider=%d mode=%x\n",
		    aux->name, *current_level, bl->max, bl->pwm_freq_pre_divider, *current_mode);
	DRM_DEBUG_DP(
		    "%s: Backlight caps: pwmgen_bit_count=%d lsb_reg_used=%d aux_enable=%d\n",
		    aux->name, bl->pwmgen_bit_count, bl->lsb_reg_used, bl->aux_enable);
	return 0;
}
EXPORT_SYMBOL(drm_edp_backlight_init);
#endif /* DRM_EDP_BACKLIGHT_NOT_PRESENT */

#ifndef MAX_FLR_NOT_PRESENT
/**
 * drm_hdmi_sink_max_frl_rate - get the max frl rate, if supported
 * @connector - connector with HDMI sink
 *
 * RETURNS:
 * max frl rate supported by the HDMI sink, 0 if FRL not supported
 */
int drm_hdmi_sink_max_frl_rate(struct drm_connector *connector)
{
        int max_lanes = connector->display_info.hdmi.max_lanes;
        int rate_per_lane = connector->display_info.hdmi.max_frl_rate_per_lane;

        return max_lanes * rate_per_lane;
}
EXPORT_SYMBOL(drm_hdmi_sink_max_frl_rate);

/**
 * drm_hdmi_sink_dsc_max_frl_rate - get the max frl rate from HDMI sink with
 * DSC1.2 compression.
 * @connector - connector with HDMI sink
 *
 * RETURNS:
 * max frl rate supported by the HDMI sink with DSC1.2, 0 if FRL not supported
 */
int drm_hdmi_sink_dsc_max_frl_rate(struct drm_connector *connector)
{
        int max_dsc_lanes, dsc_rate_per_lane;

        if (!connector->display_info.hdmi.dsc_cap.v_1p2)
                return 0;

        max_dsc_lanes = connector->display_info.hdmi.dsc_cap.max_lanes;
        dsc_rate_per_lane = connector->display_info.hdmi.dsc_cap.max_frl_rate_per_lane;

        return max_dsc_lanes * dsc_rate_per_lane;
}
EXPORT_SYMBOL(drm_hdmi_sink_dsc_max_frl_rate);
#endif

/**
 * drm_connector_atomic_hdr_metadata_equal - checks if the hdr metadata changed
 * @old_state: old connector state to compare
 * @new_state: new connector state to compare
 *
 * This is used by HDR-enabled drivers to test whether the HDR metadata
 * have changed between two different connector state (and thus probably
 * requires a full blown mode change).
 *
 * Returns:
 * True if the metadata are equal, False otherwise
 */
bool drm_connector_atomic_hdr_metadata_equal(struct drm_connector_state *old_state,
					     struct drm_connector_state *new_state)
{
	struct drm_property_blob *old_blob = old_state->hdr_output_metadata;
	struct drm_property_blob *new_blob = new_state->hdr_output_metadata;

	if (!old_blob || !new_blob)
		return old_blob == new_blob;

	if (old_blob->length != new_blob->length)
		return false;

	return !memcmp(old_blob->data, new_blob->data, old_blob->length);
}
EXPORT_SYMBOL(drm_connector_atomic_hdr_metadata_equal);

/**
 * drm_dsc_dp_rc_buffer_size - get rc buffer size in bytes
 * @rc_buffer_block_size: block size code, according to DPCD offset 62h
 * @rc_buffer_size: number of blocks - 1, according to DPCD offset 63h
 *
 * return:
 * buffer size in bytes, or 0 on invalid input
 */
int drm_dsc_dp_rc_buffer_size(u8 rc_buffer_block_size, u8 rc_buffer_size)
{
	int size = 1024 * (rc_buffer_size + 1);

	switch (rc_buffer_block_size) {
	case DP_DSC_RC_BUF_BLK_SIZE_1:
		return 1 * size;
	case DP_DSC_RC_BUF_BLK_SIZE_4:
		return 4 * size;
	case DP_DSC_RC_BUF_BLK_SIZE_16:
		return 16 * size;
	case DP_DSC_RC_BUF_BLK_SIZE_64:
		return 64 * size;
	default:
		return 0;
	}
}
EXPORT_SYMBOL(drm_dsc_dp_rc_buffer_size);

/**
 * drm_connector_attach_colorspace_property - attach "Colorspace" property
 * @connector: connector to attach the property on.
 *
 * This is used to allow the userspace to signal the output colorspace
 * to the driver.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_connector_attach_colorspace_property(struct drm_connector *connector)
{
	struct drm_property *prop = connector->colorspace_property;

	drm_object_attach_property(&connector->base, prop, DRM_MODE_COLORIMETRY_DEFAULT);

	return 0;
}
EXPORT_SYMBOL(drm_connector_attach_colorspace_property);

struct drm_property *
drm_create_scaling_filter_prop(struct drm_device *dev,
			       unsigned int supported_filters)
{
	struct drm_property *prop;
	static const struct drm_prop_enum_list props[] = {
		{ DRM_SCALING_FILTER_DEFAULT, "Default" },
		{ DRM_SCALING_FILTER_NEAREST_NEIGHBOR, "Nearest Neighbor" },
	};
	unsigned int valid_mode_mask = BIT(DRM_SCALING_FILTER_DEFAULT) |
				       BIT(DRM_SCALING_FILTER_NEAREST_NEIGHBOR);
	int i;

	if (WARN_ON((supported_filters & ~valid_mode_mask) ||
		    ((supported_filters & BIT(DRM_SCALING_FILTER_DEFAULT)) == 0)))
		return ERR_PTR(-EINVAL);

	prop = drm_property_create(dev, DRM_MODE_PROP_ENUM,
				   "SCALING_FILTER",
				   hweight32(supported_filters));
	if (!prop)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < ARRAY_SIZE(props); i++) {
		int ret;

		if (!(BIT(props[i].type) & supported_filters))
			continue;

		ret = drm_property_add_enum(prop, props[i].type,
					    props[i].name);

		if (ret) {
			drm_property_destroy(dev, prop);

			return ERR_PTR(ret);
		}
	}

	return prop;
}

/**
 * drm_crtc_create_scaling_filter_property - create a new scaling filter
 * property
 *
 * @crtc: drm CRTC
 * @supported_filters: bitmask of supported scaling filters, must include
 *		       BIT(DRM_SCALING_FILTER_DEFAULT).
 *
 * This function lets driver to enable the scaling filter property on a given
 * CRTC.
 *
 * RETURNS:
 * Zero for success or -errno
 */
int drm_crtc_create_scaling_filter_property(struct drm_crtc *crtc,
					    unsigned int supported_filters)
{
	struct drm_property *prop =
		drm_create_scaling_filter_prop(crtc->dev, supported_filters);

	if (IS_ERR(prop))
		return PTR_ERR(prop);

	drm_object_attach_property(&crtc->base, prop,
				   DRM_SCALING_FILTER_DEFAULT);
//	crtc->scaling_filter_property = prop;

	return 0;
}
EXPORT_SYMBOL(drm_crtc_create_scaling_filter_property);

/**
 * drm_plane_create_scaling_filter_property - create a new scaling filter
 * property
 *
 * @plane: drm plane
 * @supported_filters: bitmask of supported scaling filters, must include
 *		       BIT(DRM_SCALING_FILTER_DEFAULT).
 *
 * This function lets driver to enable the scaling filter property on a given
 * plane.
 *
 * RETURNS:
 * Zero for success or -errno
 */
int drm_plane_create_scaling_filter_property(struct drm_plane *plane,
					     unsigned int supported_filters)
{
	struct drm_property *prop =
		drm_create_scaling_filter_prop(plane->dev, supported_filters);

	if (IS_ERR(prop))
		return PTR_ERR(prop);

	drm_object_attach_property(&plane->base, prop,
				   DRM_SCALING_FILTER_DEFAULT);
//	plane->scaling_filter_property = prop;

	return 0;
}
EXPORT_SYMBOL(drm_plane_create_scaling_filter_property);

static u8 dp_lttpr_common_cap(const u8 caps[DP_LTTPR_COMMON_CAP_SIZE], int r)
{
	return caps[r - DP_LT_TUNABLE_PHY_REPEATER_FIELD_DATA_STRUCTURE_REV];
}

/**
 * drm_dp_lttpr_count - get the number of detected LTTPRs
 * @caps: LTTPR common capabilities
 *
 * Get the number of detected LTTPRs from the LTTPR common capabilities info.
 *
 * Returns:
 *   -ERANGE if more than supported number (8) of LTTPRs are detected
 *   -EINVAL if the DP_PHY_REPEATER_CNT register contains an invalid value
 *   otherwise the number of detected LTTPRs
 */
int drm_dp_lttpr_count(const u8 caps[DP_LTTPR_COMMON_CAP_SIZE])
{
	u8 count = dp_lttpr_common_cap(caps, DP_PHY_REPEATER_CNT);

	switch (hweight8(count)) {
	case 0:
		return 0;
	case 1:
		return 8 - ilog2(count);
	case 8:
		return -ERANGE;
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL(drm_dp_lttpr_count);

/**
 * drm_dp_lttpr_max_link_rate - get the maximum link rate supported by all LTTPRs
 * @caps: LTTPR common capabilities
 *
 * Returns the maximum link rate supported by all detected LTTPRs.
 */
int drm_dp_lttpr_max_link_rate(const u8 caps[DP_LTTPR_COMMON_CAP_SIZE])
{
	u8 rate = dp_lttpr_common_cap(caps, DP_MAX_LINK_RATE_PHY_REPEATER);

	return drm_dp_bw_code_to_link_rate(rate);
}
EXPORT_SYMBOL(drm_dp_lttpr_max_link_rate);

/**
 * drm_dp_lttpr_max_lane_count - get the maximum lane count supported by all LTTPRs
 * @caps: LTTPR common capabilities
 *
 * Returns the maximum lane count supported by all detected LTTPRs.
 */
int drm_dp_lttpr_max_lane_count(const u8 caps[DP_LTTPR_COMMON_CAP_SIZE])
{
	u8 max_lanes = dp_lttpr_common_cap(caps, DP_MAX_LANE_COUNT_PHY_REPEATER);

	return max_lanes & DP_MAX_LANE_COUNT_MASK;
}
EXPORT_SYMBOL(drm_dp_lttpr_max_lane_count);

/*
 * drm_dp_pcon_convert_rgb_to_ycbcr() - Configure the PCon to convert RGB to Ycbcr
 * @aux: displayPort AUX channel
 * @color_spc: Color-space/s for which conversion is to be enabled, 0 for disable.
 *
 * Returns 0 on success, else returns negative error code.
 */
int drm_dp_pcon_convert_rgb_to_ycbcr(struct drm_dp_aux *aux, u8 color_spc)
{
	int ret;
	u8 buf;

	ret = drm_dp_dpcd_readb(aux, DP_PROTOCOL_CONVERTER_CONTROL_2, &buf);
	if (ret < 0)
		return ret;

	if (color_spc & DP_CONVERSION_RGB_YCBCR_MASK)
		buf |= (color_spc & DP_CONVERSION_RGB_YCBCR_MASK);
	else
		buf &= ~DP_CONVERSION_RGB_YCBCR_MASK;

	ret = drm_dp_dpcd_writeb(aux, DP_PROTOCOL_CONVERTER_CONTROL_2, buf);
	if (ret < 0)
		return ret;

	return 0;
}
EXPORT_SYMBOL(drm_dp_pcon_convert_rgb_to_ycbcr);

/**
 * drm_dp_dpcd_read_phy_link_status - get the link status information for a DP PHY
 * @aux: DisplayPort AUX channel
 * @dp_phy: the DP PHY to get the link status for
 * @link_status: buffer to return the status in
 *
 * Fetch the AUX DPCD registers for the DPRX or an LTTPR PHY link status. The
 * layout of the returned @link_status matches the DPCD register layout of the
 * DPRX PHY link status.
 *
 * Returns 0 if the information was read successfully or a negative error code
 * on failure.
 */
int drm_dp_dpcd_read_phy_link_status(struct drm_dp_aux *aux,
				     enum drm_dp_phy dp_phy,
				     u8 link_status[DP_LINK_STATUS_SIZE])
{
	int ret;

	if (dp_phy == DP_PHY_DPRX) {
		ret = drm_dp_dpcd_read(aux,
				       DP_LANE0_1_STATUS,
				       link_status,
				       DP_LINK_STATUS_SIZE);

		if (ret < 0)
			return ret;

		WARN_ON(ret != DP_LINK_STATUS_SIZE);

		return 0;
	}

	ret = drm_dp_dpcd_read(aux,
			       DP_LANE0_1_STATUS_PHY_REPEATER(dp_phy),
			       link_status,
			       DP_LINK_STATUS_SIZE - 1);

	if (ret < 0)
		return ret;

	WARN_ON(ret != DP_LINK_STATUS_SIZE - 1);

	/* Convert the LTTPR to the sink PHY link status layout */
	memmove(&link_status[DP_SINK_STATUS - DP_LANE0_1_STATUS + 1],
		&link_status[DP_SINK_STATUS - DP_LANE0_1_STATUS],
		DP_LINK_STATUS_SIZE - (DP_SINK_STATUS - DP_LANE0_1_STATUS) - 1);
	link_status[DP_SINK_STATUS - DP_LANE0_1_STATUS] = 0;

	return 0;
}
EXPORT_SYMBOL(drm_dp_dpcd_read_phy_link_status);

/**
 * drm_dp_pcon_hdmi_link_active() - check if the PCON HDMI LINK status is active.
 * @aux: DisplayPort AUX channel
 *
 * Returns true if link is active else returns false.
 */
bool drm_dp_pcon_hdmi_link_active(struct drm_dp_aux *aux)
{
	u8 buf;
	int ret;

	ret = drm_dp_dpcd_readb(aux, DP_PCON_HDMI_TX_LINK_STATUS, &buf);
	if (ret < 0)
		return false;

	return buf & DP_PCON_HDMI_TX_LINK_ACTIVE;
}
EXPORT_SYMBOL(drm_dp_pcon_hdmi_link_active);

/**
 * drm_dp_get_pcon_max_frl_bw() - maximum frl supported by PCON
 * @dpcd: DisplayPort configuration data
 * @port_cap: port capabilities
 *
 * Returns maximum frl bandwidth supported by PCON in GBPS,
 * returns 0 if not supported.
 */
int drm_dp_get_pcon_max_frl_bw(const u8 dpcd[DP_RECEIVER_CAP_SIZE],
			       const u8 port_cap[4])
{
	int bw;
	u8 buf;

	buf = port_cap[2];
	bw = buf & DP_PCON_MAX_FRL_BW;

	switch (bw) {
	case DP_PCON_MAX_9GBPS:
		return 9;
	case DP_PCON_MAX_18GBPS:
		return 18;
	case DP_PCON_MAX_24GBPS:
		return 24;
	case DP_PCON_MAX_32GBPS:
		return 32;
	case DP_PCON_MAX_40GBPS:
		return 40;
	case DP_PCON_MAX_48GBPS:
		return 48;
	case DP_PCON_MAX_0GBPS:
	default:
		return 0;
	}

	return 0;
}
EXPORT_SYMBOL(drm_dp_get_pcon_max_frl_bw);

/**
 * drm_dp_downstream_rgb_to_ycbcr_conversion() - determine downstream facing port
 *                                               RGB->YCbCr conversion capability
 * @dpcd: DisplayPort configuration data
 * @port_cap: downstream facing port capabilities
 * @color_spc: Colorspace for which conversion cap is sought
 *
 * Returns: whether the downstream facing port can convert RGB->YCbCr for a given
 * colorspace.
 */
bool drm_dp_downstream_rgb_to_ycbcr_conversion(const u8 dpcd[DP_RECEIVER_CAP_SIZE],
					       const u8 port_cap[4],
					       u8 color_spc)
{
	if (!drm_dp_is_branch(dpcd))
		return false;

	if (dpcd[DP_DPCD_REV] < 0x13)
		return false;

	switch (port_cap[0] & DP_DS_PORT_TYPE_MASK) {
	case DP_DS_PORT_TYPE_HDMI:
		if ((dpcd[DP_DOWNSTREAMPORT_PRESENT] & DP_DETAILED_CAP_INFO_AVAILABLE) == 0)
			return false;

		return port_cap[3] & color_spc;
	default:
		return false;
	}
}
EXPORT_SYMBOL(drm_dp_downstream_rgb_to_ycbcr_conversion);

/**
 * drm_connector_attach_hdr_output_metadata_property - attach "HDR_OUTPUT_METADA" property
 * @connector: connector to attach the property on.
 *
 * This is used to allow the userspace to send HDR Metadata to the
 * driver.
 *
 * Returns:
 * Zero on success, negative errno on failure.
 */
int drm_connector_attach_hdr_output_metadata_property(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct drm_property *prop = dev->mode_config.hdr_output_metadata_property;

	drm_object_attach_property(&connector->base, prop, 0);

	return 0;
}
EXPORT_SYMBOL(drm_connector_attach_hdr_output_metadata_property);

static u8 dp_lttpr_phy_cap(const u8 phy_cap[DP_LTTPR_PHY_CAP_SIZE], int r)
{
	return phy_cap[r - DP_TRAINING_AUX_RD_INTERVAL_PHY_REPEATER1];
}

/**
 * drm_dp_lttpr_voltage_swing_level_3_supported - check for LTTPR vswing3 support
 * @caps: LTTPR PHY capabilities
 *
 * Returns true if the @caps for an LTTPR TX PHY indicate support for
 * voltage swing level 3.
 */
bool
drm_dp_lttpr_voltage_swing_level_3_supported(const u8 caps[DP_LTTPR_PHY_CAP_SIZE])
{
	u8 txcap = dp_lttpr_phy_cap(caps, DP_TRANSMITTER_CAPABILITY_PHY_REPEATER1);

	return txcap & DP_VOLTAGE_SWING_LEVEL_3_SUPPORTED;
}
EXPORT_SYMBOL(drm_dp_lttpr_voltage_swing_level_3_supported);

/**
 * drm_dp_lttpr_pre_emphasis_level_3_supported - check for LTTPR preemph3 support
 * @caps: LTTPR PHY capabilities
 *
 * Returns true if the @caps for an LTTPR TX PHY indicate support for
 * pre-emphasis level 3.
 */
bool
drm_dp_lttpr_pre_emphasis_level_3_supported(const u8 caps[DP_LTTPR_PHY_CAP_SIZE])
{
	u8 txcap = dp_lttpr_phy_cap(caps, DP_TRANSMITTER_CAPABILITY_PHY_REPEATER1);

	return txcap & DP_PRE_EMPHASIS_LEVEL_3_SUPPORTED;
}
EXPORT_SYMBOL(drm_dp_lttpr_pre_emphasis_level_3_supported);

void drm_dp_lttpr_link_train_clock_recovery_delay(void)
{
	usleep_range(100, 200);
}
EXPORT_SYMBOL(drm_dp_lttpr_link_train_clock_recovery_delay);

static void __drm_dp_link_train_channel_eq_delay(const struct drm_dp_aux *aux,
						 unsigned long rd_interval)
{
	if (rd_interval > 4)
		DRM_DEBUG_DP("%s: AUX interval %lu, out of range (max 4)\n",
			    aux->name, rd_interval);

	if (rd_interval == 0)
		rd_interval = 400;
	else
		rd_interval *= 4 * USEC_PER_MSEC;

	usleep_range(rd_interval, rd_interval * 2);
}

void drm_dp_lttpr_link_train_channel_eq_delay(const struct drm_dp_aux *aux,
					      const u8 phy_cap[DP_LTTPR_PHY_CAP_SIZE])
{
	u8 interval = dp_lttpr_phy_cap(phy_cap,
				       DP_TRAINING_AUX_RD_INTERVAL_PHY_REPEATER1) &
		      DP_TRAINING_AUX_RD_MASK;

	__drm_dp_link_train_channel_eq_delay(aux, interval);
}
EXPORT_SYMBOL(drm_dp_lttpr_link_train_channel_eq_delay);
