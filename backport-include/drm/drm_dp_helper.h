#ifndef _BACKPORT_DRM_DP_HELPER_H_
#define _BACKPORT_DRM_DP_HELPER_H_

#ifdef BPM_DRM_DP_HELPER_DIR_DISPLAY_PRESENT
#include_next  <drm/display/drm_dp_helper.h>
#elif defined(BPM_DRM_DP_HELPER_DIR_DP_PRESENT)
#include_next  <drm/dp/drm_dp_helper.h>
#else
#include_next  <drm/drm_dp_helper.h>
#endif /* DRM_DP_HELPER_DIR_DISPLAY_PRESENT */

// DP_MAIN_LINK_CHANNEL_CODING_SET	    0x108
# define DP_SET_ANSI_128B132B               (1 << 1)

#define HDCP_2_2_DP_CERT_READ_TIMEOUT_MS	110
#define HDCP_2_2_DP_HPRIME_READ_TIMEOUT_MS	7
#define HDCP_2_2_DP_PAIRING_READ_TIMEOUT_MS	5

/* PCON HDMI LINK STATUS */
#define DP_PCON_HDMI_TX_LINK_STATUS           0x303B
# define DP_PCON_HDMI_TX_LINK_ACTIVE          (1 << 0)
# define DP_PCON_FRL_READY		      (1 << 1)

/* DP-HDMI2.1 PCON DSC ENCODER SUPPORT */
#define DP_PCON_DSC_ENCODER_CAP_SIZE        0xD	/* 0x92 through 0x9E */
#define DP_PCON_DSC_ENCODER                 0x092
# define DP_PCON_DSC_ENCODER_SUPPORTED      (1 << 0)
# define DP_PCON_DSC_PPS_ENC_OVERRIDE       (1 << 1)

// DP_PSR_EN_CFG				0x170   /* XXX 1.2? */
# define DP_PSR_SU_REGION_SCANLINE_CAPTURE	BIT(4) /* eDP 1.4a */

// DP_MAIN_LINK_CHANNEL_CODING         0x006
# define DP_CAP_ANSI_128B132B               (1 << 1) /* 2.0 */

#define DP_128B132B_SUPPORTED_LINK_RATES       0x2215 /* 2.0 */
# define DP_UHBR10                             (1 << 0)
# define DP_UHBR20                             (1 << 1)
# define DP_UHBR13_5                           (1 << 2)

/* PCON CONFIGURE-1 FRL FOR HDMI SINK */
#define DP_PCON_HDMI_LINK_CONFIG_1             0x305A
# define DP_PCON_ENABLE_MAX_FRL_BW             (7 << 0)
# define DP_PCON_ENABLE_MAX_BW_0GBPS	       0
# define DP_PCON_ENABLE_MAX_BW_9GBPS	       1
# define DP_PCON_ENABLE_MAX_BW_18GBPS	       2
# define DP_PCON_ENABLE_MAX_BW_24GBPS	       3
# define DP_PCON_ENABLE_MAX_BW_32GBPS	       4
# define DP_PCON_ENABLE_MAX_BW_40GBPS	       5
# define DP_PCON_ENABLE_MAX_BW_48GBPS	       6
# define DP_PCON_ENABLE_SOURCE_CTL_MODE       (1 << 3)
# define DP_PCON_ENABLE_CONCURRENT_LINK       (1 << 4)
# define DP_PCON_ENABLE_SEQUENTIAL_LINK       (0 << 4)
# define DP_PCON_ENABLE_LINK_FRL_MODE         (1 << 5)
# define DP_PCON_ENABLE_HPD_READY	      (1 << 6)
# define DP_PCON_ENABLE_HDMI_LINK             (1 << 7)

/* PCON CONFIGURE-2 FRL FOR HDMI SINK */
#define DP_PCON_HDMI_LINK_CONFIG_2            0x305B
# define DP_PCON_MAX_LINK_BW_MASK             (0x3F << 0)
# define DP_PCON_FRL_BW_MASK_9GBPS            (1 << 0)
# define DP_PCON_FRL_BW_MASK_18GBPS           (1 << 1)
# define DP_PCON_FRL_BW_MASK_24GBPS           (1 << 2)
# define DP_PCON_FRL_BW_MASK_32GBPS           (1 << 3)
# define DP_PCON_FRL_BW_MASK_40GBPS           (1 << 4)
# define DP_PCON_FRL_BW_MASK_48GBPS           (1 << 5)
# define DP_PCON_FRL_LINK_TRAIN_EXTENDED      (1 << 6)
# define DP_PCON_FRL_LINK_TRAIN_NORMAL        (0 << 6)

// DP_LINK_SERVICE_IRQ_VECTOR_ESI0     0x2005   /* 1.2 */
# define RX_CAP_CHANGED                      (1 << 0)
# define LINK_STATUS_CHANGED                 (1 << 1)
# define STREAM_STATUS_CHANGED               (1 << 2)
# define HDMI_LINK_STATUS_CHANGED            (1 << 3)
# define CONNECTED_OFF_ENTRY_REQUESTED       (1 << 4)

// DP_PROTOCOL_CONVERTER_CONTROL_2		0x3052 /* DP 1.3 */
# define DP_CONVERSION_RGB_YCBCR_MASK	       (7 << 4)
# define DP_CONVERSION_BT601_RGB_YCBCR_ENABLE  (1 << 4)
# define DP_CONVERSION_BT709_RGB_YCBCR_ENABLE  (1 << 5)
# define DP_CONVERSION_BT2020_RGB_YCBCR_ENABLE (1 << 6)

// DP_DOWNSTREAM_PORT_0		    0x80
/* HDMI2.1 PCON FRL CONFIGURATION */
# define DP_PCON_MAX_FRL_BW                 (7 << 2)
# define DP_PCON_MAX_0GBPS                  (0 << 2)
# define DP_PCON_MAX_9GBPS                  (1 << 2)
# define DP_PCON_MAX_18GBPS                 (2 << 2)
# define DP_PCON_MAX_24GBPS                 (3 << 2)
# define DP_PCON_MAX_32GBPS                 (4 << 2)
# define DP_PCON_MAX_40GBPS                 (5 << 2)
# define DP_PCON_MAX_48GBPS                 (6 << 2)
# define DP_PCON_SOURCE_CTL_MODE            (1 << 5)
/*
 * VESA DP-to-HDMI PCON Specification adds caps for colorspace
 * conversion in DFP cap DPCD 83h. Sec6.1 Table-3.
 * Based on the available support the source can enable
 * color conversion by writing into PROTOCOL_COVERTER_CONTROL_2
 * DPCD 3052h.
 */
# define DP_DS_HDMI_BT601_RGB_YCBCR_CONV    (1 << 5)
# define DP_DS_HDMI_BT709_RGB_YCBCR_CONV    (1 << 6)
# define DP_DS_HDMI_BT2020_RGB_YCBCR_CONV   (1 << 7)

#define DP_EDP_MSO_LINK_CAPABILITIES        0x7a4    /* eDP 1.4 */
# define DP_EDP_MSO_NUMBER_OF_LINKS_MASK    (7 << 0)
# define DP_EDP_MSO_NUMBER_OF_LINKS_SHIFT   0
# define DP_EDP_MSO_INDEPENDENT_LINK_BIT    (1 << 3)

// DP_TRANSMITTER_CAPABILITY_PHY_REPEATER1		    0xf0021 /* 1.4a */
# define DP_VOLTAGE_SWING_LEVEL_3_SUPPORTED		    BIT(0)
# define DP_PRE_EMPHASIS_LEVEL_3_SUPPORTED		    BIT(1)

#ifdef BPM_DP_READ_LTTPR_CAPS_DPCD_ARG_NOT_PRESENT

enum drm_dp_phy {
	DP_PHY_DPRX,

	DP_PHY_LTTPR1,
	DP_PHY_LTTPR2,
	DP_PHY_LTTPR3,
	DP_PHY_LTTPR4,
	DP_PHY_LTTPR5,
	DP_PHY_LTTPR6,
	DP_PHY_LTTPR7,
	DP_PHY_LTTPR8,

	DP_MAX_LTTPR_COUNT = DP_PHY_LTTPR8,
};

#define DP_PHY_LTTPR(i)					    (DP_PHY_LTTPR1 + (i))

#define DP_LTTPR_COMMON_CAP_SIZE	8
#define DP_LTTPR_PHY_CAP_SIZE		3

#define drm_dp_read_lttpr_common_caps LINUX_I915_BACKPORT(drm_dp_read_lttpr_common_caps)
int drm_dp_read_lttpr_common_caps(struct drm_dp_aux *aux,
                                  u8 caps[DP_LTTPR_COMMON_CAP_SIZE]);

#define drm_dp_read_lttpr_phy_caps LINUX_I915_BACKPORT(drm_dp_read_lttpr_phy_caps)
int drm_dp_read_lttpr_phy_caps(struct drm_dp_aux *aux,
                               enum drm_dp_phy dp_phy,
                               u8 caps[DP_LTTPR_PHY_CAP_SIZE]);
#define drm_dp_lttpr_count LINUX_I915_BACKPORT(drm_dp_lttpr_count)
int drm_dp_lttpr_count(const u8 cap[DP_LTTPR_COMMON_CAP_SIZE]);
#define drm_dp_lttpr_max_link_rate LINUX_I915_BACKPORT(drm_dp_lttpr_max_link_rate)
int drm_dp_lttpr_max_link_rate(const u8 caps[DP_LTTPR_COMMON_CAP_SIZE]);
#define drm_dp_lttpr_max_lane_count LINUX_I915_BACKPORT(drm_dp_lttpr_max_lane_count)
int drm_dp_lttpr_max_lane_count(const u8 caps[DP_LTTPR_COMMON_CAP_SIZE]);
#define drm_dp_lttpr_voltage_swing_level_3_supported LINUX_I915_BACKPORT(drm_dp_lttpr_voltage_swing_level_3_supported)
bool drm_dp_lttpr_voltage_swing_level_3_supported(const u8 caps[DP_LTTPR_PHY_CAP_SIZE]);
#define drm_dp_lttpr_pre_emphasis_level_3_supported LINUX_I915_BACKPORT(drm_dp_lttpr_pre_emphasis_level_3_supported)
bool drm_dp_lttpr_pre_emphasis_level_3_supported(const u8 caps[DP_LTTPR_PHY_CAP_SIZE]);
#define drm_dp_lttpr_link_train_clock_recovery_delay LINUX_I915_BACKPORT(drm_dp_lttpr_link_train_clock_recovery_delay)
void drm_dp_lttpr_link_train_clock_recovery_delay(void);
#define drm_dp_lttpr_link_train_channel_eq_delay LINUX_I915_BACKPORT(drm_dp_lttpr_link_train_channel_eq_delay)
void drm_dp_lttpr_link_train_channel_eq_delay(const struct drm_dp_aux *aux,
					      const u8 caps[DP_LTTPR_PHY_CAP_SIZE]);

#define __DP_LTTPR1_BASE				    0xf0010 /* 1.3 */
#define __DP_LTTPR2_BASE				    0xf0060 /* 1.3 */
#define DP_LTTPR_BASE(dp_phy) \
	(__DP_LTTPR1_BASE + (__DP_LTTPR2_BASE - __DP_LTTPR1_BASE) * \
		((dp_phy) - DP_PHY_LTTPR1))

#define DP_LTTPR_REG(dp_phy, lttpr1_reg) \
	(DP_LTTPR_BASE(dp_phy) - DP_LTTPR_BASE(DP_PHY_LTTPR1) + (lttpr1_reg))

#define DP_TRAINING_AUX_RD_INTERVAL_PHY_REPEATER(dp_phy)	\
	DP_LTTPR_REG(dp_phy, DP_TRAINING_AUX_RD_INTERVAL_PHY_REPEATER1)

// DP_LANE0_1_STATUS_PHY_REPEATER1			    0xf0030 /* 1.3 */
#define DP_LANE0_1_STATUS_PHY_REPEATER(dp_phy) \
	DP_LTTPR_REG(dp_phy, DP_LANE0_1_STATUS_PHY_REPEATER1)

// DP_TRAINING_PATTERN_SET_PHY_REPEATER1		    0xf0010 /* 1.3 */
#define DP_TRAINING_PATTERN_SET_PHY_REPEATER(dp_phy) \
	DP_LTTPR_REG(dp_phy, DP_TRAINING_PATTERN_SET_PHY_REPEATER1)

// DP_TRAINING_LANE0_SET_PHY_REPEATER1		    0xf0011 /* 1.3 */
#define DP_TRAINING_LANE0_SET_PHY_REPEATER(dp_phy) \
	DP_LTTPR_REG(dp_phy, DP_TRAINING_LANE0_SET_PHY_REPEATER1)

#endif

#ifdef DRM_DP_GET_ADJUST_NOT_PRESENT
/* DP 2.0 128b/132b Link Layer */
#define DP_ADJUST_TX_FFE_PRESET_LANE0_MASK  (0xf << 0)
#define DP_ADJUST_TX_FFE_PRESET_LANE0_SHIFT 0
#define DP_ADJUST_TX_FFE_PRESET_LANE1_MASK  (0xf << 4)
#define DP_ADJUST_TX_FFE_PRESET_LANE1_SHIFT 4

# define DP_TX_FFE_PRESET_VALUE_MASK        (0xf << 0) /* 2.0 128b/132b Link Layer */

#define drm_dp_get_adjust_tx_ffe_preset LINUX_I915_BACKPORT(drm_dp_get_adjust_tx_ffe_preset)
u8 drm_dp_get_adjust_tx_ffe_preset(const u8 link_status[DP_LINK_STATUS_SIZE],
				   int lane);

#define DP_MAIN_LINK_CHANNEL_CODING_PHY_REPEATER        0xf0006 /* 2.0 */
#define DP_PHY_REPEATER_128B132B_SUPPORTED              (1 << 0)
/* See DP_128B132B_SUPPORTED_LINK_RATES for values */
#define DP_PHY_REPEATER_128B132B_RATES                  0xf0007 /* 2.0 */

#define DP_EDP_BACKLIGHT_CONTROL_MODE_MASK              (3 << 0)
#define DP_EDP_BACKLIGHT_CONTROL_MODE_DPCD              (2 << 0)
#define DP_EDP_BACKLIGHT_FREQ_AUX_SET_ENABLE            (1 << 3)
#define DP_EDP_BACKLIGHT_AUX_ENABLE_CAP                 (1 << 2)
#define DP_EDP_BACKLIGHT_BRIGHTNESS_BYTE_COUNT          (1 << 2)
#define DP_EDP_TCON_BACKLIGHT_ADJUSTMENT_CAP            (1 << 0)
#define DP_EDP_BACKLIGHT_BRIGHTNESS_AUX_SET_CAP         (1 << 1)
#define DP_EDP_BACKLIGHT_FREQ_AUX_SET_CAP               (1 << 5)
#define DP_EDP_BACKLIGHT_ENABLE                         (1 << 0)
#define DP_EDP_PWMGEN_BIT_COUNT_MASK                    (0x1f << 0)
#define DP_EDP_PWMGEN_BIT_COUNT                         0x724
#define DP_EDP_BACKLIGHT_FREQ_SET                       0x728
#define DP_EDP_BACKLIGHT_MODE_SET_REGISTER              0x721
#define EDP_DISPLAY_CTL_CAP_SIZE                        3
#define DP_EDP_BACKLIGHT_FREQ_BASE_KHZ                  27000
#define DP_EDP_DISPLAY_CONTROL_REGISTER                 0x720
#define DP_EDP_BACKLIGHT_BRIGHTNESS_MSB                 0x722

#endif /* DRM_DP_GET_ADJUST_NOT_PRESENT */

#ifdef DRM_EDP_BACKLIGHT_NOT_PRESENT
/**
* struct drm_edp_backlight_info - Probed eDP backlight info struct
* @pwmgen_bit_count: The pwmgen bit count
* @pwm_freq_pre_divider: The PWM frequency pre-divider value being used for this backlight, if any
* @max: The maximum backlight level that may be set
* @lsb_reg_used: Do we also write values to the DP_EDP_BACKLIGHT_BRIGHTNESS_LSB register?
* @aux_enable: Does the panel support the AUX enable cap?
*
* This structure contains various data about an eDP backlight, which can be populated by using
* drm_edp_backlight_init().
*/
#define drm_edp_backlight_info LINUX_I915_BACKPORT(drm_edp_backlight_info)
struct drm_edp_backlight_info {
		u8 pwmgen_bit_count;
		u8 pwm_freq_pre_divider;
		u16 max;

		bool lsb_reg_used : 1;
		bool aux_enable : 1;
		bool aux_set : 1;
};

#define drm_edp_backlight_init LINUX_I915_BACKPORT(drm_edp_backlight_init)
int
drm_edp_backlight_init(struct drm_dp_aux *aux, struct drm_edp_backlight_info *bl,
		       u16 driver_pwm_freq_hz, const u8 edp_dpcd[EDP_DISPLAY_CTL_CAP_SIZE],
		       u16 *current_level, u8 *current_mode);
#define drm_edp_backlight_set_level LINUX_I915_BACKPORT(drm_edp_backlight_set_level)
int drm_edp_backlight_set_level(struct drm_dp_aux *aux, const struct drm_edp_backlight_info *bl,
				u16 level);
#define drm_edp_backlight_enable LINUX_I915_BACKPORT(drm_edp_backlight_enable)
int drm_edp_backlight_enable(struct drm_dp_aux *aux, const struct drm_edp_backlight_info *bl,
			     u16 level);
#define drm_edp_backlight_disable LINUX_I915_BACKPORT(drm_edp_backlight_disable)
int drm_edp_backlight_disable(struct drm_dp_aux *aux, const struct drm_edp_backlight_info *bl);

#ifndef DRM_EDP_BACKLIGHT_SUPPORT_PRESENT
#define drm_edp_backlight_supported LINUX_I915_BACKPORT(drm_edp_backlight_supported)
static inline bool
drm_edp_backlight_supported(const u8 edp_dpcd[EDP_DISPLAY_CTL_CAP_SIZE])
{
       return (edp_dpcd[1] & DP_EDP_TCON_BACKLIGHT_ADJUSTMENT_CAP) &&
	       (edp_dpcd[2] & DP_EDP_BACKLIGHT_BRIGHTNESS_AUX_SET_CAP);
}
#endif /* DRM_EDP_BACKLIGHT_SUPPORT_PRESENT */
#endif /* DRM_EDP_BACKLIGHT_NOT_PRESENT */

#ifndef MAX_FLR_NOT_PRESENT
#define drm_hdmi_sink_max_frl_rate LINUX_I915_BACKPORT(drm_hdmi_sink_max_frl_rate)
int drm_hdmi_sink_max_frl_rate(struct drm_connector *connector);

#define drm_hdmi_sink_dsc_max_frl_rate LINUX_I915_BACKPORT(drm_hdmi_sink_dsc_max_frl_rate)
int drm_hdmi_sink_dsc_max_frl_rate(struct drm_connector *connector);
#endif

#ifdef BPM_DRM_DP_DSC_SINK_SUPPORTS_FORMAT_NOT_PRESENT
/**
 * drm_dp_dsc_sink_supports_format() - check if sink supports DSC with given output format
 * @dsc_dpcd : DSC-capability DPCDs of the sink
 * @output_format: output_format which is to be checked
 *
 * Returns true if the sink supports DSC with the given output_format, false otherwise.
 */
static inline bool
drm_dp_dsc_sink_supports_format(const u8 dsc_dpcd[DP_DSC_RECEIVER_CAP_SIZE], u8 output_format)
{
       return dsc_dpcd[DP_DSC_DEC_COLOR_FORMAT_CAP - DP_DSC_SUPPORT] & output_format;
}
#endif /* BPM_DRM_DP_DSC_SINK_SUPPORTS_FORMAT_NOT_PRESENT */

/**
 * drm_dp_has_quirk() - does the DP device have a specific quirk
 * @desc: Device descriptor filled by drm_dp_read_desc()
 * @quirk: Quirk to query for
 *
 * Return true if DP device identified by @desc has @quirk.
 */
static inline bool
LINUX_I915_BACKPORT(drm_dp_has_quirk)(const struct drm_dp_desc *desc, enum drm_dp_quirk quirk)
{
	return drm_dp_has_quirk(desc, 0, quirk);
}

#define drm_dp_has_quirk LINUX_I915_BACKPORT(drm_dp_has_quirk)

#define drm_dp_get_pcon_max_frl_bw LINUX_I915_BACKPORT(drm_dp_get_pcon_max_frl_bw)
int drm_dp_get_pcon_max_frl_bw(const u8 dpcd[DP_RECEIVER_CAP_SIZE],
			       const u8 port_cap[4]);
#define drm_dp_downstream_rgb_to_ycbcr_conversion LINUX_I915_BACKPORT(drm_dp_downstream_rgb_to_ycbcr_conversion)
bool drm_dp_downstream_rgb_to_ycbcr_conversion(const u8 dpcd[DP_RECEIVER_CAP_SIZE],
					       const u8 port_cap[4], u8 color_spc);
#define drm_dp_pcon_convert_rgb_to_ycbcr LINUX_I915_BACKPORT(drm_dp_pcon_convert_rgb_to_ycbcr)
int drm_dp_pcon_convert_rgb_to_ycbcr(struct drm_dp_aux *aux, u8 color_spc);
#define drm_dp_pcon_hdmi_link_active LINUX_I915_BACKPORT(drm_dp_pcon_hdmi_link_active)
bool drm_dp_pcon_hdmi_link_active(struct drm_dp_aux *aux);
#define drm_dp_dpcd_read_phy_link_status LINUX_I915_BACKPORT(drm_dp_dpcd_read_phy_link_status)
int drm_dp_dpcd_read_phy_link_status(struct drm_dp_aux *aux,
				     enum drm_dp_phy dp_phy,
				     u8 link_status[DP_LINK_STATUS_SIZE]);

#endif /* _BACKPORT_DRM_DP_HELPER_H_ */
