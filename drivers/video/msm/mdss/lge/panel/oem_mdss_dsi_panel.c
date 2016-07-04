/* Copyright (c) 2012-2015, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */


#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/pwm.h>
#include <linux/err.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/leds.h>

#include "oem_mdss_dsi_common.h"
#include "oem_mdss_dsi.h"
#include <linux/input/lge_touch_notify.h>
#include <soc/qcom/lge/board_lge.h>
#include <linux/mfd/p1_dsv.h>
#include <linux/syscalls.h>

DEFINE_LED_TRIGGER(bl_led_trigger);
#define PIN_DSV_ENA "DSV_ENA"
#define PIN_DISP_EN "DISP_EN"
#define PIN_RESET "RESET"
#define PANEL_SEQUENCE(name, state) do { pr_info("[PanelSequence][%s] %d\n", name, state); } while (0)

//LGE_UPDATE_S (june1014.lee@lge.com. 2015.03.04). SRE
#if defined(CONFIG_LGE_P1_SRE_SUPPORTED)
struct mdss_panel_data *pdata_sre = NULL;
static unsigned int sre_mode = 0;
struct sre_cmds_desc {
	struct dsi_panel_cmds sre_cmds[2];
};
static char *sre_lgd_dt[] = {
	"lgd,sre-cmds-off",	// sre off
	"lgd,sre-cmds-on",	// sre on
};
static char *sre_jdi_dt[] = {
	"jdi,sre-cmds-off",	// sre off
	"jdi,sre-cmds-on",	// sre on
};
static char *sre_lgdsic_dt[] = {
	"lge,sre-cmds-off",     // sre off
	"lge,sre-cmds-on",      // sre on
};
static struct sre_cmds_desc *sre_cmds_set;
#endif
//LGE_UPDATE_E (june1014.lee@lge.com. 2015.03.04). SRE

#if defined(CONFIG_LGE_BLMAP_STORE_MODE)
struct mdss_panel_info *pinfo_store_mode = NULL;
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_AOD_SUPPORT)
int req_bklt_type = 1; // BL_PWM, BL_WLED, BL_DCS_CMD, UNKNOWN_CTRL
#else
extern void mdss_dsi_panel_bklt_pwm(struct mdss_dsi_ctrl_pdata *ctrl, int level);
#endif

#if IS_ENABLED(CONFIG_BACKLIGHT_LM3697)
extern void lm3697_lcd_backlight_set_level(int level);
#endif
#if IS_ENABLED(CONFIG_BACKLIGHT_LM3632)
extern void lm3632_lcd_backlight_set_level(int level);
extern void lm3632_dsv_fd_ctrl(int dsv_fd);
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_AOD_SUPPORT)
#define FB_BLANK_U2 FB_BLANK_VSYNC_SUSPEND
#define FB_BLANK_U1 FB_BLANK_NORMAL

struct mdss_panel_data *pdata_mode = NULL;

struct mode_cmds_desc {
	struct dsi_panel_cmds mode_cmds[LGE_PANEL_CMD_CHANGE_NUM];
};
static struct mode_cmds_desc *mode_cmds_set;
static char *mode_lg4945_dt[] = {
	"lge,mode-change-cmds-u2-to-u1",
	"lge,mode-change-cmds-u3-to-u1",
	"lge,mode-change-cmds-u1-to-u2",
	"lge,mode-change-cmds-u3-to-u2",
	"lge,mode-change-cmds-u1-to-u3",
	"lge,mode-change-cmds-u2-to-u3",
	"lge,mode-change-cmds-u3-ready",
	"lge,mode-change-cmds-proximity-u2-to-u3",
	"lge,mode-change-cmds-proximity-u3-to-u2",
	"lge,mode-change-cmds-memwrite"
};

#define EXT_WATCH_LUT_MAX   7
struct ExtWatchFontLUTConfig {
	/* LUT */
	u32	RGB_blue;
	u32	RGB_green;
	u32	RGB_red;
};

struct ExtWatchFontPropertyConfig {
	u32	len;
	u32	max_num;		/* The number of LUT */
	struct	ExtWatchFontLUTConfig	LUT[EXT_WATCH_LUT_MAX];
};

struct ExtWatchFontPostionConfig {
	u32	len;
	u32	wat_startx;
	u32	wat_endx;
	u32	wat_starty;
	u32	wat_endy;
	u32	h1x_pos;		/* 1 ~ 9hour position */
	u32	h10x_pos;		/* 10, 20 hour position */
	u32	m1x_pos;		/* 1 ~ 9min position */
	u32	m10x_pos;		/* 10 ~ 50 min position */
	u32	clx_pos;		/* 1 ~ 60 second position */
};

struct img_tune_cmds_desc {
	struct dsi_panel_cmds img_tune_cmds[LGE_PANEL_IMG_TUNE_NUM];
};
static struct img_tune_cmds_desc *img_tune_cmds_set;
static char *img_tune_lg4945_dt[] = {
	"lge,sharpness-cmds-on",
	"lge,color_enhancement-cmds-on",
};
#endif

extern int mdss_dsi_parse_dcs_cmds(struct device_node *np,
		struct dsi_panel_cmds *pcmds, char *cmd_key, char *link_key);

//LGE_UPDATE_S (june1014.lee@lge.com. 2015.03.04). SRE
#if defined(CONFIG_LGE_P1_SRE_SUPPORTED)
int mdss_dsi_panel_sre_apply(unsigned int enabled) {
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_dsi_ctrl_pdata *other = NULL;

	if (pdata_sre == NULL) {
		pr_err("%s: invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata_sre, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (pdata_sre->panel_info.panel_power_state == 0) {
		pr_err("%s: Panel off state. Ignore sre command\n", __func__);
		return -EINVAL;
	}

	if (enabled == SRE_CHANGE_OFF || enabled == SRE_CHANGE_ON) {
		// sre mode changed
		pr_info("%s: send the sre cmd (%d) from kernel\n", __func__, enabled);

		if (sre_cmds_set->sre_cmds[enabled].cmd_cnt) {
			mdss_dsi_panel_cmds_send(ctrl, &sre_cmds_set->sre_cmds[enabled]);
			other = mdss_dsi_get_other_ctrl(ctrl);
			mdss_dsi_panel_cmds_send(other, &sre_cmds_set->sre_cmds[enabled]);
			pr_info("%s: SRE is changed\n", __func__);
		} else {
			pr_err("%s: sre is changed, but cmd is empty\n", __func__);
			return -EINVAL;
		}
	} else {
		pr_info("%s: SRE is not changed\n", __func__);
	}
	return 0;
}
EXPORT_SYMBOL(mdss_dsi_panel_sre_apply);

static ssize_t sre_mode_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", sre_mode);
}

static ssize_t sre_mode_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int param;
	ssize_t ret = strnlen(buf, PAGE_SIZE);

	sscanf(buf, "%u", &param);
	sre_mode = param;
	pr_debug("%s: sre mode change[%d]\n", __func__, sre_mode);
	mdss_dsi_panel_sre_apply(param);
	return ret;
}

static struct device_attribute sre_status_attrs[] = {
	__ATTR(sre_status, 0644, sre_mode_get, sre_mode_set),
};
#endif
//LGE_UPDATE_E (june1014.lee@lge.com. 2015.03.04). SRE

#if defined(CONFIG_LGE_BLMAP_STORE_MODE)
static ssize_t blmap_store_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	if (pinfo_store_mode == NULL) {
		pr_err("%s: No panel information\n", __func__);
		return -EINVAL;
	}

	return sprintf(buf, "%d\n", pinfo_store_mode->lge_pan_info.bl_store_mode);
}

static ssize_t blmap_store_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int param;
    ssize_t ret = 0;

	if (pinfo_store_mode == NULL) {
		pr_err("%s: No panel information\n", __func__);
		return -EINVAL;
	}

	ret = strnlen(buf, PAGE_SIZE);
    if( ret>0 ) {
        sscanf(buf, "%x", &param);
        pinfo_store_mode->lge_pan_info.bl_store_mode = param;
        pr_info("%s: bl_store_mode[%d]\n", __func__, pinfo_store_mode->lge_pan_info.bl_store_mode);
    }

	return ret;
}

static struct device_attribute blmap_adjust_status_attrs[] = {
	__ATTR(enable, 0644, blmap_store_mode_show, blmap_store_mode_store),
};
#endif

#if IS_ENABLED(CONFIG_LGE_DISPLAY_AOD_SUPPORT)
int lge_lg4945_panel_img_tune_cmd_send(int mode) {
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;
	struct mdss_dsi_ctrl_pdata *other = NULL;

	if (pdata_mode == NULL) {
		pr_err("%s: invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl = container_of(pdata_mode, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (img_tune_cmds_set->img_tune_cmds[mode].cmd_cnt) {
		mdss_dsi_panel_cmds_send(ctrl, &img_tune_cmds_set->img_tune_cmds[mode]);
		other = mdss_dsi_get_other_ctrl(ctrl);
		if (other)
			mdss_dsi_panel_cmds_send(other, &img_tune_cmds_set->img_tune_cmds[mode]);
		pr_debug("%s: img tune mode %d cmds send\n", __func__, mode);
	} else {
		pr_err("%s: img tune %d cmd is empty\n", __func__, mode);
		return -EINVAL;
	}

	return 0;
}

static ssize_t sharpness_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%x\n", img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[1].payload[3]);
}

static ssize_t sharpness_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	unsigned int param;
	ssize_t ret = strnlen(buf, PAGE_SIZE);

	sscanf(buf, "%x", &param);
	img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[1].payload[3] = param;
	pr_info("%s: strength=0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n", __func__,
		img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[0].payload[0],
		img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[0].payload[1],
		img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[1].payload[0],
		img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[1].payload[1],
		img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[1].payload[2],
		img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[1].payload[3],
		img_tune_cmds_set->img_tune_cmds[LGE_PANEL_SH_ON].cmds[1].payload[4]);
	lge_lg4945_panel_img_tune_cmd_send(LGE_PANEL_SH_ON);

	return ret;
}

static ssize_t color_enhancement_get(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int i=0;

	for(i=0; i<24; i++){
		sprintf(buf, "%s 0x%02X", buf, img_tune_cmds_set->img_tune_cmds[LGE_PANEL_CE_ON].cmds[2].payload[i]);
		if(((i+1)%6) == 0)
			sprintf(buf, "%s \n", buf);
	}

	return sprintf(buf, "%s\n", buf);
}

static ssize_t color_enhancement_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	ssize_t ret = strnlen(buf, PAGE_SIZE);
	int set_color_enhancement[128];
	int i, ie_function;

	memset(set_color_enhancement,0,24*sizeof(int));
	set_color_enhancement[0] = img_tune_cmds_set->img_tune_cmds[LGE_PANEL_CE_ON].cmds[2].payload[0];
	sscanf(buf, "%x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x %x",
		&ie_function,&set_color_enhancement[1],&set_color_enhancement[2],&set_color_enhancement[3],
		&set_color_enhancement[4],&set_color_enhancement[5],&set_color_enhancement[6],&set_color_enhancement[7],
		&set_color_enhancement[8],&set_color_enhancement[9],&set_color_enhancement[10],&set_color_enhancement[11],
		&set_color_enhancement[12],&set_color_enhancement[13],&set_color_enhancement[14],&set_color_enhancement[15],
		&set_color_enhancement[16],&set_color_enhancement[17],&set_color_enhancement[18],&set_color_enhancement[19],
		&set_color_enhancement[20],&set_color_enhancement[21],&set_color_enhancement[22],&set_color_enhancement[23]);

	img_tune_cmds_set->img_tune_cmds[LGE_PANEL_CE_ON].cmds[1].payload[1] = ie_function;
	pr_debug("%s: 0xF0 0x%02X ", __func__, img_tune_cmds_set->img_tune_cmds[LGE_PANEL_CE_ON].cmds[1].payload[1]);

	for(i=0; i<24; i++){
		img_tune_cmds_set->img_tune_cmds[LGE_PANEL_CE_ON].cmds[2].payload[i] = set_color_enhancement[i];
		pr_debug("0x%02X ", img_tune_cmds_set->img_tune_cmds[LGE_PANEL_CE_ON].cmds[2].payload[i]);
	}
	pr_debug("\n");

	lge_lg4945_panel_img_tune_cmd_send(LGE_PANEL_CE_ON);
	return ret;
}

static struct device_attribute panel_tuning_device_attrs[] = {
	__ATTR(sharpness, 0644, sharpness_get, sharpness_set),
	__ATTR(color_enhance, 0644, color_enhancement_get, color_enhancement_set),
};

int lge_lg4945_panel_mode_cmd_send(int switch_cmd, struct mdss_dsi_ctrl_pdata *ctrl) {
	struct mdss_dsi_ctrl_pdata *other = NULL;
	if (pdata_mode == NULL) {
		pr_err("%s: invalid input data\n", __func__);
		return -EINVAL;
	}

	if (pdata_mode->panel_info.panel_power_state == 0) {
		pr_err("%s: Panel off state. Ignore mode change cmd %d\n", __func__, switch_cmd);
		return -EINVAL;
	}

	if ((switch_cmd < 0) || (switch_cmd >= LGE_PANEL_CMD_CHANGE_NUM)) {
		pr_err("%s: invalid change cmd %d\n", __func__, switch_cmd);
		return -EINVAL;
	}

	if (ctrl == NULL) {
		ctrl = container_of(pdata_mode, struct mdss_dsi_ctrl_pdata, panel_data);
		other = mdss_dsi_get_other_ctrl(ctrl);
		if (other)
			ctrl = other;
	}

	if (mdss_dsi_split_display_enabled() && mdss_dsi_is_left_ctrl(ctrl)) {
		pr_err("%s: dsi control # is not correct\n", __func__);
		return 0;
	}

	if (mode_cmds_set->mode_cmds[switch_cmd].cmd_cnt) {
		mdss_dsi_panel_cmds_send(ctrl, &mode_cmds_set->mode_cmds[switch_cmd]);
		pr_info("[PowerMode] %s: send mode change cmd %d\n", __func__, switch_cmd);
	} else {
		pr_err("[PowerMode] %s: mode change cmd %d is empty\n", __func__, switch_cmd);
		return -EINVAL;
	}

	return 0;
}

int lge_dsi_pack_dcs_cmds(struct dsi_panel_cmds *pcmds, char *cmd_str, int blen, char *link_key) {
	int len;
	char *buf, *bp;
	struct dsi_ctrl_hdr *dchdr;
	int i, cnt;

	if (!cmd_str) {
		pr_err("%s: failed, cmd_str NULL\n", __func__);
		return -ENOMEM;
	}

	buf = kzalloc(sizeof(char) * blen, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	memcpy(buf, cmd_str, blen);

	/* scan dcs commands */
	bp = buf;
	len = blen;
	cnt = 0;
	while (len >= sizeof(*dchdr)) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		dchdr->dlen = ntohs(dchdr->dlen);
		if (dchdr->dlen > len) {
			pr_err("%s: dtsi cmd=%x error, len=%d",
				__func__, dchdr->dtype, dchdr->dlen);
			goto exit_free;
		}
		bp += sizeof(*dchdr);
		len -= sizeof(*dchdr);
		bp += dchdr->dlen;
		len -= dchdr->dlen;
		cnt++;
	}

	pr_err("%s: dcs_cmd=%x len=%d, cmd_cnt=%d link_state=%s\n", __func__,
		buf[0], blen, cnt, link_key);

	if (len != 0) {
		pr_err("%s: dcs_cmd=%x len=%d error!",
				__func__, buf[0], blen);
		goto exit_free;
	}

	pcmds->cmds = kzalloc(cnt * sizeof(struct dsi_cmd_desc),
						GFP_KERNEL);
	if (!pcmds->cmds)
		goto exit_free;

	pcmds->cmd_cnt = cnt;
	pcmds->buf = buf;
	pcmds->blen = blen;

	bp = buf;
	len = blen;
	for (i = 0; i < cnt; i++) {
		dchdr = (struct dsi_ctrl_hdr *)bp;
		len -= sizeof(*dchdr);
		bp += sizeof(*dchdr);
		pcmds->cmds[i].dchdr = *dchdr;
		pcmds->cmds[i].payload = bp;
		bp += dchdr->dlen;
		len -= dchdr->dlen;
	}

	/*Set default link state to LP Mode*/
	pcmds->link_state = DSI_LP_MODE;

	if (link_key) {
		if (!strcmp(link_key, "dsi_hs_mode"))
			pcmds->link_state = DSI_HS_MODE;
		else
			pcmds->link_state = DSI_LP_MODE;
	}

	return 0;

exit_free:
	kfree(buf);
	return -ENOMEM;

}

static int wpos_x = -1;
static int wpos_y = -1;
static struct ExtWatchFontLUTConfig wlut[EXT_WATCH_LUT_MAX];
int lgd_lg4895_panel_watch_ctl_cmd_send(int type, void *data, struct mdss_dsi_ctrl_pdata *ctrl) {
	struct ExtWatchFontPostionConfig wpos_cfg;
	struct ExtWatchFontPropertyConfig wprop_cfg;
	struct dsi_panel_cmds cmds;
	char cmd_str[64] = {0,};
	char link_key[12] = "dsi_hs_mode";
	int i;

	if (pdata_mode == NULL) {
		pr_err("%s: invalid input data\n", __func__);
		return -EINVAL;
	}

	if (pdata_mode->panel_info.panel_power_state == 0) {
		pr_err("%s: Panel off state. Ignore watch control cmd %d\n", __func__, type);
		return -EINVAL;
	}

	/*
	if (data == 0) {
		pr_err("%s: invalid watch control data. Ignore watch control cmd %d\n", __func__, type);
		return -EINVAL;
	}
	*/

	if (ctrl == NULL) {
		ctrl = container_of(pdata_mode, struct mdss_dsi_ctrl_pdata, panel_data);
	}

	if (mdss_dsi_split_display_enabled() && mdss_dsi_is_left_ctrl(ctrl)) {
		pr_err("%s: dsi control # is not correct\n", __func__);
		return 0;
	}

	switch (type) {
		case WATCH_POS_UPDATE:
			if (data) {
				wpos_cfg = *(struct ExtWatchFontPostionConfig *)data;
			} else {
				wpos_cfg.wat_startx = wpos_x;
				wpos_cfg.wat_starty = wpos_y;
			}
			// need to bias start_x position by AOD region(+200) & WATCH_START_X[10:8] as 1h(+256)
			sprintf(cmd_str,
					"%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
					0x39, 0x01, 0x00, 0x00, 0x00, 0x00, 0x12,
					0xCE, 0x04, 0x22, 0x0C, 0xFF,
					0x01, /*(char)(wpos_cfg.wat_endx-wpos_cfg.wat_startx)*/0x8C, /*(char)(wpos_cfg.wat_endy-wpos_cfg.wat_starty)*/0x32, (char)(wpos_cfg.wat_startx-456), (char)wpos_cfg.wat_starty,
					0x1D, 0x52, 0x00, 0x00, 0x00,
					0xFF, 0xFF, 0xFF);
			if (lge_dsi_pack_dcs_cmds(&cmds,cmd_str,25,link_key)==0) {
				mdss_dsi_panel_cmds_send(ctrl, &cmds);
				pr_err("%s : send watch position(%d,%d) through mipi command [%x(%d)]\n",
						__func__, wpos_cfg.wat_startx-200, wpos_cfg.wat_starty, cmd_str[0], 25);
				kfree((&cmds)->buf);
				kfree((&cmds)->cmds);

				wpos_x = wpos_cfg.wat_startx;
				wpos_y = wpos_cfg.wat_starty;
			}
			break;
		case WATCH_LUT_UPDATE:
			if (data) {
				wprop_cfg = *(struct ExtWatchFontPropertyConfig *)data;
			} else {
				for (i=0; i<7; i++) {
					wprop_cfg.LUT[i].RGB_blue = wlut[i].RGB_blue;
					wprop_cfg.LUT[i].RGB_green = wlut[i].RGB_green;
					wprop_cfg.LUT[i].RGB_red = wlut[i].RGB_red;
				}
			}
			sprintf(cmd_str,
					"%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
					0x39, 0x01, 0x00, 0x00, 0x00, 0x00, 0x16,
					0xCF, (char)wprop_cfg.LUT[0].RGB_blue, (char)wprop_cfg.LUT[0].RGB_green, (char)wprop_cfg.LUT[0].RGB_red,
					(char)wprop_cfg.LUT[1].RGB_blue, (char)wprop_cfg.LUT[1].RGB_green, (char)wprop_cfg.LUT[1].RGB_red,
					(char)wprop_cfg.LUT[2].RGB_blue, (char)wprop_cfg.LUT[2].RGB_green, (char)wprop_cfg.LUT[2].RGB_red,
					(char)wprop_cfg.LUT[3].RGB_blue, (char)wprop_cfg.LUT[3].RGB_green, (char)wprop_cfg.LUT[3].RGB_red,
					(char)wprop_cfg.LUT[4].RGB_blue, (char)wprop_cfg.LUT[4].RGB_green, (char)wprop_cfg.LUT[4].RGB_red,
					(char)wprop_cfg.LUT[5].RGB_blue, (char)wprop_cfg.LUT[5].RGB_green, (char)wprop_cfg.LUT[5].RGB_red,
					(char)wprop_cfg.LUT[6].RGB_blue, (char)wprop_cfg.LUT[6].RGB_green, (char)wprop_cfg.LUT[6].RGB_red);
			if (lge_dsi_pack_dcs_cmds(&cmds,cmd_str,29,link_key)==0) {
				mdss_dsi_panel_cmds_send(ctrl, &cmds);
				pr_err("%s : send lut info. through mipi command [%x(%d)]\n",
						__func__, cmd_str[0], 29);
				for (i=0; i<7; i++) {
					pr_err("%s : LUT[%d] = %02x%02x%02x",
							__func__, i, (char)wprop_cfg.LUT[i].RGB_blue, (char)wprop_cfg.LUT[i].RGB_green, (char)wprop_cfg.LUT[i].RGB_red );
				}
				kfree((&cmds)->buf);
				kfree((&cmds)->cmds);

				for (i=0; i<7; i++) {
					wlut[i].RGB_blue = wprop_cfg.LUT[i].RGB_blue;
					wlut[i].RGB_green = wprop_cfg.LUT[i].RGB_green;
					wlut[i].RGB_red = wprop_cfg.LUT[i].RGB_red;
				}
			}
			break;
	}

	return 0;
}

#define CHECK_FAKE_U3_PATH     "/sys/devices/virtual/input/lge_touch/u3fake"
int is_fake_u3(void) {
       const int SIZE = 128;
       char buf[SIZE];
       int value = 0;
       int fd = -1;

       mm_segment_t old_fs = get_fs();

       set_fs(KERNEL_DS);
       fd = sys_open(CHECK_FAKE_U3_PATH, O_RDONLY,0);
       if (fd < 0) {
               pr_err(" %s: u3fake node is not exist \n", __func__);
               return -1;
       }

       memset(buf,0,SIZE*sizeof(char));
       sys_read(fd, buf, sizeof(buf));
       value = simple_strtol(buf, NULL, 0);
       pr_info("%s: value:%d, buf:%s\n", __func__, value, buf);

       sys_close(fd);
       set_fs(old_fs);

    return value;
}

int lge_lg4945_panel_mode_switch(struct msm_fb_data_type *mfd, unsigned int req_blank_mode) {
	int cur_panel_mode;
	int mode_switch_cmd;

	cur_panel_mode = mfd->panel_info->lge_pan_info.cur_panel_mode;
	mode_switch_cmd = LGE_PANEL_NOT_SUPPORTED_SWITCH;
	switch(req_blank_mode) {
	case FB_BLANK_UNBLANK:
		//if(is_fake_u3())
		//	mfd->fakeu3 = 1;
		//else
			mfd->fakeu3 = 0;
		if(cur_panel_mode == LGE_PANEL_MODE_U0){
			mode_switch_cmd = LGE_PANEL_CMD_NONE;
			pr_info("[PowerMode] %s: switch u0 to u3 \n", __func__);
		} else if(cur_panel_mode == LGE_PANEL_MODE_U1){
			mode_switch_cmd = LGE_PANEL_CMD_U1_TO_U3;
			pr_info("[PowerMode] %s: switch u1 to u3 \n", __func__);
		} else if(cur_panel_mode == LGE_PANEL_MODE_U2){
				if(mfd->fakeu3){
					mode_switch_cmd = LGE_PANEL_CMD_PROXIMITY_U2_TO_U3;
					pr_info("[PowerMode] %s: switch u2 to fake u3 \n", __func__);
				}else{
					mode_switch_cmd = LGE_PANEL_CMD_U2_TO_U3;
					pr_info("[PowerMode] %s: switch u2 to u3 \n", __func__);
				}
		}
		break;
	case FB_BLANK_U2:
		if(cur_panel_mode == LGE_PANEL_MODE_U1){
			mode_switch_cmd = LGE_PANEL_CMD_U1_TO_U2;
			pr_info("[PowerMode] %s: switch u1 to u2 \n", __func__);
		}
		else if(cur_panel_mode == LGE_PANEL_MODE_U3){
				if(mfd->fakeu3){
					mode_switch_cmd = LGE_PANEL_CMD_PROXIMITY_U3_TO_U2;
					pr_info("[PowerMode] %s: switch fake u3 to u2 \n", __func__);
				}else{
					mode_switch_cmd = LGE_PANEL_CMD_U3_TO_U2;
					pr_info("[PowerMode] %s: switch u3 to u2 \n", __func__);
				}
		}
		break;
	case FB_BLANK_U1:
		if(cur_panel_mode == LGE_PANEL_MODE_U2){
			mode_switch_cmd = LGE_PANEL_CMD_U2_TO_U1;
			pr_info("[PowerMode] %s: switch u2 to u1 \n", __func__);
		}
		else if(cur_panel_mode == LGE_PANEL_MODE_U3){
			mode_switch_cmd = LGE_PANEL_CMD_U3_TO_U1;
			pr_info("[PowerMode] %s: switch u3 to u1 \n", __func__);
		}
		break;
	case FB_BLANK_POWERDOWN:
		if(cur_panel_mode == LGE_PANEL_MODE_U3){
			mode_switch_cmd = LGE_PANEL_CMD_NONE;
			pr_info("[PowerMode] %s: switch u3 to u0 \n", __func__);
		}
		else if(cur_panel_mode == LGE_PANEL_MODE_U1){
			mode_switch_cmd = LGE_PANEL_CMD_NONE;
			pr_info("[PowerMode] %s: switch u1 to u0 \n", __func__);
		}
		break;
	}

	if (LGE_PANEL_NOT_SUPPORTED_SWITCH == mode_switch_cmd){
		pr_err("Not supported mode switch! u%d -> req blank:%d\n", cur_panel_mode, req_blank_mode);
		if(cur_panel_mode == 2 && req_blank_mode == 4)
		{
			mutex_lock(&mfd->bl_lock);
			mdss_fb_set_backlight(mfd, 0);
			mfd->bl_updated = 0;
			mdss_fb_set_backlight_ex(mfd, 0);
			mfd->bl_updated_ex = 0;
			mutex_unlock(&mfd->bl_lock);
		}
	}

	return mode_switch_cmd;
}

int lge_lg4945_check_skip_onoff_cmd(struct msm_fb_data_type *mfd, int req_blank_mode){
	int cur_panel_mode;
	struct mdss_panel_info *panel_info = mfd->panel_info;

	cur_panel_mode = panel_info->lge_pan_info.cur_panel_mode;
	switch(req_blank_mode) {
	case FB_BLANK_UNBLANK: // U3 unblank
		switch (cur_panel_mode) {
		case LGE_PANEL_MODE_U0: // fall through
			panel_info->lge_pan_info.lge_panel_send_on_cmd = true;
			break;
		case LGE_PANEL_MODE_U1:
		case LGE_PANEL_MODE_U2:
			panel_info->lge_pan_info.lge_panel_send_on_cmd = true;
			break;
		}
		break;
	case FB_BLANK_VSYNC_SUSPEND: // U2 blank
		switch (cur_panel_mode) {
		case LGE_PANEL_MODE_U0:
			pr_err("error, can't called.");
			break;
		case LGE_PANEL_MODE_U1: // fall through
		case LGE_PANEL_MODE_U3:
			panel_info->lge_pan_info.lge_panel_send_off_cmd = false;
			break;
		}
	case FB_BLANK_HSYNC_SUSPEND: // U1 unblank
		switch (cur_panel_mode) {
		case LGE_PANEL_MODE_U0:
			panel_info->lge_pan_info.lge_panel_send_on_cmd = true;
			break;
		case LGE_PANEL_MODE_U2: // fall through
		case LGE_PANEL_MODE_U3:
			break;
		}
		break;
	case FB_BLANK_POWERDOWN: // U0 blank
		switch (cur_panel_mode) {
		case LGE_PANEL_MODE_U1: // fall through
		case LGE_PANEL_MODE_U2:
			break;
		case LGE_PANEL_MODE_U3:
			panel_info->lge_pan_info.lge_panel_send_off_cmd = true;
			break;
		}
		break;
	}

	return 0;
}
#endif

int lgd_lg4895_hd_video_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;
    pr_err("#### %s() start !!\n", __func__);
    if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
        rc = gpio_request(ctrl_pdata->mode_gpio, "panel_mode");
        if (rc) {
            pr_err("request panel mode gpio failed,rc=%d\n",
                    rc);
            goto mode_gpio_err;
        }
    }
    return rc;

mode_gpio_err:
	return rc;
}

int lgd_qhd_command_mdss_dsi_request_gpios(struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	int rc = 0;
	if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
		rc = gpio_request(ctrl_pdata->bklt_en_gpio,
						"bklt_enable");
		if (rc) {
			pr_err("request bklt gpio failed, rc=%d\n",
				       rc);
			goto bklt_en_gpio_err;
		}
	}
	if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
		rc = gpio_request(ctrl_pdata->mode_gpio, "panel_mode");
		if (rc) {
			pr_err("request panel mode gpio failed,rc=%d\n",
								rc);
			goto mode_gpio_err;
		}
	}
	return rc;

mode_gpio_err:
	if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
		gpio_free(ctrl_pdata->bklt_en_gpio);
bklt_en_gpio_err:
	return rc;
}

int lgd_lg4895_hd_video_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int rc = 0;
    int i = 0;

    pr_err("#### %s() start !!\n", __func__);
	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, disp_en_gpio was not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line was not configured\n",
			   __func__, __LINE__);
		return rc;
	}
	pr_info("%s: enable = %d, ndx = %d\n", __func__,
			enable, ctrl_pdata->ndx);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
		rc = lge_mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}

		if (!pinfo->cont_splash_enabled) {
			/* lm3632_dsv_ctrl */
			lm3632_dsv_fd_ctrl(1);

			if (ctrl_pdata->lge_pan_data->touch_driver_registered) {
				rc = touch_notifier_call_chain(LCD_EVENT_HW_RESET, NULL);
				pr_info("[LCD] notify to touch_driver LCD_EVENT_HW_RESET, rc=%d\n", rc);
			}
			pr_info("[LCD] start reset control when LCD on\n");
			for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
				if (gpio_is_valid(ctrl_pdata->rst_gpio)) {
					gpio_set_value((ctrl_pdata->rst_gpio),
						pdata->panel_info.rst_seq[i]);
					PANEL_SEQUENCE(PIN_RESET, pdata->panel_info.rst_seq[i]);
					if (pdata->panel_info.rst_seq[++i])
						usleep(pinfo->rst_seq[i] * 1000);
				}
			}
			pr_info("[LCD] End reset control when LCD on\n");
		}
		if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
			if (pinfo->mode_gpio_state == MODE_GPIO_HIGH)
				gpio_set_value((ctrl_pdata->mode_gpio), 1);
			else if (pinfo->mode_gpio_state == MODE_GPIO_LOW)
				gpio_set_value((ctrl_pdata->mode_gpio), 0);
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		lm3632_dsv_fd_ctrl(0);
		if (gpio_is_valid(ctrl_pdata->mode_gpio))
			gpio_free(ctrl_pdata->mode_gpio);
	}
	return rc;
}

int lgd_sic_qhd_command_mdss_dsi_panel_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	struct mdss_panel_info *pinfo = NULL;
	int rc = 0;
    int i = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
				panel_data);

	if (!gpio_is_valid(ctrl_pdata->disp_en_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
	}

	if (!gpio_is_valid(ctrl_pdata->rst_gpio)) {
		pr_debug("%s:%d, reset line not configured\n",
			   __func__, __LINE__);
		return rc;
	}
	pr_info("%s: enable = %d, ndx = %d\n", __func__,
			enable, ctrl_pdata->ndx);
	pinfo = &(ctrl_pdata->panel_data.panel_info);

	if (enable) {
		rc = lgd_qhd_command_mdss_dsi_request_gpios(ctrl_pdata);
		if (rc) {
			pr_err("gpio request failed\n");
			return rc;
		}
		if (!pinfo->cont_splash_enabled) {
            // Temporarily do nothing..
/*
			if (ts_data != NULL) {
				if (syna_ts_data != NULL) {
					if (atomic_read(&syna_ts_data->state->upgrade_state) != UPGRADE_START) {
						if (syna_ts_data->lpwg_ctrl.has_debug_module) {
							print_tci_debug_result(syna_ts_data, 0);
							print_tci_debug_result(syna_ts_data, 1);
							pr_info("[Touch] TCI Fail Reason Report when LCD on\n");
						}
					}
				}
				mutex_lock(&ts_data->pdata->thread_lock);
			}

			if (wakeup_by_swipe == false) {
				pr_info("[LCD] Start LCD power down when LCD on\n");
				if (gpio_is_valid(ctrl_pdata->rst_gpio)) {
				    gpio_set_value((ctrl_pdata->rst_gpio), 0);
					PANEL_SEQUENCE(DISPLAY_PIN_RESET, 0);
				}
				msm_dss_enable_vreg(
				    ctrl_pdata->power_data[DSI_PANEL_PM].vreg_config,
				    ctrl_pdata->power_data[DSI_PANEL_PM].num_vreg, 0);
				if (gpio_is_valid(ctrl_pdata->disp_lcd_ldo_3v0_gpio)) {
				    gpio_set_value((ctrl_pdata->disp_lcd_ldo_3v0_gpio), 0);
					PANEL_SEQUENCE(PIN_VCI, 0);
				}
				if (gpio_is_valid(ctrl_pdata->disp_lcd_ldo_1v8_gpio)) {
				    gpio_set_value((ctrl_pdata->disp_lcd_ldo_1v8_gpio), 0);
					PANEL_SEQUENCE(PIN_VIO, 0);
				}
				if (gpio_is_valid(ctrl_pdata->disp_dsv_en_gpio)) {
				    gpio_set_value((ctrl_pdata->disp_dsv_en_gpio), 0);
					PANEL_SEQUENCE(PIN_DDVDH, 0);
				}
				lm3632_dsv_output_ctrl(0);
				//lm3632_dsv_fd_ctrl();
				pr_info("[LCD] End LCD power down when LCD on\n");
				usleep(12 * 1000);
				pr_info("[LCD] Start LCD power up when LCD on\n");
				msm_dss_enable_vreg(
					ctrl_pdata->power_data[DSI_PANEL_PM].vreg_config,
					ctrl_pdata->power_data[DSI_PANEL_PM].num_vreg, 1);
				if (gpio_is_valid(ctrl_pdata->disp_lcd_ldo_3v0_gpio)) {
					gpio_set_value((ctrl_pdata->disp_lcd_ldo_3v0_gpio), 1);
					PANEL_SEQUENCE(PIN_VCI, 1);
				}
				if (gpio_is_valid(ctrl_pdata->disp_lcd_ldo_1v8_gpio)) {
					gpio_set_value((ctrl_pdata->disp_lcd_ldo_1v8_gpio), 1);
					PANEL_SEQUENCE(DISPLAY_PIN_VIO, 1);
				}
				usleep(10 * 1000);
				if (gpio_is_valid(ctrl_pdata->disp_dsv_en_gpio)) {
					gpio_set_value((ctrl_pdata->disp_dsv_en_gpio), 1);
					PANEL_SEQUENCE(DISPLAY_PIN_DDVDH, 1);
				}
				lm3632_dsv_output_ctrl(1);
				usleep(6 * 1000);
				pr_info("[LCD] End LCD power up when LCD on\n");
			} else {
				pr_info("[LCD] Skip power control in case of wakeup_by_swipe \n");
			}
*/
			pr_info("[LCD] Start reset pin high when LCD on\n");
			for (i = 0; i < pdata->panel_info.rst_seq_len; ++i) {
				gpio_set_value((ctrl_pdata->rst_gpio),
					pdata->panel_info.rst_seq[i]);
				PANEL_SEQUENCE(PIN_RESET, pdata->panel_info.rst_seq[i]);
				if (pdata->panel_info.rst_seq[++i])
					usleep(pinfo->rst_seq[i] * 1000);
			}
			pr_info("[LCD] End reset control when LCD on\n");

			if (gpio_is_valid(ctrl_pdata->bklt_en_gpio))
				gpio_set_value((ctrl_pdata->bklt_en_gpio), 1);
		}

		if (gpio_is_valid(ctrl_pdata->mode_gpio)) {
			if (pinfo->mode_gpio_state == MODE_GPIO_HIGH)
				gpio_set_value((ctrl_pdata->mode_gpio), 1);
			else if (pinfo->mode_gpio_state == MODE_GPIO_LOW)
				gpio_set_value((ctrl_pdata->mode_gpio), 0);
		}
		if (ctrl_pdata->ctrl_state & CTRL_STATE_PANEL_INIT) {
			pr_debug("%s: Panel Not properly turned OFF\n",
						__func__);
			ctrl_pdata->ctrl_state &= ~CTRL_STATE_PANEL_INIT;
			pr_debug("%s: Reset panel done\n", __func__);
		}
	} else {
		if (gpio_is_valid(ctrl_pdata->bklt_en_gpio)) {
			gpio_set_value((ctrl_pdata->bklt_en_gpio), 0);
			gpio_free(ctrl_pdata->bklt_en_gpio);
		}
		if (gpio_is_valid(ctrl_pdata->mode_gpio))
			gpio_free(ctrl_pdata->mode_gpio);
	}
	return rc;
}

int lgd_lg4895_hd_video_mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata, u32 bl_level) {
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return 0;
	}
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_AOD_SUPPORT)
	ctrl_pdata->bklt_ctrl = lge_get_bklt_type();
	pr_info("%s: bklt_ctrl=%d\n", __func__, ctrl_pdata->bklt_ctrl);
#endif
	switch (ctrl_pdata->bklt_ctrl) {
	case BL_WLED:
		lm3632_lcd_backlight_set_level(bl_level);
		break;
	case BL_PWM:
#if IS_ENABLED(CONFIG_LGE_DISPLAY_AOD_SUPPORT)
#if IS_ENABLED(CONFIG_BACKLIGHT_LM3697)
		lm3697_lcd_backlight_set_level(bl_level);
#endif
#else
		mdss_dsi_panel_bklt_pwm(ctrl_pdata, bl_level);
#endif
		break;
	default:
		pr_err("%s: Unknown bl_ctrl configuration\n",
				__func__);
		break;
	}


	return 0;
}

int lgd_qhd_command_mdss_dsi_panel_bl_ctrl(struct mdss_panel_data *pdata, u32 bl_level)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return 0;
	}
	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);

#if IS_ENABLED(CONFIG_LGE_DISPLAY_AOD_SUPPORT)
	ctrl_pdata->bklt_ctrl = lge_get_bklt_type();
	pr_info("%s: bklt_ctrl=%d\n", __func__, ctrl_pdata->bklt_ctrl);
#endif
	switch (ctrl_pdata->bklt_ctrl) {
	case BL_WLED:
		lm3632_lcd_backlight_set_level(bl_level);
		break;
	case BL_PWM:
#if IS_ENABLED(CONFIG_LGE_DISPLAY_AOD_SUPPORT)
#if IS_ENABLED(CONFIG_BACKLIGHT_LM3697)
		lm3697_lcd_backlight_set_level(bl_level);
#endif
#else
		mdss_dsi_panel_bklt_pwm(ctrl_pdata, bl_level);
#endif
		break;
	default:
		pr_err("%s: Unknown bl_ctrl configuration\n",
				__func__);
		break;
	}


	return 0;
}

int lgd_lg4895_command_post_mdss_dsi_panel_on(struct mdss_panel_data *pdata)
{
	struct mdss_dsi_ctrl_pdata *ctrl = NULL;

	ctrl = container_of(pdata, struct mdss_dsi_ctrl_pdata,
			panel_data);

#if defined(CONFIG_LGE_P1_SRE_SUPPORTED)
	if (pdata_sre == NULL)
		pdata_sre = pdata;
#endif

	if (wpos_x!=-1 && wpos_y!=-1) {
		lgd_lg4895_panel_watch_ctl_cmd_send(WATCH_POS_UPDATE, NULL, ctrl);
		lgd_lg4895_panel_watch_ctl_cmd_send(WATCH_LUT_UPDATE, NULL, ctrl);
	}

	/*
	if (swipe_status) {
		touch_notifier_call_chain(
				LCD_EVENT_TOUCH_LPWG_OFF, NULL);
	}
	*/
	return 0;
}

int lgd_qhd_command_post_mdss_dsi_panel_off(struct mdss_panel_data *pdata)
{
	int rc=0;
	return rc;
}

int lgd_qhd_command_mdss_dsi_panel_init(struct device_node *node, struct mdss_dsi_ctrl_pdata *ctrl_pdata, bool cmd_cfg_cont_splash)
{
	struct mdss_panel_info *pinfo;

//LGE_UPDATE_S (june1014.lee@lge.com. 2015.03.04). SRE
#if defined(CONFIG_LGE_P1_SRE_SUPPORTED)
	static struct class *sre = NULL;
	static struct device *sre_sysfs_dev = NULL;
#endif
//LGE_UPDATE_E (june1014.lee@lge.com. 2015.03.04). SRE

#if IS_ENABLED(CONFIG_LGE_DISPLAY_AOD_SUPPORT)
	static struct class *panel;
	static struct device *panel_sysfs_dev;
#endif

	pinfo = &ctrl_pdata->panel_data.panel_info;

//LGE_UPDATE_S (june1014.lee@lge.com. 2015.03.04). SRE
#if defined(CONFIG_LGE_P1_SRE_SUPPORTED)
		if(!sre) {
			sre = class_create(THIS_MODULE, "sre");
			if (IS_ERR(sre))
				pr_err("%s: Failed to create sre class\n", __func__);
		}

		if ( !sre_sysfs_dev ) {
			sre_sysfs_dev = device_create(sre, NULL, 0, NULL, "sre_func");
			if (IS_ERR(sre_sysfs_dev)) {
				pr_err("%s: Failed to create dev(sre_sysfs_dev)!",
						__func__);
			} else {
				if (device_create_file(sre_sysfs_dev,
						&sre_status_attrs[0]) < 0)
					pr_err("%s: Fail!", __func__);
			}
		}
#endif

#if defined(CONFIG_LGE_BLMAP_STORE_MODE)
		{
			static struct class *blmap_adjust = NULL;
			static struct device *blmode_sysfs_dev = NULL;

			if(!blmap_adjust) {
				blmap_adjust = class_create(THIS_MODULE, "blmap_adjust");
				if (IS_ERR(blmap_adjust))
					pr_err("%s: Failed to create blmap_adjust class\n", __func__);
			}

			if ( !blmode_sysfs_dev ) {
				blmode_sysfs_dev = device_create(blmap_adjust, NULL, 0, NULL, "bl_store_mode");
				if (IS_ERR(blmode_sysfs_dev)) {
					pr_err("%s: Failed to create dev(blmode_sysfs_dev)!",
							__func__);
				} else {
					if (device_create_file(blmode_sysfs_dev,
								&blmap_adjust_status_attrs[0]) < 0)
						pr_err("%s: Fail!", __func__);
				}
			}
		}
#endif

//LGE_UPDATE_E (june1014.lee@lge.com. 2015.03.04). SRE
#if IS_ENABLED(CONFIG_LGE_DISPLAY_AOD_SUPPORT)
	pinfo->lge_pan_info.lge_panel_send_on_cmd = true;
	pinfo->lge_pan_info.lge_panel_send_off_cmd = true;

	if (!panel) {
		panel = class_create(THIS_MODULE, "panel");
		if (IS_ERR(panel))
			pr_err("%s: Failed to create panel class\n", __func__);
	}

	if (!panel_sysfs_dev) {
		panel_sysfs_dev = device_create(panel, NULL, 0, NULL, "img_tune");
		if (IS_ERR(panel_sysfs_dev)) {
			pr_err("%s: Failed to create dev(panel_sysfs_dev)!", __func__);
		} else {
			if (device_create_file(panel_sysfs_dev, &panel_tuning_device_attrs[0]) < 0)
				pr_err("%s: #1 add panel tuning node fail!", __func__);
			if (device_create_file(panel_sysfs_dev, &panel_tuning_device_attrs[1]) < 0)
				pr_err("%s: #2 add panel tuning node fail!!", __func__);
		}
	}
#endif
#if defined(CONFIG_LGE_BLMAP_STORE_MODE)
	if (pinfo_store_mode == NULL)
		pinfo_store_mode = pinfo;
#endif

	return 0;
}

int lgd_lg4895_hd_video_mdss_panel_parse_dt(struct device_node *np,	struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	u32 tmp;
	int rc, i;
	u32 *array;
    const char *data;

	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

    ctrl_pdata->bklt_ctrl = UNKNOWN_CTRL;
    data = of_get_property(np, "qcom,mdss-dsi-bl-pmic-control-type", NULL);
    if (data) {
        if (!strncmp(data, "bl_ctrl_wled", 12)) {
            led_trigger_register_simple("bkl-trigger",
                    &bl_led_trigger);
            pr_debug("%s: SUCCESS-> WLED TRIGGER register\n",
                    __func__);
            ctrl_pdata->bklt_ctrl = BL_WLED;
        } else if (!strncmp(data, "bl_ctrl_pwm", 11)) {
            ctrl_pdata->bklt_ctrl = BL_PWM;
            ctrl_pdata->pwm_pmi = of_property_read_bool(np,
                    "qcom,mdss-dsi-bl-pwm-pmi");
            rc = of_property_read_u32(np,
                    "qcom,mdss-dsi-bl-pmic-pwm-frequency", &tmp);
            if (rc) {
                pr_err("%s:%d, Error, panel pwm_period\n",
                        __func__, __LINE__);
                return -EINVAL;
            }
            ctrl_pdata->pwm_period = tmp;
            if (ctrl_pdata->pwm_pmi) {
                ctrl_pdata->pwm_bl = of_pwm_get(np, NULL);
                if (IS_ERR(ctrl_pdata->pwm_bl)) {
                    pr_err("%s: Error, pwm device\n",
                            __func__);
                    ctrl_pdata->pwm_bl = NULL;
                    return -EINVAL;
                }
            } else {
                rc = of_property_read_u32(np,
                        "qcom,mdss-dsi-bl-pmic-bank-select",
                        &tmp);
                if (rc) {
                    pr_err("%s:%d, Error, lpg channel\n",
                            __func__, __LINE__);
                    return -EINVAL;
                }
                ctrl_pdata->pwm_lpg_chan = tmp;
                tmp = of_get_named_gpio(np,
                        "qcom,mdss-dsi-pwm-gpio", 0);
                ctrl_pdata->pwm_pmic_gpio = tmp;
                pr_debug("%s: Configured PWM bklt ctrl\n",
                        __func__);
            }
        } else if (!strncmp(data, "bl_ctrl_dcs", 11)) {
            ctrl_pdata->bklt_ctrl = BL_DCS_CMD;
            pr_debug("%s: Configured DCS_CMD bklt ctrl\n",
                    __func__);
        }
    }


	rc = of_property_read_u32(np, "qcom,blmap-size", &tmp);
	pinfo->lge_pan_info.blmap_size = (!rc ? tmp : 0);

	if (pinfo->lge_pan_info.blmap_size) {
		array = kzalloc(sizeof(u32) * pinfo->lge_pan_info.blmap_size, GFP_KERNEL);

		if (!array)
			return -ENOMEM;
		rc = of_property_read_u32_array(np,
			"qcom,blmap", array, pinfo->lge_pan_info.blmap_size);

		if (rc) {
			pr_err("%s:%d, unable to read backlight map\n",
					__func__, __LINE__);
			kfree(array);
			return rc;
		}

		pinfo->lge_pan_info.blmap = kzalloc(sizeof(int) * pinfo->lge_pan_info.blmap_size,
					GFP_KERNEL);
		if (!pinfo->lge_pan_info.blmap)
		{
			kfree(array);
			return -ENOMEM;
		}

		for (i = 0; i < pinfo->lge_pan_info.blmap_size; i++)
			pinfo->lge_pan_info.blmap[i] = array[i];

		kfree(array);
	} else {
		pinfo->lge_pan_info.blmap = NULL;
	}
#if defined(CONFIG_LGE_BLMAP_STORE_MODE)
	if (pinfo->lge_pan_info.blmap_size) {
		array = kzalloc(sizeof(u32) * pinfo->lge_pan_info.blmap_size, GFP_KERNEL);

		if (!array)
			return -ENOMEM;
		rc = of_property_read_u32_array(np,
			"qcom,blmap_store_mode", array, pinfo->lge_pan_info.blmap_size);

		if (rc) {
			pr_err("%s:%d, unable to read backlight map for store_mode\n",
					__func__, __LINE__);
			kfree(array);
			return rc;
		}

		pinfo->lge_pan_info.blmap_store_mode = kzalloc(sizeof(int) * pinfo->lge_pan_info.blmap_size,
					GFP_KERNEL);
		if (!pinfo->lge_pan_info.blmap_store_mode)
			return -ENOMEM;

		for (i = 0; i < pinfo->lge_pan_info.blmap_size; i++)
			pinfo->lge_pan_info.blmap_store_mode[i] = array[i];
		pinfo->lge_pan_info.bl_store_mode = 0;

		kfree(array);
	} else {
		pinfo->lge_pan_info.blmap_store_mode = NULL;
	}
#endif
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lge_pan_data->clk_on_cmds,
		"lge,mdss-dsi-clk_on-command", "lge,mdss-dsi-clk_on-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lge_pan_data->clk_off_cmds,
		"lge,mdss-dsi-clk_off-command", "lge,mdss-dsi-clk_off-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lge_pan_data->rsp_nvm_write,
		"lgd,rsp-write-nvm", "qcom,mdss-dsi-on-command-state");

#if IS_ENABLED(CONFIG_LGE_DISPLAY_AOD_SUPPORT)
	mode_cmds_set = kzalloc(sizeof(struct mode_cmds_desc), GFP_KERNEL);
	for (i = 0; i < LGE_PANEL_CMD_CHANGE_NUM; i++) {
		mdss_dsi_parse_dcs_cmds(np, &mode_cmds_set->mode_cmds[i],
			mode_lg4945_dt[i], "qcom,mode-control-dsi-state");
		pr_info("%s: parse mode_change[%d] cmd = 0x%02x\n", __func__, 
				i, ((struct mode_cmds_desc*)mode_cmds_set)->mode_cmds[i].cmd_cnt);
	}

	img_tune_cmds_set = kzalloc(sizeof(struct img_tune_cmds_desc), GFP_KERNEL);
/*
	for (i = 0; i < LGE_PANEL_IMG_TUNE_NUM; i++) {
		mdss_dsi_parse_dcs_cmds(np, &img_tune_cmds_set->img_tune_cmds[i],
			img_tune_lg4945_dt[i], "qcom,img-tune-control-dsi-state");
		pr_info("%s: parse img tune %d cmd = 0x%02X \n", __func__, i, img_tune_cmds_set->img_tune_cmds[i].cmds->payload[0]);
	}
*/
	if (pdata_mode == NULL)
		pdata_mode = &(ctrl_pdata->panel_data);
#endif

    return rc;
}

int lgd_qhd_command_mdss_panel_parse_dt(struct device_node *np,	struct mdss_dsi_ctrl_pdata *ctrl_pdata)
{
	u32 tmp;
	int rc, i;
	u32 *array;

	struct mdss_panel_info *pinfo = &(ctrl_pdata->panel_data.panel_info);

	rc = of_property_read_u32(np, "qcom,blmap-size", &tmp);
	pinfo->lge_pan_info.blmap_size = (!rc ? tmp : 0);

	if (pinfo->lge_pan_info.blmap_size) {
		array = kzalloc(sizeof(u32) * pinfo->lge_pan_info.blmap_size, GFP_KERNEL);

		if (!array)
			return -ENOMEM;
		rc = of_property_read_u32_array(np,
			"qcom,blmap", array, pinfo->lge_pan_info.blmap_size);

		if (rc) {
			pr_err("%s:%d, unable to read backlight map\n",
					__func__, __LINE__);
			kfree(array);
			return rc;
		}

		pinfo->lge_pan_info.blmap = kzalloc(sizeof(int) * pinfo->lge_pan_info.blmap_size,
					GFP_KERNEL);
		if (!pinfo->lge_pan_info.blmap)
		{
			kfree(array);
			return -ENOMEM;
		}

		for (i = 0; i < pinfo->lge_pan_info.blmap_size; i++)
			pinfo->lge_pan_info.blmap[i] = array[i];

		kfree(array);
	} else {
		pinfo->lge_pan_info.blmap = NULL;
	}
#if defined(CONFIG_LGE_BLMAP_STORE_MODE)
	if (pinfo->lge_pan_info.blmap_size) {
		array = kzalloc(sizeof(u32) * pinfo->lge_pan_info.blmap_size, GFP_KERNEL);

		if (!array)
			return -ENOMEM;
		rc = of_property_read_u32_array(np,
			"qcom,blmap_store_mode", array, pinfo->lge_pan_info.blmap_size);

		if (rc) {
			pr_err("%s:%d, unable to read backlight map for store_mode\n",
					__func__, __LINE__);
			kfree(array);
			return rc;
		}

		pinfo->lge_pan_info.blmap_store_mode = kzalloc(sizeof(int) * pinfo->lge_pan_info.blmap_size,
					GFP_KERNEL);
		if (!pinfo->lge_pan_info.blmap_store_mode)
			return -ENOMEM;

		for (i = 0; i < pinfo->lge_pan_info.blmap_size; i++)
			pinfo->lge_pan_info.blmap_store_mode[i] = array[i];
		pinfo->lge_pan_info.bl_store_mode = 0;

		kfree(array);
	} else {
		pinfo->lge_pan_info.blmap_store_mode = NULL;
	}
#endif
	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lge_pan_data->clk_on_cmds,
		"lge,mdss-dsi-clk_on-command", "lge,mdss-dsi-clk_on-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lge_pan_data->clk_off_cmds,
		"lge,mdss-dsi-clk_off-command", "lge,mdss-dsi-clk_off-command-state");

	mdss_dsi_parse_dcs_cmds(np, &ctrl_pdata->lge_pan_data->rsp_nvm_write,
		"lgd,rsp-write-nvm", "qcom,mdss-dsi-on-command-state");
//LGE_UPDATE_S (june1014.lee@lge.com. 2015.03.04). SRE
#if defined(CONFIG_LGE_P1_SRE_SUPPORTED)
	sre_cmds_set = kzalloc(sizeof(struct sre_cmds_desc), GFP_KERNEL);
	switch (pinfo->lge_pan_info.panel_type) {
		case LGD_INCELL_CMD_PANEL:	// LGD panel
			for (i = 0; i < 2; i++) {
				mdss_dsi_parse_dcs_cmds(np, &sre_cmds_set->sre_cmds[i],
						sre_lgd_dt[i], "qcom,sre-control-dsi-state");
			}
			pr_debug("%s: std_cmd=0x%02X 0x%02X\n", __func__,
				sre_cmds_set->sre_cmds[0].cmds->payload[0],
				sre_cmds_set->sre_cmds[0].cmds->payload[1]);
			break;
		case JDI_INCELL_CMD_PANEL:	// JDI panel
			for (i = 0; i < 2; i++) {
				mdss_dsi_parse_dcs_cmds(np, &sre_cmds_set->sre_cmds[i],
						sre_jdi_dt[i], "qcom,sre-control-dsi-state");
			}
			pr_debug("%s: std_cmd=0x%02X 0x%02X\n", __func__,
				sre_cmds_set->sre_cmds[0].cmds->payload[0],
				sre_cmds_set->sre_cmds[0].cmds->payload[1]);
			break;
		case LGD_SIC_INCELL_CMD_PANEL:	// LGD-SIC panel
		case LGD_LG4945_INCELL_CMD_PANEL:
			for (i = 0; i < 2; i++) {
				mdss_dsi_parse_dcs_cmds(np, &sre_cmds_set->sre_cmds[i],
						sre_lgdsic_dt[i], "qcom,sre-control-dsi-state");
			}
			pr_debug("%s: std_cmd=0x%02X 0x%02X\n", __func__,
				sre_cmds_set->sre_cmds[0].cmds->payload[0],
				sre_cmds_set->sre_cmds[0].cmds->payload[1]);
			break;
		default:
			pr_err("%s: not chosen panel type in SRE\n", __func__);
			break;
	}

	if (pdata_sre == NULL)
		pdata_sre = &(ctrl_pdata->panel_data);
#endif
//LGE_UPDATE_E (june1014.lee@lge.com. 2015.03.04). SRE

#if IS_ENABLED(CONFIG_LGE_DISPLAY_AOD_SUPPORT)
	mode_cmds_set = kzalloc(sizeof(struct mode_cmds_desc), GFP_KERNEL);
	for (i = 0; i < LGE_PANEL_CMD_CHANGE_NUM; i++) {
		mdss_dsi_parse_dcs_cmds(np, &mode_cmds_set->mode_cmds[i],
			mode_lg4945_dt[i], "qcom,mode-control-dsi-state");
	}
	img_tune_cmds_set = kzalloc(sizeof(struct img_tune_cmds_desc), GFP_KERNEL);
	for (i = 0; i < LGE_PANEL_IMG_TUNE_NUM; i++) {
		mdss_dsi_parse_dcs_cmds(np, &img_tune_cmds_set->img_tune_cmds[i],
			img_tune_lg4945_dt[i], "qcom,img-tune-control-dsi-state");
		pr_info("%s: parse img tune %d cmd = 0x%02X \n", __func__, i, img_tune_cmds_set->img_tune_cmds[i].cmds->payload[0]);
	}
	if (pdata_mode == NULL)
		pdata_mode = &(ctrl_pdata->panel_data);
#endif

    	return rc;
}

void mdss_dsi_stub_cmds_send(struct mdss_dsi_ctrl_pdata *ctrl,
			struct dsi_panel_cmds *pcmds)
{
		mdss_dsi_panel_cmds_send(ctrl, pcmds);
}

int lgd_deep_sleep(struct mdss_dsi_ctrl_pdata *ctrl_pdata, int mode,
	       int is_no_sleep)
{
	int rc = 0;
	pr_info("%s: mode : %d\n", __func__, mode);
	if (!mode) { /* NEAR :  LPWG -> DEEP SLEEP */
		if (gpio_get_value(ctrl_pdata->lge_pan_data->dsv_ena)) {
			pr_info("%s: LPWG to DEEP SLEEP\n", __func__);
			if (gpio_is_valid(ctrl_pdata->rst_gpio)) {
				gpio_set_value((ctrl_pdata->rst_gpio), 0);
				PANEL_SEQUENCE(PIN_RESET, 0);
				gpio_free(ctrl_pdata->rst_gpio);
			}

//			dw8768_lgd_fd_mode_change(1); /* fd_on */

			if (gpio_is_valid(ctrl_pdata->lge_pan_data->dsv_ena)) {
				gpio_set_value((ctrl_pdata->lge_pan_data->dsv_ena), 0);
				PANEL_SEQUENCE(PIN_DSV_ENA, 0);
				gpio_free(ctrl_pdata->lge_pan_data->dsv_ena);
			}
			if (ctrl_pdata->lge_pan_data->touch_driver_registered)
				touch_notifier_call_chain(
						LCD_EVENT_TOUCH_SLEEP_STATUS,
						(void *)&mode);
		}
	} else { /* FAR : DEEP SLEEP -> LPWG */
		if (!gpio_get_value(ctrl_pdata->lge_pan_data->dsv_ena)) {
			if (gpio_is_valid(ctrl_pdata->lge_pan_data->dsv_ena)) {
				rc = gpio_request(ctrl_pdata->lge_pan_data->dsv_ena,
						"dsv_ena");
				if (rc) {
					pr_err("%s:request dsv_ena gpio failed, rc=%d\n",
							__func__, rc);
					goto dsv_ena_err;
				}
				gpio_set_value((ctrl_pdata->lge_pan_data->dsv_ena), 1);
				PANEL_SEQUENCE(PIN_DSV_ENA, 1);
				if (gpio_is_valid(ctrl_pdata->rst_gpio)) {
					rc = gpio_request(ctrl_pdata->rst_gpio,
							"disp_enable");
					if (rc) {
						pr_err("%s:request rst_gpio gpio failed, rc=%d\n",
								__func__, rc);
						goto disp_reset_err;
					}
					gpio_set_value((ctrl_pdata->rst_gpio),
							1);
					PANEL_SEQUENCE(PIN_RESET, 1);
				}

//				dw8768_lgd_fd_mode_change(0); /* fd_off */
				if (ctrl_pdata->lge_pan_data->touch_driver_registered) {
					if (is_no_sleep == DEEP_SLEEP_TO_LPWG) {
						pr_info("%s: DEEP SLEEP to LPWG\n",
						       __func__);

						touch_notifier_call_chain(
						LCD_EVENT_TOUCH_SLEEP_STATUS,
								(void *)&mode);
					} else if (is_no_sleep
						== DEEP_SLEEP_TO_ACTIVE) {
						pr_info("%s: DEEP SLEEP to ACTIVE\n",
						       __func__);
						touch_notifier_call_chain(
						LCD_EVENT_TOUCH_SLEEP_STATUS,
							(void *)&is_no_sleep);
					}
				}
			}
		}
	}

disp_reset_err:
dsv_ena_err:
	return 0;

}

int mdss_dsi_lcd_reset(struct mdss_panel_data *pdata, int enable)
{
	struct mdss_dsi_ctrl_pdata *ctrl_pdata = NULL;
	int rc = 0;

	if (pdata == NULL) {
		pr_err("%s: Invalid input data\n", __func__);
		return -EINVAL;
	}

	pr_info("%s: Reset = %d\n", __func__, enable);

	ctrl_pdata = container_of(pdata, struct mdss_dsi_ctrl_pdata,
					panel_data);

	if (enable) {
		if (gpio_is_valid(ctrl_pdata->rst_gpio)) {
			rc = gpio_request(ctrl_pdata->rst_gpio, "disp_enable");
			if (rc) {
				pr_err("%s:request disp_en gpio failed, rc=%d\n",
					__func__, rc);
				goto disp_reset_err;
			}
			gpio_set_value((ctrl_pdata->rst_gpio), 1);
			PANEL_SEQUENCE(PIN_RESET, 1);
		}
	} else {
		if (gpio_is_valid(ctrl_pdata->rst_gpio)) {
			gpio_set_value((ctrl_pdata->rst_gpio), 0);
			PANEL_SEQUENCE(PIN_RESET, 0);
		}

		gpio_free(ctrl_pdata->rst_gpio);
	}
disp_reset_err:
	return rc;
}

#if IS_ENABLED(CONFIG_LGE_DISPLAY_AOD_SUPPORT)
int lge_get_bklt_type()
{
	return req_bklt_type;
}

int lge_set_bklt_type(int bklt_type)
{
	req_bklt_type = bklt_type;
	return 0;
}
#endif
