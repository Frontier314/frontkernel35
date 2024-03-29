/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/slab.h>

#if defined(CONFIG_EXYNOS_LPA)
extern int pixcir_i2c_ts_early_suspend(void);
extern int pixcir_i2c_ts_early_resume(void);
static bool cur_state = 1;
#endif
struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
	unsigned int		lth_brightness;
	int			(*notify)(struct device *,
					  int brightness);
	void			(*notify_after)(struct device *,
					int brightness);
	int			(*check_fb)(struct device *, struct fb_info *);
};

#ifdef CONFIG_MACH_MIXTILE4X12
#include <mach/regs-pmu.h>
static void dump_cpu_reg(void)
{
  printk("******************** dump_cpu_reg**********************\n");
  printk("EXYNOS4_ARM_CORE0_STATUS:    power %s\n", ((readl(EXYNOS4_ARM_CORE0_STATUS)&0x03) == 0x03)?"on":"off");
  printk("EXYNOS4_ARM_CORE1_STATUS:    power %s\n", ((readl(EXYNOS4_ARM_CORE1_STATUS)&0x03) == 0x03)?"on":"off");
  printk("EXYNOS4_ARM_CPU_L2_0_STATUS: power %s\n", ((readl(EXYNOS4_ARM_CPU_L2_0_STATUS)&0x03) == 0x03)?"on":"off");
  printk("EXYNOS4_ARM_CPU_L2_1_STATUS: power %s\n", ((readl(EXYNOS4_ARM_CPU_L2_1_STATUS)&0x03) == 0x03)?"on":"off");
  printk("EXYNOS4_CAM_STATUS:          power %s\n", ((readl(EXYNOS4_CAM_STATUS)&0x07) == 0x07)?"on":"off");
  printk("EXYNOS4_TV_STATUS:           power %s\n", ((readl(EXYNOS4_TV_STATUS)&0x07) == 0x07)?"on":"off");
  printk("EXYNOS4_MFC_STATUS:          power %s\n", ((readl(EXYNOS4_MFC_STATUS)&0x07) == 0x07)?"on":"off");
  printk("EXYNOS4_G3D_STATUS:          power %s\n", ((readl(EXYNOS4_G3D_STATUS)&0x07) == 0x07)?"on":"off");
  printk("EXYNOS4_LCD0_STATUS:         power %s\n", ((readl(EXYNOS4_LCD0_STATUS)&0x07) == 0x07)?"on":"off");
  printk("EXYNOS4_ISP_STATUS:         power %s\n", ((readl(EXYNOS4_ISP_STATUS)&0x07) == 0x07)?"on":"off");
  printk("EXYNOS4_MAUDIO_STATUS:         power %s\n", ((readl(EXYNOS4_MAUDIO_STATUS)&0x07) == 0x07)?"on":"off");
  printk("EXYNOS4_GPS_STATUS:          power %s\n", ((readl(EXYNOS4_GPS_STATUS)&0x07) == 0x07)?"on":"off");
  printk("EXYNOS4_GPS_ALIVE_STATUS:    power %s\n", ((readl(EXYNOS4_GPS_ALIVE_STATUS)&0x07) == 0x07)?"on":"off");
  printk("\n\n\n");
}
#endif

static int pwm_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (pb->notify)
		brightness = pb->notify(pb->dev, brightness);

	if (brightness == 0) {
		pwm_config(pb->pwm, 0, pb->period);
		pwm_disable(pb->pwm);
#if defined(CONFIG_EXYNOS_LPA)
		if (cur_state == 0)
		goto exit;
		pixcir_i2c_ts_early_suspend();
		cur_state = 0;
#endif
	} else {
		brightness = pb->lth_brightness +
			(brightness * (pb->period - pb->lth_brightness) / max);
		pwm_config(pb->pwm, brightness, pb->period);
		pwm_enable(pb->pwm);
#if defined(CONFIG_EXYNOS_LPA)
		if (cur_state == 1)
		goto exit;
		pixcir_i2c_ts_early_resume();
		cur_state = 1;
#endif
	}
#if defined(CONFIG_EXYNOS_LPA)
exit:
#endif
	if (pb->notify_after)
		pb->notify_after(pb->dev, brightness);

	return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static int pwm_backlight_check_fb(struct backlight_device *bl,
				  struct fb_info *info)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	return !pb->check_fb || pb->check_fb(pb->dev, info);
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_update_status,
	.get_brightness	= pwm_backlight_get_brightness,
	.check_fb	= pwm_backlight_check_fb,
};

static int pwm_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	int ret;

	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = devm_kzalloc(&pdev->dev, sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	pb->period = data->pwm_period_ns;
	pb->notify = data->notify;
	pb->notify_after = data->notify_after;
	pb->check_fb = data->check_fb;
	pb->lth_brightness = data->lth_brightness *
		(data->pwm_period_ns / data->max_brightness);
	pb->dev = &pdev->dev;

	pb->pwm = pwm_request(data->pwm_id, "backlight");
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM for backlight\n");
		ret = PTR_ERR(pb->pwm);
		goto err_alloc;
	} else
		dev_dbg(&pdev->dev, "got pwm for backlight\n");

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	return 0;

err_bl:
	pwm_free(pb->pwm);
err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	pwm_free(pb->pwm);
	if (data->exit)
		data->exit(&pdev->dev);
	return 0;
}

#ifdef CONFIG_PM
static int pwm_backlight_suspend(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	struct platform_pwm_backlight_data *data = dev->platform_data;

	if (pb->notify)
		pb->notify(pb->dev, 0);
	pwm_config(pb->pwm, 0, pb->period);
	if(data->exit)
		data->exit(dev);
	pwm_disable(pb->pwm);
	if (pb->notify_after)
		pb->notify_after(pb->dev, 0);
	return 0;
}

static int pwm_backlight_resume(struct device *dev)
{
	struct backlight_device *bl = dev_get_drvdata(dev);
	struct platform_pwm_backlight_data *data = dev->platform_data;
	int ret = 0;

	if(data->init){
		ret = data->init(dev);
	        if (ret < 0)
        	    return ret;
	}

	backlight_update_status(bl);
	return 0;
}

static SIMPLE_DEV_PM_OPS(pwm_backlight_pm_ops, pwm_backlight_suspend,
			 pwm_backlight_resume);

#endif

static struct platform_driver pwm_backlight_driver = {
	.driver		= {
		.name	= "pwm-backlight",
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &pwm_backlight_pm_ops,
#endif
	},
	.probe		= pwm_backlight_probe,
	.remove		= pwm_backlight_remove,
};

module_platform_driver(pwm_backlight_driver);

MODULE_DESCRIPTION("PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight");

