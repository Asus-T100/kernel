#include "rt5640.h"

#define RT5640_DET_EXT_MIC 1
#define USE_ASRC

struct rt5640_init_reg {
	u8 reg;
	u16 val;
};

static struct rt5640_init_reg init_list[] = {
#ifdef USE_ASRC
	{RT5640_GEN_CTRL1, 0x3f71},	/*fa[12:13] = 1'b; fa[8~11]=1; fa[0]=1 */
	{RT5640_JD_CTRL, 0x0003},
#else
	{RT5640_GEN_CTRL1, 0x3f01},	/*fa[12:13] = 1'b; fa[8~11]=1; fa[0]=1 */
#endif
	{RT5640_ADDA_CLK1, 0x0014},	/*73[2] = 1'b */
	{RT5640_MICBIAS, 0x3030},	/*93[5:4] = 11'b */
	{RT5640_CLS_D_OUT, 0xa000},	/*8d[11] = 0'b */
	{RT5640_CLS_D_OVCD, 0x0334},	/*8c[8] = 1'b */
	{RT5640_PRIV_INDEX, 0x001d},	/*PR1d[8] = 1'b; */
	{RT5640_PRIV_DATA, 0x0347},
	{RT5640_PRIV_INDEX, 0x003d},	/*PR3d[12] = 0'b; PR3d[9] = 1'b */
	{RT5640_PRIV_DATA, 0x2600},
	{RT5640_PRIV_INDEX, 0x0012},	/*PR12 = 0aa8'h */
	{RT5640_PRIV_DATA, 0x0aa8},
	{RT5640_PRIV_INDEX, 0x0014},	/*PR14 = 8aaa'h */
	{RT5640_PRIV_DATA, 0x8aaa},
	{RT5640_PRIV_INDEX, 0x0020},	/*PR20 = 6115'h */
	{RT5640_PRIV_DATA, 0x6115},
	{RT5640_PRIV_INDEX, 0x0023},	/*PR23 = 0804'h */
	{RT5640_PRIV_DATA, 0x0804},
	/*playback */
	{RT5640_STO_DAC_MIXER, 0x1404},	/*Dig inf 1 -> Sto DAC mixer -> DACL */
	{RT5640_OUT_L3_MIXER, 0x01fe},	/*DACL1 -> OUTMIXL */
	{RT5640_OUT_R3_MIXER, 0x01fe},	/*DACR1 -> OUTMIXR */
	{RT5640_HP_VOL, 0x8888},	/*OUTMIX -> HPVOL */
	{RT5640_HPO_MIXER, 0xc000},	/*HPVOL -> HPOLMIX */
/*	{RT5640_HPO_MIXER	, 0xa000},//DAC1 -> HPOLMIX*/
/*	{RT5640_CHARGE_PUMP	, 0x0f00},*/
	{RT5640_PRIV_INDEX, 0x0090},
	{RT5640_PRIV_DATA, 0x2000},
	{RT5640_PRIV_INDEX, 0x0091},
	{RT5640_PRIV_DATA, 0x1000},
/*	{RT5640_HP_CALIB_AMP_DET, 0x0420},*/
	{RT5640_SPK_L_MIXER, 0x0036},	/*DACL1 -> SPKMIXL */
	{RT5640_SPK_R_MIXER, 0x0036},	/*DACR1 -> SPKMIXR */
	{RT5640_SPK_VOL, 0x8b8b},	/*SPKMIX -> SPKVOL */
	{RT5640_SPO_CLSD_RATIO, 0x0001},
	{RT5640_SPO_L_MIXER, 0xe800},	/*SPKVOLL -> SPOLMIX */
	{RT5640_SPO_R_MIXER, 0x2800},	/*SPKVOLR -> SPORMIX */
/*	{RT5640_SPO_L_MIXER	, 0xb800},//DAC -> SPOLMIX*/
/*	{RT5640_SPO_R_MIXER	, 0x1800},//DAC -> SPORMIX*/
/*	{RT5640_I2S1_SDP	, 0xD000},//change IIS1 and IIS2*/
	/*record */
	{RT5640_IN1_IN2, 0x5080},	/*IN1 boost 40db and differential mode */
	{RT5640_IN3_IN4, 0x0000},	/*IN2 boost 40db and signal ended mode */
/*	{RT5640_REC_L2_MIXER	, 0x007d},//Mic1 -> RECMIXL*/
/*	{RT5640_REC_R2_MIXER	, 0x007d},//Mic1 -> RECMIXR*/
	{RT5640_REC_L2_MIXER, 0x006f},	/*Mic2 -> RECMIXL */
	{RT5640_REC_R2_MIXER, 0x006f},	/*Mic2 -> RECMIXR */
	{RT5640_STO_ADC_MIXER, 0x1000},	/*DMIC1 & AMIC */
	{RT5640_MONO_ADC_MIXER, 0x1010},
	{RT5640_ADC_DIG_VOL, 0xe2e2},
/*	{RT5640_MONO_MIXER, 0xcc00},*/	/*OUTMIX -> MONOMIX */
#if IS_ENABLED(CONFIG_SND_SOC_RT5642)
	{RT5640_DSP_PATH2, 0x0000},
#else
	{RT5640_DSP_PATH2, 0x0c00},
#endif
#if RT5640_DET_EXT_MIC
	{RT5640_MICBIAS, 0x3410},	/* disable MICBIAS short current;
					 chopper(b5) circuit disabled */
	{RT5640_GPIO_CTRL1, 0x8400},	/* set GPIO1 to IRQ */
	{RT5640_GPIO_CTRL3, 0x0004},	/* set GPIO1 output */
/*	{RT5640_GEN_CTRL2, 0x5100},*/	/* enable JD2 */
/*	{RT5640_IRQ_CTRL2, 0x8000},*/	/*set MICBIAS short current to IRQ */
	/*( if sticky set regBE : 8800 ) */
	/* for Jack Detection */
	{RT5640_JD_CTRL, 0x6003},
	{RT5640_IRQ_CTRL1, 0x8000}, /* enable jd */
#endif
};

/*
drc_mode:
0 : int mic
1 : headset mic
2 : spk
other: disable
*/
static void set_drc(struct snd_soc_codec *codec, int drc_mode)
{
	pr_debug("%s drc_mode=%d\n", __func__, drc_mode);
	switch (drc_mode) {
	case 0: /*int mic*/
		snd_soc_write(codec, RT5640_DRC_AGC_2, 0x1fab);
		snd_soc_write(codec, RT5640_DRC_AGC_3, 0x20ce);
		snd_soc_write(codec, RT5640_DRC_AGC_1, 0xc10f);
		break;
	case 1: /*headset mic*/
		snd_soc_write(codec, RT5640_DRC_AGC_2, 0x1fa4);
		snd_soc_write(codec, RT5640_DRC_AGC_3, 0x20ce);
		snd_soc_write(codec, RT5640_DRC_AGC_1, 0xc10f);
		break;
	case 2: /*spk*/
		if (snd_soc_read(codec, RT5640_PWR_DIG1) & RT5640_PWR_CLS_D) {
			snd_soc_write(codec, RT5640_DRC_AGC_2, 0x27e4);
			snd_soc_write(codec, RT5640_DRC_AGC_3, 0x0100);
			snd_soc_write(codec, RT5640_DRC_AGC_1, 0x420a);
		} else {
			snd_soc_write(codec, RT5640_DRC_AGC_1, 0x2206);
			snd_soc_write(codec, RT5640_DRC_AGC_2, 0x1f00);
			snd_soc_write(codec, RT5640_DRC_AGC_3, 0x0000);
		}
		break;
	default: /*disable drc*/
		snd_soc_write(codec, RT5640_DRC_AGC_1, 0x2206);
		snd_soc_write(codec, RT5640_DRC_AGC_2, 0x1f00);
		snd_soc_write(codec, RT5640_DRC_AGC_3, 0x0000);
		break;
	}
}
