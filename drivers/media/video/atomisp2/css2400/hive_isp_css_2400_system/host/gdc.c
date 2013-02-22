
/* The name "gdc.h is already taken" */
#include "gdc_device.h"

#include "device_access.h"

#include "assert_support.h"

/*
 * Local function declarations
 */
STORAGE_CLASS_INLINE void gdc_reg_store(
	const gdc_ID_t		ID,
	const unsigned int	reg,
	const hrt_data		value);

STORAGE_CLASS_INLINE hrt_data gdc_reg_load(
	const gdc_ID_t		ID,
	const unsigned int	reg);


#ifndef __INLINE_GDC__
#include "gdc_private.h"
#endif /* __INLINE_GDC__ */

/*
 * Exported function implementations
 */
void gdc_lut_store(
	const gdc_ID_t		ID,
	const int			data[4][HRT_GDC_N])
{
	unsigned int i, lut_offset = HRT_GDC_LUT_IDX;

assert(ID < N_GDC_ID);
assert(HRT_GDC_LUT_COEFF_OFFSET <= (4*sizeof(hrt_data)));

	for (i = 0; i < HRT_GDC_N; i++) {
		hrt_data	entry_0 = data[0][i] & HRT_GDC_BCI_COEF_MASK;
		hrt_data	entry_1 = data[1][i] & HRT_GDC_BCI_COEF_MASK;
		hrt_data	entry_2 = data[2][i] & HRT_GDC_BCI_COEF_MASK;
		hrt_data	entry_3 = data[3][i] & HRT_GDC_BCI_COEF_MASK;

		hrt_data	word_0 = entry_0 |
			(entry_1 << HRT_GDC_LUT_COEFF_OFFSET);
		hrt_data	word_1 = entry_2 |
			(entry_3 << HRT_GDC_LUT_COEFF_OFFSET);

		gdc_reg_store(ID, lut_offset++, word_0);
		gdc_reg_store(ID, lut_offset++, word_1);
	}
return;
}

int gdc_get_unity(
	const gdc_ID_t		ID)
{
assert(ID < N_GDC_ID);
	(void)ID;
return (int)(1UL << HRT_GDC_FRAC_BITS);
}


/*
 * Local function implementations
 */
STORAGE_CLASS_INLINE void gdc_reg_store(
	const gdc_ID_t		ID,
	const unsigned int	reg,
	const hrt_data		value)
{
	device_store_uint32(GDC_BASE[ID] + reg*sizeof(hrt_data), value);
return;
}

STORAGE_CLASS_INLINE hrt_data gdc_reg_load(
	const gdc_ID_t		ID,
	const unsigned int	reg)
{
return device_load_uint32(GDC_BASE[ID] + reg*sizeof(hrt_data));
}
