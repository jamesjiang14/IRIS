#include "g_force.h"
#include <stdio.h>
#include "adxl345.h"
#include "mxc_errors.h"

/* Initialize ADXL345 using the adxl345 driver. This is optional;
 * if you already initialize ADXL in main, you can ignore this helper.
 */
int gforce_init(struct adxl345_dev **dev_out,
                mxc_i2c_regs_t *i2c_inst,
                uint8_t i2c_addr,
                uint8_t range,
                uint8_t full_res)
{
    if (dev_out == NULL) return -1;

    struct adxl345_dev *dev = NULL;
    struct adxl345_init_param init_param;
    int32_t status;

    /* Use the SDK I2C type for init_param */
    init_param.i2c_inst = i2c_inst;
    init_param.i2c_addr = i2c_addr;
    init_param.dev_type = ID_ADXL345;
    init_param.selected_range = range;
    init_param.full_resolution_set = full_res;

    status = adxl345_init(&dev, init_param);
    if (status != 0) {
        return status;
    }

    adxl345_set_range_resolution(dev, init_param.selected_range, init_param.full_resolution_set);
    adxl345_set_power_mode(dev, 1);

    *dev_out = dev;
    return 0;
}

/* Read X/Y/Z in g units using adxl345_get_g_xyz (driver-provided). */
int gforce_read_g(struct adxl345_dev *dev, float *x_g, float *y_g, float *z_g)
{
    if (dev == NULL || x_g == NULL || y_g == NULL || z_g == NULL) {
        return -1;
    }

    /* adxl345_get_g_xyz from your driver returns float values in g */
    adxl345_get_g_xyz(dev, x_g, y_g, z_g);

    return 0;
}

/* Read magnitude (g) directly */
int gforce_read_magnitude(struct adxl345_dev *dev, float *mag_g)
{
    if (dev == NULL || mag_g == NULL) {
        return -1;
    }

    float x, y, z;
    int res = gforce_read_g(dev, &x, &y, &z);
    if (res != 0) return res;

    *mag_g = sqrtf(x * x + y * y + z * z);
    return 0;
}

/* Compute weighted final G using only X and Z (Y ignored).
 *
 * Interpretation:
 *   - Z weight: 65%
 *   - X weight: 35%
 *
 * This implementation uses absolute values (fabsf) so final result is non-negative.
 * The final g is computed as a linear weighted sum:
 *    final_g = 0.35 * |x_g| + 0.65 * |z_g|
 *
 * If you prefer a different interpretation (e.g. weighted Euclidean norm),
 * tell me and I can change the formula.
 */
float gforce_weighted_from_xz(float x_g, float z_g)
{
    const float w_x = 0.35f;
    const float w_z = 0.65f;

    float ax = fabsf(x_g);
    float az = fabsf(z_g);

    return w_x * ax + w_z * az;
}
