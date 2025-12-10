#ifndef G_FORCE_H
#define G_FORCE_H

#include <stdint.h>
#include <math.h>   /* for sqrtf, fabsf */
#include "i2c.h"    /* for mxc_i2c_regs_t (I2C instance type) */

/* Forward-declare ADXL device struct from your ADXL driver. */
struct adxl345_dev;

/**
 * Initialize ADXL345 through this module (optional). If you already
 * initialize ADXL externally (like your main currently does), you can
 * ignore this function and call the read/compute helpers below.
 *
 * @param dev_out  Pointer to adxl device pointer to receive handle.
 * @param i2c_inst I2C instance (e.g. MXC_I2C1).
 * @param i2c_addr 7-bit ADXL345 address (0x1D or 0x53).
 * @param range    ADXL range constant (e.g. ADXL345_RANGE_PM_8G).
 * @param full_res ADXL345_FULL_RES to enable full resolution or 0 to disable.
 * @return 0 on success, non-zero on failure.
 */
int gforce_init(struct adxl345_dev **dev_out,
                mxc_i2c_regs_t *i2c_inst,
                uint8_t i2c_addr,
                uint8_t range,
                uint8_t full_res);

/* Read x/y/z in g units from an already-initialized adxl dev.
 * Returns 0 on success, non-zero on error.
 */
int gforce_read_g(struct adxl345_dev *dev, float *x_g, float *y_g, float *z_g);

/* Read magnitude directly from device (sqrt(x^2+y^2+z^2)).
 * Returns 0 on success, non-zero on error.
 */
int gforce_read_magnitude(struct adxl345_dev *dev, float *mag_g);

/* Compute Euclidean magnitude from provided x,y,z (in g). Useful to avoid a second
 * sensor read if you already captured x,y,z.
 */
static inline float gforce_mag_from_xyz(float x_g, float y_g, float z_g)
{
    /* Compute Euclidean norm (magnitude) */
    return sqrtf(x_g * x_g + y_g * y_g + z_g * z_g);
}

/**
 * Compute weighted final G using only X and Z (Y ignored).
 *
 * Weighting:
 *   - Z contributes 65% of the final G
 *   - X contributes 35% of the final G
 *
 * The function uses absolute values of the axis readings so the final g is non-negative.
 * Returns the final weighted g value.
 */
float gforce_weighted_from_xz(float x_g, float z_g);

#endif /* G_FORCE_H */
