/*
 ******************************************************************************
 * @file    lsm9ds1_support.c
 * @author  Isaac Grossberg
 * @brief   This file implements functions for setting up and reading from
 *          the LSM9DS1 inertial and magnetic sensors
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <wiringPiI2C.h>
#include "lsm9ds1_reg.h"

/* Struct Declarations -------------------------------------------------------------*/

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

typedef struct sensor{
    uint8_t addr_accel;
    uint8_t addr_gyro;
    uint8_t addr_mag;
    int file_desc_accel;
    int file_desc_gyro;
    int file_desc_mag;
}sensor;

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis3bit16_t data_raw_magnetic_field;
static double acceleration_mg[3];
static double angular_rate_mdps[3];
static double magnetic_field_mgauss[3];
static lsm9ds1_id_t whoamI;
static lsm9ds1_status_t reg;
static uint8_t rst;

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 *   Functions declared in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

/*
 * @brief  Set up configuration registers and establish I2C communication
 *         channels for all sensors on the LSM9DS1
 *
 * @param  lsm    sensor struct containing information about
 *                I2C addresses and file descriptors for communication
 *
 */
int Configure_lsm(struct sensor* lsm)
{ 
  /* Set up I2C communication */
  uint8_t i2c_add_mag = 0x1c;
  uint8_t i2c_add_imu = 0x6a;
  int fd_xl = wiringPiI2CSetup(i2c_add_imu);
  int fd_m = wiringPiI2CSetup(i2c_add_mag);

  /* Fill in sensor struct */
  lsm->addr_accel = i2c_add_imu;
  lsm->addr_gyro = i2c_add_imu;
  lsm->addr_mag = i2c_add_mag;
  lsm->file_desc_accel = fd_xl;
  lsm->file_desc_gyro = fd_xl;
  lsm->file_desc_mag = fd_m;

  /* Initialize inertial sensors (IMU) driver interface */
  stmdev_ctx_t dev_ctx_mag;
  dev_ctx_mag.write_reg = platform_write;
  dev_ctx_mag.read_reg = platform_read;
  dev_ctx_mag.handle = (void*)&fd_m;

  /* Initialize magnetic sensors driver interface */
  stmdev_ctx_t dev_ctx_imu;
  dev_ctx_imu.write_reg = platform_write;
  dev_ctx_imu.read_reg = platform_read;
  dev_ctx_imu.handle = (void*)&fd_xl;

  /* Check device ID */
  int ret = platform_read(&fd_xl, LSM9DS1_WHO_AM_I, (uint8_t*)&(whoamI.imu), 1);
  if (ret == 0) {
    ret += platform_read(&fd_m, LSM9DS1_WHO_AM_I_M, (uint8_t*)&(whoamI.mag), 1);
  }
  
  if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID){
    printf("Device not found\n");
    return -1;
  }

  /* Restore default configuration */
  lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
  do {
    printf("Device in reset\n");
    lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
  } while (rst);

  /* Enable Block Data Update */
  ret += lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

  /* Set full scale */
  ret += lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
  ret += lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
  ret += lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);

  /* Configure filtering chain - See datasheet for filtering chain details */
  /* Accelerometer filtering chain */
  ret += lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
  ret += lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_50);
  ret += lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
  /* Gyroscope filtering chain */
  ret += lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);
  ret += lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
  ret += lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_LPF2_OUT);

  /* Set Output Data Rate / Power mode */
  ret += lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
  ret += lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);
  
  return ret; //Return value indicates number of incorrectly performed functions
}

/*
 * @brief  Read from Accelerometer
 *
 * @param  lsm    sensor struct containing information about
 *                I2C addresses and file descriptors for communication
 *
 */
double* Read_lsm_accel(struct sensor* lsm)
{ 
  int fd_xl = lsm->file_desc_accel;
  int fd_m = lsm->file_desc_mag;
  stmdev_ctx_t dev_ctx_mag;
  dev_ctx_mag.write_reg = platform_write;
  dev_ctx_mag.read_reg = platform_read;
  dev_ctx_mag.handle = (void*)&fd_m;
  stmdev_ctx_t dev_ctx_imu;
  dev_ctx_imu.write_reg = platform_write;
  dev_ctx_imu.read_reg = platform_read;
  dev_ctx_imu.handle = (void*)&fd_xl;

  //Check status of data ready on accelerometer 
  do{
    platform_read(&fd_xl, LSM9DS1_STATUS_REG, (uint8_t*)&(reg.status_imu), 1);
  } while (reg.status_imu.xlda != 1);

  double* accel_readings;
  memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
  lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw_acceleration.u8bit);

  acceleration_mg[0] = (double) data_raw_acceleration.i16bit[0] * 0.122;
  acceleration_mg[1] = (double) data_raw_acceleration.i16bit[1] * 0.122;
  acceleration_mg[2] = (double) data_raw_acceleration.i16bit[2] * 0.122;

  return acceleration_mg;
 }

/*
 * @brief  Read from Gyroscope
 *
 * @param  lsm    sensor struct containing information about
 *                I2C addresses and file descriptors for communication
 *
 */
 double* Read_lsm_gyro(struct sensor* lsm)
 {  
   int fd_xl = lsm->file_desc_gyro;
   int fd_m = lsm->file_desc_mag;
   stmdev_ctx_t dev_ctx_mag;
   dev_ctx_mag.write_reg = platform_write;
   dev_ctx_mag.read_reg = platform_read;
   dev_ctx_mag.handle = (void*)&fd_m;
   stmdev_ctx_t dev_ctx_imu;
   dev_ctx_imu.write_reg = platform_write;
   dev_ctx_imu.read_reg = platform_read;
   dev_ctx_imu.handle = (void*)&fd_xl;

   do { //Check status of data ready on gyroscope
      platform_read(&fd_xl, LSM9DS1_STATUS_REG, (uint8_t*)&(reg.status_imu), 1);
    } while (reg.status_imu.gda != 1);

    memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));
    lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw_angular_rate.u8bit);

    angular_rate_mdps[0] = (double) data_raw_angular_rate.i16bit[0] * 70.0;
    angular_rate_mdps[1] = (double) data_raw_angular_rate.i16bit[1] * 70.0;
    angular_rate_mdps[2] = (double) data_raw_angular_rate.i16bit[2] * 70.0;

    return angular_rate_mdps;
 }

/*
 * @brief  Read from Magnetometer
 *
 * @param  lsm    sensor struct containing information about
 *                I2C addresses and file descriptors for communication
 *
 */
 double* Read_lsm_mag(struct sensor* lsm)
 {
   int fd_xl = lsm->file_desc_accel;
   int fd_m = lsm->file_desc_mag; 
   stmdev_ctx_t dev_ctx_mag;
   dev_ctx_mag.write_reg = platform_write;
   dev_ctx_mag.read_reg = platform_read;
   dev_ctx_mag.handle = (void*)&fd_m;
   stmdev_ctx_t dev_ctx_imu;
   dev_ctx_imu.write_reg = platform_write;
   dev_ctx_imu.read_reg = platform_read;
   dev_ctx_imu.handle = (void*)&fd_xl;

   do { //Check status of data ready on magnetometer
      platform_read(&fd_m, LSM9DS1_STATUS_REG_M, (uint8_t*)&(reg.status_mag), 1);
    } while (reg.status_mag.zyxda != 1);

    memset(data_raw_magnetic_field.u8bit, 0x00, 3 * sizeof(int16_t));
    lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field.u8bit);

    magnetic_field_mgauss[0] = (double) data_raw_magnetic_field.i16bit[0] * 0.58;
    magnetic_field_mgauss[1] = (double) data_raw_magnetic_field.i16bit[1] * 0.58;
    magnetic_field_mgauss[2] = (double) data_raw_magnetic_field.i16bit[2] * 0.58;

    return magnetic_field_mgauss;
 }

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor I2C address.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive registers to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  int *fd = handle;

  //printf("Writing %X to register %X (len %d,fd %d)\n", *bufp, reg, len, *fd);
  if (len == 1) {
    wiringPiI2CWriteReg8(*fd, reg, *bufp);
  }
  else if (len == 2) {
    wiringPiI2CWriteReg16(*fd, reg, *bufp);
  }
  else { return 1; }

  return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor I2C address.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that stores the data read
 * @param  len       number of consecutive registers to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{

  int *fd = handle;

  int result;
  if (len == 1) {
    result = wiringPiI2CReadReg8(*fd, reg);
    *bufp = result;
  }
  else if (len == 2) {
    result = wiringPiI2CReadReg16(*fd, reg);
    *bufp = result;
  }
  else if (len == 6) {
    int x_result = wiringPiI2CReadReg16(*fd, reg);
    int y_result = wiringPiI2CReadReg16(*fd, reg+2);
    int z_result = wiringPiI2CReadReg16(*fd, reg+4);
    *bufp = x_result;
    *(bufp+2) = y_result;
    *(bufp+4) = z_result;
  }
  else { return 1; }
  //printf("Read %X from register %X (len %d,fd %d)\n", *bufp, reg, len, *fd);
  return 0;
}
