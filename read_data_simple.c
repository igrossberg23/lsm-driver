/*
 ******************************************************************************
 * @file    read_data_simple.c
 * @author  Sensors Software Solution Team
 * @brief   This file show the simplest way to get data from sensor.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3
 * - NUCLEO_F411RE + X_NUCLEO_IKS01A2
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 * NUCLEO_STM32F411RE + X_NUCLEO_IKS01A2 - Host side: UART(COM) to USB bridge
 *                                       - I2C(Default) / SPI(N/A)
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: `platform_write`, `platform_read`,
 * `tx_com` and 'platform_init' is required.
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */
//#define STEVAL_MKI109V3
//#define NUCLEO_F411RE_X_NUCLEO_IKS01A2

//#if defined(STEVAL_MKI109V3)
/* MKI109V3: Vdd and Vddio power supply values */
//#define PWM_3V3 915
//#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <wiringPiI2C.h>
#include "lsm9ds1_reg.h"

typedef union{
  int16_t i16bit[3];
  uint8_t u8bit[6];
} axis3bit16_t;

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;
static axis3bit16_t data_raw_magnetic_field;
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];
static lsm9ds1_id_t whoamI;
static lsm9ds1_status_t reg;
static uint8_t rst;
static uint8_t tx_buffer[1000];

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/
/*
 *   WARNING:
 *   Functions declared in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_init(void);


/* Main Example --------------------------------------------------------------*/
//void example_main_lsm9ds1(void)
int main()
{
  /* Initialize platform specific hardware */
  //platform_init();

  int fd_xl = wiringPiI2CSetup(0x6a);
  int fd_m = wiringPiI2CSetup(0x1c);

  //printf("fd_xl: %d\nfd_m: %d\n", fd_xl, fd_m);
  /* Initialize inertial sensors (IMU) driver interface */
  //uint8_t i2c_add_mag = LSM9DS1_MAG_I2C_ADD_L;
  uint8_t i2c_add_mag = 0x1c;
  stmdev_ctx_t dev_ctx_mag;
  dev_ctx_mag.write_reg = platform_write;
  dev_ctx_mag.read_reg = platform_read;
  //dev_ctx_mag.handle = (void*)&i2c_add_mag;
  dev_ctx_mag.handle = (void*)&fd_m;

  /* Initialize magnetic sensors driver interface */
  //uint8_t i2c_add_imu = LSM9DS1_IMU_I2C_ADD_H;
  uint8_t i2c_add_imu = 0x6a;
  stmdev_ctx_t dev_ctx_imu;
  dev_ctx_imu.write_reg = platform_write;
  dev_ctx_imu.read_reg = platform_read;
  //dev_ctx_imu.handle = (void*)&i2c_add_imu;
  dev_ctx_imu.handle = (void*)&fd_xl;

  /* Check device ID */
  lsm9ds1_dev_id_get(&dev_ctx_mag, &dev_ctx_imu, &whoamI);
  if (whoamI.imu != LSM9DS1_IMU_ID || whoamI.mag != LSM9DS1_MAG_ID){
    printf("Device not found\n");
    while(1){
      /* manage here device not found */
    }
  }

  /* Restore default configuration */
  lsm9ds1_dev_reset_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);
  do {
    printf("Device in reset\n");
    lsm9ds1_dev_reset_get(&dev_ctx_mag, &dev_ctx_imu, &rst);
  } while (rst);

  /* Enable Block Data Update */
  lsm9ds1_block_data_update_set(&dev_ctx_mag, &dev_ctx_imu, PROPERTY_ENABLE);

  /* Set full scale */
  lsm9ds1_xl_full_scale_set(&dev_ctx_imu, LSM9DS1_4g);
  lsm9ds1_gy_full_scale_set(&dev_ctx_imu, LSM9DS1_2000dps);
  lsm9ds1_mag_full_scale_set(&dev_ctx_mag, LSM9DS1_16Ga);

  /* Configure filtering chain - See datasheet for filtering chain details */
  /* Accelerometer filtering chain */
  lsm9ds1_xl_filter_aalias_bandwidth_set(&dev_ctx_imu, LSM9DS1_AUTO);
  lsm9ds1_xl_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ODR_DIV_50);
  lsm9ds1_xl_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LP_OUT);
  /* Gyroscope filtering chain */
  lsm9ds1_gy_filter_lp_bandwidth_set(&dev_ctx_imu, LSM9DS1_LP_ULTRA_LIGHT);
  lsm9ds1_gy_filter_hp_bandwidth_set(&dev_ctx_imu, LSM9DS1_HP_MEDIUM);
  lsm9ds1_gy_filter_out_path_set(&dev_ctx_imu, LSM9DS1_LPF1_HPF_LPF2_OUT);

  /* Set Output Data Rate / Power mode */
  lsm9ds1_imu_data_rate_set(&dev_ctx_imu, LSM9DS1_IMU_59Hz5);
  lsm9ds1_mag_data_rate_set(&dev_ctx_mag, LSM9DS1_MAG_UHP_10Hz);

  /* Read samples in polling mode (no int) */
  while(1)
  //int i;
  //for (i=0; i<5; i++)
  {
    /* Read device status register */
    lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);

    if ( reg.status_imu.xlda && reg.status_imu.gda )
    {
      /* Read imu data */
      memset(data_raw_acceleration.u8bit, 0x00, 3 * sizeof(int16_t));
      memset(data_raw_angular_rate.u8bit, 0x00, 3 * sizeof(int16_t));

      int ret1 = lsm9ds1_acceleration_raw_get(&dev_ctx_imu, data_raw_acceleration.u8bit);
      
      int ret2 = lsm9ds1_angular_rate_raw_get(&dev_ctx_imu, data_raw_angular_rate.u8bit);

      acceleration_mg[0] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[0]);
      acceleration_mg[1] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[1]);
      acceleration_mg[2] = lsm9ds1_from_fs4g_to_mg(data_raw_acceleration.i16bit[2]);

      angular_rate_mdps[0] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
      angular_rate_mdps[1] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
      angular_rate_mdps[2] = lsm9ds1_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

      printf("IMU - [mg]:%4.2f\t%4.2f\t%4.2f\t[mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      /*sprintf((char*)tx_buffer, "IMU - [mg]:%4.2f\t%4.2f\t%4.2f\t[mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
              acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
              angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
      tx_com(tx_buffer, strlen((char const*)tx_buffer));*/
    }

    if ( reg.status_mag.zyxda )
    {
      /* Read magnetometer data */
      memset(data_raw_magnetic_field.u8bit, 0x00, 3 * sizeof(int16_t));

      lsm9ds1_magnetic_raw_get(&dev_ctx_mag, data_raw_magnetic_field.u8bit);

      magnetic_field_mgauss[0] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[0]);
      magnetic_field_mgauss[1] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[1]);
      magnetic_field_mgauss[2] = lsm9ds1_from_fs16gauss_to_mG(data_raw_magnetic_field.i16bit[2]);

      printf("MAG - [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
              magnetic_field_mgauss[0], magnetic_field_mgauss[1], magnetic_field_mgauss[2]);
      /*sprintf((char*)tx_buffer, "MAG - [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
              magnetic_field_mgauss[0], magnetic_field_mgauss[1], magnetic_field_mgauss[2]);
      tx_com(tx_buffer, strlen((char const*)tx_buffer)); */
    }
  }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor I2C address.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
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
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
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

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to trasmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{

  /*#ifdef NUCLEO_F411RE_X_NUCLEO_IKS01A2
  HAL_UART_Transmit(&huart2, tx_buffer, len, 1000);
  #endif
  #ifdef STEVAL_MKI109V3
  CDC_Transmit_FS(tx_buffer, len);
  #endif*/
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
/*#ifdef STEVAL_MKI109V3
  TIM3->CCR1 = PWM_3V3;
  TIM3->CCR2 = PWM_3V3;
  HAL_Delay(1000);
#endif*/
}
