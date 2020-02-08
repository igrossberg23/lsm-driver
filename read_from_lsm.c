#include "lsm9ds1_reg.h"
#include <stdio.h>
#include <string.h>

/* Generic Sensor Information struct */
typedef struct sensor{
    uint8_t addr_imu;
    uint8_t addr_mag;
    int file_desc_imu;
    int file_desc_mag;
}sensor;

/* Function declarations */
int Configure_lsm(struct sensor* lsm);
double* Read_lsm_accel(struct sensor* lsm);
double* Read_lsm_gyro(struct sensor* lsm);
double* Read_lsm_mag(struct sensor* lsm);

int main() {
  struct sensor lsm9ds1;
    
  int result = Configure_lsm(&lsm9ds1);

  double lsm_accel_readings[3];
  double lsm_gyro_readings[3];
  double lsm_mag_readings[3];

  double* accel_return;
  double* gyro_return;
  double* mag_return;

  if (result == 0) {
    while(1) {
    
      accel_return = Read_lsm_accel(&lsm9ds1);
      gyro_return = Read_lsm_gyro(&lsm9ds1);
      mag_return = Read_lsm_mag(&lsm9ds1);

      lsm_accel_readings[0] = *(accel_return);
      lsm_accel_readings[1] = *(accel_return+1);
      lsm_accel_readings[2] = *(accel_return+2);

      lsm_gyro_readings[0] = *(gyro_return);
      lsm_gyro_readings[1] = *(gyro_return+1);
      lsm_gyro_readings[2] = *(gyro_return+2);

      lsm_mag_readings[0] = *(mag_return);
      lsm_mag_readings[1] = *(mag_return+1);
      lsm_mag_readings[2] = *(mag_return+2);

      printf("IMU - [mg]: x:%4.2f\ty:%4.2f\tz:%4.2f\t[mdps]: x:%4.2f\ty:%4.2f\tz:%4.2f\r\n",
              lsm_accel_readings[0], lsm_accel_readings[1], lsm_accel_readings[2],
              lsm_gyro_readings[0], lsm_gyro_readings[1], lsm_gyro_readings[2]);

      printf("MAG - [mG]: x:%4.2f\ty:%4.2f\tz:%4.2f\r\n",
              lsm_mag_readings[0], lsm_mag_readings[1], lsm_mag_readings[2]);
    }
  }
  return 0;
}
