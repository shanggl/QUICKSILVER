#include "driver/spi_bmi323.h"

#include "core/profile.h"
#include "driver/gpio.h"
#include "driver/spi.h"
#include "driver/spi_gyro.h"
#include "driver/time.h"
#include "util/util.h"


#ifdef USE_GYRO

#define SPI_SPEED_SLOW MHZ_TO_HZ(4)
#define SPI_SPEED_FAST MHZ_TO_HZ(10)

extern spi_bus_device_t gyro_bus;

static int8_t gyro_cas=0;// not ready 

static void bmi323_reset_to_spi(){
// put the device in spi mode by toggeling CS
  gpio_pin_reset(gyro_bus.nss);
  time_delay_ms(1);
  gpio_pin_set(gyro_bus.nss);
  time_delay_ms(10);
}

uint8_t bmi323_detect(){
    bmi323_reset_to_spi();

    const uint8_t whoami=bmi3_read8(BMI323_REG_CHIP_ID);
    switch (whoami)
    {
    case BMI323_WHO_AMI:
        /* code */
        return GYRO_TYPE_BMI323;
    
    default:
        return GYRO_TYPE_INVALID;
    }
}

static void bmi323_init(){
    bmi323_reset_to_spi();
    bmi3_write16(BMI323_REG_CMD,BMI323_CMD_SOFT_RESET,100);
    bmi323_reset_to_spi();

}

static void bmi323_init_config(){
    //init acc conf
    uint16_t regdata=0;
    regdata= BMI3_ACC_BW_ODR_QUARTER<<7|BMI323_ACC_RANGE_16G<<4|BMI323_ACC_ODR_6400HZ;
    regdata|=BMI3_ACC_MODE_HIGH_PERF<<12| BMI323_ACC_AVG1<<8;
    bmi3_write16(BMI323_REG_ACC_CONF,regdata,1);
    regdata=0;
    //init gyro conf
    regdata=BMI323_GYR_BW_ODR_QUARTER<<7|BMI323_GYR_RANGE_2000DPS<<4|BMI323_GYR_ODR_6400HZ;
    regdata|=BMI323_GYR_MODE_HIGH_PERF<<12|BMI323_GYR_AVG1<<8;
    bmi3_write16(BMI323_REG_GYRO_CONF,regdata,1);
    //init data ready interupt to pin int1/ push_pull /active_high  NO_LATCH(default)
    //BMI323_REG_IO_INT_CTRL push_pull/active_high
    bmi3_write8(BMI323_REG_IO_INT_CTRL,BMI3_INT_OUTPUT_ENABLE<<2|BMI3_INT_PUSH_PULL<<1|BMI3_INT_ACTIVE_HIGH,1);
    //BMI323_REG_INT_MAP2 accready /gyro_ready 
    regdata=0;
    regdata= 0x01<<10 | 0x01<<8;
    bmi3_write16(BMI323_REG_INT_MAP2,regdata,15);
}

void bmi323_configure() {
  bmi323_init();
  // skip CRT for now
  bmi323_init_config();
//   bmi323_enable_cas();
}

uint8_t bmi3_read8(uint8_t reg) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_SLOW);

  uint8_t buffer[3] = {reg | 0x80, 0x0, 0x0};

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(buffer, buffer, 3),
  };
  spi_seg_submit_wait(&gyro_bus, segs);

  return buffer[2];
}

uint16_t bmi3_read16(uint8_t reg) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_SLOW);

  uint8_t buffer[4] = {reg | 0x80, 0x0, 0x0, 0x0};

  const spi_txn_segment_t segs[] = {
      spi_make_seg_buffer(buffer, buffer, 4),
  };
  spi_seg_submit_wait(&gyro_bus, segs);

  return ((buffer[3] << 8) | buffer[2]);
}

void bmi3_write8(uint8_t reg, uint8_t data, uint32_t delay) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_SLOW);

  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg),
      spi_make_seg_const(data),
  };
  spi_seg_submit_wait(&gyro_bus, segs);

  time_delay_ms(delay);
}

void bmi3_write16(uint8_t reg, uint16_t data, uint32_t delay) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_SLOW);

  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg),
      spi_make_seg_const(data & 0xff),
      spi_make_seg_const(data >> 8),
  };
  spi_seg_submit_wait(&gyro_bus, segs);

  time_delay_ms(delay);
}

void bmi3_write_data(uint8_t reg, uint8_t *data, uint32_t size, uint32_t delay) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_SLOW);

  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg),
      spi_make_seg_buffer(NULL, data, size),
  };
  spi_seg_submit_wait(&gyro_bus, segs);

  time_delay_ms(delay);
}

void bmi3_read_data(uint8_t reg, uint8_t *data, uint32_t size) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_FAST);

  const spi_txn_segment_t segs[] = {
      spi_make_seg_const(reg | 0x80),
      spi_make_seg_const(0xFF),
      spi_make_seg_buffer(data, NULL, size),
  };
  spi_seg_submit_wait(&gyro_bus, segs);
}

void bmi323_read_gyro_data(gyro_data_t *data) {
  spi_bus_device_reconfigure(&gyro_bus, SPI_MODE_TRAILING_EDGE, SPI_SPEED_FAST);

  uint8_t buf[12];
  const spi_txn_segment_t gyro_segs[] = {
      spi_make_seg_const(BMI323_REG_ACC_DATA_X_LSB | 0x80),
      spi_make_seg_const(0xFF),
      spi_make_seg_buffer(buf, NULL, 12),
  };
  spi_seg_submit_wait(&gyro_bus, gyro_segs);

  data->accel.axis[0] = -(int16_t)((buf[1] << 8) | buf[0]);
  data->accel.axis[1] = -(int16_t)((buf[3] << 8) | buf[2]);
  data->accel.axis[2] = (int16_t)((buf[5] << 8) | buf[4]);

  int16_t gyro_data[3] = {
      (int16_t)((buf[7] << 8) | buf[6]),
      (int16_t)((buf[9] << 8) | buf[8]),
      (int16_t)((buf[11] << 8) | buf[10]),
  };

  const int32_t tempx = gyro_data[0] - (int16_t)(gyro_cas * (int16_t)(gyro_data[2]) / 512);
  if (tempx > 32767) {
    gyro_data[0] = 32767;
  } else if (tempx < -32768) {
    gyro_data[0] = -32768;
  } else {
    gyro_data[0] = tempx;
  }

  data->gyro.axis[1] = gyro_data[0];
  data->gyro.axis[0] = gyro_data[1];
  data->gyro.axis[2] = gyro_data[2];

  data->temp = 0;
}


#endif