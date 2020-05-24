#pragma once

#include "profile.h"

typedef struct {
  uint32_t time;
  uint16_t cpu_load;
  uint16_t vbat_filter;

  vec4_t rx_raw;
  vec4_t rx_filter;
  uint32_t rx_aux;

  compact_vec3_t accel_raw;
  compact_vec3_t accel_filter;

  compact_vec3_t gyro_raw;
  compact_vec3_t gyro_filter;

  float gyro_vector[3];
  float pid_output[3];
} blackbox_t;

cbor_result_t cbor_encode_blackbox_t(cbor_value_t *enc, const blackbox_t *b);

void blackbox_init();
void blackbox_update();