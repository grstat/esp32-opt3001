/* opt3001.h
 * ---------------------------------
 * ESP32 OPT3001 LIBRARY HEADER FILE
 * ---------------------------------
 * Copyright 2023 Open grStat
 * SPDX-FileCopyrightText: 2023 Open grStat https://github.com/grstat
 * SPDX-FileType: SOURCE
 * SPDX-FileContributor: Created by Adrian Borchardt
 * SPDX-License-Identifier: Apache 2.0
 */
#ifndef __OPT3001_H__
#define __OPT3001_H__

#define OPT3001_TIMEOUT_PERIOD   ((TickType_t)200 / portTICK_PERIOD_MS)

/* 8BIT REGISTER ADDRESS POINTERS */
#define OPT3001_RESULT_REG            0x00 // AMBIENT LIGHT VALUE
#define OPT3001_CONFIG_REG            0x01 // CONFIGURATION
#define OPT3001_LOW_LIMIT_REG         0x02 // LOWER COMPARISON LIMIT
#define OPT3001_HIGH_LIMIT_REG        0x03 // UPPER COMPARISON LIMIT
#define OPT3001_MANUFACTURER_ID_REG   0x7E // 0x5449 FOR TI
#define OPT3001_DEVICE_ID_REG         0x7F // 0x3001 FOR OPT3001
#define OPT3001_MANUFACTURER_ID       0x5449
#define OPT3001_DEVICE_ID             0x3001

/* OPT3001 DEVICE ADDRESSES ARE SET VIA 
 * THE ADDR PIN CONNECTED TO GND, VCC, SDA OR SCL 
 * UP TO 4 DEVICES CAN BE ON THE I2C BUS AT A TIME */
#define OPT3001_ADDR_GND    0x44
#define OPT3001_ADDR_VCC    0x45
#define OPT3001_ADDR_SDA    0x46
#define OPT3001_ADDR_SCL    0x47

#define OPT3001_RANGE_AUTO_FULL_SCALE           0x0C  //AUTOMATIC FULL-SCALE RANGE MODE
#define OPT3001_CONVERSION_TIME_100MS           0x00  //100ms CONVERSION TIME
#define OPT3001_CONVERSION_TIME_800MS           0x01  //800ms CONVERSION TIME
#define OPT3001_CONVERSION_MODE_LOWPOWER        0x00
#define OPT3001_CONVERSION_MODE_SINGLE_SHOT     0x01
#define OPT3001_CONVERSION_MODE_CONTINUOUS_A    0x02
#define OPT3001_CONVERSION_MODE_CONTINUOUS_B    0x03
#define OPT3001_OVERFLOW_FLAG_ISSET             0x01  //LIGHT EXCEEDED FULL SCALE MEASUREMENT
#define OPT3001_CONVERSION_READY                0x01  //CONVERSION IS READY TO READ
#define OPT3001_HIGH_FLAG_ISSET                 0x01  //RESULT IS HIGHER THAN THE HIGH FEILD
#define OPT3001_LOW_FLAG_ISSET                  0x01  //RESULT IS BELOW THE LOW FEILD
#define OPT3001_LATCH_TRANSPARENT_HYSTERESIS    0x00  //RESULT/HIGH/LOW WITH NO CLEAR
#define OPT3001_LATCH_WINDOW_STYLE              0x01  //USER MUST CLEAR INT TRIGGER
#define OPT3001_INT_ACTIVE_LOW                  0x00  //HIGH TO LOW TRIGGER
#define OPT3001_INT_ACTIVE_HIGH                 0x01  //LOW TO HIGH TRIGGER
#define OPT3001_DEFAULT_MASK                    0x00
#define OPT3001_ONE_FAULT_INT_TRIGGER           0x00
#define OPT3001_TWO_FAULT_INT_TRIGGER           0x01
#define OPT3001_FOUR_FAULT_INT_TRIGGER          0x02
#define OPT3001_EIGHT_FAULT_INT_TRIGGER         0x03
#define OPT3001_END_OF_CONVERSION_ENABLED       0xC0
#define OPT3001_END_OF_CONVERSION_DISABLED      0x00
#define OPT3001_HIGHEST_FEILD_VALUE             (83865.60)
#define OPT3001_LOWEST_FEILD_VALUE              (0.01)
#define OPT3001_ERR_I2C_READING_LUX             (-1.2)
#define OPT3001_ERR_TIMEOUT_READING_LUX         (-1.1)
#define OPT3001_ERR                             0xFF

/* THE CONFIG REGISTER IS 16 BITS LONG BUT THE FIRST 8 
 * BITS ARE RESERVED, SO HERE JUST PACK IN THE LAST 8 BITS */
typedef union {
  unsigned short config_register;
  struct {
    unsigned short fault_count_field : 2;   // 0->1   R/W
    unsigned short mask_exponent_field : 1; // 2      R/W
    unsigned short polarity_field : 1;      // 3      R/W
    unsigned short latch_field : 1;         // 4      R/W
    unsigned short low_field_flag : 1;      // 5      R
    unsigned short high_field_flag : 1;     // 6      R
    unsigned short conversion_ready : 1;    // 7      R
    unsigned short overflow_flag : 1;       // 8      R
    unsigned short mode_of_conversion : 2;  // 10->9  R/W
    unsigned short conversion_time : 1;     // 11     R/W
    unsigned short range_number_feild : 4;  // 15->12 R/W
  };
} opt3001_config_t;

/* THIS CAN BE USED FOR THE DEFAULT REGISTER VALUES */
#define OPT3001_REGISTER_DEFAULTS(){ \
      .fault_count_field = OPT3001_ONE_FAULT_INT_TRIGGER, \
      .mask_exponent_field = OPT3001_DEFAULT_MASK, \
      .polarity_field = OPT3001_INT_ACTIVE_LOW, \
      .latch_field = OPT3001_LATCH_WINDOW_STYLE, \
      .low_field_flag = 0, \
      .high_field_flag = 0, \
      .conversion_ready = 0, \
      .overflow_flag = 0, \
      .mode_of_conversion = OPT3001_CONVERSION_MODE_LOWPOWER, \
      .conversion_time = OPT3001_CONVERSION_TIME_800MS, \
      .range_number_feild = OPT3001_RANGE_AUTO_FULL_SCALE \
}

/* THE UPPER AND LOWER LIMITS USED 
 * FOR CONFIGURATION */
typedef struct OPT3001_FEILD_LIMITS {
  float low_limit;
  float high_limit;
} opt3001_feild_limits_t;

/* 
 * i2c_address -> OPT3001 i2c ADDRESS
 * i2c_port_number -> THE CONFIGURED i2c PORT
 * eoc_indicator -> END OF CONVERSATION MODE (0 FOR DISABLED)
 * feild_limits -> HIGH/LOW FEILD LIMITS
 */
typedef struct OPT3001_SETTINGS {
  unsigned char i2c_address;
  unsigned char i2c_port_number;
  unsigned char eoc_indicator;
  opt3001_feild_limits_t feild_limits;
} opt3001_settings_t;

esp_err_t opt3001_configure(opt3001_settings_t opt3001_settings, opt3001_config_t opt_cfg);
esp_err_t opt3001_get_config(opt3001_settings_t opt3001_settings, opt3001_config_t * opt_cfg);
float opt3001_get_lux(opt3001_settings_t opt3001_settings);
esp_err_t opt3001_get_limits(opt3001_settings_t * opt3001_settings);
esp_err_t opt3001_set_limits(opt3001_settings_t opt3001_settings);

#endif