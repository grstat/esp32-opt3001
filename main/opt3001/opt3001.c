/* opt3001.c
 * ---------------------------------------
 * ESP32 OPT3001 ALS DRIVER LIBRARY C FILE
 * ---------------------------------------
 * The OPT3001 is an Ambient Light Sensor (ALS) Manufactured by Texas Instruments
 * that can measures lux values between 0.01 and 83k and also provides a highly 
 * configurable interrupt pin.
 * -------------------------------------------------------------------------------------
 * Copyright 2023 Open grStat
 * SPDX-FileCopyrightText: 2023 Open grStat https://github.com/grstat
 * SPDX-FileType: SOURCE
 * SPDX-FileContributor: Created by Adrian Borchardt
 * SPDX-License-Identifier: Apache 2.0
 * -------------------------------------------------------------------------------------
 * Some of the information below is supplied directly from the 
 * OPT3001 DATASHEET - SBOS853A - MARCH 2017–REVISED DECEMBER 2018
 * Which is Copyright 2020, Texas Instruments Incorporated
 * SEE https://www.ti.com/product/OPT3001 FOR MORE INFORMATION
 * -------------------------------------------------------------------------------------
 */
#include <string.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <math.h>
#include "opt3001.h"

#define LOG_TAG   "OPT3001"

/* THE RESULT REGISTER IS THE 16-BIT LIGHT TO DIGITAL CONVERSION
 * A 12 BIT FRACTIONAL RESULT AND A 4 BIT EXPONENT, THE REGISTER IS
 * READ ONLY */
typedef union {
  unsigned short register_data;
  struct{
    unsigned short mantissa : 12;
    unsigned short exponent : 4;
  };
} opt3001_register_data_t;

static unsigned char get_lsb_size_from_lux(float luxValue);
static float opt3001_calculate_float(unsigned short lux_s);

/* --------------------------------------------------------------------------------------------------
 * @name esp_err_t opt3001_configure(opt3001_settings_t * opt3001_settings, opt3001_config_t opt_cfg)
 * --------------------------------------------------------------------------------------------------
 * @brief Set OPT3001 config registers
 * @param opt3001_settings -> Pointer to the opt3001_settings_t struct
 * @param opt_cfg - Device configuration from the opt3001_config_t struct
 * @return ESP_OK on success* 
 * @note The i2c bus must be setup and configured before
 *       calling this routine
 */
esp_err_t opt3001_configure(opt3001_settings_t opt3001_settings, opt3001_config_t opt_cfg){
  unsigned char rxBuff[2] = {0};
  unsigned char txBuff[2] = {0};
  //GET THE MANUFACTURER INFORMATION AND VALIDATE
  txBuff[0] = OPT3001_MANUFACTURER_ID_REG;
  esp_err_t err_ck = i2c_master_write_read_device(opt3001_settings.i2c_port_number, opt3001_settings.i2c_address,
                     (const uint8_t *)txBuff, 1, (uint8_t *)rxBuff, 2, OPT3001_TIMEOUT_PERIOD);
  if(err_ck != ESP_OK){ return err_ck; }
  if((unsigned short)((rxBuff[0] << 8) | rxBuff[1]) != OPT3001_MANUFACTURER_ID){
    // NOT A TI CHIP
    ESP_LOGE(LOG_TAG, "EXPECTED TI ID 0x%04X BUT GOT 0x%04X", OPT3001_MANUFACTURER_ID, (unsigned short)((rxBuff[0] << 8) | rxBuff[1]));
    return OPT3001_ERR;
  }
  //GET THE DEVICE ID AND VALIDATE
  txBuff[0] = OPT3001_DEVICE_ID_REG;
  err_ck = i2c_master_write_read_device(opt3001_settings.i2c_port_number, opt3001_settings.i2c_address,
           (const uint8_t *)txBuff, 1, (uint8_t *)rxBuff, 2, OPT3001_TIMEOUT_PERIOD);
  if(err_ck != ESP_OK){ return err_ck; }
  if((unsigned short)((rxBuff[0] << 8) | rxBuff[1]) != OPT3001_DEVICE_ID){
    // DOESN'T SEEM TO BE AN OPT3001
    ESP_LOGE(LOG_TAG, "EXPECTED DEVICE ID 0x%04X GOT 0x%04X, NOT AN OPT3001", OPT3001_DEVICE_ID, (unsigned short)((rxBuff[0] << 8) | rxBuff[1]));
    return OPT3001_ERR;
  }
  //GET THE CURRENT CONFIGURATION
  txBuff[0] = OPT3001_CONFIG_REG;
  err_ck = i2c_master_write_read_device(opt3001_settings.i2c_port_number, opt3001_settings.i2c_address,
           (const uint8_t *)txBuff, 1, (uint8_t *)rxBuff, 2, OPT3001_TIMEOUT_PERIOD);  
  if(err_ck != ESP_OK){ return err_ck; }
  if((unsigned short)((rxBuff[0] << 8) | rxBuff[1]) != opt_cfg.config_register){
    ESP_LOGD(LOG_TAG, "CURRENT CONFIGURATION IS 0x%04X", ((rxBuff[0] << 8) | rxBuff[1]));
    ESP_LOGD(LOG_TAG, "WRITING NEW CONFIGURATION 0x%04X", opt_cfg.config_register);
    txBuff[0] = (opt_cfg.config_register >> 8);
    txBuff[1] = (opt_cfg.config_register & 0x00FF);
    i2c_cmd_handle_t cmdlnk = i2c_cmd_link_create();
    i2c_master_start(cmdlnk);
    i2c_master_write_byte(cmdlnk, (opt3001_settings.i2c_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmdlnk, OPT3001_CONFIG_REG, true);
    i2c_master_write(cmdlnk, txBuff, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmdlnk);
    err_ck = i2c_master_cmd_begin(opt3001_settings.i2c_port_number, cmdlnk, OPT3001_TIMEOUT_PERIOD);
    i2c_cmd_link_delete(cmdlnk);
  }
  return err_ck;
}

/* --------------------------------------------------------------------------------------------------
 * @name esp_err_t opt3001_get_limits(opt3001_settings_t * opt3001_settings)
 * --------------------------------------------------------------------------------------------------
 * @brief Read the current limit registers and update the settings
 * @param opt3001_settings -> Pointer to the opt3001_settings_t struct
 * @param opt_cfg - Device configuration from the opt3001_config_t struct
 * @return ESP_OK on success* 
 * @note The i2c bus must be setup and configured before
 *       calling this routine
 */
esp_err_t opt3001_get_limits(opt3001_settings_t * opt3001_settings){
  const unsigned char cfgRegHigh = OPT3001_HIGH_LIMIT_REG;
  const unsigned char cfgRegLow = OPT3001_LOW_LIMIT_REG;
  unsigned char rxReg[2] = {0};
  esp_err_t err_ck = i2c_master_write_read_device(opt3001_settings->i2c_port_number, opt3001_settings->i2c_address,
                     &cfgRegHigh, 1, (uint8_t *)rxReg, 2, OPT3001_TIMEOUT_PERIOD);
  if(err_ck != ESP_OK){ return err_ck; }
  opt3001_settings->feild_limits.high_limit = opt3001_calculate_float((unsigned short)((rxReg[0] << 8) | rxReg[1]));
  ESP_LOGD(LOG_TAG, "READ HIGH LIMIT VALUE AS %.2f", opt3001_settings->feild_limits.high_limit);
  err_ck = i2c_master_write_read_device(opt3001_settings->i2c_port_number, opt3001_settings->i2c_address,
                     &cfgRegLow, 1, (uint8_t *)rxReg, 2, OPT3001_TIMEOUT_PERIOD);
  if(err_ck != ESP_OK){ return err_ck; }
  opt3001_settings->feild_limits.low_limit = opt3001_calculate_float((unsigned short)((rxReg[0] << 8) | rxReg[1]));
  ESP_LOGD(LOG_TAG, "READ LOW LIMIT VALUE AS %.2f", opt3001_settings->feild_limits.low_limit);
  return err_ck;
}

/* --------------------------------------------------------------------------------------------------
 * @name esp_err_t opt3001_set_limits(opt3001_settings_t opt3001_settings)
 * --------------------------------------------------------------------------------------------------
 * @brief Set OPT3001 high and low limit registers
 * @param opt3001_settings -> the opt3001_settings_t struct
 * @return ESP_OK on success* 
 * @note The i2c bus must be setup and configured before
 *       calling this routine
 */
esp_err_t opt3001_set_limits(opt3001_settings_t opt3001_settings){
  float luxExponent, luxMantissa = 0;
  unsigned char txBuff[2];
  opt3001_register_data_t hl_reg = {0};
  //TEST HIGH LIMIT AND LOW LIMIT ARE SET
  if((opt3001_settings.feild_limits.high_limit == 0) && (opt3001_settings.feild_limits.low_limit == 0)){ return ESP_OK; } //NOT CONFIGURED
  //TEST HIGH LIMIT IS NOT < LOW LIMIT
  if(opt3001_settings.feild_limits.high_limit <= opt3001_settings.feild_limits.low_limit){
    ESP_LOGE("OPT3001", "HIGH LIMIT %.2f CANNOT BE LESS OR EQUAL LOW LIMIT %.2f", opt3001_settings.feild_limits.high_limit, opt3001_settings.feild_limits.low_limit);
    return OPT3001_ERR;
  }
  //IF HIGH LIMIT IS OVER THE MAX VALUE, SET IT TO THE MAX VALUE
  if(opt3001_settings.feild_limits.high_limit > OPT3001_HIGHEST_FEILD_VALUE){opt3001_settings.feild_limits.high_limit = OPT3001_HIGHEST_FEILD_VALUE;}
  //IF LOW LIMIT IS BELOW THE LOWEST VALUE, SET IT TO THE LOWEST VALUE
  if(opt3001_settings.feild_limits.low_limit < OPT3001_LOWEST_FEILD_VALUE){opt3001_settings.feild_limits.low_limit = OPT3001_LOWEST_FEILD_VALUE;}
  //GET THE LSB_SIZE BITS FROM THE LUX VALUE
  unsigned char expVal = get_lsb_size_from_lux(opt3001_settings.feild_limits.high_limit);
  ESP_LOGD("OPT3001", "HIGH LIMIT %.2f -> EXP 0x%02X", opt3001_settings.feild_limits.high_limit, expVal);
  //CALCULATE THE EXPONENT
  luxExponent = (0.01 * pow(2, (float)expVal));
  ESP_LOGD("OPT3001", "EXPONENT %.2f", luxExponent);
  //CALCULATE THE MANTISSA
  luxMantissa = (opt3001_settings.feild_limits.high_limit / luxExponent);
  ESP_LOGD("OPT3001", "MANTISS %.2f", luxMantissa);
  //CONVERT THE FLOAT DATA TO BITS
  hl_reg.mantissa = (unsigned short)luxMantissa;
  hl_reg.exponent = expVal;
  ESP_LOGD("OPT3001", "MSG 0x%04X", hl_reg.register_data);
  //FILL THE TX BUFFER
  txBuff[0] = (hl_reg.register_data >> 8);
  txBuff[1] = (hl_reg.register_data & 0x00FF);
  ESP_LOGD("OPT3001", "BUFF 0x%02X 0x%02X", txBuff[0], txBuff[1]);
  //QUEUE UP AND SEND THE HIGH LIMIT VALUE
  i2c_cmd_handle_t cmdlnk = i2c_cmd_link_create();
  i2c_master_start(cmdlnk);
  i2c_master_write_byte(cmdlnk, (opt3001_settings.i2c_address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmdlnk, OPT3001_HIGH_LIMIT_REG, true);
  i2c_master_write(cmdlnk, txBuff, 2, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmdlnk);
  esp_err_t err_ck = i2c_master_cmd_begin(opt3001_settings.i2c_port_number, cmdlnk, OPT3001_TIMEOUT_PERIOD);
  i2c_cmd_link_delete(cmdlnk);
  if(err_ck != ESP_OK){ return err_ck; }
  //RESET TO BE SAFE
  memset(&txBuff, 0, sizeof(txBuff));
  hl_reg.register_data = 0;
  luxExponent = luxMantissa = 0;
  //RINSE AND REPEAT FOR THE LOW LIMIT
  expVal = get_lsb_size_from_lux(opt3001_settings.feild_limits.low_limit);
  ESP_LOGD("OPT3001", "LOW LIMIT %.2f -> EXP 0x%02X", opt3001_settings.feild_limits.low_limit, expVal);
  luxExponent = (0.01 * pow(2, (float)expVal));
  ESP_LOGD("OPT3001", "EXPONENT %.2f", luxExponent);
  luxMantissa = (opt3001_settings.feild_limits.low_limit / luxExponent);
  ESP_LOGD("OPT3001", "MANTISS %.2f", luxMantissa);
  hl_reg.mantissa = (unsigned short)luxMantissa;
  hl_reg.exponent = expVal;
  ESP_LOGD("OPT3001", "MSG 0x%04X", hl_reg.register_data);
  txBuff[0] = (hl_reg.register_data >> 8);
  txBuff[1] = (hl_reg.register_data & 0x00FF);
  ESP_LOGD("OPT3001", "LOW BUFF 0x%02X 0x%02X", txBuff[0], txBuff[1]);
  //CHECK TO SEE IF END OF CONVERSATION MODE SHOULD BE ENABLED
  if(opt3001_settings.eoc_indicator == OPT3001_END_OF_CONVERSION_ENABLED){
    //ENABLE END OF CONVERSATION INDICATOR MORE BY SETTING THE MSb OF THE LOW-LIMIT REGISTER LE[3:2]
    //THIS WILL RESET THE LOW LIMIT VALUES
    txBuff[0] = OPT3001_END_OF_CONVERSION_ENABLED;
    txBuff[1] = 0;
    ESP_LOGD("OPT3001", "END OF CONVERSION SET 0x%02X 0x%02X", txBuff[0], txBuff[1]);
  }
  cmdlnk = i2c_cmd_link_create();
  i2c_master_start(cmdlnk);
  i2c_master_write_byte(cmdlnk, (opt3001_settings.i2c_address << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmdlnk, OPT3001_LOW_LIMIT_REG, true);
  i2c_master_write(cmdlnk, txBuff, 2, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmdlnk);
  err_ck = i2c_master_cmd_begin(opt3001_settings.i2c_port_number, cmdlnk, OPT3001_TIMEOUT_PERIOD);
  i2c_cmd_link_delete(cmdlnk);
  return err_ck;
}

/* --------------------------------------------------------------------------------------------------
 * @name static unsigned char get_lsb_size_from_lux(float luxValue)
 * --------------------------------------------------------------------------------------------------
 * @brief Get the LSB chart value from the lux input
 * @param luxValue -> The lux value to chart the LSB to
 * @return unsigned char -> The LSB value
 * -----------------------------------------------------------------------
 * Table 8. Full-Scale Range and LSB Size as a Function of Exponent Level
 * -----------------------------------------------------------------------
 * | E3 | E2 | E1 | E0 | FULL-SCALE RANGE (lux) | LSB SIZE (lux per LSB) |
 * -----------------------------------------------------------------------
 * |  0 |  0 |  0 |  0 |                  40.95 |                   0.01 |
 * -----------------------------------------------------------------------
 * |  0 |  0 |  0 |  1 |                  81.90 |                   0.02 |
 * -----------------------------------------------------------------------
 * |  0 |  0 |  1 |  0 |                 163.80 |                   0.04 |
 * -----------------------------------------------------------------------
 * |  0 |  0 |  1 |  1 |                 327.60 |                   0.08 |
 * -----------------------------------------------------------------------
 * |  0 |  1 |  0 |  0 |                 655.20 |                   0.16 |
 * -----------------------------------------------------------------------
 * |  0 |  1 |  0 |  1 |                1310.40 |                   0.32 |
 * -----------------------------------------------------------------------
 * |  0 |  1 |  1 |  0 |                2620.80 |                   0.64 |
 * -----------------------------------------------------------------------
 * |  0 |  1 |  1 |  1 |                5241.60 |                   1.28 |
 * -----------------------------------------------------------------------
 * |  1 |  0 |  0 |  0 |               10483.20 |                   2.56 |
 * -----------------------------------------------------------------------
 * |  1 |  0 |  0 |  1 |               20966.40 |                   5.12 |
 * -----------------------------------------------------------------------
 * |  1 |  0 |  1 |  0 |               41932.80 |                  10.24 |
 * -----------------------------------------------------------------------
 * |  1 |  0 |  1 |  1 |               83865.60 |                  20.48 |
 * -----------------------------------------------------------------------
 */
static unsigned char get_lsb_size_from_lux(float luxValue){
  if(luxValue <= 40.95){
    return 0x00;
  }else if(luxValue <= 81.90){
    return 0x01;
  }else if(luxValue <= 163.80){
    return 0x02;
  }else if(luxValue <= 327.60){
    return 0x03;
  }else if(luxValue <= 655.20){
    return 0x04;
  }else if(luxValue <= 1310.40){
    return 0x05;
  }else if(luxValue <= 2620.80){
    return 0x06;
  }else if(luxValue <= 5241.60){
    return 0x07;
  }else if(luxValue <= 10483.20){
    return 0x08;
  }else if(luxValue <= 20966.40){
    return 0x09;
  }else if(luxValue <= 41932.80){
    return 0x0A;
  } 
  return 0x0B; //luxValue <= 83865.60
}


/* --------------------------------------------------------------------------------------------------
 * @name float opt3001_get_lux(opt3001_settings_t * opt3001_settings)
 * --------------------------------------------------------------------------------------------------
 * @brief Get the current ambient light reading
 * @param opt3001_settings -> Pointer to the opt3001_settings_t struct
 * @return float -> The lux value of the ambient light. 0 on Error
 */
float opt3001_get_lux(opt3001_settings_t opt3001_settings){
  const unsigned char cfgReg = OPT3001_RESULT_REG;
  unsigned char rxReg[2] = {0};
  esp_err_t err_ck = i2c_master_write_read_device(opt3001_settings.i2c_port_number, opt3001_settings.i2c_address,
                     &cfgReg, 1, (uint8_t *)rxReg, 2, OPT3001_TIMEOUT_PERIOD);
  if(err_ck != ESP_OK){
    /* CHECK AND RESPOND IF THE READ DIDN'T WORK */
    switch(err_ck){
      /* REQUEST TIMED OUT */
      case ESP_ERR_TIMEOUT: return OPT3001_ERR_TIMEOUT_READING_LUX; break;
      /* I2C IS INVALID, SLAVE DIDN'T ACK OR A SETTINGS PARAM IS INCORRECT */
      default: return OPT3001_ERR_I2C_READING_LUX;
    }
  }
  return opt3001_calculate_float((unsigned short)((rxReg[0] << 8) | rxReg[1]));
}

/* --------------------------------------------------------------------------------------------------
 * @name static float opt3001_calculate_float(unsigned short lux_s)
 * --------------------------------------------------------------------------------------------------
 * @brief Convert the register data from bits to the lux float value
 * @param lux_s -> the register data to convert
 * @return The converted data
 */
static float opt3001_calculate_float(unsigned short lux_s){
  unsigned char luxExponent = 0;
  unsigned short luxMantissa = 0;
  float luxExp_f = 0;
  luxExponent = (unsigned char)((lux_s & 0xF000) >> 12); /* EXPONENT */
  luxMantissa = (lux_s & 0x0FFF);                        /* MANTISSA */    
  luxExp_f = (0.01 * pow(2, (float)luxExponent));
  return (luxExp_f * (float)luxMantissa);
}

/* --------------------------------------------------------------------------------------------------
 * @name esp_err_t opt3001_get_config(opt3001_settings_t * opt3001_settings, opt3001_config_t opt_cfg)
 * --------------------------------------------------------------------------------------------------
 * @brief Read the current config register
 * @param opt3001_settings -> Pointer to the opt3001_settings_t struct
 * @param opt_cfg - Device configuration from the opt3001_config_t struct
 * @return ESP_OK on success* 
 * @note The i2c bus must be setup and configured before
 *       calling this routine
 */
esp_err_t opt3001_get_config(opt3001_settings_t opt3001_settings, opt3001_config_t * opt_cfg){
  const unsigned char cfgReg = OPT3001_CONFIG_REG;
  unsigned char rxReg[2] = {0};
  esp_err_t err_ck = i2c_master_write_read_device(opt3001_settings.i2c_port_number, opt3001_settings.i2c_address,
                     &cfgReg, 1, (uint8_t *)rxReg, 2, OPT3001_TIMEOUT_PERIOD);
  if(err_ck == ESP_OK){
    opt_cfg->config_register = ((rxReg[0] << 8) | rxReg[1]);
    ESP_LOGD(LOG_TAG, "RX VALUE IS: 0x%02X", opt_cfg->config_register);
  }
  return err_ck;
}