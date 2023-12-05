/*
 * opt3001 HAL Usage Example
 * 
 * This should provide you with a decent example to use the opt3001.c Library I wrote
 * It includes using an interrupt pin, an ISR and an event task thread to handle the
 * incoming msgs. Do with as you please.
 *----------------------------------------------------------------------------------------
 * SPDX-FileCopyrightText: 2023 Open grStat https://github.com/grstat
 * SPDX-FileType: SOURCE
 * SPDX-FileContributor: Created by Adrian Borchardt
 * SPDX-License-Identifier: Apache 2.0
 * ----------------------------------------------------------------------------------------
 * NOTES ARE TAKEN FROM THE OPT3001 DATASHEET - SBOS853A - MARCH 2017–REVISED DECEMBER 2018
 * SEE https://www.ti.com/product/OPT3001 FOR MORE INFORMATION
 * ----------------------------------------------------------------------------------------
 */
#include <stdio.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_event.h>
#include <driver/i2c.h>
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include "opt3001/opt3001.h"

/* I2C CONFIGURATION */
#define I2C_SCL                   (22)        /* I2C SCL IO */
#define I2C_SDA                   (21)        /* I2C SDA IO */
#define I2C_PORT_MASTER           I2C_NUM_0   /* I2C PORT NUMBER */
#define I2C_MASTER_FREQ_HZ        (400000)    /* 400kHz */
#define I2C_MASTER_TX_BUF_DISABLE (0)         /* TWEAK IF REQUIRED */
#define I2C_MASTER_RX_BUF_DISABLE (0)         /* TWEAK IF REQUIRED */

/* TIMEOUT WAIT PERIOD FOR I2C COMS */
#define I2C_READ_TIMEOUT_PERIOD   ((TickType_t)200 / portTICK_PERIOD_MS)

/* INT PIN */
#define OPT3001_INT_PIN           (33)                    /* INTERRUPT PIN ATTACHED TO THE OPT3001 */
#define OPT3001_INT_PIN_SEL       (1ULL<<OPT3001_INT_PIN) /* CONFIG INT PIN SELECTION */

/* EVENT THREAD */
#define MONIT_EVENT_STACK_SZ      (2048)  /* MONITOR EVENT THREAD STACK SIZE */
#define MONIT_EVENT_PRIORITY      (15)    /* MONITOR EVENT PRIORITY 0=IDLE 25=MAX */

/* BIT FLAGS USED FOR GROUP EVENT WAITS */
#define BIT_INT_TRIGGER  BIT0   // INT HAS TRIGGERED THE ISR
#define BIT_DATA_READY   BIT1   // DATA IS READY TO READ
#define BIT_HIGH_THRESH  BIT2   // HIGH THRESHOLD LIMIT REACHED
#define BIT_LOW_THRESH   BIT3   // LOW THRESHOLD LIMIT REACHED

/* THE BIT FLAGS THE MONITOR THREAD WILL WAIT ON */
#define BIT_FLAGS_TO_WAIT_FOR   (BIT_INT_TRIGGER | BIT_DATA_READY | BIT_HIGH_THRESH | BIT_LOW_THRESH)

EventGroupHandle_t flags_h;   // THE EVENT GROUP HANDLE
TaskHandle_t monit_th;        // THE MONITOR TASK

/* ADDITIONAL SENSORS CAN BE ADDED HERE 
 * OPT3001 SUPPORTS UP TO 4 ON A SINGLE BUS 
 * BY CONFIGURING THE ADDR PIN */
opt3001_settings_t opt_set_a; //SENSOR A SETTINGS
opt3001_config_t opt_cfg_a;   //SENSOR A CONFIG

static bool i2c_init(void);
static void monit_evt_th(void* pvParameters);
static void print_config_value(opt3001_config_t opt_cfg);
void opt3001_int_isr(void* arg);

void app_main(void){
  // KEEP EVERYTHING GOING
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_event_loop_create_default());
  // CONFIGURE YOUR I2C BUS
  if(i2c_init()){

    /* DEVICE SETUP VALUES, DO THIS FOR EACH DEVICE ON THE BUS */
    opt_set_a.i2c_address = OPT3001_ADDR_GND;     // CONFIGURED I2C ADDRESS
    opt_set_a.i2c_port_number = I2C_PORT_MASTER;  // I2C PORT NUMBER

    /* There are two major types of interrupt reporting mechanism modes: latched window-style comparison mode and
     * transparent hysteresis-style comparison mode. The configuration register latch field (L) (see the configuration
     * register, bit 4 in the datasheed) controls which of these two modes is used. 
     * An end-of-conversion mode is also associated with each major mode type. The end-of-conversion mode 
     * is active when the two most significant bits of the threshold low register are set to 11b. 
     * The mechanisms report via the flag high and flag low fields, the conversion ready field, and the INT pin. */
    /* OPTION 1 */
    opt_set_a.eoc_indicator = OPT3001_END_OF_CONVERSION_ENABLED;  // GET A TRIGGER EVERY TIME A CONVERSION HAS COMPLETED 
                                                                  // NOTE: LOW THRESHOLD TRIGGERS DISABLED IN THIS MODE
    /* OPTION 2 */
    //opt_set_a.eoc_indicator = OPT3001_END_OF_CONVERSION_DISABLED;  // NO CONVERSION END TRIGGER 
                                                                     //LOW THRESHOLD TRIGGERS ARE ENABLED

    opt_set_a.feild_limits.high_limit = 5242.88;  // THE HIGH LIMIT TRIGGER VALUE, WHEN > AN INT OCCURS

    /* OPTION WHEN END OF CONVERSION IS SET */
    opt_set_a.feild_limits.low_limit = 0;         // SET TO ZERO WHEN END OF CONVERSION IS USED
    
    /* OPTION WHEN END OF CONVERSION NOT SET */
    //opt_set_a.feild_limits.low_limit = 80.88;   // THE LOW LIMIT TRIGGER VALUE, WHEN < AN INT OCCURS 
                                                  // (END OF CONVERSION DISABLED ONLY)

    /* DEVICE CONFIGURATION VALUES - DO THIS FOR EACH DEVICE */

    /* The fault count field instructs the device as to how many consecutive fault events are required
     * to trigger the interrupt reporting mechanisms: the INT pin, the flag high field (FH), and flag low
     * field (FL). The fault events are described in the latch field (L), flag high field (FH), and flag low
     * field (FL) descriptions.
     * ----------------------------------------------------------------------------
     * OPT3001_ONE_FAULT_INT_TRIGGER = One fault count (default)
     * OPT3001_TWO_FAULT_INT_TRIGGER = Two fault counts
     * OPT3001_FOUR_FAULT_INT_TRIGGER = Four fault counts
     * OPT3001_EIGHT_FAULT_INT_TRIGGER = Eight fault counts 
     * ---------------------------------------------------------------------------- */
    opt_cfg_a.fault_count_field = OPT3001_ONE_FAULT_INT_TRIGGER;

    /* The mask exponent field forces the result register exponent field (register 00h, bits E[3:0]) to
     * 0000b when the full-scale range is manually set, which can simplify the processing of the
     * result register when the full-scale range is manually programmed. This behavior occurs when
     * the mask exponent field is set to 1 and the range number field (RN[3:0]) is set to less than
     * 1100b. NOTE: Masking is only performed to the result register. When using the interrupt
     * reporting mechanisms, the result comparison with the low-limit and high-limit registers is
     * unaffected by the ME field. */
    opt_cfg_a.mask_exponent_field = OPT3001_DEFAULT_MASK;
    /* The polarity field controls the polarity or active state of the INT pin.
     * ----------------------------------------------------------------------------
     * OPT3001_INT_ACTIVE_LOW = The INT pin reports active low, pulling the pin low upon an interrupt event.
     * OPT3001_INT_ACTIVE_HIGH = Operation of the INT pin is inverted, where the INT pin reports active high, becoming high
     *                           impedance and allowing the INT pin to be pulled high upon an interrupt event.
     * ---------------------------------------------------------------------------- */
    opt_cfg_a.polarity_field = OPT3001_INT_ACTIVE_LOW;

    /* The latch field controls the functionality of the interrupt reporting mechanisms: the INT pin, the
     * flag high field (FH), and flag low field (FL). This bit selects the reporting style between a
     * latched window-style comparison and a transparent hysteresis-style comparison.
     * ----------------------------------------------------------------------------
     * OPT3001_LATCH_TRANSPARENT_HYSTERESIS = The device functions in transparent hysteresis-style comparison operation, 
     *    where the three interrupt reporting mechanisms directly reflect the comparison of the result register with the
     *    high- and low-limit registers with no user-controlled clearing event. 
     * OPT3001_LATCH_WINDOW_STYLE = The device functions in latched window-style comparison operation, latching the interrupt
     *    reporting mechanisms until a user-controlled clearing event.
     * ---------------------------------------------------------------------------- */
    opt_cfg_a.latch_field = OPT3001_LATCH_WINDOW_STYLE;
    opt_cfg_a.low_field_flag = 0;     // READ-ONLY, THIS FLAG IS SET WHEN LOW LIMIT IS TRIGGERED
    opt_cfg_a.high_field_flag = 0;    // READ-ONLY, THIS FLAG IS SET WHEN HIGH LIMIT IS TRIGGERED
    opt_cfg_a.conversion_ready = 0;   // READ-ONLY, THIS FLAG IS SET WHEN CONVERSION COMPLETES
    opt_cfg_a.overflow_flag = 0;      // READ-ONLY, THIS FLAG IS SET WHEN AN OVERFLOW CONDITION IS TRIGGERED

    /* The mode of conversion operation field controls whether the device is operating in continuous
     * conversion, single-shot, or low-power shutdown mode. The default is 00b (shutdown mode),
     * such that upon power-up, the device only consumes operational level power after appropriately
     * programming the device.
     * When single-shot mode is selected by writing 01b to this field, the field continues to read 01b
     * while the device is actively converting. When the single-shot conversion is complete, the mode
     * of conversion operation field is automatically set to 00b and the device is shut down.
     * When the device enters shutdown mode, either by completing a single-shot conversion or by a
     * manual write to the configuration register, there is no change to the state of the reporting flags
     * (conversion ready, flag high, flag low) or the INT pin. These signals are retained for
     * subsequent read operations while the device is in shutdown mode.
     * ----------------------------------------------------------------------------
     * OPT3001_CONVERSION_MODE_LOWPOWER = Shutdown (default)
     * OPT3001_CONVERSION_MODE_SINGLE_SHOT = Single-shot
     * OPT3001_CONVERSION_MODE_CONTINUOUS_A, OPT3001_CONVERSION_MODE_CONTINUOUS_B = Continuous conversions
     * ---------------------------------------------------------------------------- */
    opt_cfg_a.mode_of_conversion = OPT3001_CONVERSION_MODE_CONTINUOUS_B;
    /* The conversion time field determines the length of the light to digital conversion process. The
     * choices are 100 ms and 800 ms. A longer integration time allows for a lower noise
     * measurement. The conversion time also relates to the effective resolution of the data conversion process. The
     * 800-ms conversion time allows for the fully specified lux resolution. The 100-ms conversion
     * time with full-scale ranges above 0101b for E[3:0] in the result and configuration registers also
     * allows for the fully specified lux resolution. The 100-ms conversion time with full-scale ranges
     * below and including 0101b for E[3:0] can reduce the effective result resolution by up to three
     * bits, as a function of the selected full-scale range. Range 0101b reduces by one bit. Ranges
     * 0100b, 0011b, 0010b, and 0001b reduces by two bits. Range 0000b reduces by three bits.
     * The result register format and associated LSB weight does not change as a function of the conversion time.
     * ----------------------------------------------------------------------------
     * OPT3001_CONVERSION_TIME_100MS = 100 ms
     * OPT3001_CONVERSION_TIME_800MS = 800 ms 
     * ---------------------------------------------------------------------------- */
    opt_cfg_a.conversion_time = OPT3001_CONVERSION_TIME_800MS;

    /* The range number field selects the full-scale lux range of the device. The format of this field is
     * the same as the result register exponent field (E[3:0]); see Table 8. When RN[3:0] is set to
     * 1100b (0Ch), the device operates in automatic full-scale setting mode, as described in the
     * Automatic Full-Scale Setting Mode section. In this mode, the automatically chosen range is
     * reported in the result exponent (register 00h, E[3:0]).
     * The device powers up as 1100 in automatic full-scale setting mode. Codes 1101b, 1110b, and
     * 1111b (0Dh, 0Eh, and 0Fh) are reserved for future use.
     * NOTE: YOU CAN IMPLEMENT YOUR OWN SCALING IF YOU REQUIRE IT
     * ----------------------------------------------------------------------------
     * OPT3001_RANGE_AUTO_FULL_SCALE = 1100bb Automatic Full Scale
     * ---------------------------------------------------------------------------- */
    opt_cfg_a.range_number_feild = OPT3001_RANGE_AUTO_FULL_SCALE;

    /* SETUP AND CONFIGURE THE DEVICE */
    esp_err_t err_msg = opt3001_configure(opt_set_a, opt_cfg_a);
    if(err_msg != ESP_OK){
      // HANDLE YOUR ERRORS
      ESP_LOGE("MAIN", "ERROR 0x%02X", err_msg);
    }

    /* CONFIGURE GPIO INT PIN */
    gpio_config_t io_opt3001_int = {
      .intr_type = GPIO_INTR_NEGEDGE,  // MAKE SURE THIS MATCHES YOUR polarity_field CONFIG VALUE
      .mode = GPIO_MODE_INPUT,
      .pin_bit_mask = OPT3001_INT_PIN_SEL,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .pull_up_en = GPIO_PULLUP_ENABLE    
    };

    /* SET THE GPIO AND CONFIGURE THE ISR SERVICE 
     * THAT IS CALLED ON THE INT PIN TRIGGER */
    ESP_ERROR_CHECK(gpio_config(&io_opt3001_int));                                  // CONFIGURE GPIO
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));                // ACCEPT L1 VECTORS LOW PRIORITY
    ESP_ERROR_CHECK(gpio_isr_handler_add(OPT3001_INT_PIN, opt3001_int_isr, NULL));  // ADD HANDLER TO INT PIN

    /* CREATE THE EVENT GROUP */
    flags_h = xEventGroupCreate();

    /* CREATE THE EVENT THREAD THAT WILL MONITOR AND PROCESS REQUESTS */
    xTaskCreate(monit_evt_th, "MONIT_THREAD", MONIT_EVENT_STACK_SZ, NULL, MONIT_EVENT_PRIORITY, &monit_th);

    /* THIS IS HOW YOU PULL IN 
     * THE CURRENT DEVICE CONFIGURATION */
    err_msg = opt3001_get_config(opt_set_a, &opt_cfg_a);
    if(err_msg == ESP_OK){ print_config_value(opt_cfg_a); } //FOR DEBUG CAN BE REMOVED

    /* THIS IS HOW YOU CAN GET THE
     * CURRENT LIMITS CONFIGURED ON THE DEVICE */
    if(opt3001_get_limits(&opt_set_a) == ESP_OK){
      //DEBUG LOG
      ESP_LOGD("MAIN", "HIGH LIMIT: %.2f   LOW LIMIT: %.2f", opt_set_a.feild_limits.high_limit, opt_set_a.feild_limits.low_limit);
      /* YOU COULD DO YOUR EVALS ON YOUR LIMITS
       * HERE I JUST CHANGE THEIR VALUE TO SOMETHING 
       * OTHER THAN THE ORIGINAL CONFIGURATION */
      opt_set_a.feild_limits.high_limit = 5242.88;
      /* IF YOU HAVE END OF CONVERSION INT ENABLED AND SET A LOW VALUE
       * THE LIB WILL AUTOMAGICALLY ZERO OUT THE LOW LIMIT FEILD 
       * FOR SAFETY */
      opt_set_a.feild_limits.low_limit = 88.80;   // SHOULDN'T HAVE SET THIS SINCE END OF CONVERSION IS ENABLED, BUT NO PROBLEM!
    }

    if(opt3001_set_limits(opt_set_a) == ESP_OK){
      //DEBUG
      ESP_LOGD("MAIN", "NEW HIGH LIMIT: %.2f   NEW LOW LIMIT: %.2f", opt_set_a.feild_limits.high_limit, opt_set_a.feild_limits.low_limit);
      /* YOU CAN TEST YOUR LIMITS AND MAKE SURE EVERYTHING WENT WELL */
    }

    /* THE opt3001_get_lux FUNCTION REQUESTS THE CURRENT AMBIENT READING 
     * AND RETURNS THE LUX VALUE - THIS FUNCTION BLOCKS, */
    float lux_value = opt3001_get_lux(opt_set_a); // REQUEST THE LUX VALUE
    ESP_LOGI("MAIN", "LUX IS: %.2f", lux_value);  // DEBUG THE READ VALUE

    /* IF THE LUX VALUE IS NEGATIVE, AN ERROR CONDITION EXISTS
     * YOU CAN EVALUATE THE ERROR TO SEE WHAT MIGHT HAVE WENT WRONG */    
    if(lux_value == OPT3001_ERR_TIMEOUT_READING_LUX){
      // I2C TIMEOUT
      ESP_LOGE("MAIN", "TIMEOUT OCCURRED ATTEMPTING TO GET THE AMBIENT READING");
    }else if(lux_value == OPT3001_ERR_I2C_READING_LUX){
      // I2C ISSUE
      ESP_LOGE("MAIN", "AN I2C ERROR OCCURRED ATTEMPTING TO GET THE AMBIENT READING");
    }
    lux_value = 0;  // RESET

  } /** END - if(i2c_init()) - NOTE: YOU CAN ELSE THIS TO DIAG THE I2C INIT */

  /** DO ANYTHING ELSE YOU NEED TO BEFORE MAIN EXITS **/

} /** END - void app_main(void) */

/* PROCEDURE USED FOR DEBUG IT SIMPLY PRINTS 
 * THE CURRENT CONFIGURATION VALUES YOU HAVE STORED */
static void print_config_value(opt3001_config_t opt_cfg){
  ESP_LOGI("CONFIG", "FAULTS: %d\nMASK: %d\nPOLARITY: %d\nLATCH: %d\nLOW FLAG: %d\nHIGH FLAG: %d\nCONVERSION RDY: %d\nOVERFLOW: %d\nCONVERSION MODE: %d\nCONVERSION TIME: %d\nRANGE: %d",
            (int)opt_cfg_a.fault_count_field, (int)opt_cfg_a.mask_exponent_field,
            (int)opt_cfg_a.polarity_field, (int)opt_cfg_a.latch_field, (int)opt_cfg_a.low_field_flag,
            (int)opt_cfg_a.high_field_flag, (int)opt_cfg_a.conversion_ready, (int)opt_cfg_a.overflow_flag,
            (int)opt_cfg_a.mode_of_conversion, (int)opt_cfg_a.conversion_time, (int)opt_cfg_a.range_number_feild);
}

/* HERE IS THE EVENT MONITORING THREAD 
 * IT LOOPS FOREVER AND BLOCKS AT THE START
 * WAITING FOR TRIGGERS FROM THE EVENT GROUP FLAGS 
 * THEN IT EVALUATES THE CONFIGURATION AND 
 * PROCESSES THE EVENTS FROM THE OPT3001 */
static void monit_evt_th(void* pvParameters){
  float lux_value = 0;
  for (;;) {
    //WAITING ON ANY GROUP NOTIFICATION
    EventBits_t xBits = xEventGroupWaitBits(flags_h, BIT_FLAGS_TO_WAIT_FOR, true, false, portMAX_DELAY);
    if((unsigned char)xBits & BIT_INT_TRIGGER){
      /* DOESN'T REALLY MATTER WHAT BIT IS SET BY THE EVENT GROUP
       * WHAT MATTERS IS WHEN THE INTERRUPT IS TRIGGERED 
       * CURRENT CONFIG REGISTERS ARE READ SO 
       * THE FLAG CONDITIONS ON THE DEVICE CAN BE EVALUATED */
      if(opt3001_get_config(opt_set_a, &opt_cfg_a) == ESP_OK){  // GET THE CONFIG
        //print_config_value(opt_cfg_a); //PRINT THE CONFIG IF YOU WANT
        if(opt_cfg_a.conversion_ready){
          /* A CONVERSION IS COMPLETE, SO IT IS 
           * SAFE TO REQUEST THE CURRENT AMBIENT 
           * LIGHT READING 
           * NOTE: IF END OF CONVERSION IS NOT ENABLED THIS WAS
           *       NOT THE ORIGINAL TRIGGER THAT CAUSED THE INTERRUPT */
          lux_value = opt3001_get_lux(opt_set_a);
          ESP_LOGI("MONIT", "LUX IS: %.2f", lux_value);
        }

        if(opt_cfg_a.fault_count_field){
          /* THE FAULT COUNT HAS REACHED THE CONFIGURED LIMIT 
           * HERE ACTION CAN BE TAKEN ON ANY FAULTS THAT MAY OCCUR 
           * FOR NOW JUST PRINT THE COUNT FOR REFERENCE */
          ESP_LOGE("MONIT", "FAULT COUNT IS: %d", (int)opt_cfg_a.fault_count_field);
        }

        if(opt_cfg_a.low_field_flag){
          /* WHEN THE AMBIENT READING IS BELOW THE CONFIGURED 
           * LOW THRESHOLD, THIS FLAG WILL BE SET AND ACTION 
           * CAN BE TAKEN */
          ESP_LOGW("MONIT", "AMBIENT LIGHT IS BELOW: %.2f lux", opt_set_a.feild_limits.low_limit);
        }

        if(opt_cfg_a.high_field_flag){
          /* WHEN THE AMBIENT READING IS ABOVE THE CONFIGURED 
           * HIGH THRESHOLD, THIS FLAG WILL BE SET AND ACTION 
           * CAN BE TAKEN */          
          ESP_LOGW("MONIT", "AMBIENT LIGHT IS ABOVE: %.2f lux", opt_set_a.feild_limits.high_limit);
        }

        if(opt_cfg_a.overflow_flag){
          /* WHEN AN OVERFLOW OCCURS, DON'T RELY ON THE 
           * LAST LUX READING, JUST TOSS IT IN THE TRASH */
          ESP_LOGW("MONIT", "OVERFLOW CONDITION HAS OCCURRED");
        }
      } /** END - if(opt3001_get_config(opt_set_a, &opt_cfg_a) == ESP_OK) - NOTE: AN else CAN BE ADDED TO EVAL THE ERROR CONDITION */
    } /** END - if((unsigned char)xBits & BIT_INT_TRIGGER) */
  }
}

/* THIS IS THE ISR TO HANDLE INT PIN TRIGGERS 
 * FROM THE OPT3001 - IT WORKS BY SETTING THE 
 * EVENT GROUP BIT WITH BIT_INT_TRIGGER
 * WHICH CAUSES THE MONITOR THREAD TO SEE
 * THE EVENT HAS OCCURRED
 * 
 * REMEMBER: A QUICK ISR IS THE BEST ISR! 
 * */
void opt3001_int_isr(void* arg){
  BaseType_t xHPTAWAKE = pdFALSE;
  //SET THE EVENT GROUP BIT FOR INT RAISED
  if(xEventGroupSetBitsFromISR(flags_h, BIT_INT_TRIGGER, &xHPTAWAKE) != pdFAIL){
    //BIT WAS SET, REQUEST THE CONTEXT SWITCH
    portYIELD_FROM_ISR(xHPTAWAKE);
  }
}

/* THIS IS THE I2C INITALIZATION ROUTINE 
 * IT PREPS THE I2C_PORT_MASTER FOR USE 
 * - THIS CAN BE ARRANGED HOWEVER YOU LIKE */
static bool i2c_init(void){
  /* I2C MASTER MODE, PULLUPS ENABLED */
  i2c_config_t i2c_conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_SDA,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_io_num = I2C_SCL,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_MASTER_FREQ_HZ
  };

  /* CONFIGURE THE PORT */
  esp_err_t err = i2c_param_config(I2C_PORT_MASTER, &i2c_conf);
  if (err != ESP_OK) {
    ESP_LOGE("I2C", "ERROR CONFIGURING I2C PORT %d", err);
    return false;
  }

  /* LOAD THE DRIVER */
  err = i2c_driver_install(I2C_PORT_MASTER, i2c_conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
  if (err != ESP_OK) {
    if(err == ESP_ERR_INVALID_ARG){
      ESP_LOGE("I2C", "ERROR INSTALLING I2C DRIVER, INVALID ARGUMENT");
    }else if(err == ESP_FAIL){
      ESP_LOGE("I2C", "I2C DRIVER INSTALLATION FAILED!");
    }
    return false;
  }
  
  return true;
}