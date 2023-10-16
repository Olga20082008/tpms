/**
 ****************************************************************************************
 *
 * @file user_proxr.c
 *
 * @brief Proximity reporter project source code.
 *
 * Copyright (C) 2015-2021 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup APP
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"
#include "gapc_task.h"
#include "user_periph_setup.h"
#include "wkupct_quadec.h"
#include "app_easy_msg_utils.h"
#include "gpio.h"
#include "app_security.h"
#include "user_proxr.h"
#include "arch.h"
#include "arch_api.h"
#if defined (__DA14531__) && (defined (CFG_APP_GOTO_HIBERNATION) || defined (CFG_APP_GOTO_STATEFUL_HIBERNATION))
#include "arch_hibernation.h"
#endif
#include "app_task.h"
#include "app_proxr.h"

#if defined (__DA14531__)
#include "rtc.h"
#include "timer1.h"
#endif

#if (BLE_SUOTA_RECEIVER)
#include "app_suotar.h"
#endif

#if defined (CFG_SPI_FLASH_ENABLE)
#include "spi_flash.h"
#endif

  mode_s_t state  __SECTION_ZERO("retention_mem_area0");;
dspi_transfer_t xfer __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY



#define NB_DATA_BYTES                 14
/* Advertising payload field sizes */
#define FLAGS_LEN                                                               3
#define COMPANY_ID_LEN                                                          2
#define BEACON_TYPE_LEN                                                         2
#define UUID_LEN                                                                16

/* Fixed advertising fields */
#define FLAG_0                                                                  0x02
#define FLAG_1                                                                  0x01
#define FLAG_2                                                                  0x06
#define LENGTH                                                                  0x1A
#define TYPE                                                                    0xFF
#define COMPANY_ID_0                                                     0x4C
#define COMPANY_ID_1                                                     0x00
#define BEACON_TYPE_0                                                    0x02
#define BEACON_TYPE_1                                                    0x15

#define TYRE_POS                                                         8

#if defined (TX_POWER_2d5Bm)
#define MEASURED_POWER                                  0xC7
#if !defined (__DA14531__)
#error "Config error: Can not define TX_POWER_2d5Bm when device selected is DA14585 or DA14586."
#endif
#else
/* Output power 0dBm */
#define MEASURED_POWER                                        0xC5
#endif

/* User defined advertising fields */
//#define UUID_STR                                               "d4070339-6da4-4e50-a375-bade13be6daa"

#define UUID_STR                                                 "585CDE93-1B01-42CC-9A13-25009BEDC65A"
#define MAJOR                                                     0x0100
//#define MAJOR                                                   0x00
#define MINOR                                                     0x0000

/* Set the advertising rate */
#define ADV_INTERVAL_ms                     300
uint8_t P_kPa_m __SECTION_ZERO("retention_mem_area0"); //@RETENTION MEMORY;

#define ADDR_FIRST_DATA_BYTE           0xCF

        /* Address to write in order to let the NTM88 CPU resume */
#define ADDR_SPIOPS                             0x38

        /* To read N data bytes, N+1 transfers are required.
         * In our implementation, one additional transfer is required
         * at the end to clear the SPIOPS register in the NTM88 memory.
         * So in total, we will perform N+2 transfers.
         */
    #define NB_16BIT_XFERS  (NB_DATA_BYTES + 2)

     static uint8_t gau8TxData[NB_16BIT_XFERS * 2] = { 0 };
       // static uint8_t gau8RxData[NB_16BIT_XFERS * 2] = { 0 };



/* Apple Proximity Beacon Specification - Release R1  */
typedef struct
        __attribute__ ((__packed__)) {
                uint8_t flags[FLAGS_LEN];
                uint8_t length;
                uint8_t type;
                uint8_t company_id[COMPANY_ID_LEN];
                uint8_t beacon_type[BEACON_TYPE_LEN];
                uint8_t uuid[UUID_LEN];
                uint16_t major;
                uint16_t minor;
                uint8_t measured_power;

        // uint8_t  power;
        } ibeacon_adv_payload_t;



/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Handles APP_WAKEUP_MSG sent when device exits deep sleep. Triggered by button press.
 ****************************************************************************************
*/
static void app_wakeup_cb(void)
{
    // If state is not idle, ignore the message
    if (ke_state_get(TASK_APP) == APP_CONNECTABLE)
    {
        default_advertise_operation();
    }

   // app_wakeup_cb

   }
static void uuid2hex(char *uuid, uint8_t *output)
     {
             uint8_t in_ix;
             uint8_t out_ix;
             for (out_ix = 0, in_ix = 0; out_ix < UUID_LEN; out_ix++, in_ix += 2) {
                     if (uuid[in_ix] == '-') {
                             in_ix++;
                     }
                     output[out_ix] = hex2byte(&uuid[in_ix]);
             }
     }

     /**
      ****************************************************************************************
      * @brief Converts ASCII character to hex equivalent.
      * @return void
      ****************************************************************************************
      */
     static uint8_t hex2byte(char *hex)
     {
             uint8_t byte = 0;
             uint8_t digits = 2;

             while (digits--) {
                     uint8_t c = *hex++;
                     if (c >= '0' && c <= '9')
                             c = c - '0';
                     else if (c >= 'a' && c <= 'f')
                             c = c - 'a' + 10;
                     else if (c >= 'A' && c <= 'F')
                             c = c - 'A' + 10;
                     else
                             c = 0;
                     byte = byte << 4;
                     byte = byte | (c & 0x0F);
             }
             return byte;
     }



void user_app_adv_start(void)
      {

        ibeacon_adv_payload_t adv_payload;
        struct gapm_start_advertise_cmd *cmd ;

  //    switch (state.suota_flag)
      //        {
        //      case 0:
              //       app_easy_timer_cancel(app_adv_data_update_timer_used );

               cmd =   app_easy_gap_non_connectable_advertise_get_active();
                             //   cmd = app_easy_gap_undirected_advertise_get_active();

              adv_payload.flags[0] = FLAG_0;
              adv_payload.flags[1] = FLAG_1;
              adv_payload.flags[2] = FLAG_2;
              adv_payload.length = LENGTH;
              adv_payload.type = TYPE;
              adv_payload.company_id[0] = COMPANY_ID_0;
              adv_payload.company_id[1] = COMPANY_ID_1;
              adv_payload.beacon_type[0] = BEACON_TYPE_0;
              adv_payload.beacon_type[1] = BEACON_TYPE_1;


              uuid2hex((char *)UUID_STR, adv_payload.uuid);

              adv_payload.major = MAJOR;
              adv_payload.minor =  (uint16_t)((xfer.P_kPa *10 /101) * 0x100); //MINOR;


              adv_payload.measured_power = MEASURED_POWER;// xfer.P_kPa; //P_kPa_m;//MEASURED_POWER;



              memcpy(cmd->info.host.adv_data, &adv_payload, sizeof(ibeacon_adv_payload_t));
              cmd->info.host.adv_data_len = sizeof(ibeacon_adv_payload_t);


              cmd->intv_min = MS_TO_BLESLOTS(ADV_INTERVAL_ms);
              cmd->intv_max = MS_TO_BLESLOTS(ADV_INTERVAL_ms);

              if (xfer.count == 0){
                         cmd = NULL;                                             //  app_add_ad_struct(cmd, &mnf_data, sizeof(struct mnf_specific_data_ad_structure), 0);
                   //   app_easy_gap_undirected_advertise_start();
                      app_easy_gap_non_connectable_advertise_start();
                       }
              else
                      app_easy_gap_update_adv_data(cmd->info.host.adv_data,
                              cmd->info.host.adv_data_len,
                              NULL,
                              0);

               xfer.count += 1;


      //    break;

                /*             case 1:

              //      struct gapm_start_advertise_cmd *cmd ;


  app_adv_data_update_timer_used = app_easy_timer(APP_ADV_DATA_UPDATE_TO, adv_data_update_timer_cb);

 // struct gapm_start_advertise_cmd* cmd;
//        cmd->op.code = GAPM_ADV_NON_CONN;
  cmd = app_easy_gap_undirected_advertise_get_active();

  // Add manufacturer data to initial advertising or scan response data, if there is enough space
  app_add_ad_struct(cmd, &mnf_data, sizeof(struct mnf_specific_data_ad_structure), 1);




  app_easy_gap_undirected_advertise_start();

                               break;

                                         case 2:
                                   break;
                              }*/



      }



/**
 ****************************************************************************************
 * @brief Routine to resume system from sleep state.
 ****************************************************************************************
 */

static void app_resume_system_from_sleep(void)
{
#if !defined (__DA14531__)
    if (GetBits16(SYS_STAT_REG, PER_IS_DOWN))
    {
        periph_init();
    }
#endif

    if (arch_ble_ext_wakeup_get())
    {
        arch_set_sleep_mode(app_default_sleep_mode);
        arch_ble_force_wakeup();
        arch_ble_ext_wakeup_off();
        app_easy_wakeup();
    }
}

/**
 ****************************************************************************************
 * @brief Button press callback function. Registered in WKUPCT driver.
 ****************************************************************************************
 */

static void app_button_press_cb(void)
{
/*#if (BLE_PROX_REPORTER)
    if (alert_state.lvl != PROXR_ALERT_NONE)
    {
        app_proxr_alert_stop();
    }
#endif
    app_resume_system_from_sleep();
*/
// if(!state.suota_flag)
        user_app_adv_start();

   // default_advertise_operation();
   // app_button_enable();
}

void app_button_enable(void)
      {
        //      app_easy_wakeup_set(app_wakeup_cb);
              wkupct_register_callback(app_button_press_cb);

              if (!GPIO_GetPinStatus(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN))
                      {
                      wkupct_enable_irq(WKUPCT_PIN_SELECT(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN), // select pin (GPIO_BUTTON_PORT, GPIO_BUTTON_PIN)
                      WKUPCT_PIN_POLARITY(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN,
                              WKUPCT_PIN_POLARITY_HIGH), // polarity low
                                1, // 1 event
                                0); // debouncing time = 0
                  }
      }

#if (BLE_SUOTA_RECEIVER)
void on_suotar_status_change(const uint8_t suotar_event)
{
#if (!SUOTAR_SPI_DISABLE)
    uint8_t dev_id;

    // Release the SPI flash memory from power down
    spi_flash_release_from_power_down();

    // Disable the SPI flash memory protection (unprotect all sectors)
    spi_flash_configure_memory_protection(SPI_FLASH_MEM_PROT_NONE);

    // Try to auto-detect the SPI flash memory
    spi_flash_auto_detect(&dev_id);

    if (suotar_event == SUOTAR_END)
    {
        // Power down the SPI flash memory
        spi_flash_power_down();
    }
#endif
}
#endif

void user_app_on_disconnect(struct gapc_disconnect_ind const *param)
{
    default_app_on_disconnect(NULL);

#if (BLE_BATT_SERVER)
    app_batt_poll_stop();
#endif

#if (BLE_SUOTA_RECEIVER)
    // Issue a platform reset when it is requested by the suotar procedure
    if (suota_state.reboot_requested)
    {
        // Reboot request will be served
        suota_state.reboot_requested = 0;

        // Platform reset
        platform_reset(RESET_AFTER_SUOTA_UPDATE);
    }
#endif

#if BLE_PROX_REPORTER
    app_proxr_alert_stop();
#endif
}

#if defined (__DA14531__)

#if defined (CFG_EXT_SLEEP_WAKEUP_RTC) || defined (CFG_DEEP_SLEEP_WAKEUP_RTC)
/**
 ****************************************************************************************
 * @brief RTC interrupt handler routine for wakeup.
 ****************************************************************************************
*/

static void rtc_interrupt_hdlr(uint8_t event)
{
#if defined (CFG_EXT_SLEEP_WAKEUP_RTC)
    app_resume_system_from_sleep();
#endif
}

/**
 ****************************************************************************************
 * @brief Configure RTC to generate an interrupt after 10 seconds.
 ****************************************************************************************
*/
static void configure_rtc_wakeup(void)
{
    rtc_time_t alarm_time;

    // Init RTC
    rtc_reset();

    // Configure the RTC clock; RCX is the RTC clock source (14420 Hz)
    rtc_clk_config(RTC_DIV_DENOM_1000, 14420);
    rtc_clock_enable();

    rtc_config_t cfg = {.hour_clk_mode = RTC_HOUR_MODE_24H, .keep_rtc = 0};

    rtc_time_t time = {.hour_mode = RTC_HOUR_MODE_24H, .pm_flag = 0, .hour = 11,
                       .minute = 55, .sec = 30, .hsec = 00};

    // Alarm interrupt in ten seconds
    alarm_time = time;
    alarm_time.sec += 10;

    // Initialize RTC, set time and data, register interrupt handler callback function and enable seconds interrupt
    rtc_init(&cfg);

    // Start RTC
    rtc_set_time_clndr(&time, NULL);
    rtc_set_alarm(&alarm_time, NULL, RTC_ALARM_EN_SEC);

    // Clear pending interrupts
    rtc_get_event_flags();
    rtc_register_intr(rtc_interrupt_hdlr, RTC_INTR_ALRM);
#if defined (CFG_EXT_SLEEP_WAKEUP_RTC)
    app_easy_wakeup_set(app_wakeup_cb);
#endif
}
#endif

#if defined (CFG_EXT_SLEEP_WAKEUP_TIMER1) || defined (CFG_DEEP_SLEEP_WAKEUP_TIMER1)
/**
 ****************************************************************************************
 * @brief Timer1 interrupt handler routine for wakeup.
 ****************************************************************************************
*/

static void timer1_interrupt_hdlr(void)
{
#if defined (CFG_EXT_SLEEP_WAKEUP_TIMER1)
    app_resume_system_from_sleep();
#endif
}

/**
 ****************************************************************************************
 * @brief Configure Timer1 to generate an interrupt when it reaches its max value.
 ****************************************************************************************
*/
static void configure_timer1_wakeup(void)
{
    timer1_count_options_t count_options = {.input_clk = TIM1_CLK_SRC_LP,
                                            .free_run = TIM1_FREE_RUN_ON,
                                            .irq_mask = TIM1_IRQ_MASK_OFF,
                                            .count_dir = TIM1_CNT_DIR_UP,
                                            .reload_val = TIM1_RELOAD_MAX,
    };
    // Enable Timer1 interrupt
    timer1_enable_irq();
#if defined (CFG_EXT_SLEEP_WAKEUP_TIMER1)
    app_easy_wakeup_set(app_wakeup_cb);
#endif
    timer1_count_config(&count_options, timer1_interrupt_hdlr);

    // Start the Timer
    timer1_start();
}
#endif

#if defined (CFG_APP_GOTO_DEEP_SLEEP)
/**
 ****************************************************************************************
 * @brief Put the system into deep sleep mode. It demonstrates the deep sleep mode usage
 *        and how the system can wake up from it. The exit from the deep sleep state causes 
 *        a system reboot.
 * @note  The system can wake up from deep sleep by:
 *          - external wake up interrupt, caused e.g. by button press (properly configured GPIO pin)
 *          - power on reset, caused e.g. by button press (properly configured GPIO pin)
 *          - interrupt generated from RTC
 *          - interrupt generated from Timer1
 *        When the system exits deep sleep state, the boot process is triggered.
 *        The application code has to be programmed in an external memory resource or
 *        in the OTP memory, in order for the system to reboot properly.
 ****************************************************************************************
*/
static void put_system_into_deep_sleep(void)
{
#if defined (CFG_DEEP_SLEEP_WAKEUP_POR)
    // Configure button for POR
    GPIO_EnablePorPin(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, GPIO_POR_PIN_POLARITY_LOW, GPIO_GetPorTime());
#endif

#if defined (CFG_DEEP_SLEEP_WAKEUP_GPIO)
    wkupct_enable_irq(WKUPCT_PIN_SELECT(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN), // Select pin
                      WKUPCT_PIN_POLARITY(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, WKUPCT_PIN_POLARITY_LOW), // Polarity low
                      1, // 1 event
                      0); // debouncing time = 0
#endif

#if defined (CFG_DEEP_SLEEP_WAKEUP_RTC)
    configure_rtc_wakeup();
#endif

#if defined (CFG_DEEP_SLEEP_WAKEUP_TIMER1)
    configure_timer1_wakeup();
#endif

    // Go to deep sleep
    arch_set_deep_sleep(CFG_DEEP_SLEEP_RAM1,
                        CFG_DEEP_SLEEP_RAM2,
                        CFG_DEEP_SLEEP_RAM3,
                        CFG_DEEP_SLEEP_PAD_LATCH_EN);
}
#endif // (CFG_APP_GOTO_DEEP_SLEEP)

#else //__DA14531__

#if defined (CFG_APP_GOTO_DEEP_SLEEP)

/**
 ****************************************************************************************
 * @brief Put the system into deep sleep mode. It demonstrates the deep sleep mode usage
 *        and how the system can wake up from it. Once the system enters deep sleep state
 *        it retains NO RAM blocks. The exit from the deep sleep state causes a system
 *        reboot.
 * @note  The system can wake up from deep sleep by:
 *          - external wake up interrupt, caused e.g. by button press (properly configured GPIO pin)
 *          - power on reset, caused e.g. by button press (properly configured GPIO pin)
 *          - H/W reset button press or power cycle (at any time)
 *        When the system exits deep sleep state, the boot process is triggered.
 *        The application code has to be programmed in an external memory resource or
 *        in the OTP memory, in order for the system to reboot properly.
 ****************************************************************************************
*/
static void put_system_into_deep_sleep(void)
{
#if defined (CFG_DEEP_SLEEP_WAKEUP_GPIO)
    // Configure button for wake-up interrupt
    app_button_enable();

    // Set deep sleep - external interrupt wake up
    arch_set_deep_sleep(true);

#elif defined (CFG_DEEP_SLEEP_WAKEUP_POR)
    // Configure button for POR
    GPIO_EnablePorPin(GPIO_BUTTON_PORT, GPIO_BUTTON_PIN, GPIO_POR_PIN_POLARITY_LOW, GPIO_GetPorTime());

    // Set deep sleep - POR wake up
    arch_set_deep_sleep(false);

#else
    // Do nothing.
    // The system will eventually enter the selected Extended sleep state.
    // A button press will wake up the system if the respective GPIO is configured as a wake up interrupt.
#endif
}

#endif //(CFG_APP_GOTO_DEEP_SLEEP)

#endif

void app_advertise_complete(const uint8_t status)
{
    if ((status == GAP_ERR_NO_ERROR) || (status == GAP_ERR_CANCELED))
    {

#if (BLE_PROX_REPORTER)
        app_proxr_alert_stop();
#endif
    }

    if (status == GAP_ERR_CANCELED)
    {
        arch_ble_ext_wakeup_on();

#if defined (__DA14531__)
    // Configure PD_TIM
#if defined (CFG_EXT_SLEEP_WAKEUP_RTC) || defined (CFG_EXT_SLEEP_WAKEUP_TIMER1) || \
    defined (CFG_DEEP_SLEEP_WAKEUP_RTC) || defined (CFG_DEEP_SLEEP_WAKEUP_TIMER1)
        // Ensure PD_TIM is open
        SetBits16(PMU_CTRL_REG, TIM_SLEEP, 0);
        // Wait until PD_TIM is opened
        while ((GetWord16(SYS_STAT_REG) & TIM_IS_UP) != TIM_IS_UP);
#else
        // Close PD_TIM
        SetBits16(PMU_CTRL_REG, TIM_SLEEP, 1);
        // Wait until PD_TIM is closed
        while ((GetWord16(SYS_STAT_REG) & TIM_IS_DOWN) != TIM_IS_DOWN);
#endif
#endif

#if defined (CFG_APP_GOTO_DEEP_SLEEP)
        // Put system into deep sleep
        put_system_into_deep_sleep();
#elif defined (__DA14531__) && defined (CFG_APP_GOTO_HIBERNATION)
        // Put system into hibernation
        arch_set_hibernation(HIB_WAKE_UP_PIN_MASK,
                             CFG_HIBERNATION_RAM1,
                             CFG_HIBERNATION_RAM2,
                             CFG_HIBERNATION_RAM3,
                             CFG_HIBERNATION_REMAP,
                             CFG_HIBERNATION_PAD_LATCH_EN);
#elif defined (__DA14531__) && defined (CFG_APP_GOTO_STATEFUL_HIBERNATION)
        // Put system into stateful hibernation
        arch_set_stateful_hibernation(HIB_WAKE_UP_PIN_MASK,
                                      CFG_STATEFUL_HIBERNATION_RAM1,
                                      CFG_STATEFUL_HIBERNATION_RAM2,
                                      CFG_STATEFUL_HIBERNATION_RAM3,
                                      CFG_STATEFUL_HIBERNATION_REMAP,
                                      CFG_STATEFUL_HIBERNATION_PAD_LATCH_EN);
        #if (DEVELOPMENT_DEBUG)
            // Turn on the orange LED (D5 on the 376-18-B Motherboard)
            SetWord16(P09_MODE_REG, ((uint32_t) OUTPUT) | ((uint32_t) PID_GPIO));
            SetWord16(P0_SET_DATA_REG, 1 << GPIO_ALERT_LED_PIN);
            // Keep it on for a couple of seconds
            for (uint32_t i = 4*2000000; i != 0; i--)
            {
                __NOP();
            }
            // Turn it off
            SetWord16(P0_RESET_DATA_REG, 1 << GPIO_ALERT_LED_PIN);
        #endif // DEVELOPMENT_DEBUG

        // Configure button to trigger wake-up interrupt from extended sleep
        app_button_enable();
#elif defined (__DA14531__) && defined (CFG_EXT_SLEEP_WAKEUP_RTC)
        // Configure RTC to trigger wake-up interrupt from extended sleep
        configure_rtc_wakeup();
#elif defined (__DA14531__) && defined (CFG_EXT_SLEEP_WAKEUP_TIMER1)
        // Configure TIMER1 to trigger wake-up interrupt from extended sleep
        configure_timer1_wakeup();
#else
        // Configure button to trigger wake-up interrupt from extended sleep
        app_button_enable();
#endif
    }
}

/// @} APP
