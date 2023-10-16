/**
 ****************************************************************************************
 *
 * @file user_periph_setup.h
 *
 * @brief Peripherals setup header file.
 *
 * Copyright (C) 2012-2020 Dialog Semiconductor.
 * This computer program includes Confidential, Proprietary Information
 * of Dialog Semiconductor. All Rights Reserved.
 *
 ****************************************************************************************
 */

#ifndef _USER_PERIPH_SETUP_H_
#define _USER_PERIPH_SETUP_H_

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "gpio.h"
#include "uart.h"
#include "spi.h"
#include "spi_flash.h"
#include "i2c.h"
#include "i2c_eeprom.h"

#define NB_DATA_BYTES                 14

/*
 * DEFINES
 ****************************************************************************************
 */


/****************************************************************************************/
/* UART2 configuration to use with arch_console print messages                          */
/****************************************************************************************/
// Define UART2 Tx Pad
#if defined (__DA14531__)
    #define UART2_TX_PORT           GPIO_PORT_0
    #define UART2_TX_PIN            GPIO_PIN_6
#else
    #define UART2_TX_PORT           GPIO_PORT_0
    #define UART2_TX_PIN            GPIO_PIN_4
#endif

// Define UART2 Settings
#define UART2_BAUDRATE              UART_BAUDRATE_115200
#define UART2_DATABITS              UART_DATABITS_8
#define UART2_PARITY                UART_PARITY_NONE
#define UART2_STOPBITS              UART_STOPBITS_1
#define UART2_AFCE                  UART_AFCE_DIS
#define UART2_FIFO                  UART_FIFO_EN
#define UART2_TX_FIFO_LEVEL         UART_TX_FIFO_LEVEL_0
#define UART2_RX_FIFO_LEVEL         UART_RX_FIFO_LEVEL_0

/****************************************************************************************/
/* SPI configuration                                                                    */
/****************************************************************************************/
// Define SPI Pads
#if defined (__DA14531__)
    #define SPI_EN_PORT             GPIO_PORT_0
    #define SPI_EN_PIN              GPIO_PIN_1

    #define SPI_CLK_PORT            GPIO_PORT_0
    #define SPI_CLK_PIN             GPIO_PIN_4

    #define SPI_DO_PORT             GPIO_PORT_0
    #define SPI_DO_PIN              GPIO_PIN_0

    #define SPI_DI_PORT             GPIO_PORT_0
    #define SPI_DI_PIN              GPIO_PIN_3

#elif !defined (__DA14586__)
    #define SPI_EN_PORT             GPIO_PORT_0
    #define SPI_EN_PIN              GPIO_PIN_3

    #define SPI_CLK_PORT            GPIO_PORT_0
    #define SPI_CLK_PIN             GPIO_PIN_0

    #define SPI_DO_PORT             GPIO_PORT_0
    #define SPI_DO_PIN              GPIO_PIN_6

    #define SPI_DI_PORT             GPIO_PORT_0
    #define SPI_DI_PIN              GPIO_PIN_5
#endif

// Define SPI Configuration
    #define SPI_MS_MODE             SPI_MS_MODE_MASTER
    #define SPI_CP_MODE             SPI_CP_MODE_0
    #define SPI_WSZ                 SPI_MODE_8BIT
    #define SPI_CS                  SPI_CS_0

#if defined (__DA14531__)
    #define SPI_SPEED_MODE          SPI_SPEED_MODE_4MHz
    #define SPI_EDGE_CAPTURE        SPI_MASTER_EDGE_CAPTURE
#else // (DA14585, DA14586)
    #define SPI_SPEED_MODE          SPI_SPEED_MODE_4MHz
#endif


/****************************************************************************************/
/* SPI Flash configuration                                                              */
/****************************************************************************************/
#if !defined (__DA14586__)
#define SPI_FLASH_DEV_SIZE          (256 * 1024)
#endif


/****************************************************************************************/
/* I2C configuration                                                                    */
/****************************************************************************************/
// Define I2C Pads
#define I2C_SCL_PORT                GPIO_PORT_0
#define I2C_SCL_PIN                 GPIO_PIN_2

#define I2C_SDA_PORT                GPIO_PORT_0
#define I2C_SDA_PIN                 GPIO_PIN_1

// Define I2C Configuration
#define I2C_SLAVE_ADDRESS           (0x50)
#define I2C_SPEED_MODE              I2C_SPEED_FAST
#define I2C_ADDRESS_MODE            I2C_ADDRESSING_7B
#define I2C_ADDRESS_SIZE            I2C_2BYTES_ADDR


/****************************************************************************************/
/* I2C EEPROM configuration                                                             */
/****************************************************************************************/
#define I2C_EEPROM_DEV_SIZE         (0x20000)
#define I2C_EEPROM_PAGE_SIZE        (256)


/****************************************************************************************/
/* Button configuration                                                                 */
/****************************************************************************************/
#if defined (__DA14531__)
    #define GPIO_BUTTON_PORT        GPIO_PORT_0
    #define GPIO_BUTTON_PIN         GPIO_PIN_5
#else
    #define GPIO_BUTTON_PORT        GPIO_PORT_1
    #define GPIO_BUTTON_PIN         GPIO_PIN_1
#endif



#define GPIO_SUOTA_PORT                GPIO_PORT_0
#define GPIO_SUOTA_PIN                 GPIO_PIN_9



///////////////////////////////////new
#define NTM88_CS_PORT                          GPIO_PORT_0
#define NTM88_CS_PIN                           GPIO_PIN_7
#define NTM88_CLK_PORT                         GPIO_PORT_0
#define NTM88_CLK_PIN                          GPIO_PIN_8
#define NTM88_MOSI_PORT                         GPIO_PORT_0
#define NTM88_MOSI_PIN                          GPIO_PIN_11
#define NTM88_MISO_PORT                         GPIO_PORT_0
#define NTM88_MISO_PIN                          GPIO_PIN_6
 // EVB_TEST

// Define SPI Configuration
#define NTM88_MS_MODE                                   SPI_MS_MODE_MASTER
#define NTM88_CP_MODE                                   SPI_CP_MODE_0
#define NTM88_WSZ                                       SPI_MODE_16BIT
#define NTM88_CS                                        SPI_CS_0
#define NTM88_SPEED_MODE                                SPI_SPEED_MODE_2MHz
#define NTM88_EDGE_CAPTURE                              SPI_MASTER_EDGE_CAPTURE




/****************************************************************************************/
/* Wake-up from hibernation configuration                                               */
/****************************************************************************************/
#if defined (__DA14531__)
    #define HIB_WAKE_UP_PORT        GPIO_PORT_0
    #define HIB_WAKE_UP_PIN         GPIO_PIN_3
    #define HIB_WAKE_UP_PIN_MASK    (1 << HIB_WAKE_UP_PIN)
#endif


/****************************************************************************************/
/* Proximity Reporter pad configuration                                                 */
/****************************************************************************************/
#define USE_BAT_LEVEL_ALERT         0

#if defined (__DA14531__)
    #define GPIO_ALERT_LED_PORT     GPIO_PORT_0
    #define GPIO_ALERT_LED_PIN      GPIO_PIN_9
    #define GPIO_BAT_LED_PORT       GPIO_PORT_0
    #define GPIO_BAT_LED_PIN        GPIO_PIN_8
#else
    #define GPIO_ALERT_LED_PORT     GPIO_PORT_1
    #define GPIO_ALERT_LED_PIN      GPIO_PIN_0
    #define GPIO_BAT_LED_PORT       GPIO_PORT_1
    #define GPIO_BAT_LED_PIN        GPIO_PIN_2
#endif

/***************************************************************************************/
/* Production debug output configuration                                               */
/***************************************************************************************/
#if PRODUCTION_DEBUG_OUTPUT
#if defined (__DA14531__)
    #define PRODUCTION_DEBUG_PORT   GPIO_PORT_0
    #define PRODUCTION_DEBUG_PIN    GPIO_PIN_7
#else
    #define PRODUCTION_DEBUG_PORT   GPIO_PORT_2
    #define PRODUCTION_DEBUG_PIN    GPIO_PIN_5
#endif
#endif

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

typedef struct /*_dspi_transfer*/  //__attribute__ ((__packed__))
{
 //   uint16_t txData[34];          /*!< Send buffer. */
      volatile uint8_t P_kPa;
      uint16_t rxData[NB_DATA_BYTES + 3];          /*!< Receive buffer. */
      uint16_t advData[NB_DATA_BYTES + 3];
      uint16_t count;
      volatile size_t dataSize; /*!< Transfer bytes. */

        uint32_t configFlags; /*!< Transfer transfer configuration flags; set from _dspi_transfer_config_flag_for_master if
                             the transfer is used for master or _dspi_transfer_config_flag_for_slave enumeration if the
                             transfer is used for slave.*/
} dspi_transfer_t;

extern dspi_transfer_t xfer;



typedef struct /*_dspi_transfer*/  //__attribute__ ((__packed__))
{

    volatile uint8_t suota_flag;
     bool checked;


} mode_s_t;

extern mode_s_t state;

/**
 ****************************************************************************************
 * @brief Enable pad and peripheral clocks assuming that peripheral power domain
 *        is down. The UART and SPI clocks are set.
 ****************************************************************************************
 */
void periph_init(void);

/**
 ****************************************************************************************
 * @brief Each application reserves its own GPIOs here.
 ****************************************************************************************
 */
void GPIO_reservations(void);

/**
 ****************************************************************************************
 * @brief Set gpio port function mode
 ****************************************************************************************
 */
void vfnSpiTriggerXfers (uint8_t u8nb_bytes);
void vfnSpiFillBufferRead (uint16_t u16address, uint8_t *i);
void vfnSpiFillBufferCmd (uint16_t u16data, uint8_t *i);
//void vfnSpiTriggerXfers (uint8_t u8nb_bytes);
static uint8_t u8SpiFillTxBuffer (void);
static bool bSpiGetParity (uint8_t parityByte);
static uint8_t u8SpiGetStatus (uint16_t u16SpiRsp);
static uint8_t u8SpiGetData (uint16_t u16SpiRsp);
static void vfnSpiFillBufferWrite (uint16_t u16data, uint8_t *i);
 static uint8_t hex2byte(char *hex);
static void uuid2hex(char *uuid, uint8_t *output);
void set_pad_functions(void);

#endif // _USER_PERIPH_SETUP_H_
