/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb.h
  * @brief          : Header for usb.c file.
  *                   This file contains the functions to configure and send/
  *                   receive data via USB
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_H
#define __USB_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
//Common macros
#define EP0_BUFFER_SIZE                                 0x40
#define EP1_BUFFER_SIZE                                 0x02

#define USBD_PRODUCT_STRING_FS                          "STM32 Learning Interface"
#define USBD_MANUFACTURER_STRING                        "STMicroelectronics"
#define USB_SIZ_STRING_SERIAL                           0x1A
#define UID_BASE                                        0x1FFFF7E8UL    /*!< Unique device ID register base address */
#define DEVICE_ID1                                      (UID_BASE)
#define DEVICE_ID2                                      (UID_BASE + 0x4)
#define DEVICE_ID3                                      (UID_BASE + 0x8)

#define EPR_NON_TOGGLE_BITS                             USB_EPREG_MASK
#define PMA_BASE_ADDR                                   0x40006000
#define BTABLE_ADDRESS                                  0x00U

#define  CUSTOM_HID_REPORT_DESC                         0x22U

#define  USB_REQ_RECIPIENT_DEVICE                       0x00U
#define  USB_REQ_RECIPIENT_INTERFACE                    0x01U

#define  USB_REQ_TYPE_STANDARD                          0x00U
#define  USB_REQ_TYPE_CLASS                             0x20U
#define  USB_REQ_TYPE_MASK                              0x60U

#define  USB_REQ_SET_ADDRESS                            0x05U
#define  USB_REQ_GET_DESCRIPTOR                         0x06U
#define  USB_REQ_SET_CONFIGURATION                      0x09U

#define  USB_DESC_TYPE_DEVICE                           0x01U
#define  USB_DESC_TYPE_CONFIGURATION                    0x02U
#define  USB_DESC_TYPE_STRING                           0x03U
#define  USB_DESC_TYPE_DEVICE_QUALIFIER                 0x06U

#define  USBD_LANGID_STR                                0x00U
#define  USBD_MFC_STR                                   0x01U
#define  USBD_PRODUCT_STR                               0x02U
#define  USBD_SERIAL_STR                                0x03U
#define  USBD_STRING_DESC_SIZE                          0x100

// Macros related to language descriptors
#define  USB_LEN_LANGID_STR_DESC                        0x04U
#define  USB_DESC_TYPE_STRING                           0x03U
#define  USBD_LANGID_STRING                             1033

// Macros related to FS descriptors
#define  USB_DESC_TYPE_CONFIGURATION                    0x02U
#define  USB_CUSTOM_HID_CONFIG_DESC_SIZ                 0x29U
#define  USB_DESC_TYPE_INTERFACE                        0x04U
#define  CUSTOM_HID_DESCRIPTOR_TYPE                     0x21U
#define  USBD_CUSTOM_HID_REPORT_DESC_SIZE               0x03U
#define  USB_DESC_TYPE_ENDPOINT                         0x05U
#define  CUSTOM_HID_EPIN_ADDR                           0x81U
#define  CUSTOM_HID_EPIN_SIZE                           0x02U
#define  CUSTOM_HID_FS_BINTERVAL                        0x05U
#define  USB_DESC_TYPE_ENDPOINT                         0x05U
#define  CUSTOM_HID_EPOUT_ADDR                          0x01U
#define  CUSTOM_HID_EPOUT_SIZE                          0x02U
#define  CUSTOM_HID_FS_BINTERVAL                        0x05U

// Device descriptor related macros
#define  USB_DESC_TYPE_DEVICE                           0x01U
#define  USB_MAX_EP0_SIZE                               64U
#define  USBD_VID                                       1155
#define  USBD_PID_FS                                    22362
#define  USBD_IDX_MFC_STR                               0x01U
#define  USBD_IDX_PRODUCT_STR                           0x02U
#define  USBD_IDX_SERIAL_STR                            0x03U
#define  USBD_MAX_NUM_CONFIGURATION                     1

// Utility macros
#define USB_EP_REG(n)               (*(__IO uint16_t *)(&(USB)->EP0R + ((n) * 2U)))

#define MIN(a, b)                   (((a) < (b)) ? (a) : (b))

#define  SWAPBYTE(addr)             (((uint16_t)(*((uint8_t *)(addr)))) + \
                                    (((uint16_t)(*(((uint8_t *)(addr)) + 1U))) << 8U))
#define  LOBYTE(x)                  ((uint8_t)((x) & 0x00FFU))
#define  HIBYTE(x)                  ((uint8_t)(((x) & 0xFF00U) >> 8U))


#define PMA_ADDR_FROM_APP(n)          (((PMAWord_t*)n - (PMAWord_t*)PMA_BASE_ADDR) << 1)
#define APP_ADDR_FROM_PMA(n)          ((PMAWord_t*)PMA_BASE_ADDR + (n >> 1))

#define SET_EP_TX_STATUS(bEpNum, wState) \
  do { \
    uint16_t _wRegVal; \
    \
    _wRegVal = USB_EP_REG(bEpNum) & USB_EPTX_DTOGMASK; \
    /* toggle first bit ? */ \
    if ((USB_EPTX_DTOG1 & (wState))!= 0U) \
    { \
      _wRegVal ^= USB_EPTX_DTOG1; \
    } \
    /* toggle second bit ?  */ \
    if ((USB_EPTX_DTOG2 & (wState))!= 0U) \
    { \
      _wRegVal ^= USB_EPTX_DTOG2; \
    } \
    USB_EP_REG(bEpNum) =  (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX); \
  } while(0)

#define SET_EP_RX_STATUS(bEpNum,wState) \
  do { \
    uint16_t _wRegVal; \
    \
    _wRegVal = USB_EP_REG(bEpNum) & USB_EPRX_DTOGMASK; \
    /* toggle first bit ? */ \
    if ((USB_EPRX_DTOG1 & (wState))!= 0U) \
    { \
      _wRegVal ^= USB_EPRX_DTOG1; \
    } \
    /* toggle second bit ? */ \
    if ((USB_EPRX_DTOG2 & (wState))!= 0U) \
    { \
      _wRegVal ^= USB_EPRX_DTOG2; \
    } \
    USB_EP_REG(bEpNum) = (_wRegVal | USB_EP_CTR_RX | USB_EP_CTR_TX); \
  } while(0)

#define CLEAR_RX_EP_CTR(bEpNum) \
  do { \
    uint16_t _wRegVal; \
    \
    _wRegVal = USB_EP_REG(bEpNum) & (0x7FFFU & USB_EPREG_MASK); \
    \
    USB_EP_REG(bEpNum) = (_wRegVal | USB_EP_CTR_TX); \
  } while(0)

#define CLEAR_TX_EP_CTR(bEpNum) \
  do { \
    uint16_t _wRegVal; \
    \
    _wRegVal = USB_EP_REG(bEpNum) & (0xFF7FU & USB_EPREG_MASK); \
    \
    USB_EP_REG(bEpNum) = (_wRegVal | USB_EP_CTR_RX); \
  } while(0)

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

typedef enum {
    EP_TYPE_CTRL,
    EP_TYPE_INTR
} EPType_t;

typedef uint32_t                    PMAWord_t;

typedef struct {
    uint8_t  request_type;
    uint8_t  request;
    uint16_t value;
    uint16_t index;
    uint16_t length;
} usb_ctrl_req_t;

/* Exported functions prototypes ---------------------------------------------*/
/* USER CODE BEGIN EFP */
void init_usb(void);
void usb_send_data(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __USB_H */
