/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb.c
  * @brief          : Program to configure the USB and communicate to a host PC
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
/* Includes ------------------------------------------------------------------*/
#include "usb.h"
#include "uart.h"
#include "string.h"
#include "stdio.h"

#define PMA_BDT_ATTR    __attribute__((section(".pma,\"aw\",%nobits//"), used, aligned(8)))

extern PMAWord_t _pma_end;
static uint8_t usb_addr = 0;
static uint8_t data[2] = {0x7F, 0x7F};

typedef struct {
    PMAWord_t tx_addrs;
    PMAWord_t tx_count;
    PMAWord_t rx_addrs;
    PMAWord_t rx_count;
} usb_buff_desc_t;

static usb_buff_desc_t  PMA_BDT_ATTR buff_desc_table[2];
static PMAWord_t *pma_ptr = NULL;

void dump_pma() {
    uint16_t *start = (uint16_t*)PMA_BASE_ADDR;
    uart1_send_string("----------- BDT ------------");
    for(uint8_t i = 0; i < 8; i++) {
        uart1_send_string("0x%X 0x%04X%04X", start, *start, *(start+1));
        start += 2;
    }

    uart1_send_string("\r\n Addr: 0x4000601C-(64byte)---");
    start = (uint16_t*)(PMA_BASE_ADDR + 0x30);
    for(uint8_t i = 0; i < 16; i++) {
        uart1_send_string("0x%X 0x%04X%04X", start, *start, *(start+1));
        start += 2;
    }
    uart1_send_string("----------------------------\r\n");
}

void dump_data(char *str, uint8_t *data, uint8_t len) {
    char buff[len * 3];

    uart1_send_string("%s----------- DATA ------------", str);
    for(uint8_t i = 0; i < len; i++) {
        snprintf(buff + i*3, 4, "%02X ", data[i]);
    }
    uart1_send_string("%s", buff);
}

static PMAWord_t *allocate_pma_buffer(uint16_t len) {
    PMAWord_t *ret = pma_ptr;
    pma_ptr += (len + 1) >> 1;
    // uart1_send_string("Allocated: %X (%X), next: %X", ret, PMA_ADDR_FROM_APP(ret), pma_ptr);
    return ret;
}

static void configure_endpoint(uint8_t endpoint, EPType_t type, uint8_t ep_addr, uint8_t is_rx) {
    uint16_t num_block;
    uint16_t pkt_size = (endpoint == 0) ? EP0_BUFFER_SIZE : EP1_BUFFER_SIZE;

    // Clear the endpoint type field
    USB_EP_REG(endpoint) &= (~USB_EP_T_FIELD) & USB_EPREG_MASK ;

    // Writing 0 to CTR_TX and CTR_RX will clear those
    // bits but writing 1 doesn't have any effect.
    // So set the value CTR_RX and CTR_TX as one everytime
    // we write the data to EPnR register.
    if(type == EP_TYPE_CTRL) {
        USB_EP_REG(endpoint) |= (uint16_t)(USB_EP_CONTROL | USB_EP_CTR_RX | USB_EP_CTR_TX);
    } else if(type == EP_TYPE_INTR) {
        USB_EP_REG(endpoint) |= (uint16_t)(USB_EP_INTERRUPT | USB_EP_CTR_RX | USB_EP_CTR_TX);
    }

    // Set endpoint address, don't touch CTR_TX and CTR_RX bits
    USB_EP_REG(endpoint) |= (uint16_t)(ep_addr | USB_EP_CTR_TX | USB_EP_CTR_RX);

    if(0 != is_rx) {

        // Set receive buffer address for the endpoint
        if(buff_desc_table[endpoint].rx_addrs == 0) {
            buff_desc_table[endpoint].rx_addrs = PMA_ADDR_FROM_APP(allocate_pma_buffer(pkt_size));
        }

        // Set maximum packet size that the endpoint can hold
        if(pkt_size > 62) {
            // BL_SIZE = 1 so that the counter value
            // become number of block * 64
            num_block = pkt_size / 64;
            buff_desc_table[endpoint].rx_count = ((num_block << 10) | USB_COUNT0_RX_BLSIZE);
        } else {
            num_block = pkt_size / 2;
            buff_desc_table[endpoint].rx_count = (num_block << 10);
        }

        // Clear DTOG_RX bit
        if(0 != (USB_EP_REG(endpoint) & USB_EP_DTOG_RX)) {
            USB_EP_REG(endpoint) |= (uint16_t)(USB_EP_DTOG_RX | USB_EP_CTR_TX | USB_EP_CTR_RX);
        }

        SET_EP_RX_STATUS(endpoint, USB_EP_RX_VALID);
    } else {
        // Set transmit buffer address for the endpoint
        if(buff_desc_table[endpoint].tx_addrs == 0) {
            buff_desc_table[endpoint].tx_addrs = PMA_ADDR_FROM_APP(allocate_pma_buffer(pkt_size));
        }

        buff_desc_table[endpoint].tx_count = 0;

        // Clear DTOG_TX bit
        if(0 != (USB_EP_REG(endpoint) & USB_EP_DTOG_TX)) {
            USB_EP_REG(endpoint) |= (uint16_t)(USB_EP_DTOG_TX | USB_EP_CTR_TX | USB_EP_CTR_RX);
        }

        SET_EP_RX_STATUS(endpoint, USB_EP_TX_NAK);
    }
}

static void deconfigure_endpoint(uint8_t endpoint, uint8_t is_rx) {
    if(0 != is_rx) {
        // Clear DTOG_RX bit
        if(0 != (USB_EP_REG(endpoint) & USB_EP_DTOG_RX)) {
            USB_EP_REG(endpoint) |= (uint16_t)(USB_EP_DTOG_RX | USB_EP_CTR_TX | USB_EP_CTR_RX);
        }

        // Set STAT_RX as disabled
        uint16_t stat_rx = USB_EP_REG(endpoint) & USB_EPRX_STAT;
        USB_EP_REG(endpoint) |= (stat_rx | USB_EP_CTR_RX | USB_EP_CTR_TX);
    } else {
        // Clear DTOG_TX bit
        if(0 != (USB_EP_REG(endpoint) & USB_EP_DTOG_TX)) {
            USB_EP_REG(endpoint) |= (uint16_t)(USB_EP_DTOG_TX | USB_EP_CTR_TX | USB_EP_CTR_RX);
        }

        // Set STAT_TX as disabled
        uint16_t stat_tx = USB_EP_REG(endpoint) & USB_EPTX_STAT;
        USB_EP_REG(endpoint) |= (stat_tx | USB_EP_CTR_RX | USB_EP_CTR_TX);
    }
}

static void usb_reset(void) {
    memset(buff_desc_table, 0, sizeof(buff_desc_table));
    pma_ptr = &_pma_end;

    // Open control endpoint
    configure_endpoint(0, EP_TYPE_CTRL, 0, 0);  // For TX endpoint
    configure_endpoint(0, EP_TYPE_CTRL, 0, 1);  // For RX endpoint

    // Close custom endpoint
    deconfigure_endpoint(1, 0);                 // For TX endpoint
    deconfigure_endpoint(1, 1);                 // For RX endpoint

    USB->DADDR = (uint16_t)USB_DADDR_EF;

    USB->CNTR = (uint16_t)(USB_CNTR_CTRM  | USB_CNTR_WKUPM |
                           USB_CNTR_SUSPM | USB_CNTR_ERRM |
                           USB_CNTR_SOFM | USB_CNTR_ESOFM);
}

static inline uint16_t get_rx_count(uint8_t endpoint) {
    if(endpoint == 0) {
        return (buff_desc_table[endpoint].rx_count & 0x3FF);
    } else {
        return 0;
    }
}

static void read_data_from_pma(uint16_t src, uint8_t* dst, uint16_t len) {
    uint16_t count = len >> 1;
    uint16_t read_val;

    __IO uint16_t *pma_addr = (__IO uint16_t*)(PMA_BASE_ADDR + 2 * src);
    for(; count != 0; count--) {
        read_val = *pma_addr;
        pma_addr++;
        pma_addr++;
        *dst = (uint8_t)(read_val & 0xFF);
        dst++;
        *dst = (uint8_t)((read_val >> 8) & 0xFF);
        dst++;
    }
}

uint8_t dev_desc[0x12]  __attribute__ ((aligned (4))) =
{
  0x12,                       /*bLength */
  USB_DESC_TYPE_DEVICE,       /*bDescriptorType*/
  0x00,                       /*bcdUSB */
  0x02,
  0x00,                       /*bDeviceClass*/
  0x00,                       /*bDeviceSubClass*/
  0x00,                       /*bDeviceProtocol*/
  USB_MAX_EP0_SIZE,           /*bMaxPacketSize*/
  LOBYTE(USBD_VID),           /*idVendor*/
  HIBYTE(USBD_VID),           /*idVendor*/
  LOBYTE(USBD_PID_FS),        /*idProduct*/
  HIBYTE(USBD_PID_FS),        /*idProduct*/
  0x00,                       /*bcdDevice rel. 2.00*/
  0x02,
  USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
  USBD_IDX_PRODUCT_STR,       /*Index of product string*/
  USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
  USBD_MAX_NUM_CONFIGURATION  /*bNumConfigurations*/
};

uint8_t fs_config[41] __attribute__ ((aligned (4))) = {
  0x09, /* bLength: Configuration Descriptor size */
  USB_DESC_TYPE_CONFIGURATION, /* bDescriptorType: Configuration */
  USB_CUSTOM_HID_CONFIG_DESC_SIZ,
  /* wTotalLength: Bytes returned */
  0x00,
  0x01,         /*bNumInterfaces: 1 interface*/
  0x01,         /*bConfigurationValue: Configuration value*/
  0x00,         /*iConfiguration: Index of string descriptor describing
  the configuration*/
  0xC0,         /*bmAttributes: bus powered */
  0x32,         /*MaxPower 100 mA: this current is used for detecting Vbus*/

  /************** Descriptor of CUSTOM HID interface ****************/
  /* 09 */
  0x09,         /*bLength: Interface Descriptor size*/
  USB_DESC_TYPE_INTERFACE,/*bDescriptorType: Interface descriptor type*/
  0x00,         /*bInterfaceNumber: Number of Interface*/
  0x00,         /*bAlternateSetting: Alternate setting*/
  0x02,         /*bNumEndpoints*/
  0x03,         /*bInterfaceClass: CUSTOM_HID*/
  0x00,         /*bInterfaceSubClass : 1=BOOT, 0=no boot*/
  0x00,         /*nInterfaceProtocol : 0=none, 1=keyboard, 2=mouse*/
  0,            /*iInterface: Index of string descriptor*/
  /******************** Descriptor of CUSTOM_HID *************************/
  /* 18 */
  0x09,         /*bLength: CUSTOM_HID Descriptor size*/
  CUSTOM_HID_DESCRIPTOR_TYPE, /*bDescriptorType: CUSTOM_HID*/
  0x11,         /*bCUSTOM_HIDUSTOM_HID: CUSTOM_HID Class Spec release number*/
  0x01,
  0x00,         /*bCountryCode: Hardware target country*/
  0x01,         /*bNumDescriptors: Number of CUSTOM_HID class descriptors to follow*/
  0x22,         /*bDescriptorType*/
  USBD_CUSTOM_HID_REPORT_DESC_SIZE,/*wItemLength: Total length of Report descriptor*/
  0x00,
  /******************** Descriptor of Custom HID endpoints ********************/
  /* 27 */
  0x07,          /*bLength: Endpoint Descriptor size*/
  USB_DESC_TYPE_ENDPOINT, /*bDescriptorType:*/

  CUSTOM_HID_EPIN_ADDR,     /*bEndpointAddress: Endpoint Address (IN)*/
  0x03,          /*bmAttributes: Interrupt endpoint*/
  CUSTOM_HID_EPIN_SIZE, /*wMaxPacketSize: 2 Byte max */
  0x00,
  CUSTOM_HID_FS_BINTERVAL,          /*bInterval: Polling Interval */
  /* 34 */

  0x07,          /* bLength: Endpoint Descriptor size */
  USB_DESC_TYPE_ENDPOINT, /* bDescriptorType: */
  CUSTOM_HID_EPOUT_ADDR,  /*bEndpointAddress: Endpoint Address (OUT)*/
  0x03, /* bmAttributes: Interrupt endpoint */
  CUSTOM_HID_EPOUT_SIZE,  /* wMaxPacketSize: 2 Bytes max  */
  0x00,
  CUSTOM_HID_FS_BINTERVAL,  /* bInterval: Polling Interval */
  /* 41 */
};

uint8_t lang_desc[USB_LEN_LANGID_STR_DESC]  __attribute__ ((aligned (4))) = {
     USB_LEN_LANGID_STR_DESC,
     USB_DESC_TYPE_STRING,
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING)
};

uint8_t serial_str[USB_SIZ_STRING_SERIAL] __attribute__ ((aligned (4))) = {
    USB_SIZ_STRING_SERIAL,
    USB_DESC_TYPE_STRING,
};

static uint8_t report_desc[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __attribute__ ((aligned (4))) = {
  0xA1, 0x01,
  0xC0
};

uint8_t str_desc[USBD_STRING_DESC_SIZE]  __attribute__ ((aligned (4)));

void parse_ctrl_msg(uint8_t *data, usb_ctrl_req_t *req) {
    req->request_type = *(uint8_t *)(data);
    req->request = *(uint8_t *)(data + 1U);
    req->value = SWAPBYTE(data + 2U);
    req->index = SWAPBYTE(data + 4U);
    req->length = SWAPBYTE(data + 6U);
}

static void write_data_to_pma(uint8_t* src, uint16_t dst, uint16_t len) {
    uint16_t count = (len + 1) >> 1;
    uint16_t write_val;

    __IO uint16_t *pma_addr = (__IO uint16_t*)(PMA_BASE_ADDR + 2 * dst);
    for(; count != 0; count--) {
        write_val = src[0];
        write_val |= src[1] << 8;
        *pma_addr = write_val;
        pma_addr++;
        pma_addr++;
        src++;
        src++;
    }

}

void usb_ctrl_send_data(uint8_t endpoint, uint8_t* buff, uint16_t len) {
    write_data_to_pma(buff, buff_desc_table[endpoint].tx_addrs, len);
    buff_desc_table[endpoint].tx_count = len;
    SET_EP_TX_STATUS(endpoint, USB_EP_TX_VALID);
}

static uint8_t get_len(uint8_t *buf) {
    uint8_t  len = 0U;

    while (*buf != '\0') {
        len++;
        buf++;
    }

    return len;
}

void convert_str_to_desc(uint8_t *desc, uint8_t *unicode, uint16_t *len) {
    uint8_t idx = 0U;

    if (desc != NULL) {
        *len = get_len(desc) * 2U + 2U;
        unicode[idx++] = *(uint8_t *)(void *)len;
        unicode[idx++] = USB_DESC_TYPE_STRING;

        while (*desc != '\0') {
            unicode[idx++] = *desc++;
            unicode[idx++] =  0U;
        }
    }
}
static void int_to_unicode(uint32_t value, uint8_t * pbuf, uint8_t len) {
  uint8_t idx = 0;

  for (idx = 0; idx < len; idx++) {
    if (((value >> 28)) < 0xA) {
      pbuf[2 * idx] = (value >> 28) + '0';
    } else {
      pbuf[2 * idx] = (value >> 28) + 'A' - 10;
    }

    value = value << 4;

    pbuf[2 * idx + 1] = 0;
  }
}

static void get_serial_num() {
  uint32_t device_serial_0;
  uint32_t device_serial_1;
  uint32_t device_serial_2;

  device_serial_0 = *(uint32_t *) DEVICE_ID1;
  device_serial_1 = *(uint32_t *) DEVICE_ID2;
  device_serial_2 = *(uint32_t *) DEVICE_ID3;

  device_serial_0 += device_serial_2;

  if (device_serial_0 != 0) {
    int_to_unicode(device_serial_0, &serial_str[2], 8);
    int_to_unicode(device_serial_1, &serial_str[18], 4);
  }
}


void set_usb_config(uint8_t cfg_idx) {
    configure_endpoint(1, EP_TYPE_INTR, 1, 0);
    configure_endpoint(1, EP_TYPE_INTR, 1, 1);
    usb_ctrl_send_data(0, NULL, 0);
}

static void process_string_request(usb_ctrl_req_t *req) {
    uint8_t *buff = NULL;
    uint16_t len;

    switch(req->value & 0xFF) {
        case USBD_LANGID_STR:
            // 80 06 00 03 00 00 FF 00
            buff = lang_desc;
            len = sizeof(lang_desc);
            usb_ctrl_send_data(0, buff, len);
            break;

        case USBD_MFC_STR:
            // 80 06 01 03 09 04 FF 00
            convert_str_to_desc((uint8_t*)USBD_MANUFACTURER_STRING, str_desc, &len);
            buff = str_desc;
            usb_ctrl_send_data(0, buff, len);
            break;

        case USBD_PRODUCT_STR:
            // 80 06 02 03 09 04 FF 00
            convert_str_to_desc((uint8_t*)USBD_PRODUCT_STRING_FS, str_desc, &len);
            buff = str_desc;
            usb_ctrl_send_data(0, buff, len);
            break;

        case USBD_SERIAL_STR:
            // 80 06 03 03 09 04 FF 00
            len = USB_SIZ_STRING_SERIAL;
            get_serial_num();
            buff = serial_str;
            usb_ctrl_send_data(0, buff, len);
            break;
    }
}

void process_descriptor_request(usb_ctrl_req_t *req) {
    // Get device descriptor request
    uint8_t *buff = NULL;
    uint16_t len;

    switch(req->value >> 8) {
        case USB_DESC_TYPE_DEVICE:
            // 80 06 00 01 00 00 40 00
            buff = dev_desc;
            len = sizeof(dev_desc);
            usb_ctrl_send_data(0, buff, len);
            break;

        case USB_DESC_TYPE_CONFIGURATION:
            // 80 06 00 02 00 00 09 00
            buff = fs_config;
            len = MIN(req->length, sizeof(fs_config));
            usb_ctrl_send_data(0, buff, len);
            break;

        case USB_DESC_TYPE_STRING:
            process_string_request(req);
            break;

        case USB_DESC_TYPE_DEVICE_QUALIFIER:
            // 80 06 00 06 00 00 0A 00
            SET_EP_TX_STATUS(0, USB_EP_TX_STALL);
            SET_EP_RX_STATUS(0, USB_EP_RX_STALL);
            break;
    }

}

static void process_std_request(usb_ctrl_req_t *req) {
    switch(req->request) {
        case USB_REQ_SET_ADDRESS:
            // 00 05 02 00 00 00 00 00
            usb_addr = req->value & 0x7F;
            usb_ctrl_send_data(0, NULL, 0);
            break;

        case USB_REQ_GET_DESCRIPTOR:
            // Get descriptor request
            process_descriptor_request(req);
            break;

        case USB_REQ_SET_CONFIGURATION:
            // 00 09 01 00 00 00 00 00
            uint8_t cfg_idx = req->value;
            set_usb_config(cfg_idx);
            break;
    }
}

static void process_setup_messages() {
    usb_ctrl_req_t request;
    uint16_t xfer_count;
    const uint8_t endpoint = 0;
    uint8_t xfer_data[16] = {0};
    uint8_t *buff = NULL;
    uint16_t len;

    // Get a setup packet
    xfer_count = get_rx_count(endpoint);
    read_data_from_pma(buff_desc_table[endpoint].rx_addrs, xfer_data, xfer_count);
    CLEAR_RX_EP_CTR(endpoint);
    parse_ctrl_msg(xfer_data, &request);
    switch(request.request_type & 0x1F) {
        case USB_REQ_RECIPIENT_DEVICE:
            // Recepient is a device
            if(USB_REQ_TYPE_STANDARD == (request.request_type & USB_REQ_TYPE_MASK)) {
                // Request type is standard
                process_std_request(&request);
            }
            break;

        case USB_REQ_RECIPIENT_INTERFACE:
            switch(request.request_type & USB_REQ_TYPE_MASK) {
                case USB_REQ_TYPE_STANDARD:
                    if(USB_REQ_GET_DESCRIPTOR == request.request) {
                        if(CUSTOM_HID_REPORT_DESC == (request.value >> 8)) {
                            // 81 06 00 22 00 00 03 00
                            buff = report_desc;
                            len = MIN(request.length, 163);
                            usb_ctrl_send_data(0, buff, len);
                        }
                    }
                    break;

                case USB_REQ_TYPE_CLASS:
                    // 21 0A 00 00 00 00 00 00
                    usb_ctrl_send_data(0, NULL, 0);
                    break;
            }
            break;
    }
}

static void process_control_messages() {
    uint16_t istr_val;
    const uint8_t endpoint = 0;
    uint16_t ep_reg_val;

    istr_val = USB->ISTR;

    if(0 == (istr_val & USB_ISTR_DIR)) {
        // DIR = 0 means CTR_TX = 1; IN transaction
        CLEAR_TX_EP_CTR(endpoint);

        if(usb_addr != 0) {
            USB->DADDR = (uint16_t)(usb_addr | USB_DADDR_EF);
            usb_addr = 0;
            SET_EP_TX_STATUS(0, USB_EP_TX_STALL);
        } else {
            SET_EP_TX_STATUS(0, USB_EP_TX_STALL);
            SET_EP_RX_STATUS(0, USB_EP_RX_VALID);
        }
    } else {
        // If DIR = 1 & CTR_RX which means a SETUP
        // transaction interrupt or OUT transaction
        // interrupt is pending

        // If DIR = 1 & (CTR_RX | CTR_TX) means, both
        // TX and RX transaction interrupts are pending
        ep_reg_val = USB_EP_REG(endpoint);

        if(ep_reg_val & USB_EP_SETUP) {
            process_setup_messages();
        } else if(0 != (ep_reg_val & USB_EP_CTR_RX)) {
            CLEAR_RX_EP_CTR(endpoint);
            buff_desc_table[endpoint].rx_count = ((1 << 10) | USB_COUNT0_RX_BLSIZE);
            SET_EP_RX_STATUS(endpoint, USB_EP_RX_VALID);
        }
    }
}

void service_correct_transfer_intr() {
    uint8_t endpoint;
    uint16_t istr_val;
    uint16_t ep_reg_val;
    uint16_t xfer_count;
    uint8_t xfer_data[16] = {0};

    while (USB->ISTR & USB_ISTR_CTR) {
        istr_val = USB->ISTR;
        endpoint = istr_val & USB_ISTR_EP_ID;
        if(endpoint == 0) {
            process_control_messages();
        } else {
            ep_reg_val = USB_EP_REG(endpoint);

            if((ep_reg_val & USB_EP_CTR_RX) != 0) {
                CLEAR_RX_EP_CTR(endpoint);
                xfer_count = buff_desc_table[endpoint].rx_count & 0x3FF;
                read_data_from_pma(buff_desc_table[endpoint].rx_addrs, xfer_data, xfer_count);
                uart1_send_string("rx from : %d - %s", xfer_count, xfer_data);
                SET_EP_RX_STATUS(endpoint, USB_EP_RX_VALID);
            } else if((ep_reg_val & USB_EP_CTR_TX) != 0) {
                CLEAR_TX_EP_CTR(endpoint);
            }
        }
    }
}
void USB_LP_CAN1_RX0_IRQHandler() {
    volatile uint16_t usb_status = USB->ISTR;

    if(usb_status & USB_ISTR_RESET) {
        usb_reset();
        USB->ISTR &= ~USB_ISTR_RESET;
        return;
    }
    if(usb_status & USB_ISTR_SOF) {
        USB->ISTR &= ~USB_ISTR_SOF;
        return;
    }

    if(usb_status & USB_ISTR_ESOF) {
        USB->ISTR &= ~USB_ISTR_ESOF;
        return;
    }

    if (usb_status & USB_ISTR_SUSP) {
        USB->ISTR &= ~USB_ISTR_SUSP;
        return;
    }

    if (usb_status & USB_ISTR_WKUP) {
        USB->ISTR &= ~USB_ISTR_WKUP;
        return;
    }

    if (usb_status & USB_ISTR_ERR) {
        USB->ISTR &= ~USB_ISTR_ERR;
        return;
    }

    if (usb_status & USB_ISTR_PMAOVR) {
        USB->ISTR &= ~USB_ISTR_PMAOVR;
        return;
    }

    if(USB->ISTR & USB_ISTR_CTR) {
        service_correct_transfer_intr();
    }
}

void init_usb(void) {
    // Enable USB Clock
    RCC->APB1ENR |= RCC_APB1ENR_USBEN;

    // Enable the interrupt
    uint32_t usb_priority_grp = NVIC_GetPriorityGrouping();
    NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, NVIC_EncodePriority(usb_priority_grp, 0, 0));
    NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

    USB->CNTR = (uint16_t)USB_CNTR_FRES;
    USB->CNTR = 0U;
    USB->BTABLE = BTABLE_ADDRESS;

    // Clear all pending interrupts and enable all
    // USB related interrupts
    USB->ISTR = 0U;
    USB->CNTR = (uint16_t)(USB_CNTR_RESETM);
}
void delay_ms(uint32_t ms) {
    for (uint32_t i = 0; i < ms * 7200; i++)
        __asm__("nop");  // No operation, just delay
}

void usb_send_data(void) {
    delay_ms(100);
    data[0]++; data[1]--;
    usb_ctrl_send_data(1, data, 2);
}
