/***************************************************************************//**
* \file usb_app.h
* \version 1.0
*
* \brief Header file providing interface definitions for the USB Slave FIFO application.
*
*******************************************************************************
* \copyright
* (c) (2026), Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.
*
* SPDX-License-Identifier: Apache-2.0
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

#ifndef _USB_APP_H_
#define _USB_APP_H_

#include "cy_debug.h"
#include "cy_usbhs_dw_wrapper.h"

#if defined(__cplusplus)
extern "C" {
#endif

#define RED                             "\033[0;31m"
#define CYAN                            "\033[0;36m"
#define COLOR_RESET                     "\033[0m"

#define LOG_COLOR(...)                  Cy_Debug_AddToLog(1,CYAN);\
                                        Cy_Debug_AddToLog(1,__VA_ARGS__); \
                                        Cy_Debug_AddToLog(1,COLOR_RESET);

#define LOG_ERROR(...)                  Cy_Debug_AddToLog(1,RED);\
                                        Cy_Debug_AddToLog(1,__VA_ARGS__); \
                                        Cy_Debug_AddToLog(1,COLOR_RESET);

#define LOG_CLR(CLR, ...)               Cy_Debug_AddToLog(1,CLR);\
                                        Cy_Debug_AddToLog(1,__VA_ARGS__); \
                                        Cy_Debug_AddToLog(1,COLOR_RESET);


#define LOG_TRACE()                     LOG_COLOR("-->[%s]:%d\r\n",__func__,__LINE__);


#define DELAY_MICRO(us)                 Cy_SysLib_DelayUs(us)
#define DELAY_MILLI(ms)                 Cy_SysLib_Delay(ms)


#define SET_BIT(byte, mask)              (byte) |= (mask)
#define CLR_BIT(byte, mask)              (byte) &= ~(mask)
#define CHK_BIT(byte, mask)              (byte) & (mask)
#define FPGA_ENABLE                      (1)

#define DMA_BUFFER_SIZE                  (SLFF_RX_MAX_BUFFER_SIZE)
#define FPGA_DMA_BUFFER_SIZE             (DMA_BUFFER_SIZE)

#define ASSERT(condition, value)        Cy_CheckStatus(__func__, __LINE__, condition, value, true);
#define ASSERT_NON_BLOCK(condition, value) Cy_CheckStatus(__func__, __LINE__, condition, value, false);
#define ASSERT_AND_HANDLE(condition, value, failureHandler) Cy_CheckStatusHandleFailure(__func__, __LINE__, condition, value, false, Cy_FailHandler);

/* Get the LS byte from a 16-bit number */
#define CY_GET_LSB(w)                           ((uint8_t)((w) & UINT8_MAX))

/* Get the MS byte from a 16-bit number */
#define CY_GET_MSB(w)                           ((uint8_t)((w) >> 8))

/* Number of RAM buffers used on the data path: 3 */
#define SLFF_RX_MAX_BUFFER_COUNT                (3)

/* Size of each RAM buffer used in the data path: 60 KB */
#define SLFF_RX_MAX_BUFFER_SIZE                 (61440)

/* Select the primary BULK-IN endpoint used for data streaming.
 * The direction bit does not need to be set and the value should be between 1 and 15 */
#define BULK_IN_ENDPOINT_1                      (0x01)

/* Select the secondary BULK-IN endpoint used for data streaming when DEVICE1_EN is set to 1.
 * The direction bit does not need to be set and the value should be between 1 and 15 */
#define BULK_IN_ENDPOINT_2                      (0x02)

/* Primary IN endpoint index. Make sure direction bit is cleared */
#define BULK_IN_EP1_INDEX                       (BULK_IN_ENDPOINT_1 & 0x0F)

/* Secondary IN endpoint index. Make sure direction bit is cleared */
#define BULK_IN_EP2_INDEX                       (BULK_IN_ENDPOINT_2 & 0x0F)

/* Depth of message queue used by the application task in number of messages */
#define CY_USB_DEVICE_MSG_QUEUE_SIZE            (16)

/* Size of each message sent to the application task from ISRs */
#define CY_USB_DEVICE_MSG_SIZE                  (sizeof (cy_stc_usbd_app_msg_t))

/* P4.0 is used for VBus detect functionality */

/* IO Port corresponding to VBus Detect pin */
#define VBUS_DETECT_GPIO_PORT                   (P4_0_PORT)

/* IO Pin corresponding to VBus Detect Pin */
#define VBUS_DETECT_GPIO_PIN                    (P4_0_PIN)

/* Interrupt source associated with the VBus Detect pin */
#define VBUS_DETECT_GPIO_INTR                   (ioss_interrupts_gpio_dpslp_4_IRQn)

/* State of VBus detect pin when supply is available */
#define VBUS_DETECT_STATE                       (0u)

#define USB_DESC_ATTRIBUTES __attribute__ ((section(".descSection"), used)) __attribute__ ((aligned (32)))
#define HBDMA_BUF_ATTRIBUTES __attribute__ ((section(".hbBufSection"), used)) __attribute__ ((aligned (32)))

/* Vendor command code used to return WinUSB specific descriptors */
#define MS_VENDOR_CODE                          (0xF0)

extern uint8_t glOsString[];
extern uint8_t glOsCompatibilityId[];
extern uint8_t glOsFeature[];

typedef struct cy_stc_usb_app_ctxt_ cy_stc_usb_app_ctxt_t;

/* FPGA Configuration mode selection */
typedef enum cy_en_fpgaConfigMode_t
{
    ACTIVE_SERIAL_MODE  = 0,                            /* FPGA configures itself in active serial mode */
    PASSIVE_SERIAL_MODE                                 /* FPGA is configured by FX2G3 in passive serial mode */
}cy_en_fpgaConfigMode_t;

/* Types of messages send to the Slave FIFO application task */
typedef enum cy_en_slfifo_app_msg_type_t {
    CY_USB_UVC_VBUS_CHANGE_INTR      = 0x01,            /* VBus change interrupt received */
    CY_USB_UVC_VBUS_CHANGE_DEBOUNCED = 0x02,            /* VBus change status debounced */
    CY_USB_STREAMING_START           = 0x03,            /* Request to notify FPGA to start data streaming */
    CY_USB_STREAMING_STOP            = 0x04             /* Request to stop data streaming */
} cy_en_slfifo_app_msg_type_t;

/*
 * Slave FIFO application data structure which is bridge between USB system and device functionality.
 */
struct cy_stc_usb_app_ctxt_
{
    bool hbChannelCreated;                              /* Whether DMA channel has been created for IN endpoints */
    bool vbusChangeIntr;                                /* Whether VBus status change interrupt has been received */
    bool vbusPresent;                                   /* Whether VBus supply is present (after debounce) */
    bool usbConnected;                                  /* Whether firmware has enabled USB connection */
    uint8_t slffInEpNum1;                               /* Index of primary IN endpoint */
    uint8_t slffInEpNum2;                               /* Index of secondary IN endpoint (where enabled) */
    uint8_t firstInitDone;                              /* Whether the application has already been initialized */
    uint8_t devAddr;                                    /* USB device address assigned to FX2G3 */
    uint8_t activeCfgNum;                               /* Active configuration index: Can be 0 or 1 */
    cy_en_usb_device_state_t devState;                  /* Current USB device state */
    cy_en_usb_device_state_t prevDevState;              /* Previous USB device state */
    cy_en_usb_speed_t devSpeed;                         /* Current USB connection speed: High-Speed or Full Speed */
    uint8_t prevAltSetting;                             /* Previous alternate setting for the only USB interface */
    cy_stc_app_endp_dma_set_t endpInDma[CY_USB_MAX_ENDP_NUMBER];        /* Data-path status for IN endpoints */
    cy_stc_app_endp_dma_set_t endpOutDma[CY_USB_MAX_ENDP_NUMBER];       /* Data-path status for OUT endpoints */
    DMAC_Type *pCpuDmacBase;                            /* Base address of DMAC register set */
    DW_Type *pCpuDw0Base;                               /* Base address of DataWire-0 register set */
    DW_Type *pCpuDw1Base;                               /* Base address of DataWire-1 register set */
    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt;          /* Pointer to High BandWidth DMA manager context structure */
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt;                  /* Pointer to USBD stack context structure */
    cy_stc_hbdma_channel_t *hbBulkInChannel[2];         /* Pointers to the primary and secondary DMA channels */
    TaskHandle_t slffTaskHandle;                        /* Handle to the Slave FIFO application task */
    QueueHandle_t usbMsgQueue;                          /* Handle to the message queue used by the application task */
    TimerHandle_t vbusDebounceTimer;                    /* Timer instance used to debounce VBus supply status */
    uint8_t fpgaVersion;                                /* Version information read from the FPGA */
};

/*
 * Details of the I2C based control registers implemented by the FPGA data source.
 * This includes register addresses as well as important fields in each of the registers.
 */
typedef enum cy_en_i2c_fpgaRegMap_t
{
    FPGA_MAJOR_VERSION_ADDRESS             = 0x00,          /* Address of FPGA Major Version register */
    FPGA_MINOR_VERSION_ADDRESS             = 0x00,          /* Address of FPGA Minor Version register */

    FPGA_UVC_SELECTION_ADDRESS             = 0x01,          /* Address of UVC function enable register */
    FPGA_UVC_ENABLE                        = 1,             /* UVC Enable */

    FPGA_HEADER_CTRL_ADDRESS               = 0x02,          /* Header Addition Control address */
    FPGA_HEADER_ENABLE                     = 1,             /* FPGA adds 32 byte UVC header */
    FPGA_HEADER_DISABLE                    = 0,             /* FX2G3 adds 32 byte UVC header */

    FPGA_ACTIVE_DEVICE_MASK_ADDRESS        = 0x08,          /* Address of streaming device selection register */

    DEVICE0_OFFSET                         = 0x20,          /* Offset of register set for FPGA Device 0 */
    DEVICE1_OFFSET                         = 0x3C,          /* Offset of register set for FPGA Device 1 */
    DEVICE2_OFFSET                         = 0x58,          /* Offset of register set for FPGA Device 2 */
    DEVICE3_OFFSET                         = 0x74,          /* Offset of register set for FPGA Device 3 */
    DEVICE4_OFFSET                         = 0x90,          /* Offset of register set for FPGA Device 4 */
    DEVICE5_OFFSET                         = 0xAC,          /* Offset of register set for FPGA Device 5 */
    DEVICE6_OFFSET                         = 0xC8,          /* Offset of register set for FPGA Device 6 */
    DEVICE7_OFFSET                         = 0xE4,          /* Offset of register set for FPGA Device 7 */

    FPGA_DEVICE_STREAM_ENABLE_OFFSET       = 0x00,         /* FPGA Data Stream Enable Address */
    DATA_DISABLE                           = 0x00,         /* Disable Data Stream */
    DMA_CH_RESET                           = 0x01,         /* DMA Channel Reset */
    DATA_ENABLE                            = 0x02,         /* Enable Data Stream */

    FPGA_DEVICE_STREAM_MODE_OFFSET         = 0x01,         /* Device Data Stream Mode  */
    NO_CONVERSION                          = 0,            /* RAW Data Stream  */

    DEVICE_IMAGE_HEIGHT_LSB_OFFSET         = 0x02,         /* Video Image Height (LSB)  */
    DEVICE_IMAGE_HEIGHT_MSB_OFFSET         = 0x03,         /* Video Image Height (MSB)  */
    DEVICE_IMAGE_WIDTH_LSB_OFFSET          = 0x04,         /* Video Image Width (LSB)  */
    DEVICE_IMAGE_WIDTH_MSB_OFFSET          = 0x05,         /* Video Image Width (MSB)  */

    DEVICE_FPS_OFFSET                      = 0x06,         /* Video frames per second */

    DEVICE_PIXEL_WIDTH_OFFSET              = 0x07,         /* Video Bits per pixel */
    VIDEO_8_BIT_PER_PIXEL                  = 8,
    VIDEO_12BIT_PER_PIXEL                  = 12,
    VIDEO_16BIT_PER_PIXEL                  = 16,
    VIDEO_24BIT_PER_PIXEL                  = 24,
    VIDEO_36BIT_PER_PIXEL                  = 36,

    DEVICE_SOURCE_TYPE_OFFSET              = 0x08,        /* Data Source Type */
    INTERNAL_COLORBAR                      = 0x00,        /* Internal Colorbar */
    MIPI_SOURCE                            = 0x02,        /* MIPI Source */

    DEVICE_FLAG_STATUS_OFFSET              = 0x09,        /* DMA Flag Status Register*/

    DEVICE_MIPI_STATUS_OFFSET              = 0x0A,        /* MIPI Interface Status Register */

    DEVICE_SOURCE_INFO_OFFSET              = 0x0B,        /* Device  Source Info Register */
    SOURCE_DISCONNECT                      = 0x00,        /* Source Disconnect */

    DEVICE_ACTIVE_THREAD_INFO_OFFSET       = 0x0F,        /* Number of GPIF threads used by this device */
    DEVICE_THREAD1_INFO_OFFSET             = 0x10,        /* Selection of first thread used by the device */
    DEVICE_THREAD2_INFO_OFFSET             = 0x11,        /* Selection of second thread used by the device */
    DEVICE_THREAD1_SOCKET_INFO_OFFSET      = 0x12,        /* Selection of socket mapped to the first thread */
    DEVICE_THREAD2_SOCKET_INFO_OFFSET      = 0x13,        /* Selection of socket mapped to the second thread */

    DEVICE_FLAG_INFO_OFFSET                = 0x14,        /* Device Info Address */
    FX2G3_READY_TO_REC_DATA                = 0x08,
    NEW_UVC_PACKET_START                   = 0x02,
    NEW_FRAME_START                        = 0x01,

    DEVICE_BUFFER_SIZE_LSB_OFFSET          = 0x16,       /* Device DMA Buffer Size (LSB) Address */
    DEVICE_BUFFER_SIZE_MSB_OFFSET          = 0x17,       /* Device DMA Buffer Size (MSB) Address */
} cy_en_i2c_fpgaRegMap_t;

/* FPGA streaming configuration macros */

#define FPGA_STREAMING_DEV_REG_COUNT    (0x1C)

#define FPGA_DEVICE_STREAM_ENABLE_ADDRESS(ndev)                 \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + FPGA_DEVICE_STREAM_ENABLE_OFFSET)

#define FPGA_DEVICE_STREAM_MODE_ADDRESS(ndev)                   \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + FPGA_DEVICE_STREAM_MODE_OFFSET)

#define DEVICE_IMAGE_HEIGHT_LSB_ADDRESS(ndev)                   \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_IMAGE_HEIGHT_LSB_OFFSET)

#define DEVICE_IMAGE_HEIGHT_MSB_ADDRESS(ndev)                   \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_IMAGE_HEIGHT_MSB_OFFSET)

#define DEVICE_IMAGE_WIDTH_LSB_ADDRESS(ndev)                    \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_IMAGE_WIDTH_LSB_OFFSET)

#define DEVICE_IMAGE_WIDTH_MSB_ADDRESS(ndev)                    \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_IMAGE_WIDTH_MSB_OFFSET)

#define DEVICE_FPS_ADDRESS(ndev)                                \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_FPS_OFFSET)

#define DEVICE_PIXEL_WIDTH_ADDRESS(ndev)                        \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_PIXEL_WIDTH_OFFSET)

#define DEVICE_SOURCE_TYPE_ADDRESS(ndev)                        \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_SOURCE_TYPE_OFFSET)

#define DEVICE_FLAG_STATUS_ADDRESS(ndev)                        \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_FLAG_STATUS_OFFSET)

#define DEVICE_MIPI_STATUS_ADDRESS(ndev)                        \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_MIPI_STATUS_OFFSET)

#define DEVICE_SOURCE_INFO_ADDRESS(ndev)                        \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_SOURCE_INFO_OFFSET)

#define DEVICE_ACTIVE_THREAD_INFO_ADDRESS(ndev)                 \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_ACTIVE_THREAD_INFO_OFFSET)

#define DEVICE_THREAD1_INFO_ADDRESS(ndev)                       \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_THREAD1_INFO_OFFSET)

#define DEVICE_THREAD2_INFO_ADDRESS(ndev)                       \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_THREAD2_INFO_OFFSET)

#define DEVICE_THREAD1_SOCKET_INFO_ADDRESS(ndev)                \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_THREAD1_SOCKET_INFO_OFFSET)

#define DEVICE_THREAD2_SOCKET_INFO_ADDRESS(ndev)                \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_THREAD2_SOCKET_INFO_OFFSET)

#define DEVICE_FLAG_INFO_ADDRESS(ndev)                          \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_FLAG_INFO_OFFSET)

#define DEVICE_BUFFER_SIZE_LSB_ADDRESS(ndev)                    \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_BUFFER_SIZE_LSB_OFFSET)

#define DEVICE_BUFFER_SIZE_MSB_ADDRESS(ndev)                    \
    (DEVICE0_OFFSET + (ndev) * FPGA_STREAMING_DEV_REG_COUNT + DEVICE_BUFFER_SIZE_MSB_OFFSET)

/**
 * \name Cy_USB_AppInit
 * \brief   This function Initializes application related data structures, register callback
 *          creates task for device function.
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer Context pointer
 * \param pCpuDmacBase DMAC base address
 * \param pCpuDw0Base DataWire 0 base address
 * \param pCpuDw1Base DataWire 1 base address
 * \param pHbDmaMgrCtxt HBDMA Manager Context
 * \retval None
 */
void Cy_USB_AppInit(cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                    DMAC_Type *pCpuDmacBase, DW_Type *pCpuDw0Base, DW_Type *pCpuDw1Base,
                    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt);

/**
 * \name Cy_USB_AppRegisterCallback
 * \brief This function will register all calback with USBD layer.
 * \param pAppCtxt application layer context pointer.
 * \retval None
 */
void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt);

/**
 * \name Cy_USB_AppSetCfgCallback
 * \brief Callback function will be invoked by USBD when set configuration is received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer.
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetCfgCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppBusResetCallback
 * \brief Callback function will be invoked by USBD when bus detects RESET
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppBusResetCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppBusResetDoneCallback
 * \brief Callback function will be invoked by USBD when RESET is completed
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppBusResetDoneCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppBusSpeedCallback
 * \brief   Callback function will be invoked by USBD when speed is identified or
 *          speed change is detected
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppBusSpeedCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppSetupCallback
 * \brief Callback function will be invoked by USBD when SETUP packet is received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetupCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppSuspendCallback
 * \brief Callback function will be invoked by USBD when Suspend signal/message is detected
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSuspendCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppResumeCallback
 * \brief Callback function will be invoked by USBD when Resume signal/message is detected
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppResumeCallback (void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppSetIntfCallback
 * \brief Callback function will be invoked by USBD when SET_INTERFACE is  received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetIntfCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, cy_stc_usb_cal_msg_t *pMsg);

/**
 * \name Cy_USB_AppInitDmaIntr
 * \brief Function to register an ISR for the DMA channel associated with an endpoint
 * \param endpNumber USB endpoint number
 * \param endpDirection Endpoint direction
 * \param userIsr ISR function pointer. Can be NULL if interrupt is to be disabled.
 * \retval None
 */
void Cy_USB_AppInitDmaIntr(uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection, cy_israddress userIsr);

/**
 * \name Cy_USB_EnableUsbHSConnection
 * \brief Enable USBHS connection
 * \param pAppCtxt Pointer to UVC application context structure.
 * \retval None
 */
bool Cy_USB_EnableUsbHSConnection(cy_stc_usb_app_ctxt_t *pAppCtxt);

/**
 * \name Cy_USB_DisableUsbHSConnection
 * \brief Disable USBHS connection
 * \retval None
 */
void Cy_USB_DisableUsbHSConnection (cy_stc_usb_app_ctxt_t *pAppCtxt);

/**
 * \name Cy_Slff_LvdsInit
 * \brief   Initialize the LVDS interface. Currently, only the SIP #0 is being initialized
 *          and configured to allow transfers into the HBW SRAM through DMA.
 * \retval None
 */
void Cy_Slff_LvdsInit(void);

/**
 * \name Cy_Slff_RxChannelDataWire_ISR(void)
 * \brief Datawire ISR for slaveFIFO RX for channels
 * \retval None
 */
void Cy_Slff_RxChannelDataWire_ISR (void);

/**
 * \name Cy_CheckStatus
 * \brief Function that handles prints error log
 * \param function Pointer to function
 * \param line Line number where error is seen
 * \param condition condition of failure
 * \param value error code
 * \param isBlocking blocking function
 * \retval None
 */
void Cy_CheckStatus(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking);

/**
 * \name Cy_CheckStatusHandleFailure
 * \brief Function that handles prints error log
 * \param function Pointer to function
 * \param line LineNumber where error is seen
 * \param condition Line number where error is seen
 * \param value error code
 * \param isBlocking blocking function
 * \param failureHandler failure handler function
 * \retval None
 */
void Cy_CheckStatusHandleFailure(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking, void (*failureHandler)());

/**
 * \name Cy_FailHandler
 * \brief Error Handler
 * \retval None
 */
void Cy_FailHandler(void);

#if defined(__cplusplus)
}
#endif

#endif /* _USB_APP_H_ */

/*[EOF]*/

