/***************************************************************************//**
* \file usb_app.c
* \version 1.0
*
* \brief Implements the USB data handling part of the Slave FIFO application.
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

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "cy_pdl.h"
#include "cy_device.h"
#include "cy_usbhs_dw_wrapper.h"
#include "cy_usb_common.h"
#include "cy_hbdma.h"
#include "cy_hbdma_mgr.h"
#include "cy_usb_usbd.h"
#include "usb_app.h"
#include "cy_debug.h"
#include "usb_i2c.h"
#include "usb_qspi.h"
#include "cy_lvds.h"

static bool glIsFPGAConfigureComplete = false;  /* Whether the FPGA FIFO master has been configured */
static bool glIsFPGARegConfigured = false;      /* Whether stream control registers on the FPGA have been updated */

#if FPGA_ENABLE

/**
 * \name Cy_Slff_ConfigFpgaRegister
 * \brief   FPGA Register Writes. FPGA is configured to send internally generated
 *          colorbar data over FX2G3's SlaveFIFO Interface
 * \retval 0 for read success, error code for unsuccess.
 */
static cy_en_scb_i2c_status_t Cy_Slff_ConfigFpgaRegister (void)
{
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;

    /* Configure FPGA to generate colorbar video pattern with a default resolution of 1920x1080 pixels */
    uint16_t width = 1920;
    uint16_t height = 1080;

    /* Disable streaming device-0 first */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_DEVICE_STREAM_ENABLE_ADDRESS(0),
            DATA_DISABLE, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

#if (!INTERLEAVE_EN && DEVICE1_EN)
    /* Disable streaming device-1 */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_DEVICE_STREAM_ENABLE_ADDRESS(1),
            DATA_DISABLE, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
#endif /* (!INTERLEAVE_EN && DEVICE1_EN) */

    /* Select data source type as UVC */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_UVC_SELECTION_ADDRESS,
            FPGA_UVC_ENABLE, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* No headers required as we are doing raw streaming */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_HEADER_CTRL_ADDRESS,
            FPGA_HEADER_DISABLE, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(500);

#if (INTERLEAVE_EN || !DEVICE1_EN)
    /* Select a single streaming device */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_ACTIVE_DEVICE_MASK_ADDRESS,
            0x01, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
#else
    /* Select two streaming devices */
    status = Cy_I2C_Write(FPGASLAVE_ADDR,FPGA_ACTIVE_DEVICE_MASK_ADDRESS,
            0x03, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
#endif /* (INTERLEAVE_EN || !DEVICE1_EN) */
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    Cy_SysLib_DelayUs(500);

    DBG_APP_INFO("FPGA: Register writes for device-0\r\n");

    /* Write FPGA register to disable format conversion */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_DEVICE_STREAM_MODE_ADDRESS(0),
            NO_CONVERSION, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_SOURCE_TYPE_ADDRESS(0),
            INTERNAL_COLORBAR, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

#if INTERLEAVE_EN
    /*
     * In interleaved mode, threads 0 and 1 are used together to receive data from the FPGA.
     */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_ACTIVE_THREAD_INFO_ADDRESS(0),
            2, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Enable thread-1 and select the socket to be used with it */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_THREAD2_SOCKET_INFO_ADDRESS(0),
            CY_LVDS_GPIF_THREAD_1, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_THREAD2_INFO_ADDRESS(0),
            1, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
#else
    /* Only one thread is used in non-interleaved mode */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_ACTIVE_THREAD_INFO_ADDRESS(0),
            1, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_THREAD2_SOCKET_INFO_ADDRESS(0),
            0, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_THREAD2_INFO_ADDRESS(0),
            0, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
#endif /* INTERLEAVE_EN */

    /* Select the first thread to be used by the streaming device */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_THREAD1_INFO_ADDRESS(0),
            CY_LVDS_GPIF_THREAD_0, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_THREAD1_SOCKET_INFO_ADDRESS(0),
            0, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* No video source connected to the FPGA */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_SOURCE_INFO_ADDRESS(0),
            SOURCE_DISCONNECT, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Update DMA buffer size used by Firmware */
    DBG_APP_INFO("SLFF: DMA Buffer Size %d\r\n", FPGA_DMA_BUFFER_SIZE);
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_BUFFER_SIZE_MSB_ADDRESS(0),
            CY_GET_MSB(FPGA_DMA_BUFFER_SIZE), FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_BUFFER_SIZE_LSB_ADDRESS(0),
            CY_GET_LSB(FPGA_DMA_BUFFER_SIZE), FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Update default resolution width size used by Firmware */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_IMAGE_WIDTH_MSB_ADDRESS(0),
            CY_GET_MSB(width), FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_IMAGE_WIDTH_LSB_ADDRESS(0),
            CY_GET_LSB(width), FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Update default resolution height size used by Firmware */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_IMAGE_HEIGHT_MSB_ADDRESS(0),
            CY_GET_MSB(height), FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_IMAGE_HEIGHT_LSB_ADDRESS(0),
            CY_GET_LSB(height), FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Update default streaming rate in fps */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_FPS_ADDRESS(0),
            60, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

#if (!INTERLEAVE_EN && DEVICE1_EN)
    DBG_APP_INFO("FPGA: Register writes for device-1\r\n");

    status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_DEVICE_STREAM_MODE_ADDRESS(1),
            NO_CONVERSION, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_SOURCE_TYPE_ADDRESS(1),
            INTERNAL_COLORBAR, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Only one thread is used to receive data from this device */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_ACTIVE_THREAD_INFO_ADDRESS(1),
            1, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_THREAD2_SOCKET_INFO_ADDRESS(1),
            0, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_THREAD2_INFO_ADDRESS(1),
            1, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Thread 1 is used to receive data from the second streaming device */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_THREAD1_INFO_ADDRESS(1),
            1, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Socket-0 of Thread 1 (effective socket 1) is used to receive data */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_THREAD1_SOCKET_INFO_ADDRESS(1),
            0, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_SOURCE_INFO_ADDRESS(1),
            SOURCE_DISCONNECT, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Update DMA buffer size used by Firmware */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_BUFFER_SIZE_MSB_ADDRESS(1),
            CY_GET_MSB(FPGA_DMA_BUFFER_SIZE), FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_BUFFER_SIZE_LSB_ADDRESS(1),
            CY_GET_LSB(FPGA_DMA_BUFFER_SIZE), FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Update default resolution width size used by Firmware */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_IMAGE_WIDTH_MSB_ADDRESS(1),
            CY_GET_MSB(width), FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_IMAGE_WIDTH_LSB_ADDRESS(1),
            CY_GET_LSB(width), FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Update default resolution height size used by Firmware */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_IMAGE_HEIGHT_MSB_ADDRESS(1),
            CY_GET_MSB(height), FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_IMAGE_HEIGHT_LSB_ADDRESS(1),
            CY_GET_LSB(height), FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);

    /* Set the default fps */
    status = Cy_I2C_Write(FPGASLAVE_ADDR, DEVICE_FPS_ADDRESS(1),
            60, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    ASSERT_NON_BLOCK(status == CY_SCB_I2C_SUCCESS, status);
#endif /* (!INTERLEAVE_EN && DEVICE1_EN) */
    return status;
} /* End of Cy_Slff_ConfigFpgaRegister() */

/**
 * \name Cy_Slff_DataStreamStartStop
 * \brief Starts/Stops FPGA data streaming
 * \param deviceNum Streaming device number.
 * \param isStreamStart Whether the data stream is to be started.
 * \retval CY_SCB_I2C_SUCCESS if FPGA update is successful, appropriate I2C error code in case of error.
 */
cy_en_scb_i2c_status_t Cy_Slff_DataStreamStartStop(uint8_t deviceNum, bool isStreamStart)
{
    cy_en_scb_i2c_status_t status = CY_SCB_I2C_SUCCESS;

    if (isStreamStart) {
        status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_DEVICE_STREAM_ENABLE_ADDRESS(deviceNum),
                DATA_ENABLE, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    } else {
        status = Cy_I2C_Write(FPGASLAVE_ADDR, FPGA_DEVICE_STREAM_ENABLE_ADDRESS(deviceNum),
                DATA_DISABLE, FPGA_I2C_ADDRESS_WIDTH, FPGA_I2C_DATA_WIDTH);
    }

    if (CY_SCB_I2C_SUCCESS == status) {
        DBG_APP_INFO("SLFF: Streaming device #%d Active=%d\r\n", deviceNum, isStreamStart);
    }

    return status;
} /* End of Cy_Slff_DataStreamStartStop() */
#endif /* FPGA_ENABLE */

/**
 * \name Cy_Slff_WaitForSocketStall
 * \brief Function which polls the specified LVCMOS socket until it is in stall state or a 5 ms timeout.
 * \param pAppCtxt application layer context pointer
 * \param socketId ID of socket to be polled.
 * \retval true if socket got stalled, false otherwise.
 */
static bool Cy_Slff_WaitForSocketStall (cy_stc_usb_app_ctxt_t *pAppCtxt, cy_hbdma_socket_id_t socketId)
{
    cy_stc_hbdma_sock_t  sckStat;
    cy_en_hbdma_status_t apiStat;
    uint32_t pollCnt = 0;

    do {
        Cy_SysLib_DelayUs(10);
        apiStat = Cy_HBDma_GetSocketStatus(pAppCtxt->pHbDmaMgrCtxt->pDrvContext, socketId, &sckStat);
        if (apiStat != CY_HBDMA_SUCCESS) {
            return false;
        }
    } while (((_FLD2VAL(LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_STATE, sckStat.status)) != 0x01UL) && (pollCnt++ < 500U));

    DBG_APP_INFO("SLFF Stop: Socket %x status is %x\r\n", socketId, sckStat.status);
    return (_FLD2VAL(LVDSSS_LVDS_ADAPTER_DMA_SCK_STATUS_STATE, sckStat.status) == 0x01UL);
}

/**
 * \name Cy_Slff_AppStop
 * \brief Stop the data stream channels
 * \param pAppCtxt application layer context pointer
 * \param pUsbdCtxt USBD layer context pointer
 * \param epNumber Index of endpoint to be stopped.
 * \retval None
 */
static void Cy_Slff_AppStop (cy_stc_usb_app_ctxt_t *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint32_t epNumber)
{
    cy_en_hbdma_mgr_status_t status = CY_HBDMA_MGR_SUCCESS;
#if INTERLEAVE_EN
    uint8_t index = 0;
#endif /* INTERLEAVE_EN */

    DBG_APP_INFO("SLFF: Stream stop on ep=0x%x\r\n", epNumber);

    /*
     * We need to reset the DMA channel and then flush any stale data present in the endpoint.
     *
     * Resetting the DMA channel will disable the socket on the LVCMOS/GPIF side. Disabling this
     * socket while data is being sent by the FPGA is likely to cause failures when the transfer
     * is restarted. So, we need to wait until the LVCMOS (producer) sockets have become stalled
     * (not able to receive data as all buffers are full) before the channel reset is performed.
     */
    if (BULK_IN_EP1_INDEX == epNumber)
    {
#if INTERLEAVE_EN
        for (index = 0; index <= SLFF_RX_MAX_BUFFER_COUNT; index++) {
            /*
             * Wait for each producer socket to get stalled. We do multiple checks to ensure that the
             * stall is not a transient condition.
             */
            Cy_Slff_WaitForSocketStall(pAppCtxt, pAppCtxt->hbBulkInChannel[0]->prodSckId[0]);
            Cy_Slff_WaitForSocketStall(pAppCtxt, pAppCtxt->hbBulkInChannel[0]->prodSckId[1]);
        }
#else
        /* Wait for the producer socket to get stalled. Single time check is sufficient in this case */
        Cy_Slff_WaitForSocketStall(pAppCtxt, pAppCtxt->hbBulkInChannel[0]->prodSckId[0]);
#endif /* INTERLEAVE_EN */

#if FPGA_ENABLE
        /* Update the FPGA so that it stops trying to send data to FX2G3 */
        Cy_Slff_DataStreamStartStop(0, false);
#endif /* FPGA_ENABLE */

        /* Reset the DMA channel through which data is received from the LVDS side */
        status = Cy_HBDma_Channel_Reset(pAppCtxt->hbBulkInChannel[0]);
        ASSERT_NON_BLOCK(CY_HBDMA_MGR_SUCCESS == status,status);
    }

    if (BULK_IN_EP2_INDEX == epNumber) {
        /* Wait for the producer socket to get stalled. Single time check is sufficient in this case */
        Cy_Slff_WaitForSocketStall(pAppCtxt, pAppCtxt->hbBulkInChannel[1]->prodSckId[0]);

#if FPGA_ENABLE
        /* Update the FPGA so that it stops trying to send data to FX2G3 */
        Cy_Slff_DataStreamStartStop(1, false);
#endif /* FPGA_ENABLE */

        /* Reset the DMA channel through which data is received from the LVDS side */
        status = Cy_HBDma_Channel_Reset(pAppCtxt->hbBulkInChannel[1]);
        ASSERT_NON_BLOCK(CY_HBDMA_MGR_SUCCESS == status,status);
    }

    /* Flush and reset the endpoint and clear the STALL bit */
    Cy_USBD_FlushEndp(pUsbdCtxt, epNumber, CY_USB_ENDP_DIR_IN);
    Cy_USBD_ResetEndp(pUsbdCtxt, epNumber, CY_USB_ENDP_DIR_IN, false);
    Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, (cy_en_usb_endp_dir_t)epNumber, CY_USB_ENDP_DIR_IN, false);
}


/**
 * \name Cy_Slff_AppHbDmaRxCallback
 * \brief Callback function for the channel receiving data into LVDS socket
 * \param handle HBDMA channel handle
 * \param cy_en_hbdma_cb_type_t HBDMA channel type
 * \param pbufStat HBDMA buffer status
 * \param userCtx user context
 * \retval None
 */
void Cy_Slff_AppHbDmaRxCallback(
        cy_stc_hbdma_channel_t *handle,
        cy_en_hbdma_cb_type_t type,
        cy_stc_hbdma_buff_status_t *pbufStat,
        void *userCtx)
{
    cy_en_hbdma_mgr_status_t   status;
    cy_stc_hbdma_buff_status_t buffStat;

    if (type == CY_HBDMA_CB_PROD_EVENT)
    {
        /* Get information about the data received from the FPGA */
        status = Cy_HBDma_Channel_GetBuffer(handle, &buffStat);
        if (status != CY_HBDMA_MGR_SUCCESS)
        {
            DBG_APP_ERR("SLFF: Get Buffer Failed %x\r\n", status);
            return;
        }

        /* Commit the buffer to send the data through to the USB host */
        status = Cy_HBDma_Channel_CommitBuffer(handle, &buffStat);
        if (status != CY_HBDMA_MGR_SUCCESS)
        {
          DBG_APP_ERR("SLFF: Commit Buffer Failed %x\r\n", status);
          return;
        }
    }
}

/**
 * \name Cy_Slff_AppTaskHandler
 * \brief Application task handler
 * \param pTaskParam task param
 * \retval None
 */
void Cy_Slff_AppTaskHandler(void *pTaskParam)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pTaskParam;
    cy_stc_usbd_app_msg_t queueMsg;
    cy_en_hbdma_mgr_status_t mgrStat;
    BaseType_t xStatus;
    uint32_t lpEntryTime = 0;

    vTaskDelay(250);

#if FPGA_ENABLE
#if FPGA_CONFIG_EN
    Cy_FPGAConfigPins(pAppCtxt,FPGA_CONFIG_MODE);
    Cy_QSPI_Start(pAppCtxt);
    Cy_SPI_FlashInit(SPI_FLASH_0, true,false);

    DBG_APP_INFO("FPGA: Configuration Start\n\r");
    glIsFPGAConfigureComplete = Cy_FPGAConfigure(pAppCtxt,FPGA_CONFIG_MODE);
#else
    glIsFPGAConfigureComplete = Cy_IsFPGAConfigured();
#endif /* FPGA_CONFIG_EN */

    if ((glIsFPGARegConfigured == false) && (glIsFPGAConfigureComplete == true))
    {
        Cy_APP_GetFPGAVersion(pAppCtxt);
        if(0 == Cy_Slff_ConfigFpgaRegister())
        {
            glIsFPGARegConfigured = true;
            DBG_APP_TRACE("FPGA: Configuration Complete \n\r");
        }
        else
        {
            LOG_ERROR("FPGA: Configuration failed \r\n");
        }
    }

#endif /* FPGA_ENABLE */

    /* Initialize the LVCMOS interface and Slave FIFO GPIF state machine */
    Cy_Slff_LvdsInit();
    vTaskDelay(100);

    /* If VBus is present, enable the USB connection */
    pAppCtxt->vbusPresent = (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);
    if (pAppCtxt->vbusPresent) {
        Cy_USB_EnableUsbHSConnection(pAppCtxt);
    }

    for (;;)
    {
        /*
         * If the link has been in USB2-L1 for more than 0.5 seconds, initiate LPM exit so that
         * transfers do not get delayed significantly.
         */
        if ((MXS40USBHSDEV_USBHSDEV->DEV_PWR_CS & USBHSDEV_DEV_PWR_CS_L1_SLEEP) != 0)
        {
            if ((Cy_USBD_GetTimerTick() - lpEntryTime) >= 500UL) {
                lpEntryTime = Cy_USBD_GetTimerTick();
                Cy_USBD_GetUSBLinkActive(pAppCtxt->pUsbdCtxt);
            }
        } else {
            lpEntryTime = Cy_USBD_GetTimerTick();
        }

        /*
         * Wait until some data is received from the queue.
         * Timeout after 100 ms.
         */
        xStatus = xQueueReceive(pAppCtxt->usbMsgQueue, &queueMsg, 100);
        if (xStatus != pdPASS) {
            continue;
        }

        switch (queueMsg.type) {

            case CY_USB_UVC_VBUS_CHANGE_INTR:
                /* Start the debounce timer */
                xTimerStart(pAppCtxt->vbusDebounceTimer, 0);
                break;

            case CY_USB_UVC_VBUS_CHANGE_DEBOUNCED:
                /* Check whether VBus state has changed */
                pAppCtxt->vbusPresent = (Cy_GPIO_Read(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN) == VBUS_DETECT_STATE);

                if (pAppCtxt->vbusPresent) {
                    if (!pAppCtxt->usbConnected) {
                        DBG_APP_INFO("USB: Enabling USB connection due to VBus detect\r\n");
                        Cy_USB_EnableUsbHSConnection(pAppCtxt);
                    }
                } else {
                    if (pAppCtxt->usbConnected) {
                        if (pAppCtxt->hbChannelCreated)
                        {
                            DBG_APP_TRACE("USB: HBDMA destroy\r\n");
                            pAppCtxt->hbChannelCreated = false;
                            Cy_HBDma_Channel_Disable(pAppCtxt->hbBulkInChannel[0]);
                            Cy_HBDma_Channel_Destroy(pAppCtxt->hbBulkInChannel[0]);
#if (!INTERLEAVE_EN && DEVICE1_EN)
                            Cy_HBDma_Channel_Disable(pAppCtxt->hbBulkInChannel[1]);
                            Cy_HBDma_Channel_Destroy(pAppCtxt->hbBulkInChannel[1]);
#endif /* (!INTERLEAVE_EN && DEVICE1_EN) */
                        }
                    }

                    DBG_APP_INFO("USB: Disabling USB connection due to VBus removal\r\n");
                    Cy_USB_DisableUsbHSConnection(pAppCtxt);
                }
                break;

            case CY_USB_STREAMING_START:
                DBG_APP_INFO("SLFF: Start stream event for ep %x\r\n", (uint8_t)queueMsg.data[0]);
#if FPGA_ENABLE
                if((uint8_t)(queueMsg.data[0]) == BULK_IN_EP1_INDEX)
                {
                    mgrStat = Cy_HBDma_Channel_Enable(pAppCtxt->hbBulkInChannel[0], 0);
                    DBG_APP_INFO("SLFF: Channel#1 enable status: %x\r\n", mgrStat);
                    ASSERT_NON_BLOCK(mgrStat == CY_HBDMA_MGR_SUCCESS, mgrStat);
                    Cy_Slff_DataStreamStartStop(0, true);
                }
                else if((uint8_t)(queueMsg.data[0]) == BULK_IN_EP2_INDEX)
                {
                    mgrStat = Cy_HBDma_Channel_Enable(pAppCtxt->hbBulkInChannel[1], 0);
                    DBG_APP_INFO("SLFF: Channel#1 enable status: %x\r\n", mgrStat);
                    ASSERT_NON_BLOCK(mgrStat == CY_HBDMA_MGR_SUCCESS, mgrStat);
                    Cy_Slff_DataStreamStartStop(1, true);
                }
#endif /* FPGA_ENABLE */
                break;

            case CY_USB_STREAMING_STOP:
                DBG_APP_INFO("SLFF: Start stop event for ep %x\r\n", (uint8_t)queueMsg.data[0]);
                if((uint8_t)(queueMsg.data[0]) == BULK_IN_EP1_INDEX)
                {
                    Cy_Slff_AppStop(pAppCtxt, pAppCtxt->pUsbdCtxt, BULK_IN_EP1_INDEX);
                }
                else if((uint8_t)(queueMsg.data[0]) == BULK_IN_EP2_INDEX)
                {
                    Cy_Slff_AppStop(pAppCtxt, pAppCtxt->pUsbdCtxt, BULK_IN_EP2_INDEX);
                }
                break;

            default:
                break;
        }
    } /* End of for(;;) */
}

/**
 * \name Cy_USB_VbusDebounceTimerCallback
 * \brief Timer used to do debounce on VBus changed interrupt notification.
 * \param xTimer Timer Handle
 * \retval None
 */
void
Cy_USB_VbusDebounceTimerCallback (TimerHandle_t xTimer)
{
    cy_stc_usb_app_ctxt_t *pAppCtxt = (cy_stc_usb_app_ctxt_t *)pvTimerGetTimerID(xTimer);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    cy_stc_usbd_app_msg_t xMsg;

    DBG_APP_INFO("USB: VBUS Timer CB\r\n");
    if (pAppCtxt->vbusChangeIntr) {
        /* Notify the VCOM task that VBus debounce is complete */
        xMsg.type = CY_USB_UVC_VBUS_CHANGE_DEBOUNCED;
        xQueueSendFromISR(pAppCtxt->usbMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));

        /* Clear and re-enable the interrupt */
        pAppCtxt->vbusChangeIntr = false;
        Cy_GPIO_ClearInterrupt(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN);
        Cy_GPIO_SetInterruptMask(VBUS_DETECT_GPIO_PORT, VBUS_DETECT_GPIO_PIN, 1);
    }
}   /* end of function  */

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
                    cy_stc_hbdma_mgr_context_t *pHbDmaMgrCtxt)
{
    uint32_t index;
    BaseType_t status = pdFALSE;
    cy_stc_app_endp_dma_set_t *pEndpInDma;
    cy_stc_app_endp_dma_set_t *pEndpOutDma;

    pAppCtxt->devState = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->prevDevState = CY_USB_DEVICE_STATE_DISABLE;
    pAppCtxt->devSpeed = CY_USBD_USB_DEV_FS;
    pAppCtxt->devAddr = 0x00;
    pAppCtxt->activeCfgNum = 0x00;
    pAppCtxt->prevAltSetting = 0x00;
    pAppCtxt->pHbDmaMgrCtxt = pHbDmaMgrCtxt;
    pAppCtxt->pCpuDmacBase = pCpuDmacBase;
    pAppCtxt->pCpuDw0Base = pCpuDw0Base;
    pAppCtxt->pCpuDw1Base = pCpuDw1Base;
    pAppCtxt->pUsbdCtxt = pUsbdCtxt;
    pAppCtxt->hbChannelCreated = false;

    for (index = 0x00; index < CY_USB_MAX_ENDP_NUMBER; index++)
    {
        pEndpInDma = &(pAppCtxt->endpInDma[index]);
        memset((void *)pEndpInDma, 0, sizeof(cy_stc_app_endp_dma_set_t));

        pEndpOutDma = &(pAppCtxt->endpOutDma[index]);
        memset((void *)pEndpOutDma, 0, sizeof(cy_stc_app_endp_dma_set_t));
    }

    /*
     * Callbacks registered with USBD layer. These callbacks will be called
     * based on appropriate event.
     */
    Cy_USB_AppRegisterCallback(pAppCtxt);

    if (!(pAppCtxt->firstInitDone))
    {

        /* Create the message queue and register it with the kernel */
        pAppCtxt->usbMsgQueue = xQueueCreate(CY_USB_DEVICE_MSG_QUEUE_SIZE,
                CY_USB_DEVICE_MSG_SIZE);
        if (pAppCtxt->usbMsgQueue == NULL) {
            DBG_APP_ERR("SLFF: Queue create failed\r\n");
            return;
        }

        vQueueAddToRegistry(pAppCtxt->usbMsgQueue, "DeviceMsgQueue");
        /* Create task and check status to confirm task created properly */
        status = xTaskCreate(Cy_Slff_AppTaskHandler, "SlaveFIFODeviceTask", 2048,
                             (void *)pAppCtxt, 5, &(pAppCtxt->slffTaskHandle));

        if (status != pdPASS)
        {
            DBG_APP_ERR("SLFF: Task create failed \r\n");
            return;
        }

        pAppCtxt->vbusDebounceTimer = xTimerCreate("VbusDebounceTimer", 200, pdFALSE,
                (void *)pAppCtxt, Cy_USB_VbusDebounceTimerCallback);
        if (pAppCtxt->vbusDebounceTimer == NULL) {
            DBG_APP_ERR("USB: Timer create failed\r\n");
            return;
        }
        DBG_APP_INFO("USB: VBus debounce timer created\r\n");
        pAppCtxt->firstInitDone = 0x01;
    }
} /* end of function */

/**
 * \name Cy_USB_AppRegisterCallback
 * \brief This function will register all calback with USBD layer.
 * \param pAppCtxt application layer context pointer.
 * \retval None
 */
void Cy_USB_AppRegisterCallback(cy_stc_usb_app_ctxt_t *pAppCtxt)
{
    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt = pAppCtxt->pUsbdCtxt;

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET, Cy_USB_AppBusResetCallback);

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESET_DONE, Cy_USB_AppBusResetDoneCallback);

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_BUS_SPEED, Cy_USB_AppBusSpeedCallback);

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SETUP, Cy_USB_AppSetupCallback);

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SUSPEND, Cy_USB_AppSuspendCallback);

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_RESUME, Cy_USB_AppResumeCallback);

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_CONFIG, Cy_USB_AppSetCfgCallback);

    Cy_USBD_RegisterCallback(pUsbdCtxt, CY_USB_USBD_CB_SET_INTF, Cy_USB_AppSetIntfCallback);

}

/**
 * \name Cy_USB_AppSetupEndpDmaParamsHs
 * \brief Configure and enable HBW DMA channels.
 * \param pAppCtxt application layer context pointer.
 * \param pEndpDscr Endpoint descriptor pointer
 * \retval None
 */
static void Cy_USB_AppSetupEndpDmaParamsHs (cy_stc_usb_app_ctxt_t *pUsbApp, uint8_t *pEndpDscr)
{
    cy_stc_hbdma_chn_config_t dmaConfig;
    cy_en_hbdma_mgr_status_t mgrStat;
    uint32_t endpNumber, dir;
    uint16_t maxPktSize;

    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNumber, &maxPktSize, &dir);

    if (endpNumber == BULK_IN_EP1_INDEX) {
        pUsbApp->slffInEpNum1   = (uint8_t)endpNumber;
        dmaConfig.size          = SLFF_RX_MAX_BUFFER_SIZE;      /* DMA Buffer Size in bytes */
        dmaConfig.count         = SLFF_RX_MAX_BUFFER_COUNT;     /* DMA Buffer Count */
        dmaConfig.bufferMode    = true;                         /* DMA buffer mode enabled */
        dmaConfig.prodHdrSize   = 0;                            /* No header to be added */
        dmaConfig.prodBufSize   = SLFF_RX_MAX_BUFFER_SIZE;      /* Same as actual buffer size */
        dmaConfig.chType        = CY_HBDMA_TYPE_IP_TO_IP;
        dmaConfig.usbMaxPktSize = maxPktSize;
        dmaConfig.eventEnable   = 0;                            /* Event enable is not supported on FX2G3 */
        dmaConfig.intrEnable    = LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_PRODUCE_EVENT_Msk |
                                  LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_CONSUME_EVENT_Msk;

        dmaConfig.consSckCount  = 1;                            /* No. of consumer Sockets */
        dmaConfig.consSck[0]    = (cy_hbdma_socket_id_t)(CY_HBDMA_USBHS_IN_EP_00 + endpNumber);
        dmaConfig.consSck[1]    = (cy_hbdma_socket_id_t)0;      /* Second consumer socket is not valid */

#if INTERLEAVE_EN
        dmaConfig.prodSckCount  = 2;                            /* No. of producer sockets */
        dmaConfig.prodSck[0]    = CY_HBDMA_LVDS_SOCKET_00;      /* Data received through LVCMOS sockets #0 and #1 */
        dmaConfig.prodSck[1]    = CY_HBDMA_LVDS_SOCKET_01;
#else
        dmaConfig.prodSckCount  = 1;                            /* No. of producer sockets */
        dmaConfig.prodSck[0]    = CY_HBDMA_LVDS_SOCKET_00;      /* Data received through LVCMOS socket #0 */
        dmaConfig.prodSck[1]    = (cy_hbdma_socket_id_t)0;      /* Second producer socket is invalid */
#endif /* INTERLEAVE_EN */

        dmaConfig.cb            = Cy_Slff_AppHbDmaRxCallback;   /* HB-DMA callback */
        dmaConfig.userCtx       = (void *)(pUsbApp);            /* Pass the application context as user context */

        mgrStat = Cy_HBDma_Channel_Create(pUsbApp->pUsbdCtxt->pHBDmaMgr,
                                          &(pUsbApp->endpInDma[endpNumber].hbDmaChannel),
                                          &dmaConfig);

        if (mgrStat != CY_HBDMA_MGR_SUCCESS) {
            DBG_APP_ERR("SLFF: BulkIn channel create failed 0x%x\r\n", mgrStat);
            return;
        } else {
            /* Store the DMA channel pointer */
            pUsbApp->hbBulkInChannel[0] = &(pUsbApp->endpInDma[endpNumber].hbDmaChannel);
        }
    }

    if (endpNumber == BULK_IN_EP2_INDEX) {
        pUsbApp->slffInEpNum2   = (uint8_t)endpNumber;
        dmaConfig.size          = SLFF_RX_MAX_BUFFER_SIZE;      /* DMA Buffer Size in bytes */
        dmaConfig.count         = SLFF_RX_MAX_BUFFER_COUNT;     /* DMA Buffer Count */
        dmaConfig.bufferMode    = true;                         /* DMA buffer mode enabled */
        dmaConfig.prodHdrSize   = 0;                            /* No header to be added */
        dmaConfig.prodBufSize   = SLFF_RX_MAX_BUFFER_SIZE;      /* Same as actual buffer size */
        dmaConfig.chType        = CY_HBDMA_TYPE_IP_TO_IP;
        dmaConfig.usbMaxPktSize = maxPktSize;
        dmaConfig.eventEnable   = 0;                            /* Event enable is not supported on FX2G3 */
        dmaConfig.intrEnable    = LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_PRODUCE_EVENT_Msk |
                                  LVDSSS_LVDS_ADAPTER_DMA_SCK_INTR_CONSUME_EVENT_Msk;
        dmaConfig.consSckCount  = 1;                            /* No. of consumer Sockets */
        dmaConfig.consSck[0]    = (cy_hbdma_socket_id_t)(CY_HBDMA_USBHS_IN_EP_00 + endpNumber);
        dmaConfig.consSck[1]    = (cy_hbdma_socket_id_t)0;      /* Second consumer socket is not valid */
        dmaConfig.prodSckCount  = 1;                            /* No. of producer sockets */
        dmaConfig.prodSck[0]    = CY_HBDMA_LVDS_SOCKET_01;      /* Data received through LVCMOS socket #1 */
        dmaConfig.prodSck[1]    = (cy_hbdma_socket_id_t)0;      /* Second producer socket is not valid */
        dmaConfig.cb            = Cy_Slff_AppHbDmaRxCallback;   /* HB-DMA callback */
        dmaConfig.userCtx       = (void *)(pUsbApp);            /* Pass the application context as user context */

        mgrStat = Cy_HBDma_Channel_Create(pUsbApp->pUsbdCtxt->pHBDmaMgr,
                                          &(pUsbApp->endpInDma[endpNumber].hbDmaChannel),
                                          &dmaConfig);

        if (mgrStat != CY_HBDMA_MGR_SUCCESS) {
            DBG_APP_ERR("SLFF: BulkIn channel create failed 0x%x\r\n", mgrStat);
            return;
        } else {
            /* Store the DMA channel pointer */
            pUsbApp->hbBulkInChannel[1] = &(pUsbApp->endpInDma[endpNumber].hbDmaChannel);
        }
    }

    /* Set flag to indicate DMA channels have been created */
    pUsbApp->hbChannelCreated = true;

    return;
} /* end of function  */


/**
 * \name Cy_USB_AppConfigureEndp
 * \brief Configure all endpoints used by application (except EP0)
 * \param pUsbdCtxt USBD layer context pointer
 * \param pEndpDscr Endpoint descriptor pointer
 * \retval None
 */
void Cy_USB_AppConfigureEndp(cy_stc_usb_usbd_ctxt_t *pUsbdCtxt, uint8_t *pEndpDscr)
{
    cy_stc_usb_endp_config_t endpConfig;
    cy_en_usb_endp_dir_t endpDirection;
    bool valid;
    uint32_t endpType;
    uint32_t endpNumber, dir;
    uint16_t maxPktSize;
    uint32_t isoPkts = 0x00;
    uint8_t interval = 0x00;
    cy_en_usbd_ret_code_t usbdRetCode;

    /* Do nothing if we do not have a valid endpoint descriptor */
    if (!Cy_USBD_EndpDscrValid(pEndpDscr)) {
        return;
    }

    Cy_USBD_GetEndpNumMaxPktDir(pEndpDscr, &endpNumber, &maxPktSize, &dir);
    endpDirection = (dir) ? CY_USB_ENDP_DIR_IN : CY_USB_ENDP_DIR_OUT;

    Cy_USBD_GetEndpType(pEndpDscr, &endpType);

    if ((CY_USB_ENDP_TYPE_ISO == endpType) || (CY_USB_ENDP_TYPE_INTR == endpType))
    {
        /* The ISOINPKS setting in the USBHS register is the actual packets per microframe value */
        isoPkts = ((*((uint8_t *)(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_MAX_PKT + 1)) & CY_USB_ENDP_ADDL_XN_MASK) >> CY_USB_ENDP_ADDL_XN_POS) + 1;
    }

    valid = 0x01;
    Cy_USBD_GetEndpInterval(pEndpDscr, &interval);

    /* Prepare endpointConfig parameter */
    endpConfig.endpType           = (cy_en_usb_endp_type_t)endpType;
    endpConfig.endpDirection      = endpDirection;
    endpConfig.valid              = valid;
    endpConfig.endpNumber         = endpNumber;
    endpConfig.maxPktSize         = (uint32_t)maxPktSize;
    endpConfig.isoPkts            = isoPkts;
    endpConfig.burstSize          = 0;
    endpConfig.streamID           = 0;
    endpConfig.interval           = interval;
    endpConfig.allowNakTillDmaRdy = true;       /* OUT endpoints only receive data once DMA channel is enabled */
    usbdRetCode = Cy_USB_USBD_EndpConfig(pUsbdCtxt, endpConfig);

    /* Print status of the endpoint configuration to help debug */
    DBG_APP_INFO("USB: Ep Number %d, configure status %x\r\n", endpNumber, usbdRetCode);
    return;
} /* end of function */

/**
 * \name Cy_USB_AppSetCfgCallback
 * \brief Callback function will be invoked by USBD when set configuration is received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer.
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetCfgCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;
    uint8_t *pActiveCfg, *pIntfDscr, *pEndpDscr;
    uint8_t index, numOfIntf, numOfEndp;
    cy_stc_usbd_app_msg_t xMsg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    DBG_APP_INFO("USB: Set Configuration CB\r\n");

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);

    /*
     * Based on type of application as well as how data flows,
     * data wire can be used so initialize datawire.
     */
    Cy_DMA_Enable(pUsbApp->pCpuDw0Base);
    Cy_DMA_Enable(pUsbApp->pCpuDw1Base);

    pActiveCfg = Cy_USB_USBD_GetActiveCfgDscr(pUsbdCtxt);
    if (!pActiveCfg)
    {
        /* Set config should be called when active config value > 0x00 */
        return;
    }
    numOfIntf = Cy_USBD_FindNumOfIntf(pActiveCfg);
    if (numOfIntf == 0x00)
    {
        return;
    }

    for (index = 0x00; index < numOfIntf; index++)
    {
        /* During Set Config command always altSetting 0 will be active */
        pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, index, 0x00);
        if (pIntfDscr == NULL)
        {
            DBG_APP_INFO("USB: Get Intf failed \r\n");
            return;
        }

        numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
        if (numOfEndp == 0x00)
        {
            DBG_APP_INFO("USB: No. of ep: 0\r\n");
            continue;
        }

        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00)
        {
            Cy_USB_AppConfigureEndp(pUsbdCtxt, pEndpDscr);
            Cy_USB_AppSetupEndpDmaParamsHs(pAppCtxt, pEndpDscr);
            numOfEndp--;
            pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));

        }
    }

    /* Register ISR for and enable DMA interrupt handlers for the DataWire channels used for USB IN endpoints */
    Cy_USB_AppInitDmaIntr(BULK_IN_EP1_INDEX, CY_USB_ENDP_DIR_IN, Cy_Slff_RxChannelDataWire_ISR);
#if DEVICE1_EN
    Cy_USB_AppInitDmaIntr(BULK_IN_EP2_INDEX, CY_USB_ENDP_DIR_IN, Cy_Slff_RxChannelDataWire_ISR);
#endif /* DEVICE1_EN */

    pUsbApp->prevDevState = CY_USB_DEVICE_STATE_CONFIGURED;
    pUsbApp->devState = CY_USB_DEVICE_STATE_CONFIGURED;

    /* Enable data from FPGA*/
    if(pUsbApp->slffInEpNum1 == BULK_IN_EP1_INDEX)
    {
        DBG_APP_INFO("SLFF: Enable Device - 0\r\n");
        xMsg.type = CY_USB_STREAMING_START;
        xMsg.data[0] = BULK_IN_EP1_INDEX;
        xQueueSendFromISR(pUsbApp->usbMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
    }
#if DEVICE1_EN
    if (pUsbApp->slffInEpNum2 == BULK_IN_EP2_INDEX)
    {
        DBG_APP_INFO("SLFF: Enable Device - 1\r\n");
        xMsg.type = CY_USB_STREAMING_START;
        xMsg.data[0] = BULK_IN_EP2_INDEX;
        xQueueSendFromISR(pUsbApp->usbMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
    }
#endif /*DEVICE1_EN */

    Cy_USBD_LpmDisable(pUsbdCtxt);
    return;
} /* end of function */

/**
 * \name Cy_USB_AppBusResetCallback
 * \brief Callback function will be invoked by USBD when bus detects RESET
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppBusResetCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    DBG_APP_INFO("USB: Bus Reset CB\r\n");

    /* Stop and destroy the high bandwidth DMA channel if present. To be done before AppInit is called */
    if (pUsbApp->hbChannelCreated)
    {
        DBG_APP_TRACE("HBDMA destroy\r\n");
        pUsbApp->hbChannelCreated = false;
        Cy_HBDma_Channel_Disable(pUsbApp->hbBulkInChannel[0]);
        Cy_HBDma_Channel_Destroy(pUsbApp->hbBulkInChannel[0]);
#if (!INTERLEAVE_EN && DEVICE1_EN)
        Cy_HBDma_Channel_Disable(pUsbApp->hbBulkInChannel[1]);
        Cy_HBDma_Channel_Destroy(pUsbApp->hbBulkInChannel[1]);
#endif /* (!INTERLEAVE_EN && DEVICE1_EN) */
    }

    /*
     * USBD layer takes care of reseting its own data structure as well as
     * takes care of calling CAL reset APIs. Application needs to take care
     * of reseting its own data structure as well as "device function".
     */
    Cy_USB_AppInit(pUsbApp, pUsbdCtxt, pUsbApp->pCpuDmacBase, pUsbApp->pCpuDw0Base, pUsbApp->pCpuDw1Base,
            pUsbApp->pHbDmaMgrCtxt);

    pUsbApp->devState = CY_USB_DEVICE_STATE_RESET;
    pUsbApp->prevDevState = CY_USB_DEVICE_STATE_RESET;

    return;
} /* end of function */

/**
 * \name Cy_USB_AppBusResetDoneCallback
 * \brief Callback function will be invoked by USBD when RESET is completed
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD layer context pointer
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppBusResetDoneCallback(void *pAppCtxt,
                                    cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                    cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    DBG_APP_INFO("USB: Bus Reset Done CB\r\n");

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devState = CY_USB_DEVICE_STATE_DEFAULT;
    pUsbApp->prevDevState = pUsbApp->devState;
    return;
} /* end of function */

/**
 * \name Cy_USB_AppBusSpeedCallback
 * \brief   Callback function will be invoked by USBD when speed is identified or
 *          speed change is detected
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppBusSpeedCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                                cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->devState = CY_USB_DEVICE_STATE_DEFAULT;
    pUsbApp->devSpeed = Cy_USBD_GetDeviceSpeed(pUsbdCtxt);
    return;
} /* end of function */

/**
 * \name Cy_USB_AppSetupCallback
 * \brief Callback function will be invoked by USBD when SETUP packet is received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetupCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                             cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;
    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    uint8_t bRequest, bReqType;
    uint8_t bType, bTarget;
    uint16_t wValue, wIndex, wLength;
    bool isReqHandled = false;
    cy_stc_usbd_app_msg_t xMsg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    cy_en_usbd_ret_code_t retStatus = CY_USBD_STATUS_SUCCESS;
    cy_en_usb_endp_dir_t epDir = CY_USB_ENDP_DIR_INVALID;
    BaseType_t status = 0;

    DBG_APP_TRACE("USB: Setup CB\r\n");

    /* Only requests addressed to the interface, class, vendor and unknown control requests are received by this
     * function. The rest are handled inside the stack itself.
     */

    /* Decode the fields from the setup request */
    bReqType = pUsbdCtxt->setupReq.bmRequest;
    bType    = ((bReqType & CY_USB_CTRL_REQ_TYPE_MASK) >> CY_USB_CTRL_REQ_TYPE_POS);
    bTarget  = (bReqType & CY_USB_CTRL_REQ_RECIPENT_OTHERS);
    bRequest = pUsbdCtxt->setupReq.bRequest;
    wValue   = pUsbdCtxt->setupReq.wValue;
    wIndex   = pUsbdCtxt->setupReq.wIndex;
    wLength  = pUsbdCtxt->setupReq.wLength;

    if (bType == CY_USB_CTRL_REQ_STD)
    {
        DBG_APP_TRACE("USB: Standard request\r\n");
        if (bRequest == CY_USB_SC_SET_FEATURE)
        {
            DBG_APP_INFO("USB: Set Feature request\r\n");
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && (wValue == 0))
            {
                DBG_APP_TRACE("USB: Set Feature request: Target-interface\r\n");
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);

                isReqHandled = true;
            }

            /* SET-FEATURE(EP-HALT) is only supported to facilitate Chapter 9 compliance tests */
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) && (wValue == CY_USB_FEATURE_ENDP_HALT))
            {
                DBG_APP_TRACE("USB: Set Feature request : Target-endpoint\r\n");
                epDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) : (CY_USB_ENDP_DIR_OUT));
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, ((uint32_t)wIndex & 0x7FUL),
                        epDir, true);

                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }
        }

        if (bRequest == CY_USB_SC_CLEAR_FEATURE)
        {
            DBG_APP_INFO("USB: Clear Feature request\r\n");
            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_INTF) && (wValue == 0))
            {
                DBG_APP_TRACE("USB: Clear Feature request: Target-interface\r\n");
                Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
                isReqHandled = true;
            }

            if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_ENDP) && (wValue == CY_USB_FEATURE_ENDP_HALT))
            {
                DBG_APP_TRACE("USB: Clear Feature request: Target-endpoint\r\n");
                epDir = ((wIndex & 0x80UL) ? (CY_USB_ENDP_DIR_IN) : (CY_USB_ENDP_DIR_OUT));

                if (((wIndex & 0x7F) == pUsbApp->slffInEpNum1) && (epDir == CY_USB_ENDP_DIR_IN))
                {
                    /* Send requests to the application task to first stop and then re-start the data stream */
                    xMsg.type = CY_USB_STREAMING_STOP;
                    xMsg.data[0] = pUsbApp->slffInEpNum1;
                    status = xQueueSendFromISR(pUsbApp->usbMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
                    ASSERT_NON_BLOCK(pdTRUE == status,status);

                    xMsg.type = CY_USB_STREAMING_START;
                    xMsg.data[0] = pUsbApp->slffInEpNum1;
                    status = xQueueSendFromISR(pUsbApp->usbMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
                    ASSERT_NON_BLOCK(pdTRUE == status,status);
                }

#if DEVICE1_EN
                if (((wIndex & 0x7F) == pUsbApp->slffInEpNum2) && (epDir == CY_USB_ENDP_DIR_IN))
                {
                    /* Send requests to the application task to first stop and then re-start the data stream */
                    xMsg.type = CY_USB_STREAMING_STOP;
                    xMsg.data[0] = pUsbApp->slffInEpNum2;
                    status = xQueueSendFromISR(pUsbApp->usbMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
                    ASSERT_NON_BLOCK(pdTRUE == status,status);

                    xMsg.type = CY_USB_STREAMING_START;
                    xMsg.data[0] = pUsbApp->slffInEpNum2;
                    status = xQueueSendFromISR(pUsbApp->usbMsgQueue, &(xMsg), &(xHigherPriorityTaskWoken));
                    ASSERT_NON_BLOCK(pdTRUE == status,status);
                }
#endif /* DEVICE1_EN */

                Cy_USBD_SendAckSetupDataStatusStage(pUsbdCtxt);
                isReqHandled = true;
            }
        }

        /* Handle Microsoft OS String Descriptor request */
        if ((bTarget == CY_USB_CTRL_REQ_RECIPENT_DEVICE) &&
                (bRequest == CY_USB_SC_GET_DESCRIPTOR) &&
                (wValue == ((CY_USB_STRING_DSCR << 8) | 0xEE))) {

            /* Make sure we do not send more data than requested */
            if (wLength > glOsString[0]) {
                wLength = glOsString[0];
            }

            DBG_APP_INFO("USB: OS String\r\n");
            retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)glOsString, wLength);
            if(retStatus == CY_USBD_STATUS_SUCCESS) {
                isReqHandled = true;
            }
        }

    }

    if (bType == CY_USB_CTRL_REQ_VENDOR) {
        /* If trying to bind to WinUSB driver, we need to support additional control requests */
        /* Handle OS Compatibility and OS Feature requests */

        if (bRequest == MS_VENDOR_CODE) {
            if (wIndex == 0x04) {
                if (wLength > *((uint16_t *)glOsCompatibilityId)) {
                    wLength = *((uint16_t *)glOsCompatibilityId);
                }

                DBG_APP_INFO("USB: OSCompat\r\n");
                retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)glOsCompatibilityId, wLength);
                if(retStatus == CY_USBD_STATUS_SUCCESS) {
                    isReqHandled = true;
                }
            }
            else if (wIndex == 0x05) {
                if (wLength > *((uint16_t *)glOsFeature)) {
                    wLength = *((uint16_t *)glOsFeature);
                }

                DBG_APP_INFO("USB: OSFeature\r\n");
                retStatus = Cy_USB_USBD_SendEndp0Data(pUsbdCtxt, (uint8_t *)glOsFeature, wLength);
                if(retStatus == CY_USBD_STATUS_SUCCESS) {
                    isReqHandled = true;
                }
            }
        }

        if (isReqHandled) {
            return;
        }
    }

    /* If Request is not handled by the callback, Stall the command */
    if (!isReqHandled)
    {
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
    }
} /* end of function */

/**
 * \name Cy_USB_AppSuspendCallback
 * \brief Callback function will be invoked by USBD when Suspend signal/message is detected
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSuspendCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;
    pUsbApp->prevDevState = pUsbApp->devState;
    pUsbApp->devState = CY_USB_DEVICE_STATE_SUSPEND;
} /* end of function */

/**
 * \name Cy_USB_AppResumeCallback
 * \brief Callback function will be invoked by USBD when Resume signal/message is detected
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppResumeCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                              cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_app_ctxt_t *pUsbApp;
    cy_en_usb_device_state_t tempState;

    pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    tempState = pUsbApp->devState;
    pUsbApp->devState = pUsbApp->prevDevState;
    pUsbApp->prevDevState = tempState;
    return;
} /* end of function */

/**
 * \name Cy_USB_AppSetIntfCallback
 * \brief Callback function will be invoked by USBD when SET_INTERFACE is  received
 * \param pAppCtxt application layer context pointer.
 * \param pUsbdCtxt USBD context
 * \param pMsg USB Message
 * \retval None
 */
void Cy_USB_AppSetIntfCallback(void *pAppCtxt, cy_stc_usb_usbd_ctxt_t *pUsbdCtxt,
                               cy_stc_usb_cal_msg_t *pMsg)
{
    cy_stc_usb_setup_req_t *pSetupReq;
    uint8_t intfNum, altSetting;
    int8_t numOfEndp;
    uint8_t *pIntfDscr, *pEndpDscr;
    uint32_t endpNumber;
    cy_en_usb_endp_dir_t endpDirection;
    cy_stc_usb_app_ctxt_t *pUsbApp = (cy_stc_usb_app_ctxt_t *)pAppCtxt;

    DBG_APP_INFO("USB: Set Interface CB\r\n");
    pSetupReq = &(pUsbdCtxt->setupReq);
    /*
     * Get interface and alt setting info. If new setting same as previous
     * then return.
     * If new alt setting came then first Unconfigure previous settings
     * and then configure new settings.
     */
    intfNum = pSetupReq->wIndex;
    altSetting = pSetupReq->wValue;

    if (altSetting == pUsbApp->prevAltSetting)
    {
        DBG_APP_INFO("USB: SameAltSetting\r\n");
        Cy_USB_USBD_EndpSetClearStall(pUsbdCtxt, 0x00, CY_USB_ENDP_DIR_IN, TRUE);
        return;
    }

    /* New altSetting is different than previous one so unconfigure previous */
    pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, intfNum, pUsbApp->prevAltSetting);
    DBG_APP_INFO("USB: Unconfig PrevAltSet\r\n");
    if (pIntfDscr == NULL)
    {
        DBG_APP_INFO("USB: pIntfDscrNull\r\n");
        return;
    }
    numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
    if (numOfEndp == 0x00)
    {
        DBG_APP_INFO("USB:prevNumEp 0\r\n");
    }
    else
    {
        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00)
        {
            if (*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS) & 0x80)
            {
                endpDirection = CY_USB_ENDP_DIR_IN;
            }
            else
            {
                endpDirection = CY_USB_ENDP_DIR_OUT;
            }
            endpNumber =
                (uint32_t)((*(pEndpDscr + CY_USB_ENDP_DSCR_OFFSET_ADDRESS)) & 0x7F);

            /* with FALSE, unconfgure previous settings */
            Cy_USBD_EnableEndp(pUsbdCtxt, endpNumber, endpDirection, FALSE);

            numOfEndp--;
            pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
        }
    }

    /* Now take care of different config with new alt setting */
    pUsbApp->prevAltSetting = altSetting;
    pIntfDscr = Cy_USBD_GetIntfDscr(pUsbdCtxt, intfNum, altSetting);
    if (pIntfDscr == NULL)
    {
        DBG_APP_INFO("USB: pIntfDscrNull\r\n");
        return;
    }

    numOfEndp = Cy_USBD_FindNumOfEndp(pIntfDscr);
    if (numOfEndp == 0x00)
    {
        DBG_APP_INFO("USB:numEp 0\r\n");
    }
    else
    {
        pUsbApp->prevAltSetting = altSetting;
        pEndpDscr = Cy_USBD_GetEndpDscr(pUsbdCtxt, pIntfDscr);
        while (numOfEndp != 0x00)
        {
            Cy_USB_AppConfigureEndp(pUsbdCtxt, pEndpDscr);
            Cy_USB_AppSetupEndpDmaParamsHs(pAppCtxt, pEndpDscr);
            numOfEndp--;
            pEndpDscr = (pEndpDscr + (*(pEndpDscr + CY_USB_DSCR_OFFSET_LEN)));
        }
    }

    return;
} /* end of function */

/**
 * \name Cy_USB_AppInitDmaIntr
 * \brief Function to register an ISR for the DMA channel associated with an endpoint
 * \param endpNumber USB endpoint number
 * \param endpDirection Endpoint direction
 * \param userIsr ISR function pointer. Can be NULL if interrupt is to be disabled.
 * \retval None
 */
void Cy_USB_AppInitDmaIntr(uint32_t endpNumber, cy_en_usb_endp_dir_t endpDirection,
                           cy_israddress userIsr)
{
    cy_stc_sysint_t intrCfg;
    if ((endpNumber > 0) && (endpNumber < CY_USB_MAX_ENDP_NUMBER))
    {
#if (!CY_CPU_CORTEX_M4)
        if (endpDirection == CY_USB_ENDP_DIR_IN)
        {
            intrCfg.intrPriority = 3;
            intrCfg.intrSrc = NvicMux1_IRQn;

            /* DW1 channels 0 onwards are used for IN endpoints */
            intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw1_0_IRQn + endpNumber);
        }
        else
        {
            intrCfg.intrPriority = 3;
            intrCfg.intrSrc = NvicMux6_IRQn;

            /* DW0 channels 0 onwards are used for OUT endpoints */
            intrCfg.cm0pSrc = (cy_en_intr_t)(cpuss_interrupts_dw0_0_IRQn + endpNumber);
        }
#else
        intrCfg.intrPriority = 5;
        if (endpDirection == CY_USB_ENDP_DIR_IN)
        {
            /* DW1 channels 0 onwards are used for IN endpoints */
            intrCfg.intrSrc = (IRQn_Type)(cpuss_interrupts_dw1_0_IRQn + endpNumber);
        }
        else
        {
            /* DW0 channels 0 onwards are used for OUT endpoints */
            intrCfg.intrSrc = (IRQn_Type)(cpuss_interrupts_dw0_0_IRQn + endpNumber);
        }
#endif /* (!CY_CPU_CORTEX_M4) */

        if (userIsr != NULL)
        {
            /* If an ISR is provided, register it and enable the interrupt */
            Cy_SysInt_Init(&intrCfg, userIsr);
            NVIC_EnableIRQ(intrCfg.intrSrc);
        }
        else
        {
            /* ISR is NULL. Disable the interrupt */
            NVIC_DisableIRQ(intrCfg.intrSrc);
        }
    }
}

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
void Cy_CheckStatus(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking)
{
    if (!condition)
    {
        /* Application failed with the error code status */
        Cy_Debug_AddToLog(1, RED);
        Cy_Debug_AddToLog(1, "Function %s failed at line %d with status = 0x%x\r\n", function, line, value);
        Cy_Debug_AddToLog(1, COLOR_RESET);

        if (isBlocking)
        {
            /* Loop indefinitely */
            for (;;)
            {
            }
        }
    }
}

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
void Cy_CheckStatusHandleFailure(const char *function, uint32_t line, uint8_t condition, uint32_t value, uint8_t isBlocking, void (*failureHandler)(void))
{
    if (!condition)
    {
        /* Application failed with the error code status */
        Cy_Debug_AddToLog(1, RED);
        Cy_Debug_AddToLog(1, "Function %s failed at line %d with status = 0x%x\r\n", function, line, value);
        Cy_Debug_AddToLog(1, COLOR_RESET);

        if(failureHandler != NULL)
        {
            (*failureHandler)();
        }
        if (isBlocking)
        {
            /* Loop indefinitely */
            for (;;)
            {
            }
        }
    }
}

/**
 * \name Cy_USB_FailHandler
 * \brief Error Handler
 * \retval None
 */
void Cy_FailHandler(void)
{
    DBG_APP_ERR("Reset Done\r\n");
}

/* [] END OF FILE */
