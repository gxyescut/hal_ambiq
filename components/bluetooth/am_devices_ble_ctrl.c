//*****************************************************************************
//
//! @file am_devices_ble_ctrl.c
//!
//! @brief An implementation of the Apollo inteface to Controller using the IOM.
//!
//! @addtogroup cooper Controller BLE Device Driver
//! @ingroup devices
//! @{
//
//*****************************************************************************

//*****************************************************************************
//
// ${copyright}
//
// This is part of revision ${version} of the AmbiqSuite Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/init.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/util.h>
#include <am_mcu_apollo.h>
#include "am_hal_security.h"
#include "am_devices_ble_ctrl.h"
#include "ble_fw_image.h"

#include <zephyr/sys/printk.h>
//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
am_devices_ble_ctrl_buffer(2) sLengthBytes;
static am_devices_ble_ctrl_sbl_update_state_t gsSblUpdateState;
static am_devices_ble_ctrl_sbl_update_data_t     g_sFwImage =
{
    (uint8_t*)&ble_fw_image_bin,
    sizeof(ble_fw_image_bin),
    AM_DEVICES_BLE_CTRL_SBL_UPDATE_IMAGE_TYPE_FW,
    0
};
static bool g_rxAck = false;

//*****************************************************************************
//
// Global functions.
//
//*****************************************************************************
extern int spi_blocking_send(uint8_t *data, uint32_t len);

//*****************************************************************************
//
// Receive a packet from the Controller SBL.
//
//*****************************************************************************
void am_devices_ble_ctrl_handshake_recv(uint8_t* pBuf)
{
    am_secboot_wired_msghdr_t* msg;
    uint32_t crc32;
    // Verify the received data CRC
    msg = (am_secboot_wired_msghdr_t*)pBuf;
    am_hal_crc32((uint32_t)&msg->msgType, msg->length - sizeof(uint32_t), &crc32);
    gsSblUpdateState.pWorkBuf = (uint32_t*)msg;

    g_rxAck = (crc32 == msg->crc32);
}

//*****************************************************************************
//
// Send a "HELLO" packet.
//
//*****************************************************************************
void send_hello(void)
{
    am_secboot_wired_msghdr_t msg;
    msg.msgType = AM_SBL_HOST_MSG_HELLO;
    msg.length = sizeof(am_secboot_wired_msghdr_t);

    am_hal_crc32((uint32_t)&msg.msgType, msg.length - sizeof(uint32_t), &msg.crc32);
    spi_blocking_send((uint8_t*)&msg, sizeof(msg));
}

//*****************************************************************************
//
// Send a "UPDATE" packet.
//
//*****************************************************************************
void send_update(uint32_t imgBlobSize)
{
    am_sbl_host_msg_update_t msg;
    msg.msgHdr.msgType = AM_SBL_HOST_MSG_UPDATE;
    msg.msgHdr.msgLength = sizeof(am_sbl_host_msg_update_t);
    msg.imageSize = imgBlobSize;
    // Check if we are downloading a newer FW versiion
    if ((gsSblUpdateState.ui32ControllerFWImageVersion < g_sFwImage.version)
         || (gsSblUpdateState.ui32ControllerVerRollBackConfig & 0x00000001))
    {
        msg.versionNumber = g_sFwImage.version;
    }
    else
    {
        msg.versionNumber = gsSblUpdateState.ui32ControllerFWImageVersion;
    }
    msg.NumPackets = gsSblUpdateState.ui32TotalPackets + 1; // One addition packet as header will be a seperate packet

    // imageSize will be zero if Apollo4 has no available image/patch for Controller to load
    // set maxPacketSize to invalid parameter to let Controller to reply NACK and clear signature
    if ( msg.imageSize == 0 )
    {
        msg.maxPacketSize = AM_DEVICES_BLE_CTRL_SBL_UPADTE_INVALID_PSI_PKT_SIZE;
    }
    else
    {
        msg.maxPacketSize = AM_DEVICES_BLE_CTRL_SBL_UPADTE_MAX_SPI_PKT_SIZE;
    }

    am_hal_crc32((uint32_t)&msg.msgHdr.msgType, msg.msgHdr.msgLength - sizeof(uint32_t), &msg.msgHdr.msgCrc);
    spi_blocking_send((uint8_t*)&msg, sizeof(msg));
}

//*****************************************************************************
//
// Send a "Data" packet.
//
//*****************************************************************************
void send_data(uint32_t address, uint32_t size, uint32_t pktNumber)
{
    // reuse same buffer for receiving
    am_sbl_host_msg_data_t* msg = (am_sbl_host_msg_data_t*)gsSblUpdateState.pWorkBuf;
    msg->msgHdr.msgType = AM_SBL_HOST_MSG_DATA;
    msg->msgHdr.msgLength = sizeof(am_sbl_host_msg_data_t) + size;
    msg->packetNumber = pktNumber;
    memcpy((uint8_t*)msg->data, (uint8_t*)address, size);

    am_hal_crc32((uint32_t) & (msg->msgHdr.msgType), msg->msgHdr.msgLength - sizeof(uint32_t), &msg->msgHdr.msgCrc);
    spi_blocking_send((uint8_t*)msg, (sizeof(am_sbl_host_msg_data_t) + size));
}

//*****************************************************************************
//
// Send a "Reset" packet.
//
//*****************************************************************************
void send_reset(void)
{
    am_sbl_host_msg_reset_t msg;
    msg.msgHdr.msgType = AM_SBL_HOST_MSG_RESET;
    msg.msgHdr.msgLength = sizeof(am_sbl_host_msg_reset_t);

    am_hal_crc32((uint32_t)&msg.msgHdr.msgType, msg.msgHdr.msgLength - sizeof(uint32_t), &msg.msgHdr.msgCrc);
    spi_blocking_send((uint8_t*)&msg, sizeof(msg));
}

//*****************************************************************************
//
// Send a "FW Continue  packet.
//
//*****************************************************************************
void send_fwContinue(void)
{
    am_sbl_host_msg_fw_continue_t msg;
    msg.msgHdr.msgType = AM_SBL_HOST_MSG_FW_CONTINUE;
    msg.msgHdr.msgLength = sizeof(am_sbl_host_msg_fw_continue_t);

    am_hal_crc32((uint32_t)&msg.msgHdr.msgType, msg.msgHdr.msgLength - sizeof(uint32_t), &msg.msgHdr.msgCrc);
    spi_blocking_send((uint8_t*)&msg, sizeof(msg));
}

//*****************************************************************************
//
// Update the state machine based on the image to download
//
//*****************************************************************************
static bool am_devices_ble_ctrl_sbl_update_state_data(uint32_t ui32updateType)
{
    // Pointer to the data to be updated
    am_devices_ble_ctrl_sbl_update_data_t* p_sUpdateImageData = NULL;
    if ( ui32updateType == AM_DEVICES_BLE_CTRL_SBL_UPDATE_IMAGE_TYPE_FW )
    {
        p_sUpdateImageData = &g_sFwImage;
    }
    else
    {
        return false;
    }
    // Check if the data is valid
    if (    (p_sUpdateImageData != NULL)                &&
            (p_sUpdateImageData->pImageAddress != 0 )    &&
            (p_sUpdateImageData->imageSize != 0 )       &&
            (p_sUpdateImageData->imageType == ui32updateType) )
    {
        // Load the INFO 0 Patch address
        gsSblUpdateState.pImageBuf          = p_sUpdateImageData->pImageAddress;
        // Image size
        gsSblUpdateState.ui32ImageSize      = p_sUpdateImageData->imageSize;
        // image type
        gsSblUpdateState.ui32ImageType      = p_sUpdateImageData->imageType;
        // Get the size of the data without headers
        gsSblUpdateState.ui32DataSize       = gsSblUpdateState.ui32ImageSize - AM_DEVICES_BLE_CTRL_SBL_UPADTE_IMAGE_HDR_SIZE;
        // Get the start address of the data without headers
        gsSblUpdateState.pDataBuf           = gsSblUpdateState.pImageBuf + AM_DEVICES_BLE_CTRL_SBL_UPADTE_IMAGE_HDR_SIZE;
        // Calculate number of packets
        gsSblUpdateState.ui32TotalPackets   = gsSblUpdateState.ui32DataSize / AM_DEVICES_BLE_CTRL_SBL_UPADTE_MAX_SPI_PKT_SIZE;
        if (  (gsSblUpdateState.ui32DataSize % AM_DEVICES_BLE_CTRL_SBL_UPADTE_MAX_SPI_PKT_SIZE) != 0 )
        {
            gsSblUpdateState.ui32TotalPackets++;
        }
        gsSblUpdateState.ui32PacketNumber = 0;
        return true;
    }

    return false;
}
//*****************************************************************************
//
//  Initialize the Firmware Update state machine
//
//*****************************************************************************
void am_devices_ble_ctrl_fw_update_init(void)
{
    // Initialize state machine
    gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_BLE_CTRL_SBL_UPDATE_STATE_INIT;
    // Load the image address
    gsSblUpdateState.pImageBuf          = NULL;
    // Image size
    gsSblUpdateState.ui32ImageSize      = 0;
    // image type
    gsSblUpdateState.ui32ImageType      = AM_DEVICES_BLE_CTRL_SBL_UPDATE_IMAGE_TYPE_NONE;
    // Get the size of the data without headers
    gsSblUpdateState.ui32DataSize       = 0;
    // Get the start address of the data without headers
    gsSblUpdateState.pDataBuf           = NULL;
    // Calculate number of packets
    gsSblUpdateState.ui32TotalPackets   = 0;
    // Initialize Packet number in progress
    gsSblUpdateState.ui32PacketNumber   = 0;
    // Initialize the processing buffer
    gsSblUpdateState.pWorkBuf           = NULL;

    am_devices_ble_ctrl_get_fw_image(&g_sFwImage);
}

//*****************************************************************************
//
// @breif Update BLE Controller firmare
// @return uint32_t
//
//*****************************************************************************
uint32_t am_devices_ble_ctrl_fw_update(void)
{
    uint32_t     ui32dataPktSize = 0;
    uint32_t     ui32Size        = 0;
    uint32_t     ui32Ret         = AM_DEVICES_BLE_CTRL_SBL_STATUS_INIT;
    am_sbl_host_msg_status_t*    psStatusMsg;
    am_sbl_host_msg_ack_nack_t*  psAckMsg;
    switch (gsSblUpdateState.ui32SblUpdateState)
    {
        case AM_DEVICES_BLE_CTRL_SBL_UPDATE_STATE_INIT:
            //
            // Send the "HELLO" message to connect to the interface.
            //
            send_hello();
            gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_BLE_CTRL_SBL_UPDATE_STATE_HELLO;
            // Tell application that we are not done with SBL
            ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_IN_PROGRESS;
            break;
        case AM_DEVICES_BLE_CTRL_SBL_UPDATE_STATE_HELLO:
            // Read the "STATUS" response from the IOS and check for CRC Error
            if ( g_rxAck == false )
            {
                // Increment the Error Counter
                gsSblUpdateState.ui32ErrorCounter++;
                // Check if the Error has happened more than the limit
                if ( gsSblUpdateState.ui32ErrorCounter > AM_DEVICES_BLE_CTRL_SBL_MAX_COMM_ERR_COUNT )
                {
                    // Return fail
                    ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_FAIL;
                }
                else
                {
                    // Resend the previous message
                    send_hello();
                    // Tell application that we are not done with SBL
                    ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_IN_PROGRESS;
                }
            }
            else
            {
                // No CRC error and if there was one, then reset the error counter
                if ( gsSblUpdateState.ui32ErrorCounter )
                {
                    gsSblUpdateState.ui32ErrorCounter = 0;
                }
                // Check the status
                psStatusMsg = (am_sbl_host_msg_status_t*) (gsSblUpdateState.pWorkBuf);
                gsSblUpdateState.ui32ControllerSblStatus = psStatusMsg->bootStatus;
                // Get the Controller FW version
                if ( psStatusMsg->versionNumber == AM_DEVICES_BLE_CTRL_SBL_DEFAULT_FW_VERSION )
                {
                    gsSblUpdateState.ui32ControllerFWImageVersion = 0;
                }
                else
                {
                    gsSblUpdateState.ui32ControllerFWImageVersion = psStatusMsg->versionNumber;
                }
                printk("BLE Controller Info:\n");
                /*
                 * Before Controller firmware version 1.19 (0x00000113), only the lower 16-bit of 32-bit Controller firmware version
                 * word was used to identify Controller firmware. It was limited to distinguish the difference of testing binaries.
                 * To restructure the Controller firmware version to a.b.c.d from a.b may solve this problem.
                 * The higher 16-bit is used to identify the major and minor version of based release firmware.
                 * The lower 16-bit is used to identify the version for testing before next release.
                 * Originally the code only prints the lower 16-bit of FW version, need to print all the bytes
                 * based on new structure of firmware version now.
                 */
                if ((psStatusMsg->versionNumber & 0xFFFF0000) == 0)
                {
                    printk("\tFW Ver:      %d.%d\n", (psStatusMsg->versionNumber & 0xF00) >> 8, psStatusMsg->versionNumber & 0xFF);
                }
                else
                {
                    printk("\tFW Ver:      %d.%d.%d.%d\n", (psStatusMsg->versionNumber & 0xFF000000) >> 24, (psStatusMsg->versionNumber & 0xFF0000) >> 16,
                                                                        (psStatusMsg->versionNumber & 0xFF00) >> 8, psStatusMsg->versionNumber & 0xFF);
                }
                if (ui32Size == sizeof(am_sbl_host_msg_status_t))
                {
                    // Get the version rollback configuration
                    gsSblUpdateState.ui32ControllerVerRollBackConfig = psStatusMsg->verRollBackStatus;
                    printk("\tChip ID0:    0x%x\n", psStatusMsg->copperChipIdWord0);
                    printk("\tChip ID1:    0x%x\n\n", psStatusMsg->copperChipIdWord1);

                    gsSblUpdateState.ui32copperChipIdWord0 = psStatusMsg->copperChipIdWord0;
                    gsSblUpdateState.ui32copperChipIdWord1 = psStatusMsg->copperChipIdWord1;

                }
                else
                {
                    gsSblUpdateState.ui32ControllerVerRollBackConfig = 0x0;
                }

                // check if the Boot Status is success
                if ( psStatusMsg->bootStatus == AM_DEVICES_BLE_CTRL_SBL_STAT_RESP_SUCCESS )
                {
                    // Check if we have some FW available
                    if (  am_devices_ble_ctrl_sbl_update_state_data(AM_DEVICES_BLE_CTRL_SBL_UPDATE_IMAGE_TYPE_FW) == true )
                    {
                        // Check if we have a newer FW version
                        if ( psStatusMsg->versionNumber < g_sFwImage.version )
                        {
                            // We have newer FW available, Letus upgrade
                            ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_UPDATE_FW;
                            if ((g_sFwImage.version & 0xFFFF0000) == 0)
                            {
                                printk("Received new BLE Controller FW version = %d.%d Going for upgrade\n", (g_sFwImage.version & 0xF00) >> 8, g_sFwImage.version & 0xFF);
                            }
                            else
                            {
                                printk("Received new BLE Controller FW version = %d.%d.%d.%d Going for upgrade\n", (g_sFwImage.version & 0xFF000000) >> 24, (g_sFwImage.version & 0xFF0000) >> 16,
                                                                                                                            (g_sFwImage.version & 0xFF00) >> 8, g_sFwImage.version & 0xFF);
                            }
                        }
                    }
                    // If we don't have any FW or any newer FW then continue with the current FW in Controller
                    if ( ui32Ret != AM_DEVICES_BLE_CTRL_SBL_STATUS_UPDATE_FW )
                    {
                        // We don't have any other FW, so continue with one already there is Controller device
                        gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_BLE_CTRL_SBL_UPDATE_STATE_IMAGE_OK;
                        // Not done yet
                        ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_IN_PROGRESS;
                        printk("No new image to upgrade\n");
                        // Send the command to continue to FW
                        send_fwContinue();
                        printk("BLE Controller FW Auth Passed, Continue with FW\n");
                    }
                }
                else if ( psStatusMsg->bootStatus == AM_DEVICES_BLE_CTRL_SBL_STAT_RESP_FW_UPDATE_REQ )
                {
                    printk("BLE Controller Requires FW\n");
                    if (  am_devices_ble_ctrl_sbl_update_state_data(AM_DEVICES_BLE_CTRL_SBL_UPDATE_IMAGE_TYPE_FW) == true )
                    {
                        ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_UPDATE_FW;
                    }
                    else
                    {
                        ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_UPDATE_IMAGE_FAIL;
                    }
                }
                else
                {
                    printk("BLE Controller Wrong Response\n");
                    ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_FAIL;
                }
            }
            if (  (ui32Ret == AM_DEVICES_BLE_CTRL_SBL_STATUS_OK) || (ui32Ret == AM_DEVICES_BLE_CTRL_SBL_STATUS_FAIL) ||
                  (gsSblUpdateState.ui32SblUpdateState == AM_DEVICES_BLE_CTRL_SBL_UPDATE_STATE_IMAGE_OK) )
            {
                // Do nothing
            }
            else
            {
                // for the case ui32Ret == AM_DEVICES_BLE_CTRL_SBL_STATUS_UPDATE_IMAGE_FAIL,
                // it indicates Controller has available FW/Info0/Info1 signature and requests update,
                // but Apollo4 does not have such image/patch at this moment, gsSblUpdateState.ui32ImageSize should be zero.
                // Need to send_update with invalid parameter to let Controller reply NACK and clear signature

                // Update the state machine
                gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_BLE_CTRL_SBL_UPDATE_STATE_UPDATE;
                // Send the update message
                send_update(gsSblUpdateState.ui32ImageSize);
                ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_IN_PROGRESS;
            }
            break;
        case AM_DEVICES_BLE_CTRL_SBL_UPDATE_STATE_UPDATE:
            // Read the "ACK/NACK" response from the IOS and check for CRC Error
            if ( g_rxAck == false )
            {
                // Increment the Error Counter
                gsSblUpdateState.ui32ErrorCounter++;
                // Check if the Error has happened more than the limit
                if ( gsSblUpdateState.ui32ErrorCounter > AM_DEVICES_BLE_CTRL_SBL_MAX_COMM_ERR_COUNT )
                {
                    // Return fail
                    ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_FAIL;
                }
                else
                {
                    // Resend the previous message
                    send_update(gsSblUpdateState.ui32ImageSize);
                    // Tell application that we are not done with SBL
                    ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_IN_PROGRESS;
                }
            }
            else
            {
                // No CRC error and if there was one, then reset the error counter
                if ( gsSblUpdateState.ui32ErrorCounter )
                {
                    gsSblUpdateState.ui32ErrorCounter = 0;
                }
                // Get the response status
                psAckMsg = (am_sbl_host_msg_ack_nack_t*)(gsSblUpdateState.pWorkBuf);
                // Process the response
                if ( (psAckMsg->msgHdr.msgType == AM_SBL_HOST_MSG_ACK) && (NULL != gsSblUpdateState.pImageBuf))
                {
                    // Save the status
                    gsSblUpdateState.ui32ControllerSblStatus = psAckMsg->status;
                    // Change the state
                    gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_BLE_CTRL_SBL_UPDATE_STATE_DATA;
                    // Send the Encrypted image header - first 64 bytes
                    send_data((uint32_t)gsSblUpdateState.pImageBuf,
                            AM_DEVICES_BLE_CTRL_SBL_UPADTE_IMAGE_HDR_SIZE, gsSblUpdateState.ui32PacketNumber);
                    printk("BLE controller upgrade in progress, wait...\n");
                    ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_IN_PROGRESS;
                }
                else if ( (psAckMsg->msgHdr.msgType == AM_SBL_HOST_MSG_NACK) && (psAckMsg->status == AM_DEVICES_BLE_CTRL_SBL_ACK_RESP_INVALID_PARAM) )
                {
                    printk("Clear Controller Signature, reset Controller and talk with SBL again\n");
                    // Add some delay for Controller SBL to clear signature
                    // am_util_delay_ms(1200);
                    // am_devices_ble_ctrl_reset();
                    gsSblUpdateState.pImageBuf        = NULL;
                    gsSblUpdateState.ui32ImageSize    = 0;
                    gsSblUpdateState.ui32ImageType    = AM_DEVICES_BLE_CTRL_SBL_UPDATE_IMAGE_TYPE_NONE;
                    gsSblUpdateState.ui32DataSize     = 0;
                    gsSblUpdateState.pDataBuf         = NULL;
                    gsSblUpdateState.ui32TotalPackets = 0;
                    gsSblUpdateState.ui32PacketNumber = 0;

                    // Send the "HELLO" message to connect to the interface.
                    send_hello();
                    gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_BLE_CTRL_SBL_UPDATE_STATE_HELLO;
                    ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_IN_PROGRESS;
                }
                else
                {
                    printk("Update Failed !!!\n");
                    ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_FAIL;
                }
            }
            break;
        case AM_DEVICES_BLE_CTRL_SBL_UPDATE_STATE_DATA:
            // Read the "ACK/NACK" response from the IOS.
            if ( g_rxAck == false )
            {
                // Increment the Error Counter
                gsSblUpdateState.ui32ErrorCounter++;
                // Check if the Error has happened more than the limit
                if ( gsSblUpdateState.ui32ErrorCounter > AM_DEVICES_BLE_CTRL_SBL_MAX_COMM_ERR_COUNT )
                {
                    // Return fail
                    ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_FAIL;
                }
                else
                {
                    // Resend the previous message
                    if ( gsSblUpdateState.ui32PacketNumber == 0 )
                    {
                        // Send the Encrypted image header - first 64 bytes
                        send_data((uint32_t)gsSblUpdateState.pImageBuf,
                                  AM_DEVICES_BLE_CTRL_SBL_UPADTE_IMAGE_HDR_SIZE, gsSblUpdateState.ui32PacketNumber);
                    }
                    else
                    {
                        // Reset the packet counters to the previous ones, to resend the packet
                        //gsSblUpdateState.ui32TotalPackets++;
                        // increment the packet number as we have already sent the header
                        //gsSblUpdateState.ui32PacketNumber--;
                        //Check if this is the last packet - Increase by one as we have already decremented after TX
                        if (  (gsSblUpdateState.ui32TotalPackets + 1) == 1 )
                        {
                            // Get the size of the leftover data
                            ui32dataPktSize = gsSblUpdateState.ui32DataSize % AM_DEVICES_BLE_CTRL_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                            if (ui32dataPktSize == 0)
                            {
                                ui32dataPktSize = AM_DEVICES_BLE_CTRL_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                            }
                        }
                        else
                        {
                            ui32dataPktSize = AM_DEVICES_BLE_CTRL_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                        }
                        // Resend the same packet - Need to decrement the packet numbers as those are already incremented
                        send_data((uint32_t) gsSblUpdateState.pDataBuf + ( (gsSblUpdateState.ui32PacketNumber - 1) * AM_DEVICES_BLE_CTRL_SBL_UPADTE_MAX_SPI_PKT_SIZE),
                                  ui32dataPktSize, gsSblUpdateState.ui32PacketNumber);
                    }
                    // Tell application that we are not done with SBL
                    ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_IN_PROGRESS;
                }
            }
            else
            {
                // No CRC error and if there was one, then reset the error counter
                if ( gsSblUpdateState.ui32ErrorCounter )
                {
                    gsSblUpdateState.ui32ErrorCounter = 0;
                }
                // Get the response status
                psAckMsg = (am_sbl_host_msg_ack_nack_t*)(gsSblUpdateState.pWorkBuf);
                // Save the status
                gsSblUpdateState.ui32ControllerSblStatus = psAckMsg->status;
                if (  (psAckMsg->srcMsgType == AM_SBL_HOST_MSG_DATA ) || (psAckMsg->srcMsgType == AM_SBL_HOST_MSG_UPDATE_STATUS) )
                {
                    if (  (psAckMsg->status == AM_DEVICES_BLE_CTRL_SBL_ACK_RESP_SUCCESS) || (psAckMsg->status == AM_DEVICES_BLE_CTRL_SBL_ACK_RESP_SEQ) )
                    {
                        if ( gsSblUpdateState.ui32TotalPackets > 0 )
                        {
                            //Check if this is the last packet
                            if ( gsSblUpdateState.ui32TotalPackets == 1 )
                            {
                                // Get the size of the left over data
                                ui32dataPktSize = gsSblUpdateState.ui32DataSize % AM_DEVICES_BLE_CTRL_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                                if (ui32dataPktSize == 0)
                                {
                                    ui32dataPktSize = AM_DEVICES_BLE_CTRL_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                                }
                            }
                            else
                            {
                                ui32dataPktSize = AM_DEVICES_BLE_CTRL_SBL_UPADTE_MAX_SPI_PKT_SIZE;
                            }
                            send_data((uint32_t) gsSblUpdateState.pDataBuf + (gsSblUpdateState.ui32PacketNumber * AM_DEVICES_BLE_CTRL_SBL_UPADTE_MAX_SPI_PKT_SIZE),
                                      ui32dataPktSize, gsSblUpdateState.ui32PacketNumber + 1);
                            gsSblUpdateState.ui32TotalPackets--;
                            // increment the packet number as we have already sent the header
                            gsSblUpdateState.ui32PacketNumber++;
                            ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_IN_PROGRESS;
                        }
                        else
                        {
                            if ( psAckMsg->status == AM_DEVICES_BLE_CTRL_SBL_ACK_RESP_SUCCESS )
                            {
                                // If FW is updated successfuly, then jump to BLE image
                                if ( gsSblUpdateState.ui32ImageType == AM_DEVICES_BLE_CTRL_SBL_UPDATE_IMAGE_TYPE_FW )
                                {
                                    gsSblUpdateState.ui32SblUpdateState = AM_DEVICES_BLE_CTRL_SBL_UPDATE_STATE_IMAGE_OK;
                                    gsSblUpdateState.ui32ControllerFWImageVersion = g_sFwImage.version;
                                    // Not done yet
                                    ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_IN_PROGRESS;
                                    // Send the command to continue to FW
                                    send_fwContinue();
                                }
                                else
                                {
                                    ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_OK;
                                }
                            }
                            else
                            {
                                printk("Update fails status = 0x%x\n", psAckMsg->status);
                                ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_FAIL;
                            }
                        }
                    }
                    else
                    {
                        printk("Update fails status = 0x%x\n", psAckMsg->status);
                        // We have received NACK
                        ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_FAIL;
                    }
                }
                else
                {
                    // Wrong Response type
                    ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_FAIL;
                }
            }
            break;
        case AM_DEVICES_BLE_CTRL_SBL_UPDATE_STATE_IMAGE_OK:
        {
            // Read the "ACK/NACK" response from the IOS and check for CRC Error
            if ( g_rxAck == false )
            {
                // Increment the Error Counter
                gsSblUpdateState.ui32ErrorCounter++;
                // Check if the Error has happened more than the limit
                if ( gsSblUpdateState.ui32ErrorCounter > AM_DEVICES_BLE_CTRL_SBL_MAX_COMM_ERR_COUNT )
                {
                    // Return fail
                    ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_FAIL;
                }
                else
                {
                    // Resend the previous message
                    send_fwContinue();
                    // Tell application that we are not done with SBL
                    ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_IN_PROGRESS;
                }
            }
            else
            {
                // No CRC error and if there was one, then reset the error counter
                if ( gsSblUpdateState.ui32ErrorCounter )
                {
                    gsSblUpdateState.ui32ErrorCounter = 0;
                }
            }
            // Get the response status
            psAckMsg = (am_sbl_host_msg_ack_nack_t*)(gsSblUpdateState.pWorkBuf);
            // Save the status
            gsSblUpdateState.ui32ControllerSblStatus = psAckMsg->status;
            if ( psAckMsg->status == AM_DEVICES_BLE_CTRL_SBL_ACK_RESP_SUCCESS )
            {
                // FW has gone to BLE, end the SBL driver state machine
                ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_OK;
                printk("BLE Controller Init Done\n");
            }
            else
            {
                ui32Ret = AM_DEVICES_BLE_CTRL_SBL_STATUS_FAIL;
                printk("BLE Controller Init Fail\n");
            }
        }
        break;
        default:
            // Bad state, update the state machine
            break;
    }
    return ui32Ret;
}

//*****************************************************************************
//
//  Get cooper firmware image from local binary
//
//*****************************************************************************
bool am_devices_ble_ctrl_get_fw_image(am_devices_ble_ctrl_sbl_update_data_t *pFwImage )
{
    if (pFwImage != NULL)
    {
        memcpy(&g_sFwImage, pFwImage, sizeof(am_devices_ble_ctrl_sbl_update_data_t));
        // Get version from the firmware image
        g_sFwImage.version = (pFwImage->pImageAddress[27] << 24) | (pFwImage->pImageAddress[26] << 16) | (pFwImage->pImageAddress[25] << 8) | (pFwImage->pImageAddress[24]);
    }

    return (pFwImage != NULL);
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************

