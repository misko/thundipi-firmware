#include "em_common.h"
#include "sl_bluetooth.h"
#include "gatt_db.h"
#include "app.h"
#include "sl_app_log.h"
#include "btl_interface.h"
#include "btl_interface_storage.h"

// The advertising set handle allocated from Bluetooth stack.
static uint8_t advertising_set_handle = 0xff;

// These variables are used for the connection handle and the notification parameters to send to the OTA Server
uint8_t conn_handle;
uint8_t notification_data[1] = {0};
uint16_t notification_len = 0;
uint8_t notifyEnabled = false;

static BootloaderInformation_t bldInfo;
static BootloaderStorageSlot_t slotInfo;

/* OTA variables */
static uint32_t ota_image_position = 0;
static uint8_t ota_in_progress = 0;
static uint8_t ota_image_finished = 0;
static uint16_t ota_time_elapsed = 0;




static int32_t get_slot_info()
{
  int32_t err;

  bootloader_getInfo(&bldInfo);
  sl_app_log("Gecko bootloader version: %u.%u\r\n", (bldInfo.version & 0xFF000000) >> 24, (bldInfo.version & 0x00FF0000) >> 16);

  err = bootloader_getStorageSlotInfo(0, &slotInfo);

  if(err == BOOTLOADER_OK)
  {
      sl_app_log("Slot 0 starts @ 0x%8.8x, size %u bytes\r\n", slotInfo.address, slotInfo.length);
  }
  else
  {
      sl_app_log("Unable to get storage slot info, error %x\r\n", err);
  }

  return(err);
}

static void erase_slot_if_needed()
{
  uint32_t offset = 0;
  uint8_t buffer[256];
  int i;
  int dirty = 0;
  int32_t err = BOOTLOADER_OK;
  int num_blocks = 0;

  /* check the download area content by reading it in 256-byte blocks */

  num_blocks = slotInfo.length / 256;

  while((dirty == 0) && (offset < 256*num_blocks) && (err == BOOTLOADER_OK))
  {
    err = bootloader_readStorage(0, offset, buffer, 256);
    if(err == BOOTLOADER_OK)
    {
      i=0;
      while(i<256)
      {
        if(buffer[i++] != 0xFF)
        {
          dirty = 1;
          break;
        }
      }
      offset += 256;
    }
    sl_app_log(".");
  }

  if(err != BOOTLOADER_OK)
  {
      sl_app_log("error reading flash! %x\r\n", err);
  }
  else if(dirty)
  {
      sl_app_log("download area is not empty, erasing...\r\n");
    bootloader_eraseStorageSlot(0);
    sl_app_log("done\r\n");
  }
  else
  {
      sl_app_log("download area is empty\r\n");
  }

  return;
}

static void print_progress()
{
  // estimate transfer speed in kbps
  int kbps = ota_image_position*8/(1024*ota_time_elapsed);

  sl_app_log("pos: %u, time: %u, kbps: %u\r\n", ota_image_position, ota_time_elapsed, kbps);
}

/**************************************************************************//**
 * Application Init.
 *****************************************************************************/
void ota_init(void)
{
  /////////////////////////////////////////////////////////////////////////////
  // Put your additional application init code here!                         //
  // This is called once during start-up.                                    //
  /////////////////////////////////////////////////////////////////////////////
  /* bootloader init must be called before calling other bootloader_xxx API calls */
  bootloader_init();

  /* read slot information from bootloader */
  if(get_slot_info() == BOOTLOADER_OK)
  {
    /* the download area is erased here (if needed), prior to any connections are opened */
    erase_slot_if_needed();
  }
  else
  {
      sl_app_log("Check that you have installed correct type of Gecko bootloader!\r\n");
  }
}
