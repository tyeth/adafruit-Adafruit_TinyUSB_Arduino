/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2026 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* This example demonstrates per-port power switching on a USB hub using
 * TinyUSB host APIs (similar to Linux hub-ctrl/usbhubctl style control).
 *
 * Board notes:
 * - Works with TinyUSB Host capable boards such as Waveshare ESP32-P4-WIFI6,
 *   Adafruit Feather RP2040 USB Host, and Adafruit Fruit Jam.
 * - For boards with a single USB controller, Serial may be unavailable,
 *   therefore this example uses Serial1 for logging/commands.
 *
 * Commands on Serial1 (115200):
 *   list
 *   on <hub_addr> <port>
 *   off <hub_addr> <port>
 *   cycle <hub_addr> <port>
 */

#include "Adafruit_TinyUSB.h"
#include "host/hub.h"
#include <stdio.h>
#include <string.h>

#ifndef USE_TINYUSB_HOST
  #error This example requires usb stack configured as host in "Tools -> USB Stack -> Adafruit TinyUSB Host"
#endif

#define CYCLE_DELAY_MS 250
#define CMD_NAME_MAX_CHARS 7
#define HUB_POWER_SWITCHING_MASK 0x3u
#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)
#define CMD_SCANF_FMT "%" STR(CMD_NAME_MAX_CHARS) "s %d %d"

typedef struct {
  bool mounted;
  uint8_t port_count;
  bool individual_power;
  hub_desc_cs_t desc;
} hub_info_t;

Adafruit_USBH_Host USBHost;

static tusb_desc_device_t _dev_desc[CFG_TUH_DEVICE_MAX] = { 0 };
static tusb_control_request_t _hub_desc_req[CFG_TUH_DEVICE_MAX] = { 0 };
static hub_info_t _hubs[CFG_TUH_DEVICE_MAX] = { 0 };
static hub_port_status_response_t _port_status[CFG_TUH_DEVICE_MAX] = { 0 };

static bool _op_in_progress = false;
static uint8_t _op_hub_addr = 0;
static uint8_t _op_port = 0;
static bool _op_target_on = false;

static bool _cycle_pending = false;
static uint8_t _cycle_hub_addr = 0;
static uint8_t _cycle_port = 0;
static uint32_t _cycle_start_ms = 0;

static inline uint8_t _hub_index(uint8_t daddr) {
  return (uint8_t) (daddr - 1);
}

static void print_hubs(void);
static bool queue_port_power(uint8_t hub_addr, uint8_t port, bool power_on);

void setup() {
  Serial1.begin(115200);
  Serial1.println("TinyUSB Host: Hub Port Power Control");
  Serial1.println("Commands: list | on <hub> <port> | off <hub> <port> | cycle <hub> <port>");

  // Init USB Host on native controller roothub port 0.
  USBHost.begin(0);
}

void loop() {
  USBHost.task();

  if (_cycle_pending && !_op_in_progress && (millis() - _cycle_start_ms >= CYCLE_DELAY_MS)) {
    _cycle_pending = false;
    queue_port_power(_cycle_hub_addr, _cycle_port, true);
  }

  if (Serial1.available()) {
    String line = Serial1.readStringUntil('\n');
    line.trim();
    line.toLowerCase();

    if (!line.length()) return;

    if (line == "list") {
      print_hubs();
      return;
    }

    char cmd[CMD_NAME_MAX_CHARS + 1] = { 0 };
    int hub_addr = 0;
    int port = 0;
    int args = sscanf(line.c_str(), CMD_SCANF_FMT, cmd, &hub_addr, &port);

    if (args == 3 && hub_addr > 0 && hub_addr <= CFG_TUH_DEVICE_MAX && port > 0) {
      if (!strcmp(cmd, "on")) {
        queue_port_power((uint8_t) hub_addr, (uint8_t) port, true);
      } else if (!strcmp(cmd, "off")) {
        queue_port_power((uint8_t) hub_addr, (uint8_t) port, false);
      } else if (!strcmp(cmd, "cycle")) {
        if (queue_port_power((uint8_t) hub_addr, (uint8_t) port, false)) {
          _cycle_pending = true;
          _cycle_hub_addr = (uint8_t) hub_addr;
          _cycle_port = (uint8_t) port;
          _cycle_start_ms = millis();
        }
      } else {
        Serial1.println("Unknown command. Use list/on/off/cycle");
      }
    } else {
      Serial1.println("Usage: list | on <hub_addr> <port> | off <hub_addr> <port> | cycle <hub_addr> <port>");
    }
  }

  Serial1.flush();
}

static void print_hubs(void) {
  bool found = false;
  for (uint8_t daddr = 1; daddr <= CFG_TUH_DEVICE_MAX; daddr++) {
    hub_info_t* hub = &_hubs[_hub_index(daddr)];
    if (!hub->mounted) continue;

    found = true;
    Serial1.printf("Hub %u: ports=%u, power switching=%s\r\n",
                   daddr, hub->port_count,
                   hub->individual_power ? "individual" : "ganged/global");
  }

  if (!found) {
    Serial1.println("No mounted hub found");
  }
}

static void hub_port_status_complete(tuh_xfer_t* xfer);

static void hub_port_power_complete(tuh_xfer_t* xfer) {
  if (xfer->result != XFER_RESULT_SUCCESS) {
    Serial1.printf("Hub %u port %u power request failed\r\n", _op_hub_addr, _op_port);
    _op_in_progress = false;
    return;
  }

  if (!hub_port_get_status(_op_hub_addr, _op_port, &_port_status[_hub_index(_op_hub_addr)], hub_port_status_complete, 0)) {
    Serial1.printf("Hub %u port %u status request failed\r\n", _op_hub_addr, _op_port);
    _op_in_progress = false;
  }
}

static void hub_port_status_complete(tuh_xfer_t* xfer) {
  (void) xfer;

  hub_port_status_response_t* status = &_port_status[_hub_index(_op_hub_addr)];
  Serial1.printf("Hub %u port %u now %s (bit=%u)\r\n",
                 _op_hub_addr, _op_port,
                 _op_target_on ? "ON" : "OFF",
                 status->status.port_power);

  _op_in_progress = false;
}

static bool queue_port_power(uint8_t hub_addr, uint8_t port, bool power_on) {
  if (_op_in_progress) {
    Serial1.println("Busy: wait for current request to finish");
    return false;
  }

  hub_info_t* hub = &_hubs[_hub_index(hub_addr)];
  if (!hub->mounted) {
    Serial1.printf("Hub %u is not mounted\r\n", hub_addr);
    return false;
  }

  if (port > hub->port_count) {
    Serial1.printf("Hub %u has %u port(s), requested port %u\r\n", hub_addr, hub->port_count, port);
    return false;
  }

  if (!hub->individual_power) {
    Serial1.printf("Hub %u reports %s power switching, per-port control may not be supported\r\n",
                   hub_addr, "ganged/global");
  }

  _op_in_progress = true;
  _op_hub_addr = hub_addr;
  _op_port = port;
  _op_target_on = power_on;

  bool queued = power_on
                ? hub_port_set_feature(hub_addr, port, HUB_FEATURE_PORT_POWER, hub_port_power_complete, 0)
                : hub_port_clear_feature(hub_addr, port, HUB_FEATURE_PORT_POWER, hub_port_power_complete, 0);

  if (!queued) {
    _op_in_progress = false;
    Serial1.printf("Failed to queue Hub %u port %u %s request\r\n", hub_addr, port, power_on ? "ON" : "OFF");
    return false;
  }

  Serial1.printf("Queued Hub %u port %u power %s\r\n", hub_addr, port, power_on ? "ON" : "OFF");
  return true;
}

static void hub_desc_complete(tuh_xfer_t* xfer) {
  if (xfer->result != XFER_RESULT_SUCCESS) {
    Serial1.printf("Hub descriptor request failed for address %u\r\n", xfer->daddr);
    return;
  }

  hub_info_t* hub = &_hubs[_hub_index(xfer->daddr)];
  uint16_t characteristics = tu_le16toh(hub->desc.wHubCharacteristics);

  hub->mounted = true;
  hub->port_count = hub->desc.bNbrPorts;
  hub->individual_power = (characteristics & HUB_POWER_SWITCHING_MASK) == HUB_CHARS_POWER_INDIVIDUAL_SWITCHING;

  Serial1.printf("Hub mounted at address %u with %u ports (%s power switching)\r\n",
                 xfer->daddr,
                 hub->port_count,
                 hub->individual_power ? "individual" : "ganged/global");
}

static void dev_desc_complete(tuh_xfer_t* xfer) {
  if (xfer->result != XFER_RESULT_SUCCESS) {
    Serial1.printf("Device descriptor request failed for address %u\r\n", xfer->daddr);
    return;
  }

  uint8_t daddr = xfer->daddr;
  tusb_desc_device_t* desc = &_dev_desc[_hub_index(daddr)];

  if (desc->bDeviceClass != TUSB_CLASS_HUB) {
    return;
  }

  tusb_control_request_t* req = &_hub_desc_req[_hub_index(daddr)];
  memset(req, 0, sizeof(tusb_control_request_t));
  req->bmRequestType_bit.recipient = TUSB_REQ_RCPT_DEVICE;
  req->bmRequestType_bit.type = TUSB_REQ_TYPE_CLASS;
  req->bmRequestType_bit.direction = TUSB_DIR_IN;
  req->bRequest = HUB_REQUEST_GET_DESCRIPTOR;
  req->wValue = 0;
  req->wIndex = 0;
  req->wLength = sizeof(hub_desc_cs_t);

  tuh_xfer_t hub_desc_xfer;
  memset(&hub_desc_xfer, 0, sizeof(hub_desc_xfer));
  hub_desc_xfer.daddr = daddr;
  hub_desc_xfer.ep_addr = 0;
  hub_desc_xfer.setup = req;
  hub_desc_xfer.buffer = (uint8_t*) &_hubs[_hub_index(daddr)].desc;
  hub_desc_xfer.complete_cb = hub_desc_complete;
  hub_desc_xfer.user_data = 0;

  if (!tuh_control_xfer(&hub_desc_xfer)) {
    Serial1.printf("Failed to start hub descriptor request for address %u\r\n", daddr);
  }
}

// Invoked when device is mounted
void tuh_mount_cb(uint8_t daddr) {
  Serial1.printf("Device attached, address = %u\r\n", daddr);

  if (!tuh_descriptor_get_device(daddr, &_dev_desc[_hub_index(daddr)], sizeof(tusb_desc_device_t), dev_desc_complete, 0)) {
    Serial1.printf("Failed to request device descriptor for address %u\r\n", daddr);
  }
}

// Invoked when device is unmounted
void tuh_umount_cb(uint8_t daddr) {
  Serial1.printf("Device removed, address = %u\r\n", daddr);

  hub_info_t* hub = &_hubs[_hub_index(daddr)];
  hub->mounted = false;
  hub->port_count = 0;
  hub->individual_power = false;

  if (_op_hub_addr == daddr) {
    _op_in_progress = false;
  }

  if (_cycle_hub_addr == daddr) {
    _cycle_pending = false;
  }
}
