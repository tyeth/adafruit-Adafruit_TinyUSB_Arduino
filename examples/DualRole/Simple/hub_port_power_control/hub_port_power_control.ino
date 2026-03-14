/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2026 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* This example demonstrates USB hub per-port power switching, similar to
 * usbhubctl on Linux, where:
 * - Device runs on the native USB controller (roothub port0)
 * - Host depends on the MCU:
 *   - rp2040/rp2350 boards such as the Adafruit Feather USB Host or Fruit Jam
 *     use Pico-PIO-USB on roothub port1
 *   - samd21/51, nrf52840, esp32 use a MAX3421E host controller on roothub port1
 *
 * Requirements:
 * - Plug in a USB hub that advertises individual port power switching support.
 * - Open the serial monitor at 115200 baud.
 * - Use the commands "status", "on <port>", and "off <port>" to inspect and
 *   change a downstream port's power state.
 */

// USBHost is defined in usbh_helper.h
#include "usbh_helper.h"

#include "host/hub.h"
#include <stdio.h>
#include <string.h>

typedef enum {
  HUB_COMMAND_NONE,
  HUB_COMMAND_STATUS_ALL,
  HUB_COMMAND_VERIFY_PORT,
} hub_command_t;

typedef struct {
  uint8_t hub_addr;
  uint8_t port_count;
  uint8_t current_port;
  hub_command_t command;
  bool mounted;
  bool ready;
  bool busy;
  hub_desc_cs_t desc;
} hub_info_t;

static hub_info_t hub_info = { 0 };
static tusb_desc_device_t device_desc[CFG_TUH_DEVICE_MAX + CFG_TUH_HUB] = { 0 };
static char command_line[32];
static uint8_t command_len = 0;

//--------------------------------------------------------------------------
// Helpers
//--------------------------------------------------------------------------
static inline uint16_t hub_characteristics(void) {
  return tu_le16toh(hub_info.desc.wHubCharacteristics);
}

static inline bool hub_has_individual_power_switching(void) {
  return (hub_characteristics() & 0x3u) == HUB_CHARS_POWER_INDIVIDUAL_SWITCHING;
}

static void print_help(void) {
  Serial.println("Commands:");
  Serial.println("  help            - show this help");
  Serial.println("  info            - print hub descriptor summary");
  Serial.println("  status          - print status for all hub ports");
  Serial.println("  on <port>       - turn a downstream port on");
  Serial.println("  off <port>      - turn a downstream port off");
}

static void print_hub_summary(void) {
  if (!hub_info.ready) {
    Serial.println("No hub descriptor available yet.");
    return;
  }

  Serial.printf("Hub address %u with %u downstream ports\r\n", hub_info.hub_addr, hub_info.port_count);
  Serial.printf("  Power switching: %s\r\n",
                hub_has_individual_power_switching() ? "individual" : "ganged");
  Serial.printf("  Power-on to power-good delay: %u ms\r\n", hub_info.desc.bPwrOn2PwrGood * 2u);
}

static void finish_command(void) {
  hub_info.busy = false;
  hub_info.command = HUB_COMMAND_NONE;
  hub_info.current_port = 0;
}

static void print_port_status(uint8_t port_num) {
  hub_port_status_response_t port_status;
  if (!hub_port_get_status_local(hub_info.hub_addr, port_num, &port_status)) {
    Serial.printf("Failed to read cached status for port %u\r\n", port_num);
    return;
  }

  Serial.printf("Port %u: power=%s connection=%s enabled=%s over-current=%s\r\n",
                port_num,
                port_status.status.port_power ? "on" : "off",
                port_status.status.connection ? "device" : "empty",
                port_status.status.port_enable ? "yes" : "no",
                port_status.status.over_current ? "yes" : "no");
}

static bool request_port_status(uint8_t port_num, hub_command_t command);

static void port_status_complete(tuh_xfer_t* xfer) {
  if (xfer->result != XFER_RESULT_SUCCESS) {
    Serial.printf("Failed to read port %u status\r\n", hub_info.current_port);
    finish_command();
    return;
  }

  print_port_status(hub_info.current_port);

  if (hub_info.command == HUB_COMMAND_STATUS_ALL && hub_info.current_port < hub_info.port_count) {
    request_port_status((uint8_t) (hub_info.current_port + 1), HUB_COMMAND_STATUS_ALL);
    return;
  }

  finish_command();
}

static bool request_port_status(uint8_t port_num, hub_command_t command) {
  if ((port_num == 0) || (port_num > hub_info.port_count)) {
    Serial.println("No hub ports are available to query.");
    finish_command();
    return false;
  }

  hub_info.busy = true;
  hub_info.command = command;
  hub_info.current_port = port_num;
  if (!hub_port_get_status(hub_info.hub_addr, port_num, NULL, port_status_complete, 0)) {
    Serial.printf("Unable to start a status request for port %u\r\n", port_num);
    finish_command();
    return false;
  }

  return true;
}

static void port_power_complete(tuh_xfer_t* xfer) {
  if (xfer->result != XFER_RESULT_SUCCESS) {
    Serial.printf("Failed to change power on port %u\r\n", hub_info.current_port);
    finish_command();
    return;
  }

  request_port_status(hub_info.current_port, HUB_COMMAND_VERIFY_PORT);
}

static bool change_port_power(uint8_t port_num, bool power_on) {
  if (!hub_info.ready) {
    Serial.println("Connect a USB hub first.");
    return false;
  }

  if (!hub_has_individual_power_switching()) {
    Serial.println("This hub reports ganged power switching, so per-port control is unavailable.");
    return false;
  }

  if ((port_num == 0) || (port_num > hub_info.port_count)) {
    Serial.printf("Port must be between 1 and %u\r\n", hub_info.port_count);
    return false;
  }

  hub_info.busy = true;
  hub_info.command = HUB_COMMAND_VERIFY_PORT;
  hub_info.current_port = port_num;

  bool const started = power_on
                         ? hub_port_set_feature(hub_info.hub_addr, port_num, HUB_FEATURE_PORT_POWER,
                                                port_power_complete, 0)
                         : hub_port_clear_feature(hub_info.hub_addr, port_num, HUB_FEATURE_PORT_POWER,
                                                  port_power_complete, 0);

  if (!started) {
    Serial.printf("Unable to start power %s on port %u\r\n", power_on ? "on" : "off", port_num);
    finish_command();
  }

  return started;
}

static void hub_descriptor_complete(tuh_xfer_t* xfer) {
  if (xfer->result != XFER_RESULT_SUCCESS) {
    Serial.printf("Failed to read hub descriptor for device %u\r\n", xfer->daddr);
    hub_info.mounted = false;
    hub_info.ready = false;
    finish_command();
    return;
  }

  hub_info.port_count = hub_info.desc.bNbrPorts;
  hub_info.ready = true;

  Serial.printf("Hub detected at address %u\r\n", hub_info.hub_addr);
  print_hub_summary();
  print_help();

  finish_command();
  if (hub_info.port_count) {
    request_port_status(1, HUB_COMMAND_STATUS_ALL);
  }
}

static void fetch_hub_descriptor(uint8_t hub_addr) {
  tusb_control_request_t const request = {
    .bmRequestType_bit = {
      .recipient = TUSB_REQ_RCPT_DEVICE,
      .type      = TUSB_REQ_TYPE_CLASS,
      .direction = TUSB_DIR_IN
    },
    .bRequest = HUB_REQUEST_GET_DESCRIPTOR,
    .wValue   = 0,
    .wIndex   = 0,
    .wLength  = sizeof(hub_desc_cs_t)
  };

  tuh_xfer_t xfer = {
    .daddr       = hub_addr,
    .ep_addr     = 0,
    .setup       = &request,
    .buffer      = (uint8_t*) &hub_info.desc,
    .complete_cb = hub_descriptor_complete,
    .user_data   = 0
  };

  hub_info.busy = true;
  hub_info.command = HUB_COMMAND_NONE;
  if (!tuh_control_xfer(&xfer)) {
    Serial.printf("Unable to request the hub descriptor for device %u\r\n", hub_addr);
    hub_info.mounted = false;
    finish_command();
  }
}

static void device_descriptor_complete(tuh_xfer_t* xfer) {
  if (xfer->result != XFER_RESULT_SUCCESS) {
    Serial.printf("Failed to read the device descriptor for address %u\r\n", xfer->daddr);
    return;
  }

  tusb_desc_device_t const* desc = &device_desc[xfer->daddr - 1];
  if (desc->bDeviceClass != TUSB_CLASS_HUB) {
    return;
  }

  if (hub_info.mounted && hub_info.hub_addr != xfer->daddr) {
    Serial.printf("Ignoring extra hub at address %u; this example manages one hub at a time.\r\n", xfer->daddr);
    return;
  }

  hub_info.mounted = true;
  hub_info.ready = false;
  hub_info.hub_addr = xfer->daddr;
  fetch_hub_descriptor(xfer->daddr);
}

static void service_serial(void) {
  while (Serial.available()) {
    char const ch = (char) Serial.read();

    if ((ch == '\r') || (ch == '\n')) {
      if (command_len == 0) {
        continue;
      }

      command_line[command_len] = 0;
      command_len = 0;

      if (hub_info.busy) {
        Serial.println("Busy talking to the hub, try again in a moment.");
        continue;
      }

      if (strcmp(command_line, "help") == 0) {
        print_help();
      } else if (strcmp(command_line, "info") == 0) {
        print_hub_summary();
      } else if (strcmp(command_line, "status") == 0) {
        if (!hub_info.ready) {
          Serial.println("Connect a USB hub first.");
        } else if (hub_info.port_count == 0) {
          Serial.println("This hub does not report any downstream ports.");
        } else {
          request_port_status(1, HUB_COMMAND_STATUS_ALL);
        }
      } else {
        char action[8];
        unsigned int port_num = 0;
        if ((sscanf(command_line, "%7s %u", action, &port_num) == 2) && (strcmp(action, "on") == 0)) {
          change_port_power((uint8_t) port_num, true);
        } else if ((sscanf(command_line, "%7s %u", action, &port_num) == 2) && (strcmp(action, "off") == 0)) {
          change_port_power((uint8_t) port_num, false);
        } else {
          Serial.printf("Unknown command: %s\r\n", command_line);
          print_help();
        }
      }
    } else if (command_len < sizeof(command_line) - 1) {
      command_line[command_len++] = ch;
    }
  }
}

//--------------------------------------------------------------------------
// Setup and loop
//--------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

#if defined(CFG_TUH_MAX3421) && CFG_TUH_MAX3421
  USBHost.begin(1);
#endif

  Serial.println("TinyUSB Dual: Hub Port Power Control Example");
  Serial.println("Connect a hub with individual port power switching support.");
  print_help();
}

#if defined(CFG_TUH_MAX3421) && CFG_TUH_MAX3421
void loop() {
  USBHost.task();
  service_serial();
  Serial.flush();
}

#elif defined(ARDUINO_ARCH_RP2040)
void loop() {
  service_serial();
}

void setup1() {
  rp2040_configure_pio_usb();
  USBHost.begin(1);
}

void loop1() {
  USBHost.task();
}
#endif

//--------------------------------------------------------------------------
// TinyUSB Host callbacks
//--------------------------------------------------------------------------
void tuh_mount_cb(uint8_t daddr) {
  Serial.printf("Device attached, address = %d\r\n", daddr);
  tuh_descriptor_get_device(daddr, &device_desc[daddr - 1], sizeof(tusb_desc_device_t),
                            device_descriptor_complete, 0);
}

void tuh_umount_cb(uint8_t daddr) {
  Serial.printf("Device removed, address = %d\r\n", daddr);

  if (hub_info.hub_addr == daddr) {
    hub_info = (hub_info_t) { 0 };
    Serial.println("Managed hub disconnected.");
  }
}
