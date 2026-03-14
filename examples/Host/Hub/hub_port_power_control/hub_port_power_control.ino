/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2024 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
 *********************************************************************/

/* USB Hub Per-Port Power control demo (TinyUSB Host)
 *
 * This sketch mimics the basic functionality of Linux's hub control
 * utilities by letting you query and toggle a hub's individual port
 * power switches.
 *
 * Tested targets:
 * - Waveshare-P4-WIFI-6 (ESP32-P4 host port)
 * - Adafruit RP2040 USB Host Feather (PIO-USB host on GPIO16/17, VBUS EN on GPIO18)
 * - Adafruit Fruit Jam (USB host capable)
 *
 * Notes:
 * - Select "Adafruit TinyUSB Host" in Tools -> USB Stack.
 * - Use Serial1 for logging (the native USB port is in host mode).
 * - If your board gates VBUS, set PIN_5V_EN / PIN_5V_EN_STATE below.
 * - For RP2040 the sketch enables PIO-USB on PIN_USB_HOST_DP (default 16).
 */

#include "Adafruit_TinyUSB.h"
#include "host/hub.h"
#include <stdlib.h>
#include <string.h>

#ifndef USE_TINYUSB_HOST
  #error This example requires usb stack configured as host in "Tools -> USB Stack -> Adafruit TinyUSB Host"
#endif

#define HOST_SERIAL Serial1
#define HOST_BAUD   115200

//--------------------------------------------------------------------+
// Optional board helpers (PIO-USB + VBUS enable)
//--------------------------------------------------------------------+
#ifdef ARDUINO_ARCH_RP2040
  #include "pio_usb.h"

  // Pin D+ for host, D- is D+ + 1
  #ifndef PIN_USB_HOST_DP
  #define PIN_USB_HOST_DP 16
  #endif

  // Pin that enables 5V to the downstream port (if your board has it)
  #ifndef PIN_5V_EN
  #define PIN_5V_EN 18
  #endif

  #ifndef PIN_5V_EN_STATE
  #define PIN_5V_EN_STATE 1
  #endif

  static void rp2040_host_setup(void) {
    // Turn on VBUS switch if present
    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, PIN_5V_EN_STATE);

    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp = PIN_USB_HOST_DP;
    USBHost.configure_pio_usb(1, &pio_cfg);
  }
#else
  static void rp2040_host_setup(void) {}
#endif

//--------------------------------------------------------------------+
// Globals
//--------------------------------------------------------------------+
Adafruit_USBH_Host USBHost;

static uint8_t hub_addr = 0;
static uint8_t hub_ports = 0;
static hub_desc_cs_t hub_descriptor = { 0 };
static tusb_desc_device_t device_descs[CFG_TUH_DEVICE_MAX] = { 0 };

enum ControlAction : uint8_t {
  ACTION_NONE      = 0,
  ACTION_POWER_ON  = 1,
  ACTION_POWER_OFF = 2,
  ACTION_STATUS    = 3
};

static char input_line[32] = { 0 };
static uint8_t input_len = 0;

static uintptr_t encode_action(ControlAction action, uint8_t port) {
  return (uintptr_t) ((action << 8) | port);
}

static ControlAction decode_action(uintptr_t value) {
  return (ControlAction) ((value >> 8) & 0xff);
}

static uint8_t decode_port(uintptr_t value) {
  return (uint8_t) (value & 0xff);
}

//--------------------------------------------------------------------+
// Forward declarations
//--------------------------------------------------------------------+
void print_help(void);
void handle_command(const char *line);
bool queue_hub_descriptor(uint8_t daddr);
void device_descriptor_complete(tuh_xfer_t *xfer);
void hub_descriptor_complete(tuh_xfer_t *xfer);
void port_status_complete(tuh_xfer_t *xfer);
void port_power_complete(tuh_xfer_t *xfer);

//--------------------------------------------------------------------+
// Setup / Loop
//--------------------------------------------------------------------+
void setup() {
  HOST_SERIAL.begin(HOST_BAUD);
  HOST_SERIAL.println("TinyUSB Host: Hub Port Power Control");

  rp2040_host_setup();

  // Init USB Host on native controller roothub port0
  USBHost.begin(0);

  print_help();
}

void loop() {
  USBHost.task();
  HOST_SERIAL.flush();

  while (HOST_SERIAL.available()) {
    char ch = HOST_SERIAL.read();
    if (ch == '\r' || ch == '\n') {
      if (input_len) {
        input_line[input_len] = '\0';
        handle_command(input_line);
        input_len = 0;
      }
    } else if (input_len < sizeof(input_line) - 1) {
      input_line[input_len++] = ch;
    }
  }
}

//--------------------------------------------------------------------+
// Command helpers
//--------------------------------------------------------------------+
void print_help(void) {
  HOST_SERIAL.println();
  HOST_SERIAL.println("Commands:");
  HOST_SERIAL.println("  ports            - print detected hub and port count");
  HOST_SERIAL.println("  status <port>    - query port power/connection state");
  HOST_SERIAL.println("  on <port>        - turn port power on");
  HOST_SERIAL.println("  off <port>       - turn port power off");
  HOST_SERIAL.println();
  HOST_SERIAL.println("Tip: connect your USB hub to the host port first.");
}

static bool hub_ready(void) {
  if (hub_addr == 0) {
    HOST_SERIAL.println("No hub detected yet. Plug a hub into the host port.");
    return false;
  }

  if (hub_ports == 0) {
    HOST_SERIAL.println("Hub detected, reading descriptor...");
    queue_hub_descriptor(hub_addr);
    return false;
  }

  return true;
}

static bool valid_port(uint8_t port) {
  if (port == 0 || port > hub_ports) {
    HOST_SERIAL.printf("Port %u is out of range (1..%u)\r\n", port, hub_ports);
    return false;
  }
  return true;
}

static void request_port_status(uint8_t port) {
  if (!hub_ready() || !valid_port(port)) return;

  if (!hub_port_get_status(hub_addr, port, NULL, port_status_complete,
                           encode_action(ACTION_STATUS, port))) {
    HOST_SERIAL.println("Unable to queue port status request (bus busy?)");
  }
}

static void set_port_power(uint8_t port, bool enable) {
  if (!hub_ready() || !valid_port(port)) return;

  bool ok = false;
  uintptr_t user = encode_action(enable ? ACTION_POWER_ON : ACTION_POWER_OFF, port);

  if (enable) {
    ok = hub_port_set_feature(hub_addr, port, HUB_FEATURE_PORT_POWER, port_power_complete, user);
  } else {
    ok = hub_port_clear_feature(hub_addr, port, HUB_FEATURE_PORT_POWER, port_power_complete, user);
  }

  if (!ok) {
    HOST_SERIAL.println("Failed to queue power request");
  }
}

void handle_command(const char *line) {
  if (!strcmp(line, "ports")) {
    if (hub_addr == 0) {
      HOST_SERIAL.println("No hub detected.");
    } else if (hub_ports == 0) {
      HOST_SERIAL.printf("Hub present at address %u, reading descriptor...\r\n", hub_addr);
      queue_hub_descriptor(hub_addr);
    } else {
      HOST_SERIAL.printf("Hub address %u, %u downstream port(s)\r\n", hub_addr, hub_ports);
    }
    return;
  }

  if (!strncmp(line, "status", 6)) {
    uint8_t port = (uint8_t) atoi(line + 6);
    request_port_status(port);
    return;
  }

  if (!strncmp(line, "on", 2)) {
    uint8_t port = (uint8_t) atoi(line + 2);
    set_port_power(port, true);
    return;
  }

  if (!strncmp(line, "off", 3)) {
    uint8_t port = (uint8_t) atoi(line + 3);
    set_port_power(port, false);
    return;
  }

  HOST_SERIAL.println("Unknown command. Type any command without args to reprint help.");
}

static void print_port_status(uint8_t port, const hub_port_status_response_t &ps) {
  const char *speed = "FS";
  if (ps.status.high_speed)      speed = "HS";
  else if (ps.status.low_speed)  speed = "LS";

  HOST_SERIAL.printf("Port %u: power=%s, enabled=%s, connected=%s, overcurrent=%s, reset=%s, speed=%s\r\n",
                     port,
                     ps.status.port_power ? "ON" : "OFF",
                     ps.status.port_enable ? "yes" : "no",
                     ps.status.connection ? "yes" : "no",
                     ps.status.over_current ? "YES" : "no",
                     ps.status.reset ? "yes" : "no",
                     speed);
}

//--------------------------------------------------------------------+
// USB Host callbacks
//--------------------------------------------------------------------+
void tuh_mount_cb(uint8_t daddr) {
  tusb_desc_device_t *desc = &device_descs[daddr - 1];

  if (!tuh_descriptor_get_device(daddr, desc, sizeof(tusb_desc_device_t), device_descriptor_complete, 0)) {
    HOST_SERIAL.printf("Failed to queue descriptor read for address %u\r\n", daddr);
  }
}

void tuh_umount_cb(uint8_t daddr) {
  HOST_SERIAL.printf("Device removed, address = %u\r\n", daddr);
  if (daddr == hub_addr) {
    hub_addr = 0;
    hub_ports = 0;
    memset(&hub_descriptor, 0, sizeof(hub_descriptor));
    HOST_SERIAL.println("Hub disconnected. Reconnect to continue.");
  }
}

//--------------------------------------------------------------------+
// Control transfer helpers
//--------------------------------------------------------------------+
bool queue_hub_descriptor(uint8_t daddr) {
  tusb_control_request_t const request = {
    .bmRequestType_bit = {
      .recipient = TUSB_REQ_RCPT_DEVICE,
      .type      = TUSB_REQ_TYPE_CLASS,
      .direction = TUSB_DIR_IN
    },
    .bRequest = HUB_REQUEST_GET_DESCRIPTOR,
    .wValue   = 0,
    .wIndex   = 0,
    .wLength  = sizeof(hub_descriptor)
  };

  tuh_xfer_t xfer = {
    .daddr       = daddr,
    .ep_addr     = 0,
    .setup       = &request,
    .buffer      = &hub_descriptor,
    .complete_cb = hub_descriptor_complete,
    .user_data   = 0
  };

  return tuh_control_xfer(&xfer);
}

void device_descriptor_complete(tuh_xfer_t *xfer) {
  if (xfer->result != XFER_RESULT_SUCCESS) {
    HOST_SERIAL.printf("Device descriptor request failed (addr %u)\r\n", xfer->daddr);
    return;
  }

  uint8_t daddr = xfer->daddr;
  tusb_desc_device_t *desc = &device_descs[daddr - 1];

  if (desc->bDeviceClass == TUSB_CLASS_HUB) {
    hub_addr = daddr;
    hub_ports = 0;
    HOST_SERIAL.printf("Hub detected on address %u, reading hub descriptor...\r\n", hub_addr);
    queue_hub_descriptor(daddr);
  } else {
    HOST_SERIAL.printf("Device %u mounted (class 0x%02X)\r\n", daddr, desc->bDeviceClass);
  }
}

void hub_descriptor_complete(tuh_xfer_t *xfer) {
  if (xfer->result != XFER_RESULT_SUCCESS) {
    HOST_SERIAL.println("Hub descriptor request failed");
    return;
  }

  uint8_t daddr = xfer->daddr;
  tusb_desc_device_t *desc = &device_descs[daddr - 1];

  if (desc->bDeviceClass != TUSB_CLASS_HUB) return;

  hub_addr = daddr;
  hub_ports = hub_descriptor.bNbrPorts;
  HOST_SERIAL.printf("Hub detected on address %u with %u port(s)\r\n", hub_addr, hub_ports);
  HOST_SERIAL.println("Use `status <port>`, `on <port>`, or `off <port>`.");
}

void port_status_complete(tuh_xfer_t *xfer) {
  uint8_t port = decode_port(xfer->user_data);

  if (xfer->result != XFER_RESULT_SUCCESS) {
    HOST_SERIAL.printf("Port %u status request failed\r\n", port);
    return;
  }

  hub_port_status_response_t status;
  hub_port_get_status_local(hub_addr, port, &status);
  print_port_status(port, status);
}

void port_power_complete(tuh_xfer_t *xfer) {
  ControlAction action = decode_action(xfer->user_data);
  uint8_t port = decode_port(xfer->user_data);

  if (xfer->result != XFER_RESULT_SUCCESS) {
    HOST_SERIAL.printf("Port %u power %s failed\r\n",
                       port, action == ACTION_POWER_OFF ? "OFF" : "ON");
    return;
  }

  HOST_SERIAL.printf("Port %u power %s\r\n",
                     port, action == ACTION_POWER_OFF ? "OFF" : "ON");
  request_port_status(port);
}
