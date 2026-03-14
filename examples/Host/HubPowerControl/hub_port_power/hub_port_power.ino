/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

/* This example demonstrates USB Hub per-port power control using TinyUSB
 * Host, similar to the Linux "uhubctl" utility. It allows querying hub
 * and port status and toggling power to individual hub ports via serial
 * commands.
 *
 * Supported/Tested boards:
 *   - Adafruit Feather RP2040 USB Host
 *   - Adafruit FruitJam (RP2350)
 *   - Waveshare P4-WIFI-6
 *   - Any board with native USB host support
 *
 * Requirements:
 *   - A USB hub connected to the host port. Not all hubs support per-port
 *     power switching; many use ganged (all-or-nothing) power switching.
 *     See the uhubctl compatible device list for known-good hubs:
 *       https://github.com/mvp/uhubctl#user-content-compatible-usb-hubs
 *   - Serial console on Serial1 (UART) at 115200 baud, since native USB
 *     is used as the host port.
 *
 * Serial Commands (115200 baud via Serial1):
 *   help                    - Show available commands
 *   status                  - Show all hubs and port status
 *   power <addr> <port> on  - Turn on power to a hub port
 *   power <addr> <port> off - Turn off power to a hub port
 *
 * Example session:
 *   > status
 *   Hub addr 5: ID 214b:7250, 4 port(s), power switching: per-port
 *     Port 1: power=ON,  device=full-speed, enabled=yes
 *     Port 2: power=ON,  no device
 *     Port 3: power=OFF, no device
 *     Port 4: power=ON,  device=low-speed, enabled=yes
 *
 *   > power 5 2 off
 *   Port 2 on hub 5: power OFF
 *
 *   > power 5 2 on
 *   Port 2 on hub 5: power ON
 */

#include "Adafruit_TinyUSB.h"

#ifndef USE_TINYUSB_HOST
  #error This example requires usb stack configured as host in "Tools -> USB Stack -> Adafruit TinyUSB Host"
#endif

// Hub header for hub types and port control functions
#include "host/hub.h"

#if !defined(CFG_TUH_HUB) || CFG_TUH_HUB == 0
  #error This example requires CFG_TUH_HUB > 0 (hub support enabled in host config)
#endif

// Serial console on UART since native USB is the host port
#define SerialConsole Serial1

//--------------------------------------------------------------------+
// Configuration
//--------------------------------------------------------------------+

// Maximum serial command length
#define CMD_BUFSIZE 64

//--------------------------------------------------------------------+
// Hub Tracking
//--------------------------------------------------------------------+

typedef struct {
  bool mounted;
  uint8_t num_ports;
  bool per_port_power; // true if hub supports per-port power switching
  uint16_t pwr_on_to_good_ms; // power-on to power-good time in ms
} tracked_hub_t;

// Track up to CFG_TUH_HUB hubs
static tracked_hub_t tracked_hubs[CFG_TUH_HUB];

// Buffer for hub descriptor control transfer (static for async lifetime)
static uint8_t hub_desc_buf[sizeof(hub_desc_cs_t)] __attribute__((aligned(4)));

//--------------------------------------------------------------------+
// Async Operation State
//--------------------------------------------------------------------+

static volatile bool xfer_pending = false;

// State for iterating port status display
static uint8_t status_hub_idx = 0;
static uint8_t status_port = 0;

//--------------------------------------------------------------------+
// Serial Command Buffer
//--------------------------------------------------------------------+

static char cmd_buf[CMD_BUFSIZE];
static uint8_t cmd_len = 0;

//--------------------------------------------------------------------+
// USB Host Instance
//--------------------------------------------------------------------+

Adafruit_USBH_Host USBHost;

//--------------------------------------------------------------------+
// Forward Declarations
//--------------------------------------------------------------------+

void process_command(char* cmd);
void cmd_help(void);
void cmd_status(void);
void cmd_power(uint8_t hub_addr, uint8_t port, bool on);
void get_hub_descriptor(uint8_t hub_addr);

//--------------------------------------------------------------------+
// setup() & loop()
//--------------------------------------------------------------------+

void setup() {
  SerialConsole.begin(115200);
  SerialConsole.println("TinyUSB Host: Hub Port Power Control (uhubctl-style)");
  SerialConsole.println("Type 'help' for available commands");
  SerialConsole.println();

  // Clear hub tracking
  memset(tracked_hubs, 0, sizeof(tracked_hubs));

  // Init USB Host on native controller roothub port0
  USBHost.begin(0);
}

void loop() {
  USBHost.task();

  // Poll for newly mounted/unmounted hubs
  // Hub addresses range from (CFG_TUH_DEVICE_MAX + 1) to
  // (CFG_TUH_DEVICE_MAX + CFG_TUH_HUB) in TinyUSB
  for (uint8_t i = 0; i < CFG_TUH_HUB; i++) {
    uint8_t hub_addr = CFG_TUH_DEVICE_MAX + 1 + i;
    bool is_mounted = tuh_mounted(hub_addr);

    if (is_mounted && !tracked_hubs[i].mounted) {
      // New hub detected - fetch its descriptor
      tracked_hubs[i].mounted = true;

      uint16_t vid, pid;
      tuh_vid_pid_get(hub_addr, &vid, &pid);
      SerialConsole.printf("Hub detected at address %u (ID %04x:%04x)\r\n",
                           hub_addr, vid, pid);

      if (!xfer_pending) {
        get_hub_descriptor(hub_addr);
      }
    } else if (!is_mounted && tracked_hubs[i].mounted) {
      // Hub removed
      SerialConsole.printf("Hub removed at address %u\r\n", hub_addr);
      memset(&tracked_hubs[i], 0, sizeof(tracked_hub_t));
    }
  }

  // Read and process serial commands
  while (SerialConsole.available()) {
    char c = SerialConsole.read();
    if (c == '\n' || c == '\r') {
      if (cmd_len > 0) {
        cmd_buf[cmd_len] = '\0';
        process_command(cmd_buf);
        cmd_len = 0;
      }
    } else if (cmd_len < CMD_BUFSIZE - 1) {
      cmd_buf[cmd_len++] = c;
    }
  }

  SerialConsole.flush();
}

//--------------------------------------------------------------------+
// Hub Descriptor Retrieval
//--------------------------------------------------------------------+

static void hub_desc_complete(tuh_xfer_t* xfer) {
  xfer_pending = false;

  if (xfer->result != XFER_RESULT_SUCCESS) {
    SerialConsole.println("  Failed to get hub descriptor");
    return;
  }

  uint8_t hub_addr = xfer->daddr;
  uint8_t hub_idx = hub_addr - CFG_TUH_DEVICE_MAX - 1;
  if (hub_idx >= CFG_TUH_HUB) {
    return;
  }

  hub_desc_cs_t const* desc = (hub_desc_cs_t const*) xfer->buffer;

  tracked_hubs[hub_idx].num_ports = desc->bNbrPorts;
  tracked_hubs[hub_idx].pwr_on_to_good_ms = desc->bPwrOn2PwrGood * 2;

  // wHubCharacteristics bits [0:1] = power switching mode
  //   0 = ganged, 1 = per-port, 2/3 = no power switching
  uint8_t power_mode = desc->wHubCharacteristics & 0x03;
  tracked_hubs[hub_idx].per_port_power = (power_mode == 1);

  const char* power_mode_str;
  switch (power_mode) {
    case 0:  power_mode_str = "ganged"; break;
    case 1:  power_mode_str = "per-port"; break;
    default: power_mode_str = "not switchable"; break;
  }

  SerialConsole.printf("  Ports: %u, Power switching: %s, PwrOn2PwrGood: %u ms\r\n",
                       desc->bNbrPorts, power_mode_str,
                       tracked_hubs[hub_idx].pwr_on_to_good_ms);
}

// Fetch the hub class descriptor to determine port count and capabilities
void get_hub_descriptor(uint8_t hub_addr) {
  tusb_control_request_t request;
  memset(&request, 0, sizeof(request));
  request.bmRequestType_bit.recipient = TUSB_REQ_RCPT_DEVICE;
  request.bmRequestType_bit.type      = TUSB_REQ_TYPE_CLASS;
  request.bmRequestType_bit.direction = TUSB_DIR_IN;
  request.bRequest = HUB_REQUEST_GET_DESCRIPTOR;
  // wValue = 0 follows TinyUSB's internal hub driver convention (hub.c hub_set_config)
  request.wValue   = 0;
  request.wIndex   = 0;
  request.wLength  = sizeof(hub_desc_cs_t);

  tuh_xfer_t xfer;
  memset(&xfer, 0, sizeof(xfer));
  xfer.daddr       = hub_addr;
  xfer.ep_addr     = 0;
  xfer.setup       = &request;
  xfer.buffer      = hub_desc_buf;
  xfer.complete_cb = hub_desc_complete;
  xfer.user_data   = 0;

  xfer_pending = true;
  if (!tuh_control_xfer(&xfer)) {
    SerialConsole.println("  Failed to send hub descriptor request");
    xfer_pending = false;
  }
}

//--------------------------------------------------------------------+
// Port Status Query
//--------------------------------------------------------------------+

static void port_status_complete(tuh_xfer_t* xfer);

void cmd_status(void) {
  if (xfer_pending) {
    SerialConsole.println("Busy with previous operation, please wait...");
    return;
  }

  bool found_hub = false;

  for (uint8_t i = 0; i < CFG_TUH_HUB; i++) {
    if (!tracked_hubs[i].mounted) continue;
    found_hub = true;

    uint8_t hub_addr = CFG_TUH_DEVICE_MAX + 1 + i;
    uint16_t vid, pid;
    tuh_vid_pid_get(hub_addr, &vid, &pid);

    const char* power_str = tracked_hubs[i].per_port_power ? "per-port" : "ganged";

    SerialConsole.printf("Hub addr %u: ID %04x:%04x, %u port(s), power switching: %s\r\n",
                         hub_addr, vid, pid,
                         tracked_hubs[i].num_ports, power_str);

    // Start querying port status for the first hub found with ports
    if (tracked_hubs[i].num_ports > 0) {
      status_hub_idx = i;
      status_port = 1;

      // Note: NULL buffer is safe for specific port queries (port >= 1)
      // because hub_port_get_status() redirects to the hub driver's
      // internal buffer. Use hub_port_get_status_local() in the
      // callback to read the cached result.
      xfer_pending = true;
      if (!hub_port_get_status(hub_addr, 1, NULL, port_status_complete, 0)) {
        SerialConsole.println("  Failed to get port 1 status");
        xfer_pending = false;
      }
      return; // Will continue with other hubs after this one completes
    }
  }

  if (!found_hub) {
    SerialConsole.println("No USB hubs detected. Connect a hub and try again.");
  }
}

static void port_status_complete(tuh_xfer_t* xfer) {
  uint8_t hub_addr = CFG_TUH_DEVICE_MAX + 1 + status_hub_idx;

  if (xfer->result == XFER_RESULT_SUCCESS) {
    // Retrieve cached port status updated by the hub driver
    hub_port_status_response_t resp;
    hub_port_get_status_local(hub_addr, status_port, &resp);

    SerialConsole.printf("  Port %u: power=%s, ", status_port,
                         resp.status.port_power ? "ON " : "OFF");

    if (resp.status.connection) {
      const char* speed_str = "full";
      if (resp.status.low_speed) speed_str = "low";
      else if (resp.status.high_speed) speed_str = "high";

      SerialConsole.printf("device=%s-speed, enabled=%s",
                           speed_str,
                           resp.status.port_enable ? "yes" : "no");
    } else {
      SerialConsole.printf("no device");
    }

    if (resp.status.suspend)      SerialConsole.printf(", suspended");
    if (resp.status.over_current) SerialConsole.printf(", OVER-CURRENT");
    if (resp.status.reset)        SerialConsole.printf(", resetting");

    SerialConsole.printf("\r\n");
  } else {
    SerialConsole.printf("  Port %u: error reading status\r\n", status_port);
  }

  // Query next port
  status_port++;
  if (status_port <= tracked_hubs[status_hub_idx].num_ports) {
    if (!hub_port_get_status(hub_addr, status_port, NULL,
                             port_status_complete, 0)) {
      SerialConsole.printf("  Port %u: failed to request status\r\n",
                           status_port);
      xfer_pending = false;
    }
  } else {
    // Done with this hub, check for more hubs to display
    xfer_pending = false;
    SerialConsole.println();

    // Continue with next hub if any
    for (uint8_t i = status_hub_idx + 1; i < CFG_TUH_HUB; i++) {
      if (!tracked_hubs[i].mounted || tracked_hubs[i].num_ports == 0) continue;

      uint8_t next_addr = CFG_TUH_DEVICE_MAX + 1 + i;
      uint16_t vid, pid;
      tuh_vid_pid_get(next_addr, &vid, &pid);

      const char* power_str = tracked_hubs[i].per_port_power ? "per-port" : "ganged";
      SerialConsole.printf("Hub addr %u: ID %04x:%04x, %u port(s), power switching: %s\r\n",
                           next_addr, vid, pid,
                           tracked_hubs[i].num_ports, power_str);

      status_hub_idx = i;
      status_port = 1;

      xfer_pending = true;
      if (!hub_port_get_status(next_addr, 1, NULL, port_status_complete, 0)) {
        SerialConsole.println("  Failed to get port 1 status");
        xfer_pending = false;
      }
      return;
    }
  }
}

//--------------------------------------------------------------------+
// Port Power Control
//--------------------------------------------------------------------+

static void power_control_complete(tuh_xfer_t* xfer) {
  xfer_pending = false;

  uint8_t hub_addr = xfer->daddr;
  uint8_t port = (uint8_t) xfer->setup->wIndex;
  bool power_on = (xfer->setup->bRequest == HUB_REQUEST_SET_FEATURE);

  if (xfer->result == XFER_RESULT_SUCCESS) {
    SerialConsole.printf("Port %u on hub %u: power %s\r\n",
                         port, hub_addr, power_on ? "ON" : "OFF");
  } else {
    SerialConsole.printf("Failed to %s power on port %u of hub %u\r\n",
                         power_on ? "enable" : "disable", port, hub_addr);
  }
}

void cmd_power(uint8_t hub_addr, uint8_t port, bool on) {
  // Validate hub address range
  if (hub_addr <= CFG_TUH_DEVICE_MAX ||
      hub_addr > CFG_TUH_DEVICE_MAX + CFG_TUH_HUB) {
    SerialConsole.printf("Invalid hub address %u. Valid range: %u-%u\r\n",
                         hub_addr,
                         CFG_TUH_DEVICE_MAX + 1,
                         CFG_TUH_DEVICE_MAX + CFG_TUH_HUB);
    return;
  }

  uint8_t hub_idx = hub_addr - CFG_TUH_DEVICE_MAX - 1;
  if (!tracked_hubs[hub_idx].mounted) {
    SerialConsole.printf("No hub mounted at address %u\r\n", hub_addr);
    return;
  }

  if (port == 0 || port > tracked_hubs[hub_idx].num_ports) {
    SerialConsole.printf("Invalid port %u. Hub has %u port(s).\r\n",
                         port, tracked_hubs[hub_idx].num_ports);
    return;
  }

  if (xfer_pending) {
    SerialConsole.println("Busy with previous operation, please wait...");
    return;
  }

  if (!tracked_hubs[hub_idx].per_port_power) {
    SerialConsole.println("Note: Hub uses ganged power switching; "
                          "change may affect all ports.");
  }

  xfer_pending = true;
  bool ok;

  if (on) {
    ok = hub_port_set_feature(hub_addr, port, HUB_FEATURE_PORT_POWER,
                              power_control_complete, 0);
  } else {
    ok = hub_port_clear_feature(hub_addr, port, HUB_FEATURE_PORT_POWER,
                                power_control_complete, 0);
  }

  if (!ok) {
    SerialConsole.println("Failed to send power control request");
    xfer_pending = false;
  }
}

//--------------------------------------------------------------------+
// Command Processing
//--------------------------------------------------------------------+

void cmd_help(void) {
  SerialConsole.println("USB Hub Port Power Control (uhubctl-style)");
  SerialConsole.println("Commands:");
  SerialConsole.println("  help                    - Show this help");
  SerialConsole.println("  status                  - Show all hubs and port status");
  SerialConsole.println("  power <addr> <port> on  - Turn on port power");
  SerialConsole.println("  power <addr> <port> off - Turn off port power");
  SerialConsole.println();
  SerialConsole.println("Notes:");
  SerialConsole.println("  - <addr> is the hub device address shown by 'status'");
  SerialConsole.println("  - <port> is 1-based port number");
  SerialConsole.println("  - Not all hubs support per-port power switching");
  SerialConsole.println();
}

void process_command(char* cmd) {
  // Skip leading whitespace
  while (*cmd == ' ') cmd++;

  if (strncmp(cmd, "help", 4) == 0) {
    cmd_help();
  } else if (strncmp(cmd, "status", 6) == 0) {
    cmd_status();
  } else if (strncmp(cmd, "power ", 6) == 0) {
    // Parse: power <addr> <port> on|off
    unsigned int addr = 0, port = 0;
    char on_off[4] = {0};

    if (sscanf(cmd + 6, "%u %u %3s", &addr, &port, on_off) == 3) {
      bool on  = (strcmp(on_off, "on") == 0  || strcmp(on_off, "1") == 0);
      bool off = (strcmp(on_off, "off") == 0 || strcmp(on_off, "0") == 0);

      if (on || off) {
        cmd_power((uint8_t) addr, (uint8_t) port, on);
      } else {
        SerialConsole.println("Usage: power <addr> <port> on|off");
      }
    } else {
      SerialConsole.println("Usage: power <addr> <port> on|off");
    }
  } else if (cmd[0] != '\0') {
    SerialConsole.printf("Unknown command: '%s'\r\n", cmd);
    SerialConsole.println("Type 'help' for available commands");
  }
}

//--------------------------------------------------------------------+
// TinyUSB Host Callbacks
// Note: tuh_mount_cb() is NOT called for hub devices in TinyUSB.
// Hub detection is handled by polling in loop() above.
//--------------------------------------------------------------------+

// Called when a non-hub device is attached
void tuh_mount_cb(uint8_t daddr) {
  uint16_t vid, pid;
  tuh_vid_pid_get(daddr, &vid, &pid);
  SerialConsole.printf("Device attached at address %u (ID %04x:%04x)\r\n",
                       daddr, vid, pid);
}

// Called when a non-hub device is removed
void tuh_umount_cb(uint8_t daddr) {
  SerialConsole.printf("Device removed from address %u\r\n", daddr);
}
