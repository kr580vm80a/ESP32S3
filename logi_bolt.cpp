#include "logi_bolt.h"

#if CONFIG_IDF_TARGET_ESP32S3
#include "usb/usb_host.h"
#include "usb/usb_helpers.h"
#include <NimBLEDevice.h>

// Forward declarations of existing KVM functions in main.cpp
void logPrint(const char* format, ...);
void updateVirtualCursorAndSend(uint8_t buttons, int16_t dx, int16_t dy, int8_t scroll, int8_t hScroll);
void keyboardNotifyCallback(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
void updateKvmPowerAndRateProfiles(String activeMac = "", bool force = false);
int getActiveClientOs();

static usb_host_client_handle_t s_usb_client_hdl = NULL;
static usb_device_handle_t s_usb_dev_hdl = NULL;
static usb_transfer_t *s_mouse_transfer = NULL;
static usb_transfer_t *s_kb_transfer = NULL;
static usb_transfer_t *s_ctrl_transfer = NULL;
static volatile bool s_ctrl_busy = false;

static bool s_is_mouse_connected = false;
static bool s_is_kb_connected = false;
static bool s_is_unifying = false;
static uint16_t s_mouse_mps = 64;
static uint16_t s_kb_mps = 64;

static void usb_ctrl_transfer_cb(usb_transfer_t *transfer) {
    s_ctrl_busy = false;
    if (transfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        logPrint("[BOLT LED] Control transfer failed status: %d", transfer->status);
    }
}

// Thread-safe rate-limiting accumulator for 1000 Hz Logi Bolt mouse input
static volatile int16_t s_accum_dx = 0;
static volatile int16_t s_accum_dy = 0;
static volatile int8_t  s_accum_scroll = 0;
static volatile int8_t  s_accum_hscroll = 0;
static volatile uint8_t s_buttons = 0;
static volatile bool    s_has_pending_data = false;
static portMUX_TYPE     s_mouse_mux = portMUX_INITIALIZER_UNLOCKED;

bool logi_bolt_is_mouse_connected() {
    return s_is_mouse_connected;
}

bool logi_bolt_is_keyboard_connected() {
    return s_is_kb_connected;
}

static void processUnifyingMouseReport(uint8_t *pData, size_t length) {
    if (!pData || length < 3) return;

    int16_t batchDx = 0;
    int16_t batchDy = 0;
    int8_t batchScroll = 0;
    int8_t batchHScroll = 0;
    uint8_t batchButtons = s_buttons;
    bool hasMovement = false;

    // Logitech Unifying batches up to eight 8-byte sub-reports in 64-byte USB transfers:
    // [0]: Report ID (0x02)
    // [1]: Buttons (Bits 0..4: Left, Right, Middle, Back, Forward)
    // [2]: Buttons high / padding (0x00)
    // [3]: X low 8 bits
    // [4]: (X high 4 bits & 0x0F) | ((Y low 4 bits & 0x0F) << 4)
    // [5]: Y high 8 bits
    // [6]: Vertical scroll (signed 8-bit)
    // [7]: Horizontal scroll (signed 8-bit)
    for (size_t offset = 0; offset + 8 <= length; offset += 8) {
        const uint8_t* r = pData + offset;
        if (r[0] != 0x02) continue;

        batchButtons = r[1] & 0x1F;

        int16_t x = (int16_t)(r[3] | ((r[4] & 0x0F) << 8));
        if (x & 0x800) x |= 0xF000;

        int16_t y = (int16_t)(((r[4] >> 4) & 0x0F) | (r[5] << 4));
        if (y & 0x800) y |= 0xF000;

        batchDx += x;
        batchDy += y;
        batchScroll += (int8_t)r[6];
        batchHScroll += (int8_t)r[7];
        hasMovement = true;
    }

    if (hasMovement) {
        portENTER_CRITICAL(&s_mouse_mux);
        s_accum_dx += batchDx;
        s_accum_dy += batchDy;
        s_accum_scroll += batchScroll;
        s_accum_hscroll += batchHScroll;
        s_buttons = batchButtons;
        s_has_pending_data = true;
        portEXIT_CRITICAL(&s_mouse_mux);
        return;
    }
}

static void processBoltMouseReport(uint8_t *pData, size_t length) {
    if (!pData || length < 3) return;

    uint8_t buttons = 0;
    int16_t x = 0;
    int16_t y = 0;
    int8_t scroll = 0;
    int8_t hScroll = 0;

    if (pData[0] == 0x02) {
        if (length >= 7) {
            // Logitech Bolt 16-bit report: [02] [btn_low] [btn_high] [dx_low] [dx_high] [dy_low] [dy_high] [v_scroll] [h_scroll]
            buttons = (pData[1] | (pData[2] << 8)) & 0x1F;
            x = (int16_t)(pData[3] | (pData[4] << 8));
            y = (int16_t)(pData[5] | (pData[6] << 8));
            scroll = (length >= 8) ? (int8_t)pData[7] : 0;
            hScroll = (length >= 9) ? (int8_t)pData[8] : 0;
        } else {
            buttons = pData[1] & 0x1F;
            x = (int8_t)pData[2];
            y = (int8_t)pData[3];
            scroll = (length >= 5) ? (int8_t)pData[4] : 0;
            hScroll = (length >= 6) ? (int8_t)pData[5] : 0;
        }
    } else if (pData[0] == 0x01 && length >= 4) {
        buttons = pData[1] & 0x1F;
        x = (int8_t)pData[2];
        y = (int8_t)pData[3];
        scroll = (length > 4) ? (int8_t)pData[4] : 0;
        hScroll = (length > 5) ? (int8_t)pData[5] : 0;
    } else {
        buttons = pData[0] & 0x1F;
        x = (int8_t)pData[1];
        y = (int8_t)pData[2];
        scroll = (length > 3) ? (int8_t)pData[3] : 0;
        hScroll = (length > 4) ? (int8_t)pData[4] : 0;
    }

    portENTER_CRITICAL(&s_mouse_mux);
    s_accum_dx += x;
    s_accum_dy += y;
    s_accum_scroll += scroll;
    s_accum_hscroll += hScroll;
    s_buttons = buttons;
    s_has_pending_data = true;
    portEXIT_CRITICAL(&s_mouse_mux);
}

static void usb_mouse_transfer_cb(usb_transfer_t *transfer) {
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED && transfer->actual_num_bytes > 0) {
        if (s_is_unifying) {
            processUnifyingMouseReport(transfer->data_buffer, transfer->actual_num_bytes);
        } else {
            processBoltMouseReport(transfer->data_buffer, transfer->actual_num_bytes);
        }
    }
    if (s_usb_dev_hdl != NULL && transfer->status != USB_TRANSFER_STATUS_NO_DEVICE) {
        transfer->num_bytes = s_mouse_mps;
        usb_host_transfer_submit(transfer);
    }
}

static void usb_kb_transfer_cb(usb_transfer_t *transfer) {
    if (transfer->status == USB_TRANSFER_STATUS_COMPLETED && transfer->actual_num_bytes > 0) {
        static uint8_t s_last_kb_report[64] = {0};
        static size_t s_last_kb_len = 0;
        
        bool isAllZero = true;
        for (int i = 0; i < transfer->actual_num_bytes; i++) {
            if (transfer->data_buffer[i] != 0) {
                isAllZero = false;
                break;
            }
        }
        
        bool isDuplicateZero = isAllZero && (s_last_kb_len == transfer->actual_num_bytes) && 
                               (memcmp(s_last_kb_report, transfer->data_buffer, transfer->actual_num_bytes) == 0);
        
        if (!isDuplicateZero) {
            memcpy(s_last_kb_report, transfer->data_buffer, min((size_t)64, (size_t)transfer->actual_num_bytes));
            s_last_kb_len = transfer->actual_num_bytes;
            keyboardNotifyCallback(nullptr, transfer->data_buffer, transfer->actual_num_bytes, false);
        }
    }
    if (s_usb_dev_hdl != NULL && transfer->status != USB_TRANSFER_STATUS_NO_DEVICE) {
        transfer->num_bytes = s_kb_mps;
        usb_host_transfer_submit(transfer);
    }
}

static void usb_host_client_event_cb(const usb_host_client_event_msg_t *event_msg, void *arg) {
    if (event_msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        logPrint("[USB HOST] *** NEW USB DEVICE DETECTED! Address: %d ***", event_msg->new_dev.address);
        esp_err_t err = usb_host_device_open(s_usb_client_hdl, event_msg->new_dev.address, &s_usb_dev_hdl);
        if (err == ESP_OK && s_usb_dev_hdl != NULL) {
            const usb_device_desc_t *dev_desc = NULL;
            err = usb_host_get_device_descriptor(s_usb_dev_hdl, &dev_desc);
            if (err == ESP_OK && dev_desc) {
                logPrint("[USB HOST] Device Opened! VID: 0x%04X, PID: 0x%04X", dev_desc->idVendor, dev_desc->idProduct);
                if (dev_desc->idVendor == 0x046D) {
                    if (dev_desc->idProduct == 0xC548) {
                        s_is_unifying = false;
                        logPrint("[USB HOST] Logi Bolt Receiver (0xC548) Confirmed!");
                    } else if (dev_desc->idProduct == 0xC52B || dev_desc->idProduct == 0xC532 ||
                               dev_desc->idProduct == 0xC534 || dev_desc->idProduct == 0xC52F) {
                        s_is_unifying = true;
                        logPrint("[USB HOST] Logitech Unifying/Nano Receiver (0x%04X) Confirmed!", dev_desc->idProduct);
                    } else {
                        s_is_unifying = false;
                        logPrint("[USB HOST] Logitech Receiver (PID: 0x%04X) Detected!", dev_desc->idProduct);
                    }
                }
            }
            
            // Automatically detect endpoint MPS (Max Packet Size) from active configuration descriptor
            const usb_config_desc_t *config_desc = NULL;
            err = usb_host_get_active_config_descriptor(s_usb_dev_hdl, &config_desc);
            if (err == ESP_OK && config_desc) {
                int offset = 0;
                const usb_ep_desc_t *ep = usb_parse_endpoint_descriptor_by_address(config_desc, 1, 0, 0x82, &offset);
                if (ep) {
                    s_mouse_mps = ep->wMaxPacketSize;
                    logPrint("[USB HOST] Mouse EP 0x82 MPS: %d", s_mouse_mps);
                } else {
                    s_mouse_mps = 64;
                }
                offset = 0;
                ep = usb_parse_endpoint_descriptor_by_address(config_desc, 0, 0, 0x81, &offset);
                if (ep) {
                    s_kb_mps = ep->wMaxPacketSize;
                    logPrint("[USB HOST] Keyboard EP 0x81 MPS: %d", s_kb_mps);
                } else {
                    s_kb_mps = 64;
                }
            } else {
                s_mouse_mps = 64;
                s_kb_mps = 64;
            }

            // Claim Interface 1 (Mouse: EP 0x82 IN)
            err = usb_host_interface_claim(s_usb_client_hdl, s_usb_dev_hdl, 1, 0);
            if (err == ESP_OK) {
                logPrint("[USB HOST] Interface 1 (Mouse) claimed successfully!");
                err = usb_host_transfer_alloc(64, 0, &s_mouse_transfer);
                if (err == ESP_OK) {
                    s_mouse_transfer->device_handle = s_usb_dev_hdl;
                    s_mouse_transfer->bEndpointAddress = 0x82; // EP 2 IN
                    s_mouse_transfer->callback = usb_mouse_transfer_cb;
                    s_mouse_transfer->context = NULL;
                    s_mouse_transfer->num_bytes = s_mouse_mps;
                    esp_err_t sub_err = usb_host_transfer_submit(s_mouse_transfer);
                    logPrint("[USB HOST] Mouse transfer submitted (rc=%d)! Ready for motion", sub_err);
                    s_is_mouse_connected = (sub_err == ESP_OK);
                }
            }

            // Claim Interface 0 (Keyboard: EP 0x81 IN)
            err = usb_host_interface_claim(s_usb_client_hdl, s_usb_dev_hdl, 0, 0);
            if (err == ESP_OK) {
                logPrint("[USB HOST] Interface 0 (Keyboard) claimed successfully!");
                err = usb_host_transfer_alloc(64, 0, &s_kb_transfer);
                if (err == ESP_OK) {
                    s_kb_transfer->device_handle = s_usb_dev_hdl;
                    s_kb_transfer->bEndpointAddress = 0x81; // EP 1 IN
                    s_kb_transfer->callback = usb_kb_transfer_cb;
                    s_kb_transfer->context = NULL;
                    s_kb_transfer->num_bytes = s_kb_mps;
                    esp_err_t sub_err = usb_host_transfer_submit(s_kb_transfer);
                    logPrint("[USB HOST] Keyboard transfer submitted (rc=%d)! Ready for typing", sub_err);
                    s_is_kb_connected = (sub_err == ESP_OK);
                }
                // Allocate control transfer for keyboard LEDs (SET_REPORT)
                err = usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + 8, 0, &s_ctrl_transfer);
                if (err == ESP_OK) {
                    s_ctrl_busy = false;
                }
            }

            updateKvmPowerAndRateProfiles("", true);
            scheduleBootCalibration();
        }
    } else if (event_msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        logPrint("[USB HOST] *** USB Device Disconnected! ***");
        s_is_mouse_connected = false;
        s_is_kb_connected = false;
        s_is_unifying = false;
        s_mouse_mps = 64;
        s_kb_mps = 64;
        if (s_mouse_transfer) {
            usb_host_transfer_free(s_mouse_transfer);
            s_mouse_transfer = NULL;
        }
        if (s_kb_transfer) {
            usb_host_transfer_free(s_kb_transfer);
            s_kb_transfer = NULL;
        }
        if (s_ctrl_transfer) {
            usb_host_transfer_free(s_ctrl_transfer);
            s_ctrl_transfer = NULL;
            s_ctrl_busy = false;
        }
        if (s_usb_dev_hdl) {
            usb_host_interface_release(s_usb_client_hdl, s_usb_dev_hdl, 0);
            usb_host_interface_release(s_usb_client_hdl, s_usb_dev_hdl, 1);
            usb_host_device_close(s_usb_client_hdl, s_usb_dev_hdl);
            s_usb_dev_hdl = NULL;
        }
        updateKvmPowerAndRateProfiles("", true);
    }
}

static void usb_lib_task(void *arg) {
    while (1) {
        uint32_t event_flags;
        esp_err_t err = usb_host_lib_handle_events(pdMS_TO_TICKS(10), &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        vTaskDelay(pdMS_TO_TICKS(1)); // Yield to allow IDLE task to feed watchdog
    }
}

static void usb_client_task(void *arg) {
    usb_host_client_config_t client_config = {
        .is_synchronous = false,
        .max_num_event_msg = 5,
        .async = {
            .client_event_callback = usb_host_client_event_cb,
            .callback_arg = NULL
        }
    };
    esp_err_t err = usb_host_client_register(&client_config, &s_usb_client_hdl);
    if (err != ESP_OK) {
        logPrint("[USB HOST] Failed to register client: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    logPrint("[USB HOST] USB Host Client Registered. Polling for Logi Bolt...");
    while (1) {
        usb_host_client_handle_events(s_usb_client_hdl, pdMS_TO_TICKS(10));
        taskYIELD();
    }
}

void logi_bolt_init() {
    logPrint("[USB HOST] Initializing ESP32-S3 USB Host stack...");
    usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        logPrint("[USB HOST] usb_host_install failed: %s", esp_err_to_name(err));
        return;
    }
    logPrint("[USB HOST] usb_host_install OK! Starting USB Host tasks on Core 1...");
    xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(usb_client_task, "usb_client", 4096, NULL, 3, NULL, 1);
}

void logi_bolt_loop() {
    static uint32_t s_lastFlush = 0;
    static uint8_t s_lastSentButtons = 0;
    uint32_t nowMs = millis();
    bool buttonChanged = (s_buttons != s_lastSentButtons);

    // Dynamic rate-matched flush strictly synchronized with active OS connection interval:
    // macOS:   15ms (exactly 1 packet per 15.00ms Apple connection event, ZERO queue backlog!)
    // Windows: 8ms (exactly 1 packet per 7.50ms Windows connection event, ZERO queue backlog!)
    uint32_t flushInterval = (getActiveClientOs() == 1 /* OS_MAC */) ? 15 : 8;

    if (s_has_pending_data && (buttonChanged || (nowMs - s_lastFlush >= flushInterval))) {
        int16_t dx = 0;
        int16_t dy = 0;
        int8_t scroll = 0;
        int8_t hScroll = 0;
        uint8_t btn = 0;

        portENTER_CRITICAL(&s_mouse_mux);
        dx = s_accum_dx; s_accum_dx = 0;
        dy = s_accum_dy; s_accum_dy = 0;
        scroll = s_accum_scroll; s_accum_scroll = 0;
        hScroll = s_accum_hscroll; s_accum_hscroll = 0;
        btn = s_buttons;
        s_has_pending_data = false;
        portEXIT_CRITICAL(&s_mouse_mux);

        s_lastFlush = nowMs;
        s_lastSentButtons = btn;
        updateVirtualCursorAndSend(btn, dx, dy, scroll, hScroll);
    }
}

void logi_bolt_set_keyboard_leds(uint8_t leds) {
    if (!s_usb_dev_hdl || !s_ctrl_transfer || !s_is_kb_connected) {
        logPrint("[USB HOST LED] Not ready: dev=%p ctrl=%p kb=%d", s_usb_dev_hdl, s_ctrl_transfer, s_is_kb_connected);
        return;
    }
    if (s_ctrl_busy) {
        logPrint("[USB HOST LED] Busy, skipping");
        return;
    }

    usb_setup_packet_t *setup = (usb_setup_packet_t *)s_ctrl_transfer->data_buffer;
    setup->bmRequestType = 0x21; // Host to Device | Class | Interface
    setup->bRequest = 0x09;      // HID SET_REPORT
    setup->wValue = 0x0200;      // Report Type: Output (0x02), Report ID: 0x00
    setup->wIndex = 0x0000;      // Interface 0
    setup->wLength = 1;          // 1 byte LED bitmask

    s_ctrl_transfer->data_buffer[sizeof(usb_setup_packet_t)] = leds;
    s_ctrl_transfer->num_bytes = sizeof(usb_setup_packet_t) + 1;
    s_ctrl_transfer->device_handle = s_usb_dev_hdl;
    s_ctrl_transfer->bEndpointAddress = 0x00; // Control EP 0
    s_ctrl_transfer->callback = usb_ctrl_transfer_cb;
    s_ctrl_transfer->context = NULL;

    s_ctrl_busy = true;
    esp_err_t err = usb_host_transfer_submit_control(s_usb_client_hdl, s_ctrl_transfer);
    if (err != ESP_OK) {
        s_ctrl_busy = false;
        logPrint("[USB HOST LED] SET_REPORT submit error rc=%d", err);
    } else {
        logPrint("[USB HOST LED] SET_REPORT submitted: 0x%02X (Caps: %d)", leds, (leds & 0x02) ? 1 : 0);
    }
}

#endif
