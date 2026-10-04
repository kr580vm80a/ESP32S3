#include "ble_server.h"
#include "ble_central.h"
#include "hid_descriptors.h"
#include "kvm_config.h"
#include "cursor_engine.h"
#include "keyboard_engine.h"
#include "cmd_processor.h"
#include "logi_bolt.h"
#include "usb_manager.h"
#include "usb_device_engine.h"
#include "led_indicator.h"
#include <esp_mac.h>
#include "nimble/nimble/host/services/gatt/include/services/gatt/ble_svc_gatt.h"
#include "nimble/nimble/host/services/gap/include/services/gap/ble_svc_gap.h"

bool isAnyPcHandshaking() {
    uint32_t now = millis();
    for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
        if (kvmClients[i].active && kvmClients[i].isHandshaking) {
            // Failsafe Watchdog: if handshake has taken > 2500ms, unlock it to prevent deadlock
            if (now - kvmClients[i].handshakeStartMs > 2500) {
                kvmClients[i].isHandshaking = false;
                logPrint("[BLE Watchdog] Handshake timeout for %s -> Failsafe unlocked", kvmClients[i].mac.c_str());
            } else {
                return true;
            }
        }
    }
    return false;
}

bool isMacInActiveLayout(const String& mac) {
    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].mac.equals(mac)) return true;
    }
    return false;
}

bool isKnownKvmClient(const String& mac) {
    if (mac.length() == 0) return false;
    if (targetMouseMac.length() > 0 && mac.equals(targetMouseMac)) return false;
    if (targetKeyboardMac.length() > 0 && mac.equals(targetKeyboardMac)) return false;
    for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
        if (kvmClients[i].mac.equals(mac)) return true;
    }
    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].mac.equals(mac)) return true;
    }
    if (NimBLEDevice::isBonded(NimBLEAddress(mac.c_str()))) {
        return true;
    }
    return false;
}

int getActiveLayoutPcCount() {
    String distinctMacs[MAX_SUPPORTED_KVM_CLIENTS];
    int count = 0;
    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].mac.length() == 0) continue;
        bool found = false;
        for (int k = 0; k < count; k++) {
            if (distinctMacs[k].equals(monitors[i].mac)) {
                found = true;
                break;
            }
        }
        if (!found && count < MAX_SUPPORTED_KVM_CLIENTS) {
            distinctMacs[count++] = monitors[i].mac;
        }
    }
    return count;
}

int getConnectedLayoutPcCount() {
    String distinctMacs[MAX_SUPPORTED_KVM_CLIENTS];
    int count = 0;
    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].mac.length() == 0) continue;
        bool found = false;
        for (int k = 0; k < count; k++) {
            if (distinctMacs[k].equals(monitors[i].mac)) {
                found = true;
                break;
            }
        }
        if (!found && count < MAX_SUPPORTED_KVM_CLIENTS) {
            distinctMacs[count++] = monitors[i].mac;
        }
    }

    int connectedCount = 0;
    for (int k = 0; k < count; k++) {
        bool isConnected = false;
        if (usb_manager_is_pc_connected() && usb_device_get_bound_mac().length() > 0 &&
            usb_device_get_bound_mac().equals(distinctMacs[k])) {
            isConnected = true;
        } else {
            for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
                if (kvmClients[i].active && kvmClients[i].mac.equals(distinctMacs[k])) {
                    isConnected = true;
                    break;
                }
            }
        }
        if (isConnected) {
            connectedCount++;
        }
    }
    return connectedCount;
}

void disconnectNonLayoutClients();

bool isConfigModeActive() {
    return isWebBleAuthenticated || (lastConfigActivityTime > 0 && (millis() - lastConfigActivityTime < 60000));
}

static bool g_webServiceActive = false;
static uint32_t g_webServiceStartTime = 0;
static bool g_webClientConnected = false;
static uint16_t g_webClientConnHandle = BLE_HS_CONN_HANDLE_NONE;

bool isWebServiceModeActive() {
    return g_webServiceActive;
}

void startHidAdvertising() {
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    if (!pAdvertising) return;
    if (pAdvertising->isAdvertising()) {
        pAdvertising->stop();
        logPrint("[BLE Server] Advertising STOPPED");
    }
    pAdvertising->reset();

    ble_svc_gap_device_name_set(BLE_DEVICE_NAME);

    pAdvertising->setName(BLE_DEVICE_NAME);
    pAdvertising->setAppearance(HID_MOUSE); // 0x03C2 Mouse Appearance to enable macOS 7.5ms low latency mode
    pAdvertising->removeServices();
    if (hidDevice && hidDevice->hidService()) {
        pAdvertising->addServiceUUID(hidDevice->hidService()->getUUID());
    }
    pAdvertising->setScanResponse(false);
    pAdvertising->start();
    logPrint("[BLE Server] Advertising HID Combo");
}

void startWebAdvertising() {
    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    if (!pAdvertising) return;
    if (pAdvertising->isAdvertising()) {
        pAdvertising->stop();
        logPrint("[BLE Server] Advertising STOPPED");
    }
    pAdvertising->reset();

    ble_svc_gap_device_name_set(BLE_WEB_SERVICE_NAME);

    NimBLEAdvertisementData advData;
    advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
    advData.setCompleteServices(NimBLEUUID(CONFIG_SERVICE_UUID));
    advData.setName(BLE_WEB_SERVICE_NAME);
    pAdvertising->setAdvertisementData(advData);
    pAdvertising->setScanResponse(false);
    pAdvertising->start();
    logPrint("[BLE Server] 🟣 Web Service Mode ACTIVE!");
}

void activateWebServiceMode() {
    if (g_webServiceActive && g_webClientConnected) {
        logPrint("[BLE Server] 🟣 Web client already connected. Holding connection.");
        return;
    }
    g_webServiceActive = true;
    g_webServiceStartTime = millis();
    g_webClientConnected = false;
    g_webClientConnHandle = BLE_HS_CONN_HANDLE_NONE;
    lastConfigActivityTime = 0;
    led_indicator_set_web_service_active(true);
    startWebAdvertising();
}

void deactivateWebServiceMode() {
    if (!g_webServiceActive) return;
    g_webServiceActive = false;
    g_webClientConnected = false;
    g_webClientConnHandle = BLE_HS_CONN_HANDLE_NONE;
    g_webServiceStartTime = 0;
    lastConfigActivityTime = 0;
    isWebBleAuthenticated = false;
    currentAuthNonce = "";
    led_indicator_set_web_service_active(false);
    logPrint("[BLE Server] ⚪ Web Service Mode DEACTIVATED. Resuming HID mode.");
    startHidAdvertising();
}

void checkWebServiceTimeout() {
    if (!g_webServiceActive) return;

    if (!g_webClientConnected) {
        if (millis() - g_webServiceStartTime >= 60000) {
            logPrint("[BLE Server] ⏳ Web Service 60s timeout expired with no browser connection.");
            deactivateWebServiceMode();
        }
    } else {
        // Connected mode: hold connection indefinitely while web page is open.
        // Inactivity watchdog: if no BLE command/ping received for 30s (e.g. browser closed/refreshed without clean disconnect), shut down.
        if (lastConfigActivityTime > 0 && (millis() - lastConfigActivityTime >= 30000)) {
            logPrint("[BLE Server] ⏳ Web client heartbeat lost (>30s no activity). Shutting down Web Service.");
            deactivateWebServiceMode();
        }
    }
}

void disconnectNonLayoutClients() {
    NimBLEServer* srv = NimBLEDevice::getServer();
    if (!srv) return;

    for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
        if (kvmClients[i].active && kvmClients[i].conn_id != BLE_HS_CONN_HANDLE_NONE) {
            if (g_webServiceActive && kvmClients[i].conn_id == g_webClientConnHandle) continue;
            if (!isMacInActiveLayout(kvmClients[i].mac)) {
                logPrint("[BLE Server] ✂ Ejecting non-layout PC %s (conn: %d) because all layout PCs are connected",
                         kvmClients[i].mac.c_str(), kvmClients[i].conn_id);
                srv->disconnect(kvmClients[i].conn_id);
            }
        }
    }
    for (int i = 0; i < MAX_NON_KVM_CLIENTS; i++) {
        if (nonKvmClients[i].conn_id != BLE_HS_CONN_HANDLE_NONE && !nonKvmClients[i].isWebConfig) {
            if (!isMacInActiveLayout(nonKvmClients[i].mac)) {
                srv->disconnect(nonKvmClients[i].conn_id);
            }
        }
    }
}

static SemaphoreHandle_t g_advMutex = NULL;

void checkAndResumeAdvertising() {
    if (g_advMutex == NULL) {
        g_advMutex = xSemaphoreCreateMutex();
    }
    if (xSemaphoreTake(g_advMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return; // Another core/task is already updating advertising safely
    }

    if (isHostConnectingPeripheral()) {
        xSemaphoreGive(g_advMutex);
        return; // Do not resume advertising while a peripheral connection/handshake is in progress!
    }

    if (g_webServiceActive) {
        if (!g_webClientConnected) {
            NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
            if (adv && !adv->isAdvertising()) {
                startWebAdvertising();
            }
        }
        xSemaphoreGive(g_advMutex);
        return;
    }

    int activeCount = 0;
    for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
        if (kvmClients[i].active) activeCount++;
    }

    int maxAllowedPcConnections = max(2, 8 - (targetMouseMac.length() > 0 ? 1 : 0) - (targetKeyboardMac.length() > 0 ? 1 : 0));
    int layoutPcs = getActiveLayoutPcCount();
    int connectedLayoutPcs = getConnectedLayoutPcCount();

    bool mouseReady = (targetMouseMac.length() == 0) || mouseConnected || logi_bolt_is_mouse_connected();
    bool kbReady = (targetKeyboardMac.length() == 0) || kbConnected;
    bool allLayoutConnected = (layoutPcs > 0) && (connectedLayoutPcs >= layoutPcs);

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();

    // If all PCs belonging to active layout are connected:
    // 1. Stop advertising (no more PCs needed, freeing radio for peripherals/traffic).
    // 2. Eject any extra PCs that do not belong to the layout.
    if (allLayoutConnected) {
        if (adv && adv->isAdvertising()) {
            adv->stop();
            logPrint("[BLE Server] Advertising STOPPED. 🛑 All layout PCs (%d/%d) connected (Mouse: %d, KB: %d)",
                     connectedLayoutPcs, layoutPcs,
                     (mouseConnected || logi_bolt_is_mouse_connected()) ? 1 : 0,
                     kbConnected ? 1 : 0);
        }
        // Eject any extra PC that is not part of active layout
        disconnectNonLayoutClients();
    } else if (activeCount < maxAllowedPcConnections) {
        // Still waiting for layout PCs (even if a non-layout PC connected, advertising stays active!)
        if (adv && !adv->isAdvertising()) {
            logPrint("[BLE Server] Advertising active (PCs: %d/%d [Layout: %d/%d] | Mouse: %d | KB: %d)", 
                     activeCount, maxAllowedPcConnections, connectedLayoutPcs, layoutPcs,
                     (mouseConnected || logi_bolt_is_mouse_connected()) ? 1 : 0, 
                     kbConnected ? 1 : 0);
            startHidAdvertising();
        }
    } else {
        if (adv && adv->isAdvertising()) {
            adv->stop();
            logPrint("[BLE Server] Advertising STOPPED. All %d connection slots full", activeCount);
        }
    }
    xSemaphoreGive(g_advMutex);
}

void markClientAsWebConfig(uint16_t connHandle) {
    if (g_webServiceActive) {
        g_webClientConnected = true;
        if (connHandle != BLE_HS_CONN_HANDLE_NONE) {
            g_webClientConnHandle = connHandle;
        }
        lastConfigActivityTime = millis();
    }
    for (int i = 0; i < MAX_NON_KVM_CLIENTS; i++) {
        if (nonKvmClients[i].conn_id != BLE_HS_CONN_HANDLE_NONE && 
            (nonKvmClients[i].conn_id == connHandle || connHandle == BLE_HS_CONN_HANDLE_NONE)) {
            if (!nonKvmClients[i].isWebConfig) {
                nonKvmClients[i].isWebConfig = true;
                logPrint("[BLE Server] 🟣 Client %s (conn: %d) identified as Web Configurator (Holding connection indefinitely)",
                         nonKvmClients[i].mac.c_str(), nonKvmClients[i].conn_id);
            }
        }
    }
}

void checkWebGracePeriod() {
    if (monitorCount == 0) return; // Allow unconfigured setup
    NimBLEServer* pServer = NimBLEDevice::getServer();
    if (!pServer) return;

    uint32_t now = millis();
    for (int i = 0; i < MAX_NON_KVM_CLIENTS; i++) {
        if (nonKvmClients[i].conn_id != BLE_HS_CONN_HANDLE_NONE) {
            // Safety check: if device is a known KVM client or is bonded/authenticated, do NOT disconnect it!
            if (isKnownKvmClient(nonKvmClients[i].mac) || NimBLEDevice::isBonded(NimBLEAddress(nonKvmClients[i].mac.c_str()))) {
                nonKvmClients[i].conn_id = BLE_HS_CONN_HANDLE_NONE;
                nonKvmClients[i].mac = "";
                nonKvmClients[i].connectedTimeMs = 0;
                nonKvmClients[i].isWebConfig = false;
                continue;
            }
            uint32_t elapsed = now - nonKvmClients[i].connectedTimeMs;
            if (!nonKvmClients[i].isWebConfig && elapsed >= 45000) { // 45 seconds to allow user to enter PIN
                logPrint("[BLE Server] ⛔ REJECTED: Unknown device %s is NOT a KVM client and no Web Config / Pairing activity after 45s! Disconnecting...",
                         nonKvmClients[i].mac.c_str());
                pServer->disconnect(nonKvmClients[i].conn_id);
                nonKvmClients[i].conn_id = BLE_HS_CONN_HANDLE_NONE;
                nonKvmClients[i].mac = "";
                nonKvmClients[i].connectedTimeMs = 0;
                nonKvmClients[i].isWebConfig = false;
            }
        }
    }
}

// --- Asynchronous BLE Link Metrics & PHY Status Logger ---
void checkAndLogPhyStatus(uint16_t connHandle, const char* deviceLabel) {
    if (connHandle == BLE_HS_CONN_HANDLE_NONE) return;
    struct PhyCheckParams {
        uint16_t handle;
        char label[32];
    };
    PhyCheckParams* params = new PhyCheckParams();
    params->handle = connHandle;
    strncpy(params->label, deviceLabel, sizeof(params->label) - 1);
    params->label[sizeof(params->label) - 1] = '\0';

    xTaskCreate([](void* param) {
        PhyCheckParams* p = (PhyCheckParams*)param;
        vTaskDelay(pdMS_TO_TICKS(1500));
        uint8_t txPhy = 0, rxPhy = 0;
        ble_gap_read_le_phy(p->handle, &txPhy, &rxPhy);
        if (txPhy != 2 && rxPhy != 2) {
            ble_gap_set_prefered_le_phy(p->handle, BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK, 0);
            vTaskDelay(pdMS_TO_TICKS(1500));
            ble_gap_read_le_phy(p->handle, &txPhy, &rxPhy);
        }
        ble_gap_conn_desc desc;
        if (ble_gap_conn_find(p->handle, &desc) == 0) {
            logPrint("[BLE SERVER] LINK METRICS %s (conn: %d) -> Itvl: %.2f ms (itvl: %d) | Latency: %d | Timeout: %d ms | TX: %s | RX: %s",
                     p->label, p->handle,
                     desc.conn_itvl * 1.25f, desc.conn_itvl,
                     desc.conn_latency,
                     desc.supervision_timeout * 10,
                     txPhy == 2 ? "2M (⚡)" : (txPhy == 1 ? "1M" : "CODED"),
                     rxPhy == 2 ? "2M (⚡)" : (rxPhy == 1 ? "1M" : "CODED"));
        }
        delete p;
        vTaskDelete(NULL);
    }, "phyCheckTask", 4096, params, 1, NULL);
}

/**
 * @brief Dynamic Link Optimization Subsystem ("Active Turbo + Background Standby")
 * When mouse is active (BLE or Logi Bolt), connected PCs are kept in Permanent Turbo (Latency 0)
 * to eliminate switch lag and ensure instantaneous 32ms/87ms border crossings.
 * When mouse is disconnected, PCs are shifted to Standby (Windows Latency 4, macOS Latency 22)
 * to conserve host battery and keep radio airtime 100% free for peripheral pairing/reconnection.
 */
void updateKvmPowerAndRateProfiles(String activeMac, bool force) {
    NimBLEServer* pServer = NimBLEDevice::getServer();
    if (!pServer) return;

    bool isMouseActive = mouseConnected || logi_bolt_is_mouse_connected();

    for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
        if (!kvmClients[i].active || kvmClients[i].conn_id == BLE_HS_CONN_HANDLE_NONE) continue;

        String clientMac = kvmClients[i].mac;
        if (targetMouseMac.length() > 0 && clientMac.equals(targetMouseMac)) continue;
        if (targetKeyboardMac.length() > 0 && clientMac.equals(targetKeyboardMac)) continue;

        // Detect OS type from monitor configuration
        int clientOs = OS_WINDOWS;
        for (int m = 0; m < monitorCount; m++) {
            if (monitors[m].mac.equals(clientMac)) {
                clientOs = monitors[m].os;
                break;
            }
        }

        bool inLayout = isMacInActiveLayout(clientMac);
        bool isUsbHost = (usb_device_is_connected() && clientMac.length() > 0 && clientMac.equals(usb_device_get_bound_mac()));
        bool shouldBeTurbo = isMouseActive && !isUsbHost && inLayout;

        uint16_t minItvl;
        uint16_t maxItvl;
        uint16_t timeout;
        uint16_t targetLatency;

        if (!inLayout) {
            // Deep Standby for PCs outside the active layout (100.00..125.00 ms, Latency: 10, Timeout: 6000 ms)
            // Keeps Windows/macOS connected without spamming radio time slots (transmits ~1 packet per 1.1s)
            minItvl = 80;
            maxItvl = 100;
            timeout = 600;
            targetLatency = 10;
        } else {
            // Rate Profiles for Active Layout PCs:
            // Mac: 15.00 ms (itvl 12). Turbo -> Latency 0 | Standby -> Latency 22
            // Windows: 7.50 ms (itvl 6). Turbo -> Latency 0 | Standby -> Latency 4
            minItvl = (clientOs == OS_MAC) ? 6 : 6;
            // Previous: maxItvl = (clientOs == OS_MAC) ? 12 : 8;
            maxItvl = (clientOs == OS_MAC) ? 12 : 6;
            timeout = (clientOs == OS_MAC) ? 216 : 600;
            targetLatency = (clientOs == OS_MAC) ? 44 : 44;
        }

        // Debounce: If a param update for this exact target mode was sent recently, don't spam while waiting for host ack
        if (kvmClients[i].isTurbo == shouldBeTurbo && (millis() - kvmClients[i].lastParamUpdateMs < 1200)) {
            continue;
        }

        // Check real hardware link state from NimBLE connection descriptor
        ble_gap_conn_desc desc;
        bool hasDesc = (ble_gap_conn_find(kvmClients[i].conn_id, &desc) == 0);
        bool alreadyTarget = hasDesc &&
                      desc.conn_latency == targetLatency &&
                      desc.conn_itvl >= minItvl && desc.conn_itvl <= maxItvl;
        kvmClients[i].isTurbo = shouldBeTurbo;
        if (!alreadyTarget) {
            kvmClients[i].lastParamUpdateMs = millis();
            pServer->updateConnParams(kvmClients[i].conn_id, minItvl, maxItvl, targetLatency, timeout);
            logPrint("[BLE Server] %s for %s PC: %s (Target: %.2f ms, Latency: %d)",
                    !inLayout ? "DEEP Standby💤" : (shouldBeTurbo ? "Enforcing TURBO⚡" : "Idle Standby💤"),
                    clientOs == OS_MAC ? "macOS" : "Windows",
                    clientMac.c_str(), minItvl * 1.25f, targetLatency);
            vTaskDelay(pdMS_TO_TICKS(100)); // Stagger connection updates to avoid L2CAP signaling collisions
        }
    }
}

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnUpdate(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
            if (kvmClients[i].conn_id == desc->conn_handle) {
                kvmClients[i].lastParamUpdateMs = 0;
                break;
            }
        }
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        bool inLayout = isMacInActiveLayout(peerMac);
        int pOs = OS_WINDOWS;
        for (int m = 0; m < monitorCount; m++) {
            if (monitors[m].mac.equals(peerMac)) {
                pOs = monitors[m].os;
                break;
            }
        }
        bool isTurboDesc = (pOs == OS_MAC) ? (desc->conn_itvl <= 8 && desc->conn_latency <= 44)
                                            : (desc->conn_itvl <= 8 && desc->conn_latency <= 44);
        const char* mode = (!inLayout) ? "DEEP STANDBY (💤)" :
                           (isTurboDesc ? "ACTIVE TURBO (⚡)" : "BACKGROUND STANDBY (💤)");
        logPrint("[BLE Server] Connection Metrics Applied -> PC: %s (conn: %d) | Itvl: %.2f ms (itvl: %d) | Latency: %d | Timeout: %d ms -> %s",
                 peerMac.c_str(), desc->conn_handle,
                 desc->conn_itvl * 1.25f, desc->conn_itvl,
                 desc->conn_latency,
                 desc->supervision_timeout * 10,
                 mode);

        // If Bolt is active, NO PC is ever allowed to sleep (PERMANENT TURBO for all PCs).
        // If BLE mouse is used, only the active PC (or sole connected PC) is kept in TURBO.
        // If NO mouse is active (e.g. Bolt unplugged and no BLE mouse), background standby is expected.
        bool boltActive = logi_bolt_is_mouse_connected();
        bool isMouseActive = mouseConnected || boltActive;
        String activeMac = (monitorCount > 0 && currentMonitorIndex < monitorCount) ? monitors[currentMonitorIndex].mac : "";
        bool isUsbHost = (usb_device_is_connected() && peerMac.length() > 0 && peerMac.equals(usb_device_get_bound_mac()));
        bool isCurrentActivePc = (activeMac.length() > 0 && peerMac.equals(activeMac));
        // Turbo only applies to PCs in the active layout
        bool shouldBeTurbo = !isUsbHost && isMouseActive && inLayout;

        if (isHostConnectingPeripheral()) {
            return; // Never trigger LLCP Turbo re-assertion while a peripheral connection is in progress!
        }

        static uint32_t lastReassertTime[MAX_SUPPORTED_KVM_CLIENTS] = {0};
        uint16_t handle = desc->conn_handle;
        uint32_t now = millis();

        if (!inLayout) {
            // Non-layout PC: if connected with fast interval (e.g. Windows default itvl 6), demote back to Deep Standby
            if (desc->conn_itvl < 80 && handle < MAX_SUPPORTED_KVM_CLIENTS && (now - lastReassertTime[handle] > 5000)) {
                lastReassertTime[handle] = now;
                logPrint("[BLE Server] Non-Layout PC %s is in fast mode (itvl: %d). Demoting to Deep Standby...", peerMac.c_str(), desc->conn_itvl);
                xTaskCreate([](void* param) {
                    uint16_t connId = (uint16_t)(uintptr_t)param;
                    vTaskDelay(pdMS_TO_TICKS(500));
                    NimBLEServer* srv = NimBLEDevice::getServer();
                    if (srv) {
                        ble_gap_conn_desc d;
                        if (ble_gap_conn_find(connId, &d) == 0) {
                            srv->updateConnParams(connId, 80, 100, 10, 600);
                        }
                    }
                    vTaskDelete(NULL);
                }, "deepStbyDemoteTask", 4096, (void*)(uintptr_t)handle, 1, NULL);
            }
            return;
        }

        int pcOs = OS_WINDOWS;
        for (int m = 0; m < monitorCount; m++) {
            if (monitors[m].mac.equals(peerMac)) {
                pcOs = monitors[m].os;
                break;
            }
        }
        // Previous (v43): bool isSubOptimal = (pcOs == OS_MAC) ? (desc->conn_itvl != 12 || desc->conn_latency > 0) : (desc->conn_itvl > 12 || desc->conn_latency > 0);
        bool isSubOptimal = (pcOs == OS_MAC) ? (desc->conn_itvl > 6 || desc->conn_latency > 44)
                                             : (desc->conn_itvl > 6 || desc->conn_latency > 44);

        if (shouldBeTurbo && isSubOptimal && handle < MAX_SUPPORTED_KVM_CLIENTS && (now - lastReassertTime[handle] > 10000)) {
            lastReassertTime[handle] = now;
            logPrint("[BLE Server] Active PC %s in Sub-Optimal Mode (itvl: %d, latency: %d, shouldBeTurbo=1). Scheduling single TURBO re-assertion...",
                     peerMac.c_str(), desc->conn_itvl, desc->conn_latency);
            xTaskCreate([](void* param) {
                uint16_t connId = (uint16_t)(uintptr_t)param;
                vTaskDelay(pdMS_TO_TICKS(500));
                NimBLEServer* srv = NimBLEDevice::getServer();
                if (srv) {
                    ble_gap_conn_desc d;
                    if (ble_gap_conn_find(connId, &d) == 0) {
                        String pMac = NimBLEAddress(d.peer_ota_addr).toString().c_str();
                        int pcOs = OS_WINDOWS;
                        for (int m = 0; m < monitorCount; m++) {
                            if (monitors[m].mac.equals(pMac)) {
                                pcOs = monitors[m].os;
                                break;
                            }
                        }
                        // Previous (v43): bool sub = (pcOs == OS_MAC) ? (d.conn_itvl != 12 || d.conn_latency > 0) : (d.conn_itvl > 12 || d.conn_latency > 0);
                        bool sub = (pcOs == OS_MAC) ? (d.conn_itvl > 6 || d.conn_latency > 44)
                                                    : (d.conn_itvl > 6 || d.conn_latency > 44);
                        if (sub) {
                            if (pcOs == OS_MAC) {
                                srv->updateConnParams(connId, 6, 6, 44, 216);
                                logPrint("[BLE Server] Re-asserted macOS TURBO for conn %d (Target: 15.00ms, Latency: 44, ⚡)!", connId);
                            } else {
                                // Previous (v100): srv->updateConnParams(connId, 6, 8, 0, 600);
                                srv->updateConnParams(connId, 6, 6, 44, 600);
                                logPrint("[BLE Server] Re-asserted Windows TURBO for conn %d (Target: 7.50 ms, Latency: 44, ⚡)!", connId);
                            }
                        }
                    }
                }
                vTaskDelete(NULL);
            }, "reTurboTask", 4096, (void*)(uintptr_t)desc->conn_handle, 1, NULL);
        }
    }
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        NimBLEAddress idAddr(desc->peer_id_addr);
        String idMac = idAddr.toString().c_str();
        String effectiveMac = (idMac.length() > 0 && idMac != "00:00:00:00:00:00") ? idMac : peerMac;

        bool isPeripheral = (targetMouseMac.length() > 0 && (effectiveMac.equals(targetMouseMac) || peerMac.equals(targetMouseMac))) ||
                            (targetKeyboardMac.length() > 0 && (effectiveMac.equals(targetKeyboardMac) || peerMac.equals(targetKeyboardMac)));
        if (isPeripheral) {
            logPrint("[BLE Server] Peripheral (%s) connected. Skipping kvmClients registration.", effectiveMac.c_str());
            return;
        }

        logPrint("[BLE Server] PC Connected! MAC: %s (ID: %s, conn_handle: %d | itvl: %d | latency: %d | timeout: %d)",
                  peerMac.c_str(), effectiveMac.c_str(), desc->conn_handle, desc->conn_itvl, desc->conn_latency, desc->supervision_timeout);

        // Save connection
        bool isBondedPeer = NimBLEDevice::isBonded(NimBLEAddress(desc->peer_ota_addr)) ||
                            NimBLEDevice::isBonded(NimBLEAddress(desc->peer_id_addr));
        bool isLayoutPc = (monitorCount == 0) || isMacInActiveLayout(peerMac) || isMacInActiveLayout(effectiveMac);
        bool isKnownPc = isLayoutPc || isKnownKvmClient(peerMac) || isKnownKvmClient(effectiveMac) || isBondedPeer;

        // Request BLE 5.0 2M PHY (2 Mbps ultra-low latency) for KVM PCs (active layout or known background)
        if (isKnownPc) {
            ble_gap_set_prefered_le_phy(desc->conn_handle, BLE_GAP_LE_PHY_2M_MASK | BLE_GAP_LE_PHY_1M_MASK, BLE_GAP_LE_PHY_2M_MASK | BLE_GAP_LE_PHY_1M_MASK, 0);
            checkAndLogPhyStatus(desc->conn_handle, effectiveMac.c_str());
        }

        if (isKnownPc) {
            bool updated = false;
            for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
                if (kvmClients[i].mac.equals(effectiveMac) || kvmClients[i].mac.equals(peerMac)) {
                    kvmClients[i].conn_id = desc->conn_handle;
                    kvmClients[i].mac = effectiveMac;
                    kvmClients[i].active = true;
                    kvmClients[i].isTurbo = false;
                    kvmClients[i].hidSubscribed = false;
                    updated = true;
                    break;
                }
            }
            if (!updated) {
                for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
                    if (kvmClients[i].mac.length() == 0) {
                        kvmClients[i].conn_id = desc->conn_handle;
                        kvmClients[i].mac = effectiveMac;
                        kvmClients[i].name = "Paired Device";
                        kvmClients[i].active = true;
                        kvmClients[i].isTurbo = false;
                        kvmClients[i].hidSubscribed = false;
                        break;
                    }
                }
            }
            if (!isLayoutPc) {
                logPrint("[BLE Server] 💤 Background KVM PC %s (conn: %d) connected (not in active layout). Scheduling Deep Standby.", effectiveMac.c_str(), desc->conn_handle);
                uint16_t connHandle = desc->conn_handle;
                xTaskCreate([](void* p) {
                    uint16_t h = (uint16_t)(uintptr_t)p;
                    vTaskDelay(pdMS_TO_TICKS(1200)); // Allow pairing/encryption and initial link exchange to complete
                    NimBLEServer* srv = NimBLEDevice::getServer();
                    if (srv) {
                        ble_gap_conn_desc d;
                        if (ble_gap_conn_find(h, &d) == 0) {
                            srv->updateConnParams(h, 80, 100, 10, 600);
                            logPrint("[BLE Server] 💤 Deep Standby applied to conn %d (Target: 100.00..125.00 ms, Latency: 10)", h);
                        }
                    }
                    vTaskDelete(NULL);
                }, "deepStbyTask", 4096, (void*)(uintptr_t)connHandle, 1, NULL);
            }

            // Check if this newly connected PC resolves USB host binding
            int pcOs = OS_WINDOWS;
            for (int m = 0; m < monitorCount; m++) {
                if (monitors[m].mac.equals(effectiveMac) || monitors[m].mac.equals(peerMac)) {
                    pcOs = monitors[m].os;
                    break;
                }
            }
            usb_device_on_ble_connect(effectiveMac, pcOs);
        } else {
            // Non-KVM Client (Candidate Web Bluetooth Configurator) - do NOT pollute kvmClients!
            bool slotted = false;
            // Remove from nonKvmClients if it was placed there during initial onConnect
            for (int i = 0; i < MAX_NON_KVM_CLIENTS; i++) {
                if (nonKvmClients[i].conn_id == BLE_HS_CONN_HANDLE_NONE || nonKvmClients[i].mac.equals(peerMac)) {
                    nonKvmClients[i].conn_id = desc->conn_handle;
                    nonKvmClients[i].mac = peerMac;
                    nonKvmClients[i].connectedTimeMs = millis();
                    nonKvmClients[i].isWebConfig = false;
                    slotted = true;
                    break;
                }
            }
            if (!slotted) {
                nonKvmClients[0].conn_id = desc->conn_handle;
                nonKvmClients[0].mac = peerMac;
                nonKvmClients[0].connectedTimeMs = millis();
                nonKvmClients[0].isWebConfig = false;
            }
            logPrint("[BLE Server] ⏳ Non-KVM Client %s (conn: %d) connected. Starting 45s Web Config & Pairing Grace Period...", effectiveMac.c_str(), desc->conn_handle);
        }

        int activeLayoutConnectedCount = 0;
        for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
            if (kvmClients[i].active && isMacInActiveLayout(kvmClients[i].mac)) activeLayoutConnectedCount++;
        }
        if (!isCalibrated && activeLayoutConnectedCount == 1 && (isMacInActiveLayout(peerMac) || isMacInActiveLayout(effectiveMac))) {
            firstConnectedPcMac = isMacInActiveLayout(effectiveMac) ? effectiveMac : peerMac;
            scheduleBootCalibration();
        }

        // Do not update connection parameters immediately in onConnect to prevent sch_prog.c assertion
        checkAndResumeAdvertising();
    }

    void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        logPrint("[BLE Server] PC Disconnected! MAC: %s (conn_handle: %d)", peerMac.c_str(), desc->conn_handle);

        bool isTheWebClient = (g_webClientConnHandle != BLE_HS_CONN_HANDLE_NONE && desc->conn_handle == g_webClientConnHandle);

        // Check if disconnected client was a non-KVM / Web client
        for (int i = 0; i < MAX_NON_KVM_CLIENTS; i++) {
            if (nonKvmClients[i].conn_id == desc->conn_handle || nonKvmClients[i].mac.equals(peerMac)) {
                if (nonKvmClients[i].isWebConfig) {
                    isTheWebClient = true;
                }
                nonKvmClients[i].conn_id = BLE_HS_CONN_HANDLE_NONE;
                nonKvmClients[i].mac = "";
                nonKvmClients[i].connectedTimeMs = 0;
                nonKvmClients[i].isWebConfig = false;
                break;
            }
        }

        if (isTheWebClient) {
            isWebBleAuthenticated = false; // Reset Web Bluetooth authorization on web client disconnect
            currentAuthNonce = "";
            logPrint("[BLE Server] 🟣 Web client (conn: %d) disconnected. Turning OFF Web Service.", desc->conn_handle);
            deactivateWebServiceMode();
        }

        for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
            if (kvmClients[i].conn_id == desc->conn_handle || kvmClients[i].mac.equals(peerMac)) {
                kvmClients[i].active = false;
                kvmClients[i].isTurbo = false;
                kvmClients[i].hidSubscribed = false;
                break;
            }
        }

        bool isStillUsbConnected = usb_manager_is_pc_connected() && 
                                   usb_device_get_bound_mac().length() > 0 && 
                                   usb_device_get_bound_mac().equals(peerMac);

        if (!isStillUsbConnected) {
            // If the disconnected PC was the current active PC, failover to another connected PC!
            if (monitorCount > 0 && monitors[currentMonitorIndex].mac.equals(peerMac)) {
                isCalibrated = false;
                firstConnectedPcMac = "";
                // Check if any other PC is connected via USB first
                if (usb_manager_is_pc_connected() && usb_device_get_bound_mac().length() > 0 &&
                    isMacInActiveLayout(usb_device_get_bound_mac())) {
                    firstConnectedPcMac = usb_device_get_bound_mac();
                } else {
                    for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
                        if (kvmClients[i].active && !kvmClients[i].mac.equals(peerMac) && isMacInActiveLayout(kvmClients[i].mac)) {
                            firstConnectedPcMac = kvmClients[i].mac;
                            break;
                        }
                    }
                }
                if (firstConnectedPcMac.length() > 0) {
                    scheduleBootCalibration();
                    logPrint("[FAILOVER] Current PC %s disconnected! Switched control to active PC %s (Cursor at %ld, %ld)", peerMac.c_str(), firstConnectedPcMac.c_str(), virtualX, virtualY);
                } else {
                    logPrint("[FAILOVER] Current PC %s disconnected and no other active PC available.", peerMac.c_str());
                }
            }
            updateKvmPowerAndRateProfiles(isCalibrated ? (monitors[currentMonitorIndex].mac) : "");
        } else {
            logPrint("[BLE Server] PC %s BLE disconnected, but USB-C connection is ACTIVE (1000Hz). Preserving active control!", peerMac.c_str());
            updateKvmPowerAndRateProfiles(monitors[currentMonitorIndex].mac);
        }
        checkAndResumeAdvertising();
    }
};

class SecurityCallbacks : public NimBLESecurityCallbacks {
    uint32_t onPassKeyRequest() override {
        logPrint("[BLE Security] =========================================");
        logPrint("[BLE Security] >>> onPassKeyRequest: RETURNING %06lu <<<", (unsigned long)BLE_PAIRING_PIN);
        logPrint("[BLE Security] =========================================");
        return BLE_PAIRING_PIN;
    }
    void onPassKeyNotify(uint32_t pass_key) override {
        logPrint("[BLE Security] =========================================");
        logPrint("[BLE Security] >>> TYPE THIS PASSKEY ON KEYBOARD: %06lu <<<", (unsigned long)pass_key);
        logPrint("[BLE Security] >>> AND PRESS ENTER ON MX KEYS S <<<");
        logPrint("[BLE Security] =========================================");
    }
    bool onConfirmPIN(uint32_t pass_key) override {
        logPrint("[BLE Security] =========================================");
        logPrint("[BLE Security] >>> onConfirmPIN: %06lu (auto-confirmed) <<<", (unsigned long)pass_key);
        logPrint("[BLE Security] =========================================");
        return true;
    }
    bool onSecurityRequest() override {
        logPrint("[BLE Security] onSecurityRequest -> Accepted");
        return true;
    }
    void onAuthenticationComplete(ble_gap_conn_desc* desc) override {
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        logPrint("[BLE Security] Auth Complete for %s (conn: %d) | Encrypted: %d | Authenticated: %d | Bonded: %d",
                  peerMac.c_str(), desc->conn_handle, desc->sec_state.encrypted, desc->sec_state.authenticated, desc->sec_state.bonded);

        if (!desc->sec_state.bonded) {
            logPrint("[BLE Security] Bonding incomplete (Bonded: 0) for %s! Clearing stale bond key to allow fresh pairing...", peerMac.c_str());
            NimBLEDevice::deleteBond(desc->peer_ota_addr);
        } else {
            NimBLEAddress idAddr(desc->peer_id_addr);
            String idMac = idAddr.toString().c_str();
            String effectiveMac = (idMac.length() > 0 && idMac != "00:00:00:00:00:00") ? idMac : peerMac;

            bool isPeripheral = (targetMouseMac.length() > 0 && (effectiveMac.equals(targetMouseMac) || peerMac.equals(targetMouseMac))) ||
                                (targetKeyboardMac.length() > 0 && (effectiveMac.equals(targetKeyboardMac) || peerMac.equals(targetKeyboardMac)));
            if (isPeripheral) {
                logPrint("[BLE Security] Peripheral (%s) authenticated. Skipping kvmClients registration.", effectiveMac.c_str());
                return;
            }

            // Automatically register newly paired device into kvmClients!
            bool alreadyKvm = false;
            for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
                if (kvmClients[i].mac.equals(effectiveMac) || kvmClients[i].mac.equals(peerMac)) {
                    kvmClients[i].conn_id = desc->conn_handle;
                    kvmClients[i].mac = effectiveMac;
                    kvmClients[i].active = true;
                    alreadyKvm = true;
                    break;
                }
            }
            if (!alreadyKvm) {
                for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
                    if (kvmClients[i].mac.length() == 0) {
                        kvmClients[i].conn_id = desc->conn_handle;
                        kvmClients[i].mac = effectiveMac;
                        kvmClients[i].name = "Paired Device";
                        kvmClients[i].active = true;
                        kvmClients[i].isTurbo = false;
                        logPrint("[BLE Security] 📱 Newly paired device %s registered into kvmClients (slot %d)!", effectiveMac.c_str(), i);
                        saveKvmClientsToPreferences();
                        break;
                    }
                }
            }

            for (int i = 0; i < MAX_NON_KVM_CLIENTS; i++) {
                if (nonKvmClients[i].conn_id == desc->conn_handle || 
                    nonKvmClients[i].mac.equals(peerMac) || 
                    nonKvmClients[i].mac.equals(effectiveMac)) {
                    nonKvmClients[i].conn_id = BLE_HS_CONN_HANDLE_NONE;
                    nonKvmClients[i].mac = "";
                    nonKvmClients[i].connectedTimeMs = 0;
                    nonKvmClients[i].isWebConfig = false;
                    break;
                }
            }

            int activeLayoutConnectedCount = 0;
            for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
                if (kvmClients[i].active && isMacInActiveLayout(kvmClients[i].mac)) activeLayoutConnectedCount++;
            }
            if (!isCalibrated && activeLayoutConnectedCount == 1 && isMacInActiveLayout(effectiveMac)) {
                firstConnectedPcMac = effectiveMac;
                scheduleBootCalibration();
            }

            // Apply rate profile safely after 2M PHY handshake and HID/LED state exchange complete without LLCP collisions
            String* pMac = new String(effectiveMac);
            xTaskCreate([](void* p) {
                String* macPtr = (String*)p;
                // Previous (v99): vTaskDelay(pdMS_TO_TICKS(500));
                vTaskDelay(pdMS_TO_TICKS(1800));
                if (macPtr) {
                    updateKvmPowerAndRateProfiles(*macPtr, false);
                    delete macPtr;
                }
                vTaskDelete(NULL);
            }, "safeRateTask", 4096, pMac, 1, NULL);
        }

        if (desc->sec_state.encrypted) {
            ble_gap_set_prefered_le_phy(desc->conn_handle, BLE_GAP_LE_PHY_2M_MASK, BLE_GAP_LE_PHY_2M_MASK, 0);
            ble_svc_gatt_changed(0x0001, 0xffff);
        }

        checkAndResumeAdvertising();
    }
};

String getMonDisplayName(int idx) {
    return monitors[idx].name;
}

int getActiveClientOs() {
    if (monitorCount > 0 && currentMonitorIndex < monitorCount) {
        return monitors[currentMonitorIndex].os;
    }
    return OS_WINDOWS;
}

uint16_t getTargetConnHandle(const String& targetMac) {
    if (targetMac.equalsIgnoreCase("USB") || targetMac.equalsIgnoreCase("USB-C")) {
        if (usb_manager_is_pc_connected()) {
            return CONN_HANDLE_USB_DEVICE;
        }
        return BLE_HS_CONN_HANDLE_NONE;
    }

    // 1. If physical USB-C cable is connected, check if targetMac is bound to USB
    if (usb_manager_is_pc_connected()) {
        String boundMac = usb_device_get_bound_mac();
        if (boundMac.length() > 0 && boundMac.equals(targetMac)) {
            return CONN_HANDLE_USB_DEVICE;
        }
    }

    // 2. First check if target MAC has an active BLE connection
    for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
        if (kvmClients[i].active && kvmClients[i].conn_id != BLE_HS_CONN_HANDLE_NONE && kvmClients[i].mac.equals(targetMac)) {
            return kvmClients[i].conn_id;
        }
    }

    // 3. Fallback: If target is not connected via BLE, but USB-C PC is connected (and no other PC is bound)
    if (usb_manager_is_pc_connected()) {
        String boundMac = usb_device_get_bound_mac();
        if (boundMac.length() == 0) {
            return CONN_HANDLE_USB_DEVICE;
        }
    }
    return BLE_HS_CONN_HANDLE_NONE;
}

// Send HID report directly to target connection handle
void sendHidReport(NimBLECharacteristic* pChar, uint16_t connHandle, const uint8_t* report, size_t length) {
    if (!pChar || !report || length == 0 || connHandle == BLE_HS_CONN_HANDLE_NONE) return;
    if (connHandle == CONN_HANDLE_USB_DEVICE) {
        if (pChar == keyboardInputChar && length >= 8) {
            usb_device_send_keyboard(report);
        } else if (pChar == mediaInputChar && length > 0) {
            uint16_t usage = (length >= 2) ? (uint16_t)(report[0] | (report[1] << 8)) : (uint16_t)report[0];
            usb_device_send_consumer(usage);
        } else if (pChar == macAbsInputChar && length >= 5) {
            usb_device_send_mac_abs(report);
        } else if (pChar == absInputChar && length >= 4) {
            usb_device_send_win_abs(report);
        }
        return;
    }
    os_mbuf *om = ble_hs_mbuf_from_flat(report, length);
    if (!om) return;
    int rc = ble_gatts_notify_custom(connHandle, pChar->getHandle(), om);
    if (rc != 0) {
        logPrint("[BLE NOTIFY ERROR] Char handle %d, conn %d, rc: %d", pChar->getHandle(), connHandle, rc);
    }
}

void checkKeepAlive() {
    static uint32_t lastKeepAliveCheck = 0;
    if (millis() - lastKeepAliveCheck < KEEPALIVE_INTERVAL_MS) return;
    lastKeepAliveCheck = millis();

    String currentActiveMac = "";
    if (monitorCount > 0 && currentMonitorIndex >= 0 && currentMonitorIndex < monitorCount) {
        currentActiveMac = monitors[currentMonitorIndex].mac;
    }

    String handledMacs[MAX_SUPPORTED_KVM_CLIENTS];
    int handledCount = 0;

    int keepAliveCandidates = 0;
    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].keepAlive && monitors[i].mac.length() > 0) {
            keepAliveCandidates++;
            String targetMac = monitors[i].mac;

            // Do not send keepAlive if cursor is on this PC and user was active within last interval
            bool isCurrentActive = false;
            if (currentActiveMac.length() > 0) {
                if (targetMac.equals(currentActiveMac)) {
                    isCurrentActive = true;
                } else if (usb_manager_is_pc_connected()) {
                    String boundMac = usb_device_get_bound_mac();
                    if (boundMac.length() > 0) {
                        if ((targetMac.equalsIgnoreCase("USB") || targetMac.equalsIgnoreCase("USB-C")) && currentActiveMac.equals(boundMac)) {
                            isCurrentActive = true;
                        } else if ((currentActiveMac.equalsIgnoreCase("USB") || currentActiveMac.equalsIgnoreCase("USB-C")) && targetMac.equals(boundMac)) {
                            isCurrentActive = true;
                        }
                    }
                }
            }
            if (isCurrentActive && (millis() - g_lastUserActivityMs < KEEPALIVE_INTERVAL_MS)) {
                continue;
            }

            // Check if we already handled this MAC in this cycle
            bool alreadyDone = false;
            for (int h = 0; h < handledCount; h++) {
                if (handledMacs[h].equals(targetMac)) {
                    alreadyDone = true;
                    break;
                }
            }
            if (alreadyDone) continue;
            if (handledCount < MAX_SUPPORTED_KVM_CLIENTS) {
                handledMacs[handledCount++] = targetMac;
            }

            uint16_t connHandle = getTargetConnHandle(targetMac);
            if (connHandle != BLE_HS_CONN_HANDLE_NONE) {
                logPrint("[KeepAlive] Sending %ds micro-jiggle to %s (conn: %d%s)", KEEPALIVE_INTERVAL_SEC, targetMac.c_str(), connHandle, isCurrentActive ? ", Active Idle" : "");
                sendRelative12Bit(connHandle, 1, 0);
                delay(10); // Allow host OS to register +1 before sending -1 compensation
                sendRelative12Bit(connHandle, -1, 0);
            }
        }
    }

    if (keepAliveCandidates == 0 && monitorCount > 0) {
        static uint32_t lastNoticeMs = 0;
        if (millis() - lastNoticeMs > 60000 || lastNoticeMs == 0) {
            lastNoticeMs = millis();
            logPrint("[KeepAlive] Inactive: All %d monitors have KeepAlive DISABLED (0). Turn ON coffee cup icon in Web UI", monitorCount);
        }
    }
}

class HidInputCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) override {
        if (!desc) return;
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        NimBLEAddress idAddr(desc->peer_id_addr);
        String idMac = idAddr.toString().c_str();
        String effectiveMac = (idMac.length() > 0 && idMac != "00:00:00:00:00:00") ? idMac : peerMac;

        logPrint("[BLE HID] Client %s (conn: %d) subscribed to char %s (subValue: %d)",
                 effectiveMac.c_str(), desc->conn_handle, pCharacteristic->getUUID().toString().c_str(), subValue);

        if (subValue > 0) {
            onBleClientHidSubscribed(effectiveMac, pCharacteristic);
        }
    }
};

void initBleServer() {
    logPrint("[BLE] Initializing NimBLE...");
    uint8_t customMac[6];
    esp_read_mac(customMac, ESP_MAC_BT);
    customMac[5] += 30; // Increment to present fresh identity to PCs to reload new 12-bit Logitech HID descriptor
    esp_base_mac_addr_set(customMac);
    logPrint("[BLE] Custom Base MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             customMac[0], customMac[1], customMac[2], customMac[3], customMac[4], customMac[5]);

    NimBLEDevice::init(BLE_DEVICE_NAME);
    NimBLEDevice::setMTU(512);
    NimBLEDevice::setSecurityAuth(true, true, true); // (bonding=true, mitm=true -> Enforces 6-digit PIN passkey 123456, sc=true)
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY); // Display Only: Prompts PC/Smartphone for PIN entry
    NimBLEDevice::setSecurityPasskey(BLE_PAIRING_PIN);
    NimBLEDevice::setSecurityCallbacks(new SecurityCallbacks());
    NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
    NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
    
    int numBonds = NimBLEDevice::getNumBonds();
    logPrint("[BLE NVS BONDS] Saved bonded devices count: %d", numBonds);
    if (numBonds >= 14) {
        logPrint("[BLE NVS BONDS] Bond table full (%d/16). Clearing stale bonds to ensure reliable pairing...", numBonds);
        NimBLEDevice::deleteAllBonds();
        numBonds = 0;
    }
    for (int i = 0; i < numBonds; i++) {
        NimBLEAddress bondAddr = NimBLEDevice::getBondedAddress(i);
        logPrint("  -> Bonded Device #%d: MAC %s", i + 1, bondAddr.toString().c_str());
    }
    syncOrphanBonds();
    
    // Setup BLE Server (Peripheral)
    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());
    hidDevice = new NimBLEHIDDevice(pServer);
    keyboardInputChar = hidDevice->inputReport(1); // Report ID 1: Standard Keyboard (6KRO)
    keyboardOutputChar = hidDevice->outputReport(1); // Report ID 1: Keyboard LEDs (Output)
    keyboardOutputChar->setCallbacks(new KeyboardOutputCallbacks());
    NimBLECharacteristic* bootOutputChar = hidDevice->bootOutput(); // UUID 0x2A32 (Boot Keyboard Output)
    bootOutputChar->setCallbacks(new KeyboardOutputCallbacks());
    inputChar = hidDevice->inputReport(2);         // Report ID 2: Relative Mouse
    absInputChar = hidDevice->inputReport(3);      // Report ID 3: Absolute Mouse / Pointer
    macAbsInputChar = hidDevice->inputReport(5);   // Report ID 5: macOS / iPadOS Digitizer
    mediaInputChar = hidDevice->inputReport(4);    // Report ID 4: Media / Consumer Keys

    HidInputCallbacks* hidCallbacks = new HidInputCallbacks();
    inputChar->setCallbacks(hidCallbacks);
    absInputChar->setCallbacks(hidCallbacks);
    macAbsInputChar->setCallbacks(hidCallbacks);
    
    hidDevice->manufacturer()->setValue("Logitech");
    hidDevice->pnp(0x01, 0x046d, 0xc52b, 0x0100);
    hidDevice->hidInfo(0x00, 0x01);
    
    hidDevice->reportMap((uint8_t*)hidReportMap, sizeof(hidReportMap));
    hidDevice->startServices();
    
    // Setup Custom Config Service ("ESP32 KVM Server")
    NimBLEService* pConfigService = pServer->createService(CONFIG_SERVICE_UUID);
    configTxChar = pConfigService->createCharacteristic(
                      CONFIG_TX_UUID,
                      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
                   );
    configTxChar->setCallbacks(new ConfigTxCallbacks());
    configRxChar = pConfigService->createCharacteristic(
                      CONFIG_RX_UUID,
                      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
                   );
    configRxChar->setCallbacks(new ConfigRxCallbacks());
    pConfigService->start();

    startHidAdvertising();
}
