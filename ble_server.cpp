#include "ble_server.h"
#include "ble_central.h"
#include "hid_descriptors.h"
#include "kvm_config.h"
#include "cursor_engine.h"
#include "keyboard_engine.h"
#include "cmd_processor.h"
#include "logi_bolt.h"
#include <esp_mac.h>
#include "nimble/nimble/host/services/gatt/include/services/gatt/ble_svc_gatt.h"

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

bool isConfigModeActive() {
    return isWebBleAuthenticated || (lastConfigActivityTime > 0 && (millis() - lastConfigActivityTime < 60000));
}

void checkAndResumeAdvertising() {
    int activeCount = 0;
    for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
        if (kvmClients[i].active) activeCount++;
    }
    for (int i = 0; i < MAX_NON_KVM_CLIENTS; i++) {
        if (nonKvmClients[i].conn_id != BLE_HS_CONN_HANDLE_NONE) activeCount++;
    }

    // Hardware ACL budget: 8 max simultaneous connections in ESP32-S3 controller (10 activities).
    int maxAllowedPcConnections = max(2, 8 - (targetMouseMac.length() > 0 ? 1 : 0) - (targetKeyboardMac.length() > 0 ? 1 : 0));

    int layoutPcs = getActiveLayoutPcCount();

    // Keep advertising active as long as hardware handles are available (up to maxAllowedPcConnections)
    // so Web Bluetooth browser interface can discover and connect to ESP32 at any time!
    if (activeCount < maxAllowedPcConnections) {
        if (NimBLEDevice::getAdvertising() && !NimBLEDevice::getAdvertising()->isAdvertising()) {
            logPrint("[BLE Server] Advertising active (PCs: %d/%d [Layout: %d] | Mouse: %d | KB: %d)", 
                     activeCount, maxAllowedPcConnections, layoutPcs,
                     mouseConnected ? 1 : 0, kbConnected ? 1 : 0);
            NimBLEDevice::getAdvertising()->start();
        }
    } else {
        if (NimBLEDevice::getAdvertising() && NimBLEDevice::getAdvertising()->isAdvertising()) {
            logPrint("[BLE Server] Advertising paused (All %d connection slots full)", activeCount);
            NimBLEDevice::getAdvertising()->stop();
        }
    }
}

void markClientAsWebConfig(uint16_t connHandle) {
    for (int i = 0; i < MAX_NON_KVM_CLIENTS; i++) {
        if (nonKvmClients[i].conn_id != BLE_HS_CONN_HANDLE_NONE && 
            (nonKvmClients[i].conn_id == connHandle || connHandle == BLE_HS_CONN_HANDLE_NONE)) {
            if (!nonKvmClients[i].isWebConfig) {
                nonKvmClients[i].isWebConfig = true;
                logPrint("[BLE Server] Client %s (conn: %d) identified as Web Configurator (Grace Period cancelled)",
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
            vTaskDelay(pdMS_TO_TICKS(1200));
            ble_gap_read_le_phy(p->handle, &txPhy, &rxPhy);
        }
        ble_gap_conn_desc desc;
        if (ble_gap_conn_find(p->handle, &desc) == 0) {
            logPrint("[BLE LINK METRICS] %s (conn: %d) -> Itvl: %.2f ms (itvl: %d) | Latency: %d | Timeout: %d ms | TX: %s | RX: %s",
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
 * Elevates the currently active PC (where the mouse cursor is located) to high-speed Turbo profile (11.25ms - 15.0ms, latency 0),
 * while shifting all background PCs to Standby profile (60ms - 80ms, latency 4) to free 90% radio bandwidth for Mouse + Active PC.
 */
void updateKvmPowerAndRateProfiles(String activeMac, bool force) {
    NimBLEServer* pServer = NimBLEDevice::getServer();
    if (!pServer) return;

    if (activeMac.length() == 0 && monitorCount > 0 && currentMonitorIndex < monitorCount) {
        activeMac = monitors[currentMonitorIndex].mac;
    }

    bool boltActive = logi_bolt_is_mouse_connected();
    bool isMouseActive = mouseConnected || boltActive;

    for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
        if (kvmClients[i].active && kvmClients[i].conn_id != BLE_HS_CONN_HANDLE_NONE) {
            String clientMac = kvmClients[i].mac;
            if (targetMouseMac.length() > 0 && clientMac.equalsIgnoreCase(targetMouseMac)) continue;
            if (targetKeyboardMac.length() > 0 && clientMac.equalsIgnoreCase(targetKeyboardMac)) continue;

            // Detect OS type from monitor configuration
            int clientOs = OS_WINDOWS;
            for (int m = 0; m < monitorCount; m++) {
                if (monitors[m].mac.equals(clientMac)) {
                    clientOs = monitors[m].os;
                    break;
                }
            }

            // DUAL-MODE SMART POWER & BANDWIDTH ALLOCATION:
            // 1. With Logi Bolt (USB): Radio has NO BLE mouse traffic -> ALL connected PCs stay in PERMANENT TURBO (0ms switch lag!)
            // 2. With BLE Mouse: Active PC gets TURBO, background PCs get STANDBY (frees 90% radio bandwidth for BLE mouse!)
            bool isCurrentActivePc = (activeMac.length() > 0 && clientMac.equals(activeMac));
            bool shouldBeTurbo = isMouseActive && (boltActive || isCurrentActivePc);

            bool wasTurbo = kvmClients[i].isTurbo;
            if (force || wasTurbo != shouldBeTurbo) {
                kvmClients[i].isTurbo = shouldBeTurbo;
                ble_gap_conn_desc desc;
                bool hasDesc = (ble_gap_conn_find(kvmClients[i].conn_id, &desc) == 0);

                if (shouldBeTurbo) {
                    if (clientOs == OS_MAC) {
                        if (hasDesc && desc.conn_latency == 0 && desc.conn_itvl == 12) {
                            continue; // Already in Apple Turbo!
                        }
                        // macOS ACTIVE TURBO: Apple strictly mandates itvl: 12 (15.00ms). Latency 0 gives 0ms slave latency!
                        pServer->updateConnParams(kvmClients[i].conn_id, 12, 12, 0, 216);
                        logPrint("[BLE Server] Enforcing macOS PERMANENT TURBO for PC: %s (Target: 15.00 ms, Latency: 0, ⚡)", kvmClients[i].mac.c_str());
                    } else {
                        if (hasDesc && desc.conn_itvl <= 8 && desc.conn_latency == 0) {
                            continue; // Already in Windows Turbo!
                        }
                        // Windows ACTIVE TURBO: 7.50ms..10.00ms (itvl 6..8), latency 0, timeout 6000ms (Ultra-fast 100..133Hz)
                        pServer->updateConnParams(kvmClients[i].conn_id, 6, 8, 0, 600);
                        logPrint("[BLE Server] Enforcing Windows PERMANENT TURBO for PC: %s (Target: 7.50..10.00 ms, Latency: 0, ⚡)", kvmClients[i].mac.c_str());
                    }
                } else {
                    // Safe idle standby only when no mouse is active at all
                    if (clientOs == OS_MAC) {
                        pServer->updateConnParams(kvmClients[i].conn_id, 12, 12, 4, 216);
                        logPrint("[BLE Server] Idle Standby for macOS PC: %s (Latency: 4, 💤)", kvmClients[i].mac.c_str());
                    } else {
                        pServer->updateConnParams(kvmClients[i].conn_id, 6, 8, 4, 600);
                        logPrint("[BLE Server] Idle Standby for Windows PC: %s (Latency: 4, 💤)", kvmClients[i].mac.c_str());
                    }
                }
            }
        }
    }
}

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnUpdate(NimBLEServer* pServer, ble_gap_conn_desc* desc) override {
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        const char* mode = (desc->conn_itvl <= 16 && desc->conn_latency == 0) ? "ACTIVE TURBO (⚡)" : "BACKGROUND STANDBY (💤)";
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
        bool isCurrentActivePc = (activeMac.length() > 0 && peerMac.equals(activeMac));
        bool shouldBeTurbo = isMouseActive && (boltActive || isCurrentActivePc);

        static uint32_t lastReassertTime[MAX_SUPPORTED_KVM_CLIENTS] = {0};
        uint16_t handle = desc->conn_handle;
        uint32_t now = millis();

        if (shouldBeTurbo && desc->conn_latency > 0 && handle < MAX_SUPPORTED_KVM_CLIENTS && (now - lastReassertTime[handle] > 15000)) {
            lastReassertTime[handle] = now;
            logPrint("[BLE Server] Active PC %s in Latency %d (shouldBeTurbo=1). Scheduling single TURBO re-assertion...",
                     peerMac.c_str(), desc->conn_latency);
            xTaskCreate([](void* param) {
                uint16_t connId = (uint16_t)(uintptr_t)param;
                vTaskDelay(pdMS_TO_TICKS(250));
                NimBLEServer* srv = NimBLEDevice::getServer();
                if (srv) {
                    ble_gap_conn_desc d;
                    if (ble_gap_conn_find(connId, &d) == 0 && d.conn_latency > 0) {
                        String pMac = NimBLEAddress(d.peer_ota_addr).toString().c_str();
                        int pcOs = OS_WINDOWS;
                        for (int m = 0; m < monitorCount; m++) {
                            if (monitors[m].mac.equals(pMac)) {
                                pcOs = monitors[m].os;
                                break;
                            }
                        }
                        if (pcOs == OS_MAC) {
                            srv->updateConnParams(connId, 12, 12, 0, 216);
                            logPrint("[BLE Server] Re-asserted macOS TURBO for conn %d (Target: 15.00ms, Latency: 0, ⚡)!", connId);
                        } else {
                            srv->updateConnParams(connId, 6, 8, 0, 600);
                            logPrint("[BLE Server] Re-asserted Windows TURBO for conn %d (Target: 7.50..10.00ms, Latency: 0, ⚡)!", connId);
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

        bool isPeripheral = (targetMouseMac.length() > 0 && (effectiveMac.equalsIgnoreCase(targetMouseMac) || peerMac.equalsIgnoreCase(targetMouseMac))) ||
                            (targetKeyboardMac.length() > 0 && (effectiveMac.equalsIgnoreCase(targetKeyboardMac) || peerMac.equalsIgnoreCase(targetKeyboardMac)));
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
                        break;
                    }
                }
            }
            if (!isLayoutPc) {
                logPrint("[BLE Server] 💤 Background KVM PC %s (conn: %d) connected (not in active layout). Staying in Standby.", effectiveMac.c_str(), desc->conn_handle);
            }
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
        if (activeLayoutConnectedCount == 1 && (isMacInActiveLayout(peerMac) || isMacInActiveLayout(effectiveMac))) {
            firstConnectedPcMac = isMacInActiveLayout(effectiveMac) ? effectiveMac : peerMac;
            isCalibrated = false;
            scheduleBootCalibration();
        }

        // Do not update connection parameters immediately in onConnect to prevent sch_prog.c assertion
        checkAndResumeAdvertising();
    }

    void onDisconnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        String peerMac = NimBLEAddress(desc->peer_ota_addr).toString().c_str();
        logPrint("[BLE Server] PC Disconnected! MAC: %s (conn_handle: %d)", peerMac.c_str(), desc->conn_handle);

        isWebBleAuthenticated = false; // Reset Web Bluetooth authorization on client disconnect
        currentAuthNonce = "";
        
        // Check if disconnected client was a non-KVM / Web client
        bool isNonKvm = false;
        for (int i = 0; i < MAX_NON_KVM_CLIENTS; i++) {
            if (nonKvmClients[i].conn_id == desc->conn_handle || nonKvmClients[i].mac.equals(peerMac)) {
                nonKvmClients[i].conn_id = BLE_HS_CONN_HANDLE_NONE;
                nonKvmClients[i].mac = "";
                nonKvmClients[i].connectedTimeMs = 0;
                nonKvmClients[i].isWebConfig = false;
                isNonKvm = true;
                break;
            }
        }

        for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
            if (kvmClients[i].conn_id == desc->conn_handle || kvmClients[i].mac.equals(peerMac)) {
                kvmClients[i].active = false;
                kvmClients[i].isTurbo = false;
                break;
            }
        }

        isCalibrated = false;
        // If the disconnected PC was the current active PC, failover to another connected PC!
        if (monitorCount > 0 && monitors[currentMonitorIndex].mac.equals(peerMac)) {
            firstConnectedPcMac = "";
            for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
                if (kvmClients[i].active && !kvmClients[i].mac.equals(peerMac) && isMacInActiveLayout(kvmClients[i].mac)) {
                    firstConnectedPcMac = kvmClients[i].mac;
                    break;
                }
            }
            if (firstConnectedPcMac.length() > 0) {
                scheduleBootCalibration();
                logPrint("[FAILOVER] Current PC %s disconnected! Switched control to active PC %s (Cursor at %ld, %ld)", peerMac.c_str(), firstConnectedPcMac.c_str(), virtualX, virtualY);
            } else {
                logPrint("[FAILOVER] Current PC %s disconnected and no other active PC available.", peerMac.c_str());
            }
        }
        updateKvmPowerAndRateProfiles(isCalibrated ? firstConnectedPcMac : "");
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

            bool isPeripheral = (targetMouseMac.length() > 0 && (effectiveMac.equalsIgnoreCase(targetMouseMac) || peerMac.equalsIgnoreCase(targetMouseMac))) ||
                                (targetKeyboardMac.length() > 0 && (effectiveMac.equalsIgnoreCase(targetKeyboardMac) || peerMac.equalsIgnoreCase(targetKeyboardMac)));
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
            if (activeLayoutConnectedCount == 1 && isMacInActiveLayout(effectiveMac)) {
                firstConnectedPcMac = effectiveMac;
                isCalibrated = false;
                scheduleBootCalibration();
            }
            // Connection is securely bonded and link layer is stable: apply rate profile safely!
            updateKvmPowerAndRateProfiles(effectiveMac, true);
        }

        if (desc->sec_state.encrypted) {
            ble_gap_set_prefered_le_phy(desc->conn_handle, BLE_GAP_LE_PHY_2M_MASK | BLE_GAP_LE_PHY_1M_MASK, BLE_GAP_LE_PHY_2M_MASK | BLE_GAP_LE_PHY_1M_MASK, 0);
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
    for (int i = 0; i < MAX_SUPPORTED_KVM_CLIENTS; i++) {
        if (kvmClients[i].active && kvmClients[i].mac.equals(targetMac)) {
            return kvmClients[i].conn_id;
        }
    }
    return BLE_HS_CONN_HANDLE_NONE;
}

// Send HID report directly to target connection handle
void sendHidReport(NimBLECharacteristic* pChar, uint16_t connHandle, const uint8_t* report, size_t length) {
    if (!pChar || !report || length == 0 || connHandle == BLE_HS_CONN_HANDLE_NONE) return;
    os_mbuf *om = ble_hs_mbuf_from_flat(report, length);
    if (!om) return;
    int rc = ble_gatts_notify_custom(connHandle, pChar->getHandle(), om);
    if (rc != 0) {
        os_mbuf_free_chain(om);
    }
}

void checkKeepAlive() {
    static uint32_t lastKeepAliveCheck = 0;
    if (millis() - lastKeepAliveCheck < 60000) return;
    lastKeepAliveCheck = millis();

    String currentActiveMac = "";
    if (monitorCount > 0 && currentMonitorIndex >= 0 && currentMonitorIndex < monitorCount) {
        currentActiveMac = monitors[currentMonitorIndex].mac;
    }

    String handledMacs[MAX_SUPPORTED_KVM_CLIENTS];
    int handledCount = 0;

    for (int i = 0; i < monitorCount; i++) {
        if (monitors[i].keepAlive && monitors[i].mac.length() > 0) {
            String targetMac = monitors[i].mac;

            // Do not send keepAlive if cursor is currently on this PC!
            if (currentActiveMac.length() > 0 && targetMac.equals(currentActiveMac)) {
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
                logPrint("[KeepAlive] Sending 60s micro-jiggle to %s (conn: %d)", targetMac.c_str(), connHandle);
                sendRelative12Bit(connHandle, 1, 0);
                sendRelative12Bit(connHandle, -1, 0);
            }
        }
    }
}

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

    NimBLEAdvertising* pAdvertising = pServer->getAdvertising();
    pAdvertising->setAppearance(0x03C0); // HID Generic / Combo (Mouse + Keyboard)
    pAdvertising->addServiceUUID(hidDevice->hidService()->getUUID());
    pAdvertising->addServiceUUID(CONFIG_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->start();
    logPrint("[BLE Server] Advertising HID Combo '%s' & Web Bluetooth Service...", BLE_DEVICE_NAME);
}
