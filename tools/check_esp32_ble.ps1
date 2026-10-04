# =====================================================================
# ESP32 BLE Connection Diagnostics for Windows
# Queries live Bluetooth Low Energy Link Layer parameters directly
# from the Windows Kernel via WinRT API.
# =====================================================================

[System.Reflection.Assembly]::LoadWithPartialName("System.Runtime.WindowsRuntime") | Out-Null
[Windows.Devices.Bluetooth.BluetoothLEDevice, Windows.Devices.Bluetooth, ContentType = WindowsRuntime] | Out-Null

$macHex = "288485442944"
$addr = [Convert]::ToUInt64($macHex, 16)

function Show-BleStatus {
    Clear-Host
    Write-Host "============================================================" -ForegroundColor Cyan
    Write-Host "             ESP32 KVM BLE Connection Monitor               " -ForegroundColor Yellow
    Write-Host "============================================================" -ForegroundColor Cyan
    Write-Host " Target MAC: 28:84:85:44:29:44 (esp combo)" -ForegroundColor Gray
    Write-Host ""

    try {
        $op = [Windows.Devices.Bluetooth.BluetoothLEDevice]::FromBluetoothAddressAsync($addr)
        $asTaskGeneric = [System.WindowsRuntimeSystemExtensions].GetMethods() | Where-Object {
            $_.Name -eq 'AsTask' -and $_.GetParameters().Count -eq 1 -and $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncOperation`1'
        } | Select-Object -First 1

        $asTask = $asTaskGeneric.MakeGenericMethod([Windows.Devices.Bluetooth.BluetoothLEDevice])
        $task = $asTask.Invoke($null, @($op))
        $device = $task.GetAwaiter().GetResult()

        if ($null -eq $device) {
            Write-Host " [!] Device 'esp combo' not paired or not found." -ForegroundColor Red
            return
        }

        $status = $device.ConnectionStatus
        if ($status -eq "Connected") {
            Write-Host " Connection Status: " -NoNewline
            Write-Host "CONNECTED [ONLINE]" -ForegroundColor Green
        } else {
            Write-Host " Connection Status: " -NoNewline
            Write-Host "DISCONNECTED [OFFLINE]" -ForegroundColor Red
            return
        }

        $params = $device.GetConnectionParameters()
        if ($null -eq $params) {
            Write-Host " [!] Unable to read live connection parameters." -ForegroundColor Yellow
            return
        }

        $itvl = $params.ConnectionInterval
        $lat  = $params.ConnectionLatency
        $to   = $params.LinkTimeout

        $itvlMs = $itvl * 1.25
        $pollFreq = if ($itvlMs -gt 0) { [Math]::Round(1000.0 / $itvlMs, 1) } else { 0 }
        $idleSleepMs = [Math]::Round(($lat + 1) * $itvlMs, 1)

        Write-Host ""
        Write-Host " --- [Live Link Layer Metrics] ---" -ForegroundColor Cyan
        Write-Host ("   Connection Interval : {0} ({1:F2} ms, ~{2} Hz)" -f $itvl, $itvlMs, $pollFreq) -ForegroundColor White
        
        Write-Host "   Connection Latency  : " -NoNewline
        if ($lat -le 4) {
            Write-Host ("{0}  [TURBO / Low Latency ⚡]" -f $lat) -ForegroundColor Green
        } elseif ($lat -le 20) {
            Write-Host ("{0}  [STANDBY / Power Save 💤]" -f $lat) -ForegroundColor Yellow
        } else {
            Write-Host ("{0}  [DEEP SLEEP]" -f $lat) -ForegroundColor Magenta
        }

        Write-Host ("   Supervision Timeout : {0} ({1} ms)" -f $to, ($to * 10)) -ForegroundColor White
        Write-Host ("   Idle Poll Interval  : ~{0} ms (1 packet per {0} ms when quiet)" -f $idleSleepMs) -ForegroundColor Gray
        Write-Host ""

        if ($lat -le 4) {
            Write-Host " Mode Assessment: TURBO ACTIVE! PC responds instantly to cursor movements." -ForegroundColor Green
        } else {
            Write-Host " Mode Assessment: STANDBY ACTIVE. Link is saving radio power (~$idleSleepMs ms idle sleep)." -ForegroundColor Yellow
        }

    } catch {
        Write-Host " Error querying Bluetooth stack: $_" -ForegroundColor Red
    }

    Write-Host "============================================================" -ForegroundColor Cyan
    Write-Host " [R] Refresh  |  [L] Live Auto-Refresh (1s)  |  [Q] Exit" -ForegroundColor Yellow
}

# Initial display
Show-BleStatus

# Interactive loop
while ($true) {
    if ([Console]::KeyAvailable) {
        $key = [Console]::ReadKey($true).Key
        if ($key -eq [ConsoleKey]::Q) {
            break
        } elseif ($key -eq [ConsoleKey]::R) {
            Show-BleStatus
        } elseif ($key -eq [ConsoleKey]::L) {
            Write-Host "`n Starting Live Auto-Refresh mode (press any key to stop)..." -ForegroundColor Cyan
            Start-Sleep -Milliseconds 500
            while (-not [Console]::KeyAvailable) {
                Show-BleStatus
                Write-Host " [Live Mode] Updating every 1s... Press any key to stop." -ForegroundColor Magenta
                Start-Sleep -Seconds 1
            }
            [Console]::ReadKey($true) | Out-Null
            Show-BleStatus
        }
    }
    Start-Sleep -Milliseconds 100
}
