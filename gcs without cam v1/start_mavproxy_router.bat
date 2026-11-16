@echo off
setlocal

set TELEMETRY_PORT=COM7
set TELEMETRY_BAUD=57600
set QGC_PORT=14550
set CUSTOM_GCS_PORT=14551

echo ==================================================
echo MAVLink telemetry router
echo ==================================================
echo Input:       %TELEMETRY_PORT% at %TELEMETRY_BAUD%
echo QGroundControl UDP port: %QGC_PORT%
echo Custom GCS UDP port:     %CUSTOM_GCS_PORT%
echo ==================================================
echo.

mavproxy.exe --master=%TELEMETRY_PORT%,%TELEMETRY_BAUD% --out=udp:127.0.0.1:%QGC_PORT% --out=udp:127.0.0.1:%CUSTOM_GCS_PORT%

pause
