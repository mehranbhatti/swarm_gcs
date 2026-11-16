@echo off
setlocal

rem Change COM7 only if Windows assigns a different port to the Holybro ground radio.
set TELEMETRY_PORT=COM7
set TELEMETRY_BAUD=57600

echo MAVProxy telemetry router
echo Input:  %TELEMETRY_PORT% at %TELEMETRY_BAUD%
echo QGC:    UDP 127.0.0.1:14550
echo Custom: UDP 127.0.0.1:14551
echo.

mavproxy.exe --master=%TELEMETRY_PORT%,%TELEMETRY_BAUD% --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551
pause
