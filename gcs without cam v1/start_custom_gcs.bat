@echo off
setlocal

cd /d "%~dp0"

if exist ".venv\Scripts\activate.bat" (
    call ".venv\Scripts\activate.bat"
)

python custom_gcs_gui.py

if errorlevel 1 (
    echo.
    echo The GCS stopped with an error.
    pause
)
