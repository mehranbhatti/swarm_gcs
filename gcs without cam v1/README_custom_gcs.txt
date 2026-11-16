AeroLink Custom GCS - telemetry-only first version

1. Copy these files into your project folder:
   - custom_gcs_gui.py
   - requirements_custom_gcs.txt
   - start_custom_gcs.bat

2. Activate your existing virtual environment and install dependencies:

   .\.venv\Scripts\Activate.ps1
   python -m pip install -r requirements_custom_gcs.txt

3. Start MAVProxy first:

   mavproxy.exe --master=COM7,57600 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551

4. Start the GUI:

   python custom_gcs_gui.py

The GUI listens on:
   udpin:127.0.0.1:14551

QGroundControl can continue listening on UDP port 14550.

This version is read-only and intentionally contains no arm, disarm, mission,
mode-change, or camera controls yet.
