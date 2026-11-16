PROFESSIONAL MAVLINK GROUND CONTROL
===================================

Purpose
-------
This build is a map-first custom GCS for the current Holybro/MAVProxy setup.
It intentionally contains no camera/video code yet.

Telemetry topology
------------------
Flight Controller -> Holybro 433 MHz telemetry -> Ground PC -> MAVProxy
  -> UDP 14550 -> QGroundControl (optional)
  -> UDP 14551 -> professional_mavlink_gcs.py

Main interface
--------------
- Full moving GPS map in the center
- Live aircraft marker rotated with heading
- Flight track and HOME marker when HOME_POSITION is received
- Compact top status chips: link, arm state, flight mode, GPS, battery
- Left navigation rail instead of large page headings
- Dockable flight-data panel
- Dockable control panel
- Small artificial horizon and compass dock
- Bottom MESSAGES drawer
- Bottom link/arming status strip

Vehicle controls
----------------
- ARM: press and hold
- DISARM: press and hold
- STABILIZE
- ALT HOLD
- LOITER
- RTL: press and hold
- LAND: press and hold

ARM/DISARM use normal MAVLink arming commands. Force arm/disarm is not implemented.
The application displays flight-controller COMMAND_ACK and STATUSTEXT messages.

Responsive behavior
-------------------
The UI uses QMainWindow dock widgets and Qt layouts rather than absolute positions.
Controls therefore resize without overlapping. Below about 1050 px window width,
the small right instrument dock hides automatically so the map remains usable.
Enlarge the window to restore it.

Map controls
------------
FOLLOW / FREE  - choose whether the map follows the aircraft
Target icon    - center on aircraft
+ / -          - zoom
Track icon     - clear the displayed track

The current 2D map uses OpenStreetMap tiles through Leaflet and therefore requires
internet access for map tiles. MAVLink telemetry and vehicle commands do not depend
on the map being online.

Installation
------------
1. Put these files in your project folder.
2. Activate the existing venv:

   .\.venv\Scripts\Activate.ps1

3. Install dependencies:

   python -m pip install -r requirements_professional_gcs.txt

Running
-------
1. Start MAVProxy first:

   mavproxy.exe --master=COM7,57600 --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551

2. Wait until MAVProxy sees the vehicle.
3. Start the custom GCS:

   python professional_mavlink_gcs.py

The GCS auto-connects to:

   udpin:127.0.0.1:14551

You can also use start_professional_gcs.bat.

Safety
------
First control testing should be done with propellers removed. RTL and LAND are
real flight commands. The software does not bypass FC safety/pre-arm checks.
