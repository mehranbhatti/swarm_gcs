#!/usr/bin/env python3
"""Responsive MAVLink ground station with map and essential controls.

Data flow:
    Flight Controller -> Holybro radio -> MAVProxy
        -> UDP 14550 -> QGroundControl (optional)
        -> UDP 14551 -> this application

The application displays essential telemetry, a live GPS map, compact flight
instruments, and standard MAVLink arm/disarm and ArduCopter mode controls.
It never uses force-arm or force-disarm parameters.
"""

from __future__ import annotations

import math
import queue
import sys
import time
from dataclasses import asdict, dataclass
from typing import Any

from pymavlink import mavutil
from PySide6.QtCore import QPointF, QRectF, Qt, QThread, QTimer, QUrl, Signal
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSizePolicy,
    QStatusBar,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

try:
    from PySide6.QtWebEngineWidgets import QWebEngineView

    WEBENGINE_AVAILABLE = True
except ImportError:
    QWebEngineView = None  # type: ignore[assignment]
    WEBENGINE_AVAILABLE = False


DEFAULT_CONNECTION = "udpin:127.0.0.1:14551"

MODE_BUTTONS = (
    ("STABILIZE", "STABILIZE"),
    ("ALT HOLD", "ALT_HOLD"),
    ("LOITER", "LOITER"),
    ("RTL", "RTL"),
    ("LAND", "LAND"),
)

GPS_FIX_NAMES = {
    0: "NO GPS",
    1: "NO FIX",
    2: "2D FIX",
    3: "3D FIX",
    4: "DGPS",
    5: "RTK FLOAT",
    6: "RTK FIXED",
    7: "STATIC",
    8: "PPP",
}


@dataclass
class TelemetryState:
    connected: bool = False
    system_id: int = 0
    component_id: int = 0
    mode: str = "UNKNOWN"
    armed: bool = False

    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    heading_deg: float | None = None

    latitude: float | None = None
    longitude: float | None = None
    gps_fix: int | None = None
    satellites: int | None = None

    relative_altitude_m: float | None = None
    airspeed_mps: float | None = None
    battery_voltage_v: float | None = None

    last_message: str = "-"
    last_status_text: str = "Waiting for flight controller"
    last_update_monotonic: float = 0.0

class TelemetryWorker(QThread):
    telemetry = Signal(dict)
    link_status = Signal(str, str)
    log_message = Signal(str)
    command_status = Signal(str, str)

    def __init__(self, connection_string: str, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.connection_string = connection_string
        self._stop_requested = False
        self._vehicle: Any = None
        self._state = TelemetryState()
        self._commands: queue.Queue[tuple[str, Any]] = queue.Queue()
        self._pending_commands: dict[int, str] = {}

    def request_stop(self) -> None:
        self._stop_requested = True
        try:
            if self._vehicle is not None:
                self._vehicle.close()
        except Exception:
            pass

    def queue_arm(self, arm: bool) -> None:
        self._commands.put(("arm", arm))

    def queue_mode(self, mode_name: str) -> None:
        self._commands.put(("mode", mode_name))

    def run(self) -> None:
        self._stop_requested = False
        self.link_status.emit("connecting", "Opening MAVLink connection")
        self.log_message.emit(f"Opening {self.connection_string}")

        try:
            self._vehicle = mavutil.mavlink_connection(
                self.connection_string,
                source_system=254,
                source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
            )

            self.link_status.emit("connecting", "Waiting for FC heartbeat")
            heartbeat = self._wait_for_autopilot_heartbeat(timeout=20.0)
            if heartbeat is None:
                raise TimeoutError(
                    "No flight-controller heartbeat received for 20 seconds. "
                    "Check MAVProxy and UDP port 14551."
                )

            if self._stop_requested:
                return

            self._state.connected = True
            self._parse_message(heartbeat)
            self.link_status.emit("connected", "Flight controller connected")
            self.log_message.emit(
                f"Heartbeat received from system {self._state.system_id}, "
                f"component {self._state.component_id}"
            )
            self.telemetry.emit(asdict(self._state))

            last_emit = 0.0
            last_packet = time.monotonic()
            last_gcs_heartbeat = 0.0
            stale_announced = False

            while not self._stop_requested:
                now = time.monotonic()

                self._process_commands()

                if now - last_gcs_heartbeat >= 1.0:
                    self._send_gcs_heartbeat()
                    last_gcs_heartbeat = now

                message = self._vehicle.recv_match(blocking=True, timeout=0.10)
                now = time.monotonic()

                if message is not None:
                    if stale_announced:
                        self.link_status.emit("connected", "Flight controller connected")
                        self.log_message.emit("MAVLink telemetry stream resumed")
                    last_packet = now
                    stale_announced = False
                    self._parse_message(message)

                if now - last_emit >= 0.10:
                    self.telemetry.emit(asdict(self._state))
                    last_emit = now

                if now - last_packet > 4.0 and not stale_announced:
                    self.link_status.emit("warning", "Telemetry stream is stale")
                    self.log_message.emit("No MAVLink packet received for more than 4 seconds")
                    stale_announced = True

        except Exception as error:
            if not self._stop_requested:
                self.link_status.emit("error", str(error))
                self.log_message.emit(f"ERROR: {error}")
        finally:
            self._state.connected = False
            self.telemetry.emit(asdict(self._state))
            try:
                if self._vehicle is not None:
                    self._vehicle.close()
            except Exception:
                pass
            self._vehicle = None
            if self._stop_requested:
                self.link_status.emit("disconnected", "Disconnected")

    def _wait_for_autopilot_heartbeat(self, timeout: float) -> Any | None:
        started = time.monotonic()
        while not self._stop_requested and time.monotonic() - started < timeout:
            message = self._vehicle.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=1.0,
            )
            if message is None:
                continue

            vehicle_type = int(getattr(message, "type", -1))
            autopilot_type = int(getattr(message, "autopilot", -1))
            is_gcs = vehicle_type == mavutil.mavlink.MAV_TYPE_GCS
            valid_autopilot = autopilot_type != mavutil.mavlink.MAV_AUTOPILOT_INVALID
            if not is_gcs and valid_autopilot:
                return message
        return None

    def _process_commands(self) -> None:
        while not self._commands.empty():
            try:
                action, value = self._commands.get_nowait()
            except queue.Empty:
                return

            try:
                if action == "arm":
                    self._send_arm_disarm(bool(value))
                elif action == "mode":
                    self._send_mode(str(value))
            except Exception as error:
                self.command_status.emit("error", str(error))
                self.log_message.emit(f"COMMAND ERROR: {error}")

    def _target_ids(self) -> tuple[int, int]:
        system_id = int(self._state.system_id or self._vehicle.target_system or 0)
        component_id = int(
            self._state.component_id
            or self._vehicle.target_component
            or mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1
        )
        if system_id <= 0:
            raise RuntimeError("Flight-controller system ID is not available")
        return system_id, component_id

    def _send_arm_disarm(self, arm: bool) -> None:
        system_id, component_id = self._target_ids()
        label = "ARM" if arm else "DISARM"

        self._vehicle.mav.command_long_send(
            system_id,
            component_id,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1.0 if arm else 0.0,
            0.0,  # standard safety checks; never force arm/disarm
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        self._pending_commands[
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
        ] = label
        self.command_status.emit("pending", f"{label} command sent")
        self.log_message.emit(f"Sent {label} command")

    def _send_mode(self, mode_name: str) -> None:
        system_id, component_id = self._target_ids()
        mode_mapping = self._vehicle.mode_mapping() or {}

        normalized_mapping = {
            str(name).upper().replace(" ", "_"): int(number)
            for name, number in mode_mapping.items()
        }
        normalized_name = mode_name.upper().replace(" ", "_")

        if normalized_name not in normalized_mapping:
            available = ", ".join(sorted(normalized_mapping)) or "none"
            raise RuntimeError(
                f"Mode {normalized_name} is not available for this vehicle. "
                f"Reported modes: {available}"
            )

        mode_number = normalized_mapping[normalized_name]
        self._vehicle.mav.command_long_send(
            system_id,
            component_id,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            float(mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
            float(mode_number),
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        self._pending_commands[mavutil.mavlink.MAV_CMD_DO_SET_MODE] = normalized_name
        self.command_status.emit("pending", f"Requested {normalized_name}")
        self.log_message.emit(f"Requested flight mode {normalized_name}")

    def _send_gcs_heartbeat(self) -> None:
        self._vehicle.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def _parse_message(self, message: Any) -> None:
        if message is None:
            return

        message_type = message.get_type()
        if message_type == "BAD_DATA":
            return

        self._state.last_message = message_type
        self._state.last_update_monotonic = time.monotonic()

        if message_type == "HEARTBEAT":
            vehicle_type = int(getattr(message, "type", -1))
            autopilot_type = int(getattr(message, "autopilot", -1))
            if (
                vehicle_type == mavutil.mavlink.MAV_TYPE_GCS
                or autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_INVALID
            ):
                return

            self._state.mode = mavutil.mode_string_v10(message)
            self._state.armed = bool(
                message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            self._state.system_id = int(message.get_srcSystem())
            self._state.component_id = int(message.get_srcComponent())

        elif message_type == "ATTITUDE":
            self._state.roll_deg = math.degrees(float(message.roll))
            self._state.pitch_deg = math.degrees(float(message.pitch))
            self._state.heading_deg = math.degrees(float(message.yaw)) % 360.0

        elif message_type == "GLOBAL_POSITION_INT":
            self._state.latitude = float(message.lat) / 1e7
            self._state.longitude = float(message.lon) / 1e7
            self._state.relative_altitude_m = float(message.relative_alt) / 1000.0
            heading = int(getattr(message, "hdg", 65535))
            if heading != 65535:
                self._state.heading_deg = heading / 100.0

        elif message_type == "GPS_RAW_INT":
            self._state.gps_fix = int(message.fix_type)
            satellites = int(message.satellites_visible)
            self._state.satellites = None if satellites == 255 else satellites
            raw_lat = int(getattr(message, "lat", 0))
            raw_lon = int(getattr(message, "lon", 0))
            if raw_lat != 0 or raw_lon != 0:
                self._state.latitude = raw_lat / 1e7
                self._state.longitude = raw_lon / 1e7

        elif message_type == "SYS_STATUS":
            voltage = int(message.voltage_battery)
            self._state.battery_voltage_v = (
                None if voltage == 65535 else voltage / 1000.0
            )

        elif message_type == "BATTERY_STATUS":
            valid_cells = [
                int(value)
                for value in getattr(message, "voltages", [])
                if int(value) not in (0, 65535)
            ]
            if valid_cells:
                self._state.battery_voltage_v = sum(valid_cells) / 1000.0

        elif message_type == "VFR_HUD":
            self._state.airspeed_mps = float(message.airspeed)
            self._state.heading_deg = float(message.heading) % 360.0

        elif message_type == "STATUSTEXT":
            text = message.text
            if isinstance(text, bytes):
                text = text.decode(errors="replace")
            text = str(text).replace("\x00", "").strip()
            if text:
                self._state.last_status_text = text
                self.log_message.emit(f"FC: {text}")

        elif message_type == "COMMAND_ACK":
            command_id = int(message.command)
            result_id = int(message.result)
            result_name = self._enum_name("MAV_RESULT", result_id)
            command_name = self._pending_commands.pop(command_id, None)
            if command_name is not None:
                accepted = result_id in (
                    mavutil.mavlink.MAV_RESULT_ACCEPTED,
                    mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                )
                level = "success" if accepted else "error"
                text = f"{command_name}: {result_name}"
                self.command_status.emit(level, text)
                self.log_message.emit(f"Command acknowledgement: {text}")

    @staticmethod
    def _enum_name(enum_name: str, value: int) -> str:
        enum_map = mavutil.mavlink.enums.get(enum_name, {})
        item = enum_map.get(value)
        if item is None:
            return str(value)
        return str(getattr(item, "name", value)).replace(f"{enum_name}_", "")



class DroneIconWidget(QWidget):
    """Small, text-free quadcopter icon for the top bar."""

    def __init__(self) -> None:
        super().__init__()
        self.setFixedSize(66, 44)
        self.setToolTip("MAVLink vehicle")

    def paintEvent(self, event: Any) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        center = QPointF(self.width() / 2, self.height() / 2 + 2)

        painter.setPen(QPen(QColor("#66e3c4"), 4, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        painter.drawLine(QPointF(center.x() - 20, center.y() - 11), QPointF(center.x() + 20, center.y() + 11))
        painter.drawLine(QPointF(center.x() + 20, center.y() - 11), QPointF(center.x() - 20, center.y() + 11))

        painter.setBrush(QColor("#0f2b35"))
        painter.setPen(QPen(QColor("#a7f3d0"), 2))
        painter.drawRoundedRect(QRectF(center.x() - 10, center.y() - 7, 20, 14), 5, 5)

        painter.setBrush(QColor("#071018"))
        for dx, dy in ((-23, -13), (23, -13), (-23, 13), (23, 13)):
            rotor_center = QPointF(center.x() + dx, center.y() + dy)
            painter.setPen(QPen(QColor("#8fb3cb"), 2))
            painter.drawEllipse(rotor_center, 8, 4)
            painter.setBrush(QColor("#66e3c4"))
            painter.drawEllipse(rotor_center, 2.5, 2.5)


class TelemetryCard(QFrame):
    def __init__(self, title: str, value: str = "--", unit: str = "") -> None:
        super().__init__()
        self.setObjectName("telemetryCard")
        self.setMinimumHeight(84)
        self.setMinimumWidth(135)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(13, 9, 13, 9)
        layout.setSpacing(2)

        title_label = QLabel(title.upper())
        title_label.setObjectName("cardTitle")

        value_row = QHBoxLayout()
        value_row.setSpacing(5)
        self.value_label = QLabel(value)
        self.value_label.setObjectName("cardValue")
        self.value_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.unit_label = QLabel(unit)
        self.unit_label.setObjectName("cardUnit")
        self.unit_label.setAlignment(Qt.AlignmentFlag.AlignBottom)

        value_row.addWidget(self.value_label)
        value_row.addWidget(self.unit_label)
        value_row.addStretch(1)
        layout.addWidget(title_label)
        layout.addLayout(value_row)

    def set_value(self, value: str, unit: str | None = None) -> None:
        self.value_label.setText(value)
        if unit is not None:
            self.unit_label.setText(unit)


class ArtificialHorizon(QWidget):
    """Compact roll/pitch indicator."""

    def __init__(self) -> None:
        super().__init__()
        self.roll = 0.0
        self.pitch = 0.0
        self.setMinimumSize(220, 145)
        self.setMaximumSize(285, 185)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def set_attitude(self, roll: float, pitch: float) -> None:
        self.roll = roll
        self.pitch = max(-45.0, min(45.0, pitch))
        self.update()

    def paintEvent(self, event: Any) -> None:
        from PySide6.QtGui import QPainterPath

        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = self.rect().adjusted(5, 5, -5, -5)
        center = QPointF(rect.center())

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor("#08131f"))
        painter.drawRoundedRect(QRectF(rect), 14, 14)

        painter.save()
        path = QPainterPath()
        path.addRoundedRect(QRectF(rect), 14, 14)
        painter.setClipPath(path)
        painter.translate(center)
        painter.rotate(-self.roll)
        painter.translate(0, self.pitch * (rect.height() / 70.0))

        world = QRectF(-rect.width(), -rect.height() * 2, rect.width() * 2, rect.height() * 4)
        painter.fillRect(QRectF(world.left(), world.top(), world.width(), rect.height() * 2), QColor("#2f7fb7"))
        painter.fillRect(QRectF(world.left(), 0, world.width(), rect.height() * 2), QColor("#7c4d2a"))
        painter.setPen(QPen(QColor("#f8fafc"), 2))
        painter.drawLine(QPointF(world.left(), 0), QPointF(world.right(), 0))

        for degree in range(-30, 35, 10):
            if degree == 0:
                continue
            y = -(degree * rect.height() / 70.0)
            painter.setPen(QPen(QColor(255, 255, 255, 210), 1.5))
            painter.drawLine(QPointF(-34, y), QPointF(34, y))
        painter.restore()

        painter.setPen(QPen(QColor("#f7c948"), 3))
        painter.drawLine(QPointF(center.x() - 42, center.y()), QPointF(center.x() - 10, center.y()))
        painter.drawLine(QPointF(center.x() + 10, center.y()), QPointF(center.x() + 42, center.y()))
        painter.drawLine(QPointF(center.x(), center.y() - 7), QPointF(center.x(), center.y() + 10))

        painter.setPen(QPen(QColor("#dbeafe"), 1.5))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawRoundedRect(QRectF(rect), 14, 14)

        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.DemiBold))
        painter.setPen(QColor("#f8fafc"))
        painter.drawText(
            rect.adjusted(10, 7, -10, -7),
            Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignHCenter,
            f"R {self.roll:+.1f}°   P {self.pitch:+.1f}°",
        )


class CompassWidget(QWidget):
    """Compact heading indicator."""

    def __init__(self) -> None:
        super().__init__()
        self.heading = 0.0
        self.setFixedSize(150, 150)

    def set_heading(self, heading: float | None) -> None:
        self.heading = 0.0 if heading is None else heading % 360.0
        self.update()

    def paintEvent(self, event: Any) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        side = min(self.width(), self.height()) - 12
        center = QPointF(self.width() / 2, self.height() / 2)
        radius = side / 2

        painter.setBrush(QColor("#08131f"))
        painter.setPen(QPen(QColor("#254158"), 2))
        painter.drawEllipse(center, radius, radius)

        painter.save()
        painter.translate(center)
        painter.rotate(-self.heading)
        for degree in range(0, 360, 30):
            painter.save()
            painter.rotate(degree)
            painter.setPen(QPen(QColor("#dbeafe"), 1.5))
            painter.drawLine(QPointF(0, -radius + 8), QPointF(0, -radius + 18))
            painter.restore()

        painter.setFont(QFont("Segoe UI", 8, QFont.Weight.Bold))
        for degree, label in {0: "N", 90: "E", 180: "S", 270: "W"}.items():
            angle = math.radians(degree - 90)
            x = math.cos(angle) * (radius - 27)
            y = math.sin(angle) * (radius - 27)
            painter.setPen(QColor("#fb7185" if label == "N" else "#f8fafc"))
            painter.drawText(QRectF(x - 10, y - 9, 20, 18), Qt.AlignmentFlag.AlignCenter, label)
        painter.restore()

        painter.setBrush(QColor("#f7c948"))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawPolygon(
            QPolygonF(
                [
                    QPointF(center.x(), center.y() - radius + 2),
                    QPointF(center.x() - 6, center.y() - radius + 15),
                    QPointF(center.x() + 6, center.y() - radius + 15),
                ]
            )
        )

        painter.setPen(QColor("#f8fafc"))
        painter.setFont(QFont("Segoe UI", 15, QFont.Weight.Bold))
        painter.drawText(
            QRectF(center.x() - 45, center.y() - 18, 90, 36),
            Qt.AlignmentFlag.AlignCenter,
            f"{self.heading:03.0f}°",
        )


MAP_HTML = r"""
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"
      integrity="sha256-p4NxAoJBhIIN+hmNHrzRCf9tD/miZyoHS5obTRR9BMY=" crossorigin="">
<style>
html, body, #map { width:100%; height:100%; margin:0; background:#071018; }
.leaflet-container { background:#071018; font-family:Segoe UI, sans-serif; }
.drone-marker-shell { background:transparent; border:none; }
.drone-marker { width:42px; height:42px; transform-origin:21px 21px; filter:drop-shadow(0 2px 4px rgba(0,0,0,.75)); }
.waiting {
  position:absolute; z-index:1000; top:12px; left:50%; transform:translateX(-50%);
  padding:7px 12px; border-radius:12px; color:#b8d4e5; background:rgba(7,16,24,.86);
  border:1px solid #27455e; font-size:12px; pointer-events:none;
}
</style>
</head>
<body>
<div id="map"></div>
<div id="waiting" class="waiting">Waiting for GPS position</div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"
        integrity="sha256-20nQCchB9co0qIjJZRGuk2/Z9VM+kNiyxNV1lvTlZBo=" crossorigin=""></script>
<script>
const map = L.map('map', { zoomControl: true, preferCanvas: true }).setView([20, 0], 2);
L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 19,
    attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
}).addTo(map);

let droneMarker = null;
let trail = [];
let trailLine = L.polyline([], {color:'#20d4b4', weight:3, opacity:0.78}).addTo(map);
let firstPosition = true;

function droneSvg(heading) {
    return `<div class="drone-marker" style="transform:rotate(${heading}deg)">
      <svg width="42" height="42" viewBox="0 0 42 42" xmlns="http://www.w3.org/2000/svg">
        <circle cx="9" cy="9" r="6" fill="#071018" stroke="#9bdacb" stroke-width="2"/>
        <circle cx="33" cy="9" r="6" fill="#071018" stroke="#9bdacb" stroke-width="2"/>
        <circle cx="9" cy="33" r="6" fill="#071018" stroke="#9bdacb" stroke-width="2"/>
        <circle cx="33" cy="33" r="6" fill="#071018" stroke="#9bdacb" stroke-width="2"/>
        <path d="M12 12 L30 30 M30 12 L12 30" stroke="#66e3c4" stroke-width="3" stroke-linecap="round"/>
        <rect x="15" y="16" width="12" height="10" rx="4" fill="#0f2b35" stroke="#e7fff8" stroke-width="2"/>
        <path d="M21 3 L17 12 L25 12 Z" fill="#f7c948"/>
      </svg>
    </div>`;
}

function makeIcon(heading) {
    return L.divIcon({
        className:'drone-marker-shell',
        html:droneSvg(heading),
        iconSize:[42,42],
        iconAnchor:[21,21]
    });
}

function setDroneLocation(lat, lon, heading, follow) {
    const pos = [lat, lon];
    document.getElementById('waiting').style.display = 'none';

    if (!droneMarker) {
        droneMarker = L.marker(pos, {icon:makeIcon(heading), zIndexOffset:1000}).addTo(map);
    } else {
        droneMarker.setLatLng(pos);
        droneMarker.setIcon(makeIcon(heading));
    }

    const last = trail.length ? trail[trail.length - 1] : null;
    if (!last || Math.abs(last[0]-lat) > 0.000001 || Math.abs(last[1]-lon) > 0.000001) {
        trail.push(pos);
        if (trail.length > 300) trail.shift();
        trailLine.setLatLngs(trail);
    }

    if (firstPosition) {
        map.setView(pos, 17);
        firstPosition = false;
    } else if (follow) {
        map.panTo(pos, {animate:true, duration:0.35});
    }
}

function centerOnDrone() {
    if (droneMarker) map.setView(droneMarker.getLatLng(), Math.max(map.getZoom(), 17));
}

setTimeout(() => map.invalidateSize(), 250);
window.addEventListener('resize', () => map.invalidateSize());
</script>
</body>
</html>
"""


class MapWidget(QFrame):
    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("mapPanel")
        self.setMinimumHeight(390)
        self._loaded = False
        self._latest_position: tuple[float, float, float] | None = None
        self._follow = True

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        if WEBENGINE_AVAILABLE and QWebEngineView is not None:
            self.web_view = QWebEngineView()
            self.web_view.setContextMenuPolicy(Qt.ContextMenuPolicy.NoContextMenu)
            self.web_view.loadFinished.connect(self._on_loaded)
            self.web_view.setHtml(MAP_HTML, QUrl("https://unpkg.com/"))
            layout.addWidget(self.web_view, 1)
        else:
            self.web_view = None
            fallback = QLabel(
                "Map component is unavailable.\nInstall or repair PySide6, including Qt WebEngine."
            )
            fallback.setAlignment(Qt.AlignmentFlag.AlignCenter)
            fallback.setWordWrap(True)
            fallback.setObjectName("mapFallback")
            layout.addWidget(fallback, 1)

        footer = QFrame()
        footer.setObjectName("mapFooter")
        footer_layout = QHBoxLayout(footer)
        footer_layout.setContentsMargins(10, 7, 10, 7)
        footer_layout.setSpacing(8)

        self.position_label = QLabel("GPS position unavailable")
        self.position_label.setObjectName("mapPosition")
        self.position_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.center_button = QPushButton("CENTER DRONE")
        self.center_button.setMaximumWidth(140)
        self.center_button.clicked.connect(self.center_on_drone)

        footer_layout.addWidget(self.position_label, 1)
        footer_layout.addWidget(self.center_button)
        layout.addWidget(footer)

    def _on_loaded(self, ok: bool) -> None:
        self._loaded = ok
        if ok and self._latest_position is not None:
            lat, lon, heading = self._latest_position
            self.update_position(lat, lon, heading)

    def update_position(self, latitude: float, longitude: float, heading: float | None) -> None:
        heading_value = 0.0 if heading is None else float(heading) % 360.0
        self._latest_position = (latitude, longitude, heading_value)
        self.position_label.setText(f"{latitude:.7f}, {longitude:.7f}")
        if self.web_view is None or not self._loaded:
            return
        script = (
            f"setDroneLocation({latitude:.8f}, {longitude:.8f}, "
            f"{heading_value:.2f}, {str(self._follow).lower()});"
        )
        self.web_view.page().runJavaScript(script)

    def set_gps_text(self, text: str) -> None:
        if self._latest_position is None:
            self.position_label.setText(text)

    def center_on_drone(self) -> None:
        if self.web_view is not None and self._loaded:
            self.web_view.page().runJavaScript("centerOnDrone();")


class StatusPill(QLabel):
    def __init__(self) -> None:
        super().__init__("DISCONNECTED")
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setMinimumWidth(130)
        self.set_status("disconnected", "Disconnected")

    def set_status(self, state: str, text: str) -> None:
        colors = {
            "connected": ("#123b2a", "#4ade80"),
            "connecting": ("#3d3211", "#facc15"),
            "warning": ("#45260c", "#fb923c"),
            "error": ("#47151d", "#fb7185"),
            "disconnected": ("#1f2937", "#94a3b8"),
        }
        background, foreground = colors.get(state, colors["disconnected"])
        self.setText(text.upper())
        self.setStyleSheet(
            f"background:{background}; color:{foreground}; padding:7px 11px; "
            "border-radius:13px; font-weight:700;"
        )


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Custom MAVLink GCS")
        self.resize(1320, 860)
        self.setMinimumSize(640, 560)

        self.worker: TelemetryWorker | None = None
        self.current_state = TelemetryState()
        self.cards: dict[str, TelemetryCard] = {}
        self.mode_buttons: dict[str, QPushButton] = {}
        self._responsive_mode: tuple[int, int, int] | None = None
        self._last_map_update = 0.0

        self._build_ui()
        self._apply_style()
        self._set_controls_enabled(False)
        self._apply_responsive_layout(force=True)

        self.ui_timer = QTimer(self)
        self.ui_timer.timeout.connect(self._refresh_link_age)
        self.ui_timer.start(500)

        QTimer.singleShot(450, self.connect_telemetry)

    def _build_ui(self) -> None:
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.Shape.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.setCentralWidget(scroll)

        root = QWidget()
        scroll.setWidget(root)
        self.root_layout = QVBoxLayout(root)
        self.root_layout.setContentsMargins(14, 12, 14, 12)
        self.root_layout.setSpacing(10)

        self.header = QFrame()
        self.header.setObjectName("header")
        self.header_layout = QGridLayout(self.header)
        self.header_layout.setContentsMargins(14, 9, 14, 9)
        self.header_layout.setHorizontalSpacing(10)
        self.header_layout.setVerticalSpacing(8)

        self.drone_icon = DroneIconWidget()
        self.connection_input = QLineEdit(DEFAULT_CONNECTION)
        self.connection_input.setMinimumWidth(210)
        self.connection_input.setToolTip("MAVProxy UDP output for this GCS")

        self.connect_button = QPushButton("DISCONNECT")
        self.connect_button.setObjectName("primaryButton")
        self.connect_button.clicked.connect(self.toggle_connection)
        self.status_pill = StatusPill()
        self.root_layout.addWidget(self.header)

        self.summary_layout = QGridLayout()
        self.summary_layout.setHorizontalSpacing(8)
        self.summary_layout.setVerticalSpacing(8)
        self.cards["state"] = TelemetryCard("State")
        self.cards["mode"] = TelemetryCard("Mode")
        self.cards["voltage"] = TelemetryCard("Battery", "--", "V")
        self.cards["gps"] = TelemetryCard("GPS", "--", "")
        self.cards["altitude"] = TelemetryCard("Altitude", "--", "m")
        self.cards["airspeed"] = TelemetryCard("Air speed", "--", "m/s")
        self.root_layout.addLayout(self.summary_layout)

        self.body_layout = QGridLayout()
        self.body_layout.setHorizontalSpacing(10)
        self.body_layout.setVerticalSpacing(10)

        self.map_widget = MapWidget()

        self.controls_panel = QFrame()
        self.controls_panel.setObjectName("panel")
        self.controls_panel.setMinimumWidth(300)
        controls_layout = QVBoxLayout(self.controls_panel)
        controls_layout.setContentsMargins(12, 12, 12, 12)
        controls_layout.setSpacing(10)

        self.instrument_layout = QGridLayout()
        self.instrument_layout.setHorizontalSpacing(10)
        self.instrument_layout.setVerticalSpacing(8)
        self.horizon = ArtificialHorizon()
        self.compass = CompassWidget()
        controls_layout.addLayout(self.instrument_layout)

        self.arm_layout = QGridLayout()
        self.arm_layout.setHorizontalSpacing(8)
        self.arm_layout.setVerticalSpacing(8)
        self.arm_button = QPushButton("ARM")
        self.arm_button.setObjectName("armButton")
        self.arm_button.setMinimumHeight(50)
        self.arm_button.clicked.connect(self.request_arm)
        self.disarm_button = QPushButton("DISARM")
        self.disarm_button.setObjectName("disarmButton")
        self.disarm_button.setMinimumHeight(50)
        self.disarm_button.clicked.connect(self.request_disarm)
        controls_layout.addLayout(self.arm_layout)

        self.mode_layout = QGridLayout()
        self.mode_layout.setHorizontalSpacing(8)
        self.mode_layout.setVerticalSpacing(8)
        for button_text, mode_name in MODE_BUTTONS:
            button = QPushButton(button_text)
            button.setProperty("modeButton", True)
            button.setProperty("modeActive", False)
            button.setMinimumHeight(43)
            button.setMinimumWidth(105)
            if mode_name in ("RTL", "LAND"):
                button.setProperty("criticalMode", True)
            button.clicked.connect(
                lambda checked=False, name=mode_name: self.request_mode(name)
            )
            self.mode_buttons[mode_name] = button
        controls_layout.addLayout(self.mode_layout)

        self.command_status_label = QLabel("No command sent")
        self.command_status_label.setObjectName("commandStatus")
        self.command_status_label.setWordWrap(True)
        controls_layout.addWidget(self.command_status_label)
        controls_layout.addStretch(1)

        self.root_layout.addLayout(self.body_layout, 1)

        message_panel = QFrame()
        message_panel.setObjectName("panel")
        message_layout = QVBoxLayout(message_panel)
        message_layout.setContentsMargins(12, 9, 12, 10)
        message_header = QHBoxLayout()
        message_title = QLabel("MESSAGES")
        message_title.setObjectName("messagesTitle")
        clear_button = QPushButton("CLEAR")
        clear_button.setMaximumWidth(90)
        clear_button.clicked.connect(lambda: self.message_log.clear())
        message_header.addWidget(message_title)
        message_header.addStretch(1)
        message_header.addWidget(clear_button)
        self.message_log = QTextEdit()
        self.message_log.setReadOnly(True)
        self.message_log.setMaximumHeight(125)
        self.message_log.setPlaceholderText("Flight-controller and command messages appear here.")
        message_layout.addLayout(message_header)
        message_layout.addWidget(self.message_log)
        self.root_layout.addWidget(message_panel)

        status_bar = QStatusBar()
        self.setStatusBar(status_bar)
        self.status_text = QLabel("Ready")
        self.packet_text = QLabel("Last MAVLink message: -")
        status_bar.addWidget(self.status_text, 1)
        status_bar.addPermanentWidget(self.packet_text)

    @staticmethod
    def _clear_layout(layout: QGridLayout) -> None:
        while layout.count():
            layout.takeAt(0)

    def _apply_responsive_layout(self, force: bool = False) -> None:
        width = max(self.width(), 1)
        summary_columns = 6 if width >= 1160 else 3 if width >= 780 else 2
        body_stacked = 1 if width < 1030 else 0
        button_columns = 3 if width >= 1180 and body_stacked else 2 if width >= 700 else 1
        mode = (summary_columns, body_stacked, button_columns)
        if not force and mode == self._responsive_mode:
            return
        self._responsive_mode = mode

        self._clear_layout(self.header_layout)
        if width >= 880:
            self.header_layout.addWidget(self.drone_icon, 0, 0)
            self.header_layout.addWidget(self.connection_input, 0, 1)
            self.header_layout.setColumnStretch(1, 1)
            self.header_layout.addWidget(self.connect_button, 0, 2)
            self.header_layout.addWidget(self.status_pill, 0, 3)
        else:
            self.header_layout.addWidget(self.drone_icon, 0, 0)
            self.header_layout.addWidget(self.status_pill, 0, 1, 1, 2)
            self.header_layout.setColumnStretch(1, 1)
            self.header_layout.addWidget(self.connection_input, 1, 0, 1, 2)
            self.header_layout.addWidget(self.connect_button, 1, 2)

        self._clear_layout(self.summary_layout)
        ordered_keys = ("state", "mode", "voltage", "gps", "altitude", "airspeed")
        for index, key in enumerate(ordered_keys):
            row, column = divmod(index, summary_columns)
            self.summary_layout.addWidget(self.cards[key], row, column)
        for column in range(summary_columns):
            self.summary_layout.setColumnStretch(column, 1)

        self._clear_layout(self.body_layout)
        if body_stacked:
            self.body_layout.addWidget(self.map_widget, 0, 0)
            self.body_layout.addWidget(self.controls_panel, 1, 0)
            self.body_layout.setColumnStretch(0, 1)
        else:
            self.body_layout.addWidget(self.map_widget, 0, 0)
            self.body_layout.addWidget(self.controls_panel, 0, 1)
            self.body_layout.setColumnStretch(0, 3)
            self.body_layout.setColumnStretch(1, 2)

        self._clear_layout(self.instrument_layout)
        if width < 700:
            self.instrument_layout.addWidget(self.horizon, 0, 0, 1, 1, Qt.AlignmentFlag.AlignCenter)
            self.instrument_layout.addWidget(self.compass, 1, 0, 1, 1, Qt.AlignmentFlag.AlignCenter)
        else:
            self.instrument_layout.addWidget(self.horizon, 0, 0, 1, 1, Qt.AlignmentFlag.AlignCenter)
            self.instrument_layout.addWidget(self.compass, 0, 1, 1, 1, Qt.AlignmentFlag.AlignCenter)
            self.instrument_layout.setColumnStretch(0, 1)

        self._clear_layout(self.arm_layout)
        if width < 700:
            self.arm_layout.addWidget(self.arm_button, 0, 0)
            self.arm_layout.addWidget(self.disarm_button, 1, 0)
        else:
            self.arm_layout.addWidget(self.arm_button, 0, 0)
            self.arm_layout.addWidget(self.disarm_button, 0, 1)
            self.arm_layout.setColumnStretch(0, 1)
            self.arm_layout.setColumnStretch(1, 1)

        self._clear_layout(self.mode_layout)
        for index, (mode_name, button) in enumerate(self.mode_buttons.items()):
            row, column = divmod(index, button_columns)
            self.mode_layout.addWidget(button, row, column)
        for column in range(button_columns):
            self.mode_layout.setColumnStretch(column, 1)

        QTimer.singleShot(80, self.map_widget.center_on_drone)

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QMainWindow, QWidget {
                background: #071018;
                color: #dbeafe;
                font-family: "Segoe UI";
                font-size: 10pt;
            }
            QFrame#header, QFrame#panel, QFrame#mapPanel {
                background: #0b1722;
                border: 1px solid #1d3345;
                border-radius: 11px;
            }
            QFrame#mapPanel { background: #071018; }
            QLabel#messagesTitle {
                color: #b8d4e5;
                font-size: 10pt;
                font-weight: 800;
                letter-spacing: 1px;
            }
            QLineEdit {
                background: #071018;
                border: 1px solid #29445a;
                border-radius: 8px;
                padding: 8px 10px;
                color: #e6f4ff;
            }
            QLineEdit:focus { border-color: #66e3c4; }
            QPushButton {
                background: #152737;
                border: 1px solid #2a455a;
                border-radius: 8px;
                padding: 8px 12px;
                color: #dbeafe;
                font-weight: 650;
            }
            QPushButton:hover:enabled {
                background: #1d3447;
                border-color: #4c718c;
            }
            QPushButton:disabled {
                color: #516473;
                background: #0d1922;
                border-color: #182a38;
            }
            QPushButton#primaryButton {
                background: #17675e;
                border-color: #2f9b8b;
                color: white;
            }
            QPushButton#armButton {
                background: #14532d;
                border-color: #22c55e;
                color: #dcfce7;
                font-size: 12pt;
            }
            QPushButton#disarmButton {
                background: #5b1821;
                border-color: #ef4444;
                color: #fee2e2;
                font-size: 12pt;
            }
            QPushButton[modeButton="true"] {
                background: #132536;
                border-color: #2b4a60;
            }
            QPushButton[modeButton="true"][modeActive="true"] {
                background: #17675e;
                border: 2px solid #66e3c4;
                color: white;
            }
            QPushButton[criticalMode="true"] { border-color: #b45309; }
            QFrame#telemetryCard {
                background: #0d1c28;
                border: 1px solid #1d3547;
                border-radius: 9px;
            }
            QLabel#cardTitle {
                color: #7894aa;
                font-size: 8pt;
                font-weight: 700;
                letter-spacing: 0.6px;
            }
            QLabel#cardValue {
                color: #f3f8fc;
                font-size: 16pt;
                font-weight: 750;
            }
            QLabel#cardUnit {
                color: #6f8da5;
                font-size: 8pt;
                padding-bottom: 3px;
            }
            QLabel#commandStatus {
                background: #102232;
                border: 1px solid #27455e;
                border-radius: 8px;
                color: #a9c5d8;
                padding: 9px;
            }
            QFrame#mapFooter {
                background: #0b1722;
                border-top: 1px solid #1d3345;
            }
            QLabel#mapPosition { color: #a9c5d8; font-family: Consolas; }
            QLabel#mapFallback { color: #8fb3cb; padding: 25px; }
            QTextEdit {
                background: #050c12;
                border: 1px solid #1d3345;
                border-radius: 8px;
                color: #b8d4e5;
                font-family: Consolas;
                padding: 7px;
            }
            QScrollArea { border: none; background: #071018; }
            QStatusBar {
                background: #08131d;
                border-top: 1px solid #152b3c;
                color: #7894aa;
            }
            """
        )

    def resizeEvent(self, event: Any) -> None:
        super().resizeEvent(event)
        self._apply_responsive_layout()

    def toggle_connection(self) -> None:
        if self.worker is not None and self.worker.isRunning():
            self.disconnect_telemetry()
        else:
            self.connect_telemetry()

    def connect_telemetry(self) -> None:
        if self.worker is not None and self.worker.isRunning():
            return

        connection = self.connection_input.text().strip()
        if not connection:
            QMessageBox.warning(self, "Missing connection", "Enter a MAVLink connection string.")
            return

        self.connection_input.setEnabled(False)
        self.connect_button.setText("DISCONNECT")
        self.worker = TelemetryWorker(connection, self)
        self.worker.telemetry.connect(self.update_telemetry)
        self.worker.link_status.connect(self.update_link_status)
        self.worker.log_message.connect(self.append_log)
        self.worker.command_status.connect(self.update_command_status)
        self.worker.finished.connect(self._worker_finished)
        self.worker.start()

    def disconnect_telemetry(self) -> None:
        if self.worker is not None:
            self.worker.request_stop()
            self.connect_button.setEnabled(False)
            self.worker.wait(2500)

    def _worker_finished(self) -> None:
        self.connection_input.setEnabled(True)
        self.connect_button.setEnabled(True)
        self.connect_button.setText("CONNECT")
        self._set_controls_enabled(False)
        if self.worker is not None:
            self.worker.deleteLater()
            self.worker = None

    def update_link_status(self, state: str, text: str) -> None:
        short_text = {
            "connected": "Connected",
            "connecting": "Connecting",
            "warning": "Link stale",
            "error": "Connection error",
            "disconnected": "Disconnected",
        }.get(state, text)
        self.status_pill.set_status(state, short_text)
        self.status_text.setText(text)
        self._set_controls_enabled(state == "connected")
        if state == "error":
            self.connect_button.setText("CONNECT")

    def append_log(self, text: str) -> None:
        timestamp = time.strftime("%H:%M:%S")
        self.message_log.append(f"[{timestamp}] {text}")

    def update_command_status(self, level: str, text: str) -> None:
        colors = {
            "pending": ("#3d3211", "#facc15", "#6b5a20"),
            "success": ("#123b2a", "#4ade80", "#216942"),
            "error": ("#47151d", "#fb7185", "#7f2634"),
        }
        background, foreground, border = colors.get(
            level, ("#102232", "#a9c5d8", "#27455e")
        )
        self.command_status_label.setText(text)
        self.command_status_label.setStyleSheet(
            f"background:{background}; color:{foreground}; border:1px solid {border}; "
            "border-radius:8px; padding:9px; font-weight:700;"
        )

    def update_telemetry(self, data: dict) -> None:
        self.current_state = TelemetryState(**data)
        state = self.current_state

        self.cards["state"].set_value("ARMED" if state.armed else "DISARMED")
        self.cards["mode"].set_value(state.mode)
        self.cards["voltage"].set_value(self._fmt(state.battery_voltage_v, 2))

        fix_name = GPS_FIX_NAMES.get(state.gps_fix, "UNKNOWN")
        sats = "--" if state.satellites is None else str(state.satellites)
        self.cards["gps"].set_value(fix_name, f"{sats} sats")
        self.cards["altitude"].set_value(self._fmt(state.relative_altitude_m, 1))
        self.cards["airspeed"].set_value(self._fmt(state.airspeed_mps, 1))

        self.horizon.set_attitude(state.roll_deg, state.pitch_deg)
        self.compass.set_heading(state.heading_deg)
        self.packet_text.setText(f"Last MAVLink message: {state.last_message}")

        valid_position = (
            state.latitude is not None
            and state.longitude is not None
            and not (abs(state.latitude) < 0.000001 and abs(state.longitude) < 0.000001)
            and (state.gps_fix is None or state.gps_fix >= 2)
        )
        now = time.monotonic()
        if valid_position and now - self._last_map_update >= 0.5:
            self.map_widget.update_position(
                float(state.latitude),
                float(state.longitude),
                state.heading_deg,
            )
            self._last_map_update = now
        elif not valid_position:
            self.map_widget.set_gps_text(f"{fix_name} / {sats} satellites")

        connected = state.connected and self.worker is not None
        self._set_controls_enabled(connected)
        self.arm_button.setEnabled(connected and not state.armed)
        self.disarm_button.setEnabled(connected and state.armed)
        self._highlight_current_mode(state.mode)

    def _set_controls_enabled(self, enabled: bool) -> None:
        self.arm_button.setEnabled(enabled and not self.current_state.armed)
        self.disarm_button.setEnabled(enabled and self.current_state.armed)
        for button in self.mode_buttons.values():
            button.setEnabled(enabled)

    def _highlight_current_mode(self, current_mode: str) -> None:
        normalized = current_mode.upper().replace(" ", "_")
        for mode_name, button in self.mode_buttons.items():
            button.setProperty("modeActive", mode_name == normalized)
            button.style().unpolish(button)
            button.style().polish(button)

    def request_arm(self) -> None:
        if not self._require_connection():
            return
        answer = QMessageBox.warning(
            self,
            "Confirm arm",
            "Arm the vehicle?\n\nConfirm the propeller area is clear and all pre-arm checks are satisfied.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if answer == QMessageBox.StandardButton.Yes and self.worker is not None:
            self.worker.queue_arm(True)

    def request_disarm(self) -> None:
        if not self._require_connection():
            return
        answer = QMessageBox.critical(
            self,
            "Confirm disarm",
            "Disarm the vehicle?\n\nUse this only when the vehicle is safely landed. "
            "This application sends a standard disarm command and does not force disarm.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if answer == QMessageBox.StandardButton.Yes and self.worker is not None:
            self.worker.queue_arm(False)

    def request_mode(self, mode_name: str) -> None:
        if not self._require_connection():
            return

        display_name = mode_name.replace("_", " ")
        if self.current_state.mode.upper().replace(" ", "_") == mode_name:
            self.update_command_status("success", f"Already in {display_name}")
            return

        warning = "Change flight mode to " + display_name + "?"
        if mode_name == "RTL":
            warning += "\n\nThe vehicle will attempt to return to its configured home position."
        elif mode_name == "LAND":
            warning += "\n\nThe vehicle will begin its landing procedure."
        else:
            warning += "\n\nMake sure this mode is appropriate for the current flight condition."

        answer = QMessageBox.question(
            self,
            "Confirm mode change",
            warning,
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if answer == QMessageBox.StandardButton.Yes and self.worker is not None:
            self.worker.queue_mode(mode_name)

    def _require_connection(self) -> bool:
        if self.worker is None or not self.worker.isRunning() or not self.current_state.connected:
            QMessageBox.warning(
                self,
                "Not connected",
                "Connect to the flight controller before sending commands.",
            )
            return False
        return True

    def _refresh_link_age(self) -> None:
        state = self.current_state
        if state.last_update_monotonic <= 0:
            return
        age = time.monotonic() - state.last_update_monotonic
        self.packet_text.setText(f"Last MAVLink: {state.last_message} - {age:.1f}s ago")

    @staticmethod
    def _fmt(value: float | int | None, decimals: int) -> str:
        if value is None:
            return "--"
        if decimals == 0:
            return str(int(round(float(value))))
        return f"{float(value):.{decimals}f}"

    def closeEvent(self, event: Any) -> None:
        if self.worker is not None and self.worker.isRunning():
            self.worker.request_stop()
            self.worker.wait(2500)
        event.accept()


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName("Custom MAVLink GCS")
    app.setOrganizationName("Custom GCS")
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
