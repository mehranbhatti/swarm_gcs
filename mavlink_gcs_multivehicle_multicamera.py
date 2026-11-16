#!/usr/bin/env python3


from __future__ import annotations

import json
import math
import queue
import sys
import time
from dataclasses import asdict, dataclass
from typing import Any, Callable

from pymavlink import mavutil
from PySide6.QtCore import QPointF, QRectF, Qt, QThread, QTimer, QUrl, Signal
from PySide6.QtGui import (
    QAction,
    QColor,
    QFont,
    QIcon,
    QPainter,
    QPen,
    QPixmap,
    QPolygonF,
)
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDockWidget,
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
    QToolBar,
    QVBoxLayout,
    QWidget,
)

try:
    from PySide6.QtWebEngineWidgets import QWebEngineView

    WEBENGINE_AVAILABLE = True
except ImportError:
    QWebEngineView = None  # type: ignore[assignment]
    WEBENGINE_AVAILABLE = False

try:
    from PySide6.QtMultimedia import QCamera, QMediaCaptureSession, QMediaDevices
    from PySide6.QtMultimediaWidgets import QVideoWidget

    MULTIMEDIA_AVAILABLE = True
except ImportError:
    QCamera = None  # type: ignore[assignment]
    QMediaCaptureSession = None  # type: ignore[assignment]
    QMediaDevices = None  # type: ignore[assignment]
    QVideoWidget = None  # type: ignore[assignment]
    MULTIMEDIA_AVAILABLE = False


DEFAULT_CONNECTION = "udpin:127.0.0.1:14551"

MODE_BUTTONS: tuple[tuple[str, str], ...] = (
    ("STABILIZE", "STABILIZE"),
    ("ALT HOLD", "ALT_HOLD"),
    ("LOITER", "LOITER"),
)

GPS_FIX_NAMES = {
    0: "NO GPS",
    1: "NO FIX",
    2: "2D",
    3: "3D",
    4: "DGPS",
    5: "RTK FLOAT",
    6: "RTK FIXED",
    7: "STATIC",
    8: "PPP",
}


def friendly_mav_type(type_id: int | None) -> str:
    """Return a compact operator-facing name for MAVLink HEARTBEAT.type."""
    if type_id is None:
        return "UNKNOWN"
    friendly = {
        getattr(mavutil.mavlink, "MAV_TYPE_FIXED_WING", -100): "FIXED WING",
        getattr(mavutil.mavlink, "MAV_TYPE_QUADROTOR", -101): "QUADROTOR",
        getattr(mavutil.mavlink, "MAV_TYPE_HEXAROTOR", -102): "HEXAROTOR",
        getattr(mavutil.mavlink, "MAV_TYPE_OCTOROTOR", -103): "OCTOROTOR",
        getattr(mavutil.mavlink, "MAV_TYPE_TRICOPTER", -104): "TRICOPTER",
        getattr(mavutil.mavlink, "MAV_TYPE_HELICOPTER", -105): "HELICOPTER",
        getattr(mavutil.mavlink, "MAV_TYPE_VTOL_TAILSITTER_DUOROTOR", -106): "VTOL",
        getattr(mavutil.mavlink, "MAV_TYPE_VTOL_TAILSITTER_QUADROTOR", -107): "VTOL",
        getattr(mavutil.mavlink, "MAV_TYPE_VTOL_TILTROTOR", -108): "VTOL",
        getattr(mavutil.mavlink, "MAV_TYPE_VTOL_FIXEDROTOR", -109): "VTOL",
    }
    if int(type_id) in friendly:
        return friendly[int(type_id)]
    try:
        item = mavutil.mavlink.enums.get("MAV_TYPE", {}).get(int(type_id))
        name = str(getattr(item, "name", "UNKNOWN"))
        return name.replace("MAV_TYPE_", "").replace("_", " ")
    except Exception:
        return "UNKNOWN"


# Original palette inspired by modern dark aviation / telemetry interfaces.
BG = "#16191b"
SURFACE = "#202426"
SURFACE_2 = "#292e31"
SURFACE_3 = "#111416"
BORDER = "#353b3e"
MUTED = "#9aa3a8"
TEXT = "#edf2f4"
ACCENT = "#37a8db"
ACCENT_DARK = "#1d607d"
SUCCESS = "#59aa29"
WARNING = "#f5a623"
DANGER = "#d94343"
TEAL = "#35c6b0"
VIOLET = "#9d7be8"

# Replace this one string with your company/product name when ready.
BRAND_NAME = "GROUND CONTROL STATION"
BRAND_SUBTITLE = ""


def distance_m(lat1: float | None, lon1: float | None, lat2: float | None, lon2: float | None) -> float | None:
    """Great-circle horizontal distance between two WGS84 coordinates."""
    if None in (lat1, lon1, lat2, lon2):
        return None
    r = 6371000.0
    phi1 = math.radians(float(lat1))
    phi2 = math.radians(float(lat2))
    dphi = math.radians(float(lat2) - float(lat1))
    dlambda = math.radians(float(lon2) - float(lon1))
    a = (
        math.sin(dphi / 2.0) ** 2
        + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2.0) ** 2
    )
    return 2.0 * r * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))


@dataclass
class TelemetryState:
    connected: bool = False
    system_id: int = 0
    component_id: int = 0
    mode: str = "UNKNOWN"
    armed: bool = False
    vehicle_type_id: int | None = None
    vehicle_type: str = "UNKNOWN"

    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    heading_deg: float | None = None

    latitude: float | None = None
    longitude: float | None = None
    home_latitude: float | None = None
    home_longitude: float | None = None

    gps_fix: int | None = None
    satellites: int | None = None
    hdop: float | None = None

    relative_altitude_m: float | None = None
    airspeed_mps: float | None = None
    groundspeed_mps: float | None = None
    climb_mps: float | None = None

    battery_voltage_v: float | None = None
    battery_remaining_pct: int | None = None

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
        self.link_status.emit("connecting", "Opening MAVLink endpoint")
        self.log_message.emit(f"Opening MAVLink endpoint {self.connection_string}")

        try:
            self._vehicle = mavutil.mavlink_connection(
                self.connection_string,
                source_system=254,
                source_component=mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
            )

            self.link_status.emit("connecting", "Waiting for flight controller")
            heartbeat = self._wait_for_autopilot_heartbeat(timeout=20.0)
            if heartbeat is None:
                raise TimeoutError(
                    "No autopilot heartbeat received for 20 seconds. "
                    "Confirm MAVProxy is forwarding to UDP 14551."
                )

            if self._stop_requested:
                return

            self._state.connected = True
            self._parse_message(heartbeat)
            self._request_home_position()
            self.link_status.emit("connected", "MAVLink connected")
            self.log_message.emit(
                f"Connected to system {self._state.system_id}, "
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
                    last_packet = now
                    if stale_announced:
                        self.link_status.emit("connected", "MAVLink connected")
                        self.log_message.emit("Telemetry stream resumed")
                    stale_announced = False
                    self._parse_message(message)

                if now - last_emit >= 0.10:
                    self.telemetry.emit(asdict(self._state))
                    last_emit = now

                if now - last_packet > 4.0 and not stale_announced:
                    self.link_status.emit("warning", "Telemetry stale")
                    self.log_message.emit("No MAVLink packets for more than 4 seconds")
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
            message = self._vehicle.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)
            if message is None:
                continue
            vehicle_type = int(getattr(message, "type", -1))
            autopilot_type = int(getattr(message, "autopilot", -1))
            if vehicle_type == mavutil.mavlink.MAV_TYPE_GCS:
                continue
            if autopilot_type == mavutil.mavlink.MAV_AUTOPILOT_INVALID:
                continue
            return message
        return None

    def _process_commands(self) -> None:
        while True:
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
            raise RuntimeError("Flight-controller system ID is unavailable")
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
            0.0,  # no force-arm/disarm magic number
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        self._pending_commands[mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM] = label
        self.command_status.emit("pending", f"{label} sent - waiting for FC")
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
                f"Mode {normalized_name} is unavailable. FC reports: {available}"
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
        self.command_status.emit("pending", f"{normalized_name} requested")
        self.log_message.emit(f"Requested mode {normalized_name}")

    def _send_gcs_heartbeat(self) -> None:
        self._vehicle.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )

    def _request_home_position(self) -> None:
        """Ask the FC for HOME_POSITION so distance-from-home becomes available."""
        if self._vehicle is None:
            return
        try:
            self._vehicle.mav.command_long_send(
                self._state.system_id or self._vehicle.target_system,
                self._state.component_id or self._vehicle.target_component,
                mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_HOME_POSITION,
                0, 0, 0, 0, 0, 0,
            )
        except Exception as error:
            self.log_message.emit(f"Could not request home position: {error}")

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
            self._state.vehicle_type_id = vehicle_type
            self._state.vehicle_type = friendly_mav_type(vehicle_type)
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
            eph = int(getattr(message, "eph", 65535))
            self._state.hdop = None if eph == 65535 else eph / 100.0
            raw_lat = int(getattr(message, "lat", 0))
            raw_lon = int(getattr(message, "lon", 0))
            if raw_lat != 0 or raw_lon != 0:
                self._state.latitude = raw_lat / 1e7
                self._state.longitude = raw_lon / 1e7

        elif message_type == "HOME_POSITION":
            raw_lat = int(getattr(message, "latitude", 0))
            raw_lon = int(getattr(message, "longitude", 0))
            if raw_lat != 0 or raw_lon != 0:
                self._state.home_latitude = raw_lat / 1e7
                self._state.home_longitude = raw_lon / 1e7

        elif message_type == "SYS_STATUS":
            voltage = int(message.voltage_battery)
            self._state.battery_voltage_v = (
                None if voltage == 65535 else voltage / 1000.0
            )
            remaining = int(getattr(message, "battery_remaining", -1))
            self._state.battery_remaining_pct = None if remaining < 0 else remaining

        elif message_type == "BATTERY_STATUS":
            valid_cells = [
                int(value)
                for value in getattr(message, "voltages", [])
                if int(value) not in (0, 65535)
            ]
            if valid_cells:
                self._state.battery_voltage_v = sum(valid_cells) / 1000.0
            remaining = int(getattr(message, "battery_remaining", -1))
            if remaining >= 0:
                self._state.battery_remaining_pct = remaining

        elif message_type == "VFR_HUD":
            self._state.airspeed_mps = float(message.airspeed)
            self._state.groundspeed_mps = float(message.groundspeed)
            self._state.climb_mps = float(message.climb)
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


# ---------------------------------------------------------------------------
# Small vector icons - no image files required
# ---------------------------------------------------------------------------


def make_icon(kind: str, color: str = "#c8d0d4", size: int = 24) -> QIcon:
    pixmap = QPixmap(size, size)
    pixmap.fill(Qt.GlobalColor.transparent)
    p = QPainter(pixmap)
    p.setRenderHint(QPainter.RenderHint.Antialiasing)
    pen = QPen(QColor(color), 1.8)
    pen.setCapStyle(Qt.PenCapStyle.RoundCap)
    p.setPen(pen)
    p.setBrush(Qt.BrushStyle.NoBrush)
    c = size / 2

    if kind == "flight":
        p.drawEllipse(QPointF(c, c), 8, 8)
        p.drawLine(QPointF(c - 8, c), QPointF(c + 8, c))
        p.drawLine(QPointF(c, c - 8), QPointF(c, c + 8))
        p.drawLine(QPointF(c - 5, c - 5), QPointF(c + 5, c + 5))
    elif kind == "control":
        p.drawRoundedRect(QRectF(4, 6, size - 8, size - 12), 3, 3)
        p.drawLine(QPointF(8, 10), QPointF(size - 8, 10))
        p.drawLine(QPointF(8, 15), QPointF(size - 11, 15))
        p.setBrush(QColor(color))
        p.drawEllipse(QPointF(size - 8, 15), 2.2, 2.2)
    elif kind == "link":
        p.drawArc(QRectF(4, 8, 10, 10), 45 * 16, 270 * 16)
        p.drawArc(QRectF(10, 6, 10, 10), 225 * 16, 270 * 16)
    elif kind == "messages":
        p.drawRoundedRect(QRectF(4, 5, size - 8, size - 11), 3, 3)
        p.drawLine(QPointF(8, 10), QPointF(size - 8, 10))
        p.drawLine(QPointF(8, 14), QPointF(size - 11, 14))
        p.drawLine(QPointF(8, 18), QPointF(size - 13, 18))
    elif kind == "map":
        poly = QPolygonF(
            [
                QPointF(4, 6), QPointF(10, 4), QPointF(15, 6), QPointF(20, 4),
                QPointF(20, 18), QPointF(15, 20), QPointF(10, 18), QPointF(4, 20),
            ]
        )
        p.drawPolygon(poly)
        p.drawLine(QPointF(10, 4), QPointF(10, 18))
        p.drawLine(QPointF(15, 6), QPointF(15, 20))
    elif kind == "center":
        p.drawEllipse(QPointF(c, c), 6, 6)
        p.drawLine(QPointF(c, 2), QPointF(c, 7))
        p.drawLine(QPointF(c, size - 2), QPointF(c, size - 7))
        p.drawLine(QPointF(2, c), QPointF(7, c))
        p.drawLine(QPointF(size - 2, c), QPointF(size - 7, c))
    elif kind == "plus":
        p.drawLine(QPointF(c, 5), QPointF(c, size - 5))
        p.drawLine(QPointF(5, c), QPointF(size - 5, c))
    elif kind == "minus":
        p.drawLine(QPointF(5, c), QPointF(size - 5, c))
    elif kind == "track":
        p.drawLine(QPointF(5, 18), QPointF(9, 12))
        p.drawLine(QPointF(9, 12), QPointF(14, 15))
        p.drawLine(QPointF(14, 15), QPointF(19, 6))
        p.setBrush(QColor(color))
        for x, y in ((5, 18), (9, 12), (14, 15), (19, 6)):
            p.drawEllipse(QPointF(x, y), 1.6, 1.6)
    elif kind == "close":
        p.drawLine(QPointF(6, 6), QPointF(size - 6, size - 6))
        p.drawLine(QPointF(size - 6, 6), QPointF(6, size - 6))

    p.end()
    return QIcon(pixmap)


class BrandWidget(QWidget):
    """Small vector brand mark + wordmark; no external image asset required."""

    def __init__(self) -> None:
        super().__init__()
        self.setFixedSize(242, 42)
        self.setToolTip("Flight control station")

    def paintEvent(self, event: Any) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Angular wing / navigation mark.
        cx, cy = 20.0, 21.0
        p.setPen(QPen(QColor(ACCENT), 2.3))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawPolygon(QPolygonF([
            QPointF(cx, 5),
            QPointF(7, 28),
            QPointF(cx, 23),
            QPointF(33, 28),
        ]))
        p.drawLine(QPointF(9, 31), QPointF(20, 35))
        p.drawLine(QPointF(31, 31), QPointF(20, 35))
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor(TEAL))
        p.drawEllipse(QPointF(cx, cy), 3.4, 3.4)

        p.setPen(QColor(TEXT))
        p.setFont(QFont("Segoe UI", 10, QFont.Weight.Bold))
        p.drawText(
            QRectF(43, 8, 190, 26),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            BRAND_NAME,
        )


class StatusChip(QFrame):
    def __init__(self, label: str, value: str = "--") -> None:
        super().__init__()
        self.setObjectName("statusChip")
        self.setMinimumWidth(92)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(9, 4, 9, 4)
        layout.setSpacing(6)
        self.dot = QLabel("●")
        self.dot.setFixedWidth(10)
        self.label = QLabel(label)
        self.label.setObjectName("chipLabel")
        self.value = QLabel(value)
        self.value.setObjectName("chipValue")
        layout.addWidget(self.dot)
        layout.addWidget(self.label)
        layout.addWidget(self.value)
        self.set_state("neutral")

    def set_value(self, value: str) -> None:
        self.value.setText(value)

    def set_state(self, state: str) -> None:
        colors = {
            "good": SUCCESS,
            "warn": WARNING,
            "bad": DANGER,
            "info": ACCENT,
            "neutral": MUTED,
            "violet": VIOLET,
        }
        self.dot.setStyleSheet(f"color:{colors.get(state, MUTED)};")


class MetricRow(QWidget):
    def __init__(self, label: str, value: str = "--", unit: str = "") -> None:
        super().__init__()
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 3, 0, 3)
        self.label = QLabel(label)
        self.label.setObjectName("metricLabel")
        self.value = QLabel(value)
        self.value.setObjectName("metricValue")
        self.unit = QLabel(unit)
        self.unit.setObjectName("metricUnit")
        layout.addWidget(self.label)
        layout.addStretch(1)
        layout.addWidget(self.value)
        layout.addWidget(self.unit)

    def set_value(self, value: str, unit: str | None = None) -> None:
        self.value.setText(value)
        if unit is not None:
            self.unit.setText(unit)


class MiniValue(QWidget):
    def __init__(self, label: str, unit: str) -> None:
        super().__init__()
        self.setMinimumWidth(100)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(0)
        self.label = QLabel(label)
        self.label.setObjectName("miniLabel")
        row = QHBoxLayout()
        row.setSpacing(3)
        self.value = QLabel("--")
        self.value.setObjectName("miniValue")
        self.unit = QLabel(unit)
        self.unit.setObjectName("miniUnit")
        row.addWidget(self.value)
        row.addWidget(self.unit)
        row.addStretch(1)
        layout.addWidget(self.label)
        layout.addLayout(row)

    def set_value(self, value: str) -> None:
        self.value.setText(value)


class ArtificialHorizon(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.roll = 0.0
        self.pitch = 0.0
        self.setFixedSize(164, 126)

    def set_attitude(self, roll: float, pitch: float) -> None:
        self.roll = roll
        self.pitch = max(-45.0, min(45.0, pitch))
        self.update()

    def paintEvent(self, event: Any) -> None:
        from PySide6.QtGui import QPainterPath

        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        rect = QRectF(self.rect()).adjusted(3, 3, -3, -3)
        center = rect.center()

        path = QPainterPath()
        path.addRoundedRect(rect, 9, 9)
        p.setClipPath(path)
        p.translate(center)
        p.rotate(-self.roll)
        p.translate(0, self.pitch * 1.8)

        world = QRectF(-250, -250, 500, 500)
        p.fillRect(QRectF(world.left(), world.top(), world.width(), 250), QColor("#286b91"))
        p.fillRect(QRectF(world.left(), 0, world.width(), 250), QColor("#775235"))
        p.setPen(QPen(QColor("#f2f4f5"), 1.5))
        p.drawLine(QPointF(-200, 0), QPointF(200, 0))
        for degree in (-30, -20, -10, 10, 20, 30):
            y = -(degree * 1.8)
            half = 20 if abs(degree) == 10 else 28
            p.drawLine(QPointF(-half, y), QPointF(half, y))

        p.resetTransform()
        p.setClipping(False)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(QColor(BORDER), 1.4))
        p.drawRoundedRect(rect, 9, 9)
        p.setPen(QPen(QColor(WARNING), 2.3))
        cy = self.height() / 2
        cx = self.width() / 2
        p.drawLine(QPointF(cx - 34, cy), QPointF(cx - 9, cy))
        p.drawLine(QPointF(cx + 9, cy), QPointF(cx + 34, cy))
        p.drawLine(QPointF(cx, cy - 5), QPointF(cx, cy + 7))


class MiniCompass(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.heading = 0.0
        self.setFixedSize(112, 112)

    def set_heading(self, heading: float | None) -> None:
        self.heading = 0.0 if heading is None else heading % 360.0
        self.update()

    def paintEvent(self, event: Any) -> None:
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        center = QPointF(self.width() / 2, self.height() / 2)
        radius = 48
        p.setPen(QPen(QColor(BORDER), 1.5))
        p.setBrush(QColor(SURFACE_3))
        p.drawEllipse(center, radius, radius)

        p.save()
        p.translate(center)
        p.rotate(-self.heading)
        for degree in range(0, 360, 30):
            p.save()
            p.rotate(degree)
            p.setPen(QPen(QColor("#bec8cd"), 1.2))
            p.drawLine(QPointF(0, -42), QPointF(0, -36))
            p.restore()
        p.setFont(QFont("Segoe UI", 7, QFont.Weight.DemiBold))
        for degree, label in ((0, "N"), (90, "E"), (180, "S"), (270, "W")):
            angle = math.radians(degree - 90)
            x = math.cos(angle) * 29
            y = math.sin(angle) * 29
            p.setPen(QColor(DANGER if label == "N" else TEXT))
            p.drawText(QRectF(x - 8, y - 7, 16, 14), Qt.AlignmentFlag.AlignCenter, label)
        p.restore()

        p.setBrush(QColor(WARNING))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(
            QPolygonF(
                [
                    QPointF(center.x(), 4),
                    QPointF(center.x() - 5, 15),
                    QPointF(center.x() + 5, 15),
                ]
            )
        )
        p.setPen(QColor(TEXT))
        p.setFont(QFont("Segoe UI", 12, QFont.Weight.Bold))
        p.drawText(
            QRectF(center.x() - 34, center.y() - 14, 68, 28),
            Qt.AlignmentFlag.AlignCenter,
            f"{self.heading:03.0f}°",
        )


MAP_HTML = r"""
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" crossorigin="">
<style>
html, body, #map { width:100%; height:100%; margin:0; background:#111416; }
.leaflet-container { background:#111416; font-family:'Segoe UI',sans-serif; }
.leaflet-control-attribution { background:rgba(20,23,25,.78)!important; color:#9aa3a8!important; font-size:10px!important; }
.leaflet-control-attribution a { color:#72bfe1!important; }
.vehicle-shell, .home-shell { background:transparent; border:none; }
.vehicle-tooltip {
  background:rgba(16,20,22,.92); color:#edf2f4; border:1px solid #465055;
  box-shadow:none; font-size:11px; font-weight:700; padding:4px 7px;
}
.vehicle-tooltip:before { border-top-color:#465055!important; }
.waiting {
  position:absolute; z-index:900; left:50%; top:14px; transform:translateX(-50%);
  padding:7px 12px; border-radius:4px; color:#c5d0d5; background:rgba(25,29,31,.88);
  border:1px solid #3a4246; font-size:12px; letter-spacing:.2px; pointer-events:none;
}
</style>
</head>
<body>
<div id="map"></div>
<div id="waiting" class="waiting">Waiting for vehicle GPS</div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" crossorigin=""></script>
<script>
const map = L.map('map', { zoomControl:false, preferCanvas:true, attributionControl:true }).setView([24.7136,46.6753], 6);
const satelliteLayer = L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  {maxZoom:20, attribution:'Imagery &copy; Esri, Vantor, Earthstar Geographics, and the GIS User Community'}
);
const labelsLayer = L.tileLayer(
  'https://services.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}',
  {maxZoom:20, attribution:'Reference &copy; Esri'}
);
satelliteLayer.addTo(map);
labelsLayer.addTo(map);
let activeBaseMap = 'hybrid';

function setBaseMap(name) {
  if (!map.hasLayer(satelliteLayer)) satelliteLayer.addTo(map);
  if (name === 'satellite') {
    if (map.hasLayer(labelsLayer)) map.removeLayer(labelsLayer);
    activeBaseMap = 'satellite';
  } else {
    if (!map.hasLayer(labelsLayer)) labelsLayer.addTo(map);
    activeBaseMap = 'hybrid';
  }
}

const palette = ['#37a8db','#35c6b0','#f5a623','#9d7be8','#59aa29','#e36b6b','#72bfe1','#d0b35a'];
const vehicles = {};
let selectedVehicleKey = null;
let firstPosition = true;

function makeVehicleSvg(v) {
  const selected = v.key === selectedVehicleKey;
  const edge = selected ? '#ffffff' : '#c5d0d5';
  const glow = selected ? 'filter:drop-shadow(0 0 7px rgba(55,168,219,.95));' : 'filter:drop-shadow(0 2px 4px rgba(0,0,0,.8));';
  const heading = Number(v.heading || 0);
  const isFixed = String(v.type || '').toUpperCase().includes('FIXED');
  let body = '';
  if (isFixed) {
    body = `<path d="M23 3 L28 19 L42 25 L42 29 L27 27 L27 39 L33 43 L33 46 L23 43 L13 46 L13 43 L19 39 L19 27 L4 29 L4 25 L18 19 Z" fill="${v.color}" stroke="${edge}" stroke-width="1.7"/>`;
  } else {
    body = `
      <circle cx="10" cy="10" r="6.2" fill="#111416" stroke="${edge}" stroke-width="1.8"/>
      <circle cx="36" cy="10" r="6.2" fill="#111416" stroke="${edge}" stroke-width="1.8"/>
      <circle cx="10" cy="36" r="6.2" fill="#111416" stroke="${edge}" stroke-width="1.8"/>
      <circle cx="36" cy="36" r="6.2" fill="#111416" stroke="${edge}" stroke-width="1.8"/>
      <path d="M14 14 L32 32 M32 14 L14 32" stroke="${v.color}" stroke-width="3.4" stroke-linecap="round"/>
      <rect x="17" y="18" width="12" height="10" rx="3" fill="#202426" stroke="${edge}" stroke-width="1.6"/>
      <path d="M23 2 L18.5 12 L27.5 12 Z" fill="${selected ? '#ffffff' : '#f5a623'}"/>`;
  }
  return `<div style="width:46px;height:46px;transform:rotate(${heading}deg);transform-origin:23px 23px;${glow}">
    <svg width="46" height="46" viewBox="0 0 46 46" xmlns="http://www.w3.org/2000/svg">${body}</svg>
  </div>`;
}

function makeVehicleIcon(v) {
  return L.divIcon({className:'vehicle-shell', html:makeVehicleSvg(v), iconSize:[46,46], iconAnchor:[23,23]});
}

function homeIcon(color) {
  return L.divIcon({className:'home-shell', html:`
    <svg width="30" height="30" viewBox="0 0 30 30" xmlns="http://www.w3.org/2000/svg">
      <circle cx="15" cy="15" r="12" fill="#202426" stroke="${color}" stroke-width="2"/>
      <path d="M8 15 L15 9 L22 15 V22 H17 V17 H13 V22 H8 Z" fill="${color}"/>
    </svg>`, iconSize:[30,30], iconAnchor:[15,15]});
}

function ensureVehicle(key, label, type) {
  if (!vehicles[key]) {
    const color = palette[Object.keys(vehicles).length % palette.length];
    vehicles[key] = {key:key,label:label,type:type,color:color,heading:0,marker:null,home:null,track:[],line:L.polyline([], {color:color,weight:2.4,opacity:.78}).addTo(map)};
  }
  const v = vehicles[key];
  v.label = label || v.label;
  v.type = type || v.type;
  return v;
}

function refreshVehicleAppearance(v) {
  if (v.marker) {
    v.marker.setIcon(makeVehicleIcon(v));
    v.marker.setZIndexOffset(v.key===selectedVehicleKey ? 1200 : 900);
    v.marker.unbindTooltip();
    v.marker.bindTooltip(v.label, {direction:'top', offset:[0,-19], className:'vehicle-tooltip'});
  }
  if (v.line) {
    v.line.setStyle({weight:v.key===selectedVehicleKey ? 4.0 : 2.4, opacity:v.key===selectedVehicleKey ? .95 : .68});
  }
}

function setVehicleLocation(key, label, type, lat, lon, heading, selected, follow) {
  const v = ensureVehicle(key,label,type);
  v.heading = Number(heading || 0);
  if (selected) selectedVehicleKey = key;
  const pos = [lat,lon];
  document.getElementById('waiting').style.display='none';
  if (!v.marker) {
    v.marker = L.marker(pos,{icon:makeVehicleIcon(v),zIndexOffset:selected?1200:900}).addTo(map);
    v.marker.bindTooltip(v.label,{direction:'top',offset:[0,-19],className:'vehicle-tooltip'});
  } else {
    v.marker.setLatLng(pos);
  }
  const last = v.track.length ? v.track[v.track.length-1] : null;
  if (!last || Math.abs(last[0]-lat)>0.000001 || Math.abs(last[1]-lon)>0.000001) {
    v.track.push(pos);
    if (v.track.length > 1500) v.track.shift();
    v.line.setLatLngs(v.track);
  }
  Object.values(vehicles).forEach(refreshVehicleAppearance);
  if (firstPosition) { map.setView(pos,17); firstPosition=false; }
  if (selected && follow) map.panTo(pos,{animate:true,duration:.25});
}

function setVehicleHome(key, label, type, lat, lon) {
  const v = ensureVehicle(key,label,type);
  const pos=[lat,lon];
  if(!v.home) {
    v.home=L.marker(pos,{icon:homeIcon(v.color),zIndexOffset:700}).addTo(map);
    v.home.bindTooltip(`${v.label} HOME`,{direction:'top',className:'vehicle-tooltip'});
  } else {
    v.home.setLatLng(pos);
  }
}

function setSelectedVehicle(key) {
  selectedVehicleKey = key || null;
  Object.values(vehicles).forEach(refreshVehicleAppearance);
}

function centerSelected() {
  const v = selectedVehicleKey ? vehicles[selectedVehicleKey] : null;
  if (v && v.marker) map.setView(v.marker.getLatLng(),Math.max(map.getZoom(),18));
}
function zoomIn() { map.zoomIn(); }
function zoomOut() { map.zoomOut(); }
function clearTrack() {
  const v = selectedVehicleKey ? vehicles[selectedVehicleKey] : null;
  if (v) { v.track=[]; v.line.setLatLngs([]); }
}
function removeVehicle(key) {
  const v=vehicles[key];
  if(!v) return;
  if(v.marker) map.removeLayer(v.marker);
  if(v.home) map.removeLayer(v.home);
  if(v.line) map.removeLayer(v.line);
  delete vehicles[key];
  if(selectedVehicleKey===key) selectedVehicleKey=null;
}
setTimeout(()=>map.invalidateSize(),250);
window.addEventListener('resize',()=>map.invalidateSize());
</script>
</body>
</html>
"""


class MapView(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self._loaded = False
        self._follow = True
        self._selected_key: str | None = None
        self._latest_positions: dict[str, tuple[float, float, float, str, str]] = {}
        self._latest_homes: dict[str, tuple[float, float, str, str]] = {}

        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(0)

        if WEBENGINE_AVAILABLE and QWebEngineView is not None:
            self.web = QWebEngineView()
            self.web.page().profile().setHttpUserAgent(
                "GroundControlStation/2.0 (PySide6 QWebEngine; Multi-Vehicle MAVLink GCS)"
            )
            self.web.setContextMenuPolicy(Qt.ContextMenuPolicy.NoContextMenu)
            self.web.loadFinished.connect(self._on_loaded)
            self.web.setHtml(MAP_HTML, QUrl("https://unpkg.com/"))
            grid.addWidget(self.web, 0, 0)
        else:
            self.web = None
            fallback = QLabel("Map unavailable - install PySide6 with Qt WebEngine")
            fallback.setAlignment(Qt.AlignmentFlag.AlignCenter)
            fallback.setObjectName("mapFallback")
            grid.addWidget(fallback, 0, 0)

        self.controls = QFrame()
        self.controls.setObjectName("mapControls")
        controls_layout = QVBoxLayout(self.controls)
        controls_layout.setContentsMargins(5, 5, 5, 5)
        controls_layout.setSpacing(4)

        self.follow_button = QPushButton("FOLLOW")
        self.follow_button.setObjectName("mapTextButton")
        self.follow_button.setCheckable(True)
        self.follow_button.setChecked(True)
        self.follow_button.clicked.connect(self._toggle_follow)

        self.map_style_button = QPushButton("SAT + LABELS")
        self.map_style_button.setObjectName("mapTextButton")
        self.map_style_button.setToolTip("Toggle labels over the satellite imagery")
        self.map_style_button.clicked.connect(self._toggle_map_style)
        self._map_style = "hybrid"

        center = QPushButton()
        center.setIcon(make_icon("center"))
        center.setToolTip("Center on selected aircraft")
        center.clicked.connect(self.center_on_drone)

        zoom_in = QPushButton()
        zoom_in.setIcon(make_icon("plus"))
        zoom_in.setToolTip("Zoom in")
        zoom_in.clicked.connect(lambda: self._js("zoomIn();"))

        zoom_out = QPushButton()
        zoom_out.setIcon(make_icon("minus"))
        zoom_out.setToolTip("Zoom out")
        zoom_out.clicked.connect(lambda: self._js("zoomOut();"))

        clear_track = QPushButton()
        clear_track.setIcon(make_icon("track"))
        clear_track.setToolTip("Clear selected aircraft flight track")
        clear_track.clicked.connect(lambda: self._js("clearTrack();"))

        for button in (center, zoom_in, zoom_out, clear_track):
            button.setObjectName("mapIconButton")
            button.setFixedSize(34, 34)

        controls_layout.addWidget(self.follow_button)
        controls_layout.addWidget(self.map_style_button)
        controls_layout.addWidget(center)
        controls_layout.addWidget(zoom_in)
        controls_layout.addWidget(zoom_out)
        controls_layout.addWidget(clear_track)
        controls_layout.addStretch(1)

        grid.addWidget(self.controls, 0, 0, Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignRight)

        self.position_strip = QLabel("NO VEHICLE SELECTED")
        self.position_strip.setObjectName("mapPositionStrip")
        self.position_strip.setMinimumWidth(300)
        grid.addWidget(self.position_strip, 0, 0, Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignLeft)

    def _toggle_map_style(self) -> None:
        if self._map_style == "hybrid":
            self._map_style = "satellite"
            self.map_style_button.setText("SATELLITE")
        else:
            self._map_style = "hybrid"
            self.map_style_button.setText("SAT + LABELS")
        self._js(f"setBaseMap('{self._map_style}');")

    def _toggle_follow(self, checked: bool) -> None:
        self._follow = checked
        self.follow_button.setText("FOLLOW" if checked else "FREE")
        if checked:
            self.center_on_drone()

    def _on_loaded(self, ok: bool) -> None:
        self._loaded = ok
        if not ok:
            self.position_strip.setText("Map failed to load")
            return
        for key, (lat, lon, label, vehicle_type) in self._latest_homes.items():
            self.update_vehicle_home(key, lat, lon, label, vehicle_type)
        for key, (lat, lon, hdg, label, vehicle_type) in self._latest_positions.items():
            self.update_vehicle_position(key, lat, lon, hdg, label, vehicle_type, key == self._selected_key)
        if self._selected_key:
            self.set_selected_vehicle(self._selected_key)

    def _js(self, script: str) -> None:
        if self.web is not None and self._loaded:
            self.web.page().runJavaScript(script)

    def update_vehicle_position(
        self,
        key: str,
        lat: float,
        lon: float,
        heading: float | None,
        label: str,
        vehicle_type: str,
        selected: bool = False,
    ) -> None:
        hdg = 0.0 if heading is None else float(heading) % 360.0
        self._latest_positions[key] = (lat, lon, hdg, label, vehicle_type)
        if selected:
            self._selected_key = key
            self.position_strip.setText(f"{label}   {lat:.7f}  {lon:.7f}")
        self._js(
            "setVehicleLocation(" + ",".join([
                json.dumps(key), json.dumps(label), json.dumps(vehicle_type),
                f"{lat:.8f}", f"{lon:.8f}", f"{hdg:.2f}",
                str(bool(selected)).lower(), str(bool(self._follow and selected)).lower(),
            ]) + ");"
        )

    def update_vehicle_home(self, key: str, lat: float, lon: float, label: str, vehicle_type: str) -> None:
        self._latest_homes[key] = (lat, lon, label, vehicle_type)
        self._js(
            "setVehicleHome(" + ",".join([
                json.dumps(key), json.dumps(label), json.dumps(vehicle_type),
                f"{lat:.8f}", f"{lon:.8f}",
            ]) + ");"
        )

    def set_selected_vehicle(self, key: str | None) -> None:
        self._selected_key = key
        self._js(f"setSelectedVehicle({json.dumps(key) if key else 'null'});")
        if key and key in self._latest_positions:
            lat, lon, _hdg, label, _type = self._latest_positions[key]
            self.position_strip.setText(f"{label}   {lat:.7f}  {lon:.7f}")
        elif key:
            self.position_strip.setText("Selected vehicle waiting for GPS")
        else:
            self.position_strip.setText("NO VEHICLE SELECTED")

    def set_gps_waiting(self, text: str) -> None:
        if self._selected_key and self._selected_key not in self._latest_positions:
            self.position_strip.setText(text)

    def center_on_drone(self) -> None:
        self._js("centerSelected();")

    def remove_vehicle(self, key: str) -> None:
        self._latest_positions.pop(key, None)
        self._latest_homes.pop(key, None)
        if self._selected_key == key:
            self._selected_key = None
        self._js(f"removeVehicle({json.dumps(key)});")


class FlightInfoPanel(QWidget):
    def __init__(self) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(5)

        self.state_row = MetricRow("State")
        self.mode_row = MetricRow("Mode")
        self.gps_row = MetricRow("GPS")
        self.sats_row = MetricRow("Satellites")
        self.lat_row = MetricRow("Latitude")
        self.lon_row = MetricRow("Longitude")
        self.battery_row = MetricRow("Battery", unit="V")
        self.altitude_row = MetricRow("Altitude", unit="m")
        self.airspeed_row = MetricRow("Airspeed", unit="m/s")

        for row in (
            self.state_row,
            self.mode_row,
            self.gps_row,
            self.sats_row,
            self.lat_row,
            self.lon_row,
            self.battery_row,
            self.altitude_row,
            self.airspeed_row,
        ):
            layout.addWidget(row)
        layout.addStretch(1)

    def update_state(self, s: TelemetryState) -> None:
        self.state_row.set_value("ARMED" if s.armed else "DISARMED")
        self.mode_row.set_value(s.mode)
        self.gps_row.set_value(GPS_FIX_NAMES.get(s.gps_fix, "--"))
        self.sats_row.set_value("--" if s.satellites is None else str(s.satellites))
        self.lat_row.set_value("--" if s.latitude is None else f"{s.latitude:.7f}")
        self.lon_row.set_value("--" if s.longitude is None else f"{s.longitude:.7f}")
        self.battery_row.set_value("--" if s.battery_voltage_v is None else f"{s.battery_voltage_v:.2f}")
        self.altitude_row.set_value("--" if s.relative_altitude_m is None else f"{s.relative_altitude_m:.1f}")
        self.airspeed_row.set_value("--" if s.airspeed_mps is None else f"{s.airspeed_mps:.1f}")


class InstrumentPanel(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self.horizon = ArtificialHorizon()
        self.compass = MiniCompass()

        # Keep instruments visually secondary to the map.
        self.horizon.setFixedSize(150, 112)
        self.compass.setFixedSize(92, 92)

        self.altitude = MetricRow("Altitude", unit="m")
        self.latitude = MetricRow("Latitude")
        self.longitude = MetricRow("Longitude")
        self.satellites = MetricRow("Satellites")
        self.home_distance = MetricRow("Home distance", unit="m")
        self.airspeed = MetricRow("Air speed", unit="m/s")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(7)
        layout.addWidget(self.horizon, 0, Qt.AlignmentFlag.AlignHCenter)
        layout.addWidget(self.compass, 0, Qt.AlignmentFlag.AlignHCenter)

        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.HLine)
        separator.setObjectName("separator")
        layout.addWidget(separator)

        for row in (
            self.altitude,
            self.latitude,
            self.longitude,
            self.satellites,
            self.home_distance,
            self.airspeed,
        ):
            layout.addWidget(row)

        layout.addStretch(1)

    def update_state(self, s: TelemetryState) -> None:
        self.horizon.set_attitude(s.roll_deg, s.pitch_deg)
        self.compass.set_heading(s.heading_deg)
        self.altitude.set_value("--" if s.relative_altitude_m is None else f"{s.relative_altitude_m:.1f}")
        self.latitude.set_value("--" if s.latitude is None else f"{s.latitude:.6f}")
        self.longitude.set_value("--" if s.longitude is None else f"{s.longitude:.6f}")
        self.satellites.set_value("--" if s.satellites is None else str(s.satellites))
        home_m = distance_m(s.latitude, s.longitude, s.home_latitude, s.home_longitude)
        if home_m is None:
            self.home_distance.set_value("--", "m")
        elif home_m >= 1000.0:
            self.home_distance.set_value(f"{home_m / 1000.0:.2f}", "km")
        else:
            self.home_distance.set_value(f"{home_m:.0f}", "m")
        self.airspeed.set_value("--" if s.airspeed_mps is None else f"{s.airspeed_mps:.1f}")


class VideoFeedCard(QFrame):
    """One independently selectable Windows UVC video input."""

    def __init__(self, owner: "MultiVideoPanel", feed_number: int) -> None:
        super().__init__()
        self.owner = owner
        self.feed_number = feed_number
        self.setObjectName("videoFeedCard")
        self.setMinimumWidth(390)
        self.setMaximumWidth(560)

        self._devices: list[Any] = []
        self._camera: Any = None
        self._session: Any = None
        self._active_device_id: bytes | None = None
        self._auto_started = False

        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(6)

        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        header.setSpacing(5)

        self.title = QLabel(f"FEED {feed_number}")
        self.title.setObjectName("videoFeedTitle")
        header.addWidget(self.title)

        self.device_combo = QComboBox()
        self.device_combo.setObjectName("videoDeviceCombo")
        self.device_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.device_combo.currentIndexChanged.connect(self._selection_changed)
        header.addWidget(self.device_combo, 1)

        self.refresh_button = QPushButton("↻")
        self.refresh_button.setObjectName("videoMiniButton")
        self.refresh_button.setToolTip("Refresh Windows video inputs")
        self.refresh_button.setFixedWidth(34)
        self.refresh_button.clicked.connect(self.owner.refresh_all_devices)
        header.addWidget(self.refresh_button)

        self.start_button = QPushButton("START")
        self.start_button.setObjectName("videoStartButton")
        self.start_button.setFixedWidth(62)
        self.start_button.clicked.connect(self.toggle_video)
        header.addWidget(self.start_button)

        self.remove_button = QPushButton("×")
        self.remove_button.setObjectName("videoRemoveButton")
        self.remove_button.setToolTip("Remove this video feed window")
        self.remove_button.setFixedWidth(32)
        self.remove_button.clicked.connect(lambda: self.owner.remove_feed(self))
        header.addWidget(self.remove_button)
        outer.addLayout(header)

        if MULTIMEDIA_AVAILABLE and QVideoWidget is not None:
            self.video_widget = QVideoWidget()
            self.video_widget.setObjectName("fpvVideo")
            self.video_widget.setMinimumHeight(175)
            self.video_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
            self.video_widget.setAspectRatioMode(Qt.AspectRatioMode.KeepAspectRatio)
            outer.addWidget(self.video_widget, 1)
        else:
            self.video_widget = None
            unavailable = QLabel("Qt Multimedia is unavailable in this PySide6 installation.")
            unavailable.setWordWrap(True)
            unavailable.setAlignment(Qt.AlignmentFlag.AlignCenter)
            unavailable.setMinimumHeight(165)
            unavailable.setObjectName("videoUnavailable")
            outer.addWidget(unavailable, 1)
            self.device_combo.setEnabled(False)
            self.refresh_button.setEnabled(False)
            self.start_button.setEnabled(False)

        self.status = QLabel("Select a video source")
        self.status.setObjectName("videoStatus")
        self.status.setWordWrap(True)
        outer.addWidget(self.status)

    @staticmethod
    def _device_id(device: Any) -> bytes:
        try:
            return bytes(device.id())
        except Exception:
            return str(device.description()).encode("utf-8", errors="ignore")

    @property
    def active_device_id(self) -> bytes | None:
        return self._active_device_id

    def set_feed_number(self, number: int) -> None:
        self.feed_number = number
        self.title.setText(f"FEED {number}")

    def set_remove_enabled(self, enabled: bool) -> None:
        self.remove_button.setEnabled(enabled)

    def refresh_devices(self, devices: list[Any]) -> None:
        previous_id: bytes | None = None
        current = self.device_combo.currentIndex()
        if 0 <= current < len(self._devices):
            previous_id = self._device_id(self._devices[current])

        self._devices = list(devices)
        self.device_combo.blockSignals(True)
        self.device_combo.clear()

        preferred_index = -1
        for index, device in enumerate(self._devices):
            try:
                name = str(device.description())
            except Exception:
                name = f"Camera {index + 1}"
            self.device_combo.addItem(name)
            lower = name.lower()
            if preferred_index < 0 and any(
                token in lower for token in ("skydroid", "uvc", "fpv", "receiver")
            ):
                preferred_index = index

        selected = -1
        if previous_id is not None:
            for index, device in enumerate(self._devices):
                if self._device_id(device) == previous_id:
                    selected = index
                    break
        if selected < 0 and preferred_index >= 0:
            selected = preferred_index
        if selected < 0 and self._devices:
            selected = 0

        if selected >= 0:
            self.device_combo.setCurrentIndex(selected)
        self.device_combo.blockSignals(False)

        if not self._devices:
            self.stop_video(clear_status=False)
            self.status.setText("No Windows UVC/video input detected")
            self.start_button.setEnabled(False)
            return

        self.start_button.setEnabled(True)
        try:
            selected_name = str(self._devices[selected].description()) if selected >= 0 else "video source"
        except Exception:
            selected_name = "video source"
        if self._camera is None:
            self.status.setText(f"Ready · {selected_name}")

        # Preserve the original convenient behavior for the first feed only.
        if self.feed_number == 1 and not self._auto_started:
            auto_ok = len(self._devices) == 1
            if selected >= 0:
                try:
                    lower = str(self._devices[selected].description()).lower()
                    auto_ok = auto_ok or any(
                        token in lower for token in ("skydroid", "uvc", "fpv", "receiver")
                    )
                except Exception:
                    pass
            if auto_ok:
                self._auto_started = True
                QTimer.singleShot(350, self.start_video)

    def _selection_changed(self, _index: int) -> None:
        if self._camera is not None:
            self.start_video()

    def _best_format(self, device: Any) -> Any | None:
        """Prefer a low-latency format near 640x480 / 30 fps when available."""
        try:
            formats = list(device.videoFormats())
        except Exception:
            return None
        if not formats:
            return None

        def score(fmt: Any) -> tuple[float, float, float]:
            try:
                size = fmt.resolution()
                width = float(size.width())
                height = float(size.height())
                fps = float(fmt.maxFrameRate())
            except Exception:
                return (1e12, 1e12, 1e12)
            resolution_error = abs(width - 640.0) + abs(height - 480.0)
            fps_error = abs(fps - 30.0)
            low_fps_penalty = 1000.0 if fps < 25.0 else 0.0
            return (resolution_error + low_fps_penalty, fps_error, -fps)

        return min(formats, key=score)

    def start_video(self) -> None:
        if not MULTIMEDIA_AVAILABLE or QCamera is None or QMediaCaptureSession is None:
            return

        index = self.device_combo.currentIndex()
        if index < 0 or index >= len(self._devices):
            self.status.setText("Select a video source")
            return

        device = self._devices[index]
        device_id = self._device_id(device)
        available, reason = self.owner.can_activate_device(self, device_id)
        if not available:
            self.status.setText(reason)
            return

        self.stop_video(clear_status=False)

        try:
            camera = QCamera(device, self)
            chosen_format = self._best_format(device)
            if chosen_format is not None:
                try:
                    camera.setCameraFormat(chosen_format)
                except Exception:
                    pass

            session = QMediaCaptureSession(self)
            session.setCamera(camera)
            if self.video_widget is not None:
                session.setVideoOutput(self.video_widget)

            try:
                camera.errorOccurred.connect(self._camera_error)
            except Exception:
                pass
            try:
                camera.activeChanged.connect(self._camera_active_changed)
            except Exception:
                pass

            self._camera = camera
            self._session = session
            self._active_device_id = device_id
            camera.start()
            self.start_button.setText("STOP")

            name = str(device.description())
            if chosen_format is not None:
                try:
                    size = chosen_format.resolution()
                    fps = chosen_format.maxFrameRate()
                    self.status.setText(
                        f"LIVE · {name} · {size.width()}×{size.height()} @ {fps:.0f} fps"
                    )
                except Exception:
                    self.status.setText(f"LIVE · {name}")
            else:
                self.status.setText(f"LIVE · {name}")
        except Exception as error:
            self._camera = None
            self._session = None
            self._active_device_id = None
            self.start_button.setText("START")
            self.status.setText(f"Video start failed: {error}")

    def stop_video(self, clear_status: bool = True) -> None:
        camera = self._camera
        self._camera = None
        self._active_device_id = None
        if camera is not None:
            try:
                camera.stop()
            except Exception:
                pass
            try:
                camera.deleteLater()
            except Exception:
                pass

        session = self._session
        self._session = None
        if session is not None:
            try:
                session.setCamera(None)
            except Exception:
                pass
            try:
                session.deleteLater()
            except Exception:
                pass

        self.start_button.setText("START")
        if clear_status and self._devices:
            index = self.device_combo.currentIndex()
            try:
                name = str(self._devices[index].description())
            except Exception:
                name = "video source"
            self.status.setText(f"Stopped · {name}")

    def toggle_video(self) -> None:
        if self._camera is None:
            self.start_video()
        else:
            self.stop_video()

    def _camera_error(self, _error: Any, error_string: str) -> None:
        self._active_device_id = None
        self.status.setText(f"VIDEO ERROR · {error_string}")
        self.start_button.setText("START")

    def _camera_active_changed(self, active: bool) -> None:
        if not active and self._camera is not None:
            if not self.status.text().startswith("VIDEO ERROR"):
                self.status.setText("Video source stopped")

    def shutdown(self) -> None:
        self.stop_video(clear_status=False)


class MultiVideoPanel(QFrame):
    """Bottom video workspace with dynamically addable UVC feed cards."""

    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("videoWorkspace")
        self.cards: list[VideoFeedCard] = []
        self._media_devices: Any = None

        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 6, 8, 8)
        outer.setSpacing(6)

        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        header.setSpacing(7)

        title = QLabel("FPV VIDEO FEEDS")
        title.setObjectName("videoWorkspaceTitle")
        header.addWidget(title)

        self.source_count = QLabel("0 sources")
        self.source_count.setObjectName("videoWorkspaceInfo")
        header.addWidget(self.source_count)
        header.addStretch(1)

        self.refresh_button = QPushButton("REFRESH SOURCES")
        self.refresh_button.setObjectName("compactButton")
        self.refresh_button.clicked.connect(self.refresh_all_devices)
        header.addWidget(self.refresh_button)

        self.add_button = QPushButton("+ ADD FEED")
        self.add_button.setObjectName("primaryButton")
        self.add_button.clicked.connect(self.add_feed)
        header.addWidget(self.add_button)
        outer.addLayout(header)

        self.scroll = QScrollArea()
        self.scroll.setObjectName("videoScrollArea")
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        self.container = QWidget()
        self.container.setObjectName("videoFeedContainer")
        self.feed_layout = QHBoxLayout(self.container)
        self.feed_layout.setContentsMargins(0, 0, 0, 0)
        self.feed_layout.setSpacing(8)
        self.feed_layout.addStretch(1)
        self.scroll.setWidget(self.container)
        outer.addWidget(self.scroll, 1)

        if MULTIMEDIA_AVAILABLE and QMediaDevices is not None:
            try:
                self._media_devices = QMediaDevices(self)
                self._media_devices.videoInputsChanged.connect(self.refresh_all_devices)
            except Exception:
                self._media_devices = None

        self.add_feed()
        QTimer.singleShot(250, self.refresh_all_devices)

    def video_devices(self) -> list[Any]:
        if not MULTIMEDIA_AVAILABLE or QMediaDevices is None:
            return []
        try:
            return list(QMediaDevices.videoInputs())
        except Exception:
            return []

    def refresh_all_devices(self) -> None:
        devices = self.video_devices()
        self.source_count.setText(f"{len(devices)} source{'s' if len(devices) != 1 else ''}")
        for card in list(self.cards):
            card.refresh_devices(devices)

    def add_feed(self) -> None:
        card = VideoFeedCard(self, len(self.cards) + 1)
        self.cards.append(card)
        self.feed_layout.insertWidget(max(0, self.feed_layout.count() - 1), card)
        self._renumber_cards()
        card.refresh_devices(self.video_devices())
        QTimer.singleShot(0, lambda: self.scroll.ensureWidgetVisible(card))

    def remove_feed(self, card: VideoFeedCard) -> None:
        if card not in self.cards:
            return
        if len(self.cards) <= 1:
            card.status.setText("At least one feed window is kept open")
            return
        card.shutdown()
        self.cards.remove(card)
        self.feed_layout.removeWidget(card)
        card.deleteLater()
        self._renumber_cards()

    def _renumber_cards(self) -> None:
        allow_remove = len(self.cards) > 1
        for index, card in enumerate(self.cards, start=1):
            card.set_feed_number(index)
            card.set_remove_enabled(allow_remove)

    def can_activate_device(self, requester: VideoFeedCard, device_id: bytes) -> tuple[bool, str]:
        for card in self.cards:
            if card is requester:
                continue
            if card.active_device_id == device_id:
                return False, f"This video source is already active in FEED {card.feed_number}"
        return True, ""

    def shutdown(self) -> None:
        for card in list(self.cards):
            card.shutdown()


class ConnectionRow(QFrame):
    toggle_requested = Signal(str)
    remove_requested = Signal(str)

    def __init__(self, connection_id: str, name: str, endpoint: str) -> None:
        super().__init__()
        self.connection_id = connection_id
        self.endpoint = endpoint
        self.friendly_name = name
        self.state = "disconnected"
        self.setObjectName("connectionRow")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 7, 8, 7)
        layout.setSpacing(4)

        top = QHBoxLayout()
        self.name_label = QLabel(name)
        self.name_label.setObjectName("connectionName")
        self.status = QLabel("OFF")
        self.status.setObjectName("connectionStatus")
        top.addWidget(self.name_label)
        top.addStretch(1)
        top.addWidget(self.status)
        layout.addLayout(top)

        self.identity = QLabel("SYS -- · UNKNOWN")
        self.identity.setObjectName("connectionIdentity")
        layout.addWidget(self.identity)

        self.endpoint_label = QLabel(endpoint)
        self.endpoint_label.setObjectName("connectionEndpoint")
        self.endpoint_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        layout.addWidget(self.endpoint_label)

        buttons = QHBoxLayout()
        self.toggle_button = QPushButton("CONNECT")
        self.toggle_button.setObjectName("compactButton")
        self.remove_button = QPushButton("REMOVE")
        self.remove_button.setObjectName("compactButton")
        self.toggle_button.clicked.connect(lambda: self.toggle_requested.emit(self.connection_id))
        self.remove_button.clicked.connect(lambda: self.remove_requested.emit(self.connection_id))
        buttons.addWidget(self.toggle_button)
        buttons.addWidget(self.remove_button)
        layout.addLayout(buttons)

    def set_state(self, state: str, text: str) -> None:
        self.state = state
        label_map = {
            "connected": "LIVE",
            "connecting": "WAIT",
            "warning": "STALE",
            "error": "ERROR",
            "disconnected": "OFF",
        }
        self.status.setText(label_map.get(state, state.upper()))
        self.status.setToolTip(text)
        self.status.setProperty("linkState", state)
        self.status.style().unpolish(self.status)
        self.status.style().polish(self.status)
        active = state in {"connected", "connecting", "warning"}
        self.toggle_button.setText("DISCONNECT" if active else "CONNECT")

    def set_identity(self, system_id: int, vehicle_type: str) -> None:
        sys_text = "--" if system_id <= 0 else str(system_id)
        self.identity.setText(f"SYS {sys_text} · {vehicle_type or 'UNKNOWN'}")


class LinkPanel(QWidget):
    add_requested = Signal(str, str)
    reconnect_requested = Signal(str)
    disconnect_requested = Signal(str)
    remove_requested = Signal(str)
    connect_all_requested = Signal()
    disconnect_all_requested = Signal()

    def __init__(self) -> None:
        super().__init__()
        self.rows: dict[str, ConnectionRow] = {}
        self._suggested_number = 2

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(9)

        header = QLabel("VEHICLE CONNECTIONS")
        header.setObjectName("messagesTitle")
        layout.addWidget(header)

        self.name_input = QLineEdit("Vehicle 2")
        self.name_input.setPlaceholderText("Vehicle name, e.g. Quad 2")
        self.endpoint = QLineEdit("udpin:127.0.0.1:14552")
        self.endpoint.setPlaceholderText("udpin:127.0.0.1:14552")
        layout.addWidget(self.name_input)
        layout.addWidget(self.endpoint)

        add = QPushButton("+ ADD & CONNECT")
        add.setObjectName("primaryButton")
        add.clicked.connect(self._add_clicked)
        layout.addWidget(add)

        bulk = QHBoxLayout()
        connect_all = QPushButton("CONNECT ALL")
        disconnect_all = QPushButton("DISCONNECT ALL")
        connect_all.setObjectName("compactButton")
        disconnect_all.setObjectName("compactButton")
        connect_all.clicked.connect(self.connect_all_requested.emit)
        disconnect_all.clicked.connect(self.disconnect_all_requested.emit)
        bulk.addWidget(connect_all)
        bulk.addWidget(disconnect_all)
        layout.addLayout(bulk)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.row_container = QWidget()
        self.row_layout = QVBoxLayout(self.row_container)
        self.row_layout.setContentsMargins(0, 0, 0, 0)
        self.row_layout.setSpacing(7)
        self.row_layout.addStretch(1)
        self.scroll.setWidget(self.row_container)
        layout.addWidget(self.scroll, 1)

    def _add_clicked(self) -> None:
        name = self.name_input.text().strip() or f"Vehicle {self._suggested_number}"
        endpoint = self.endpoint.text().strip()
        if endpoint:
            self.add_requested.emit(name, endpoint)

    def add_connection_row(self, connection_id: str, name: str, endpoint: str) -> None:
        row = ConnectionRow(connection_id, name, endpoint)
        row.toggle_requested.connect(self._toggle_row)
        row.remove_requested.connect(self.remove_requested.emit)
        self.rows[connection_id] = row
        self.row_layout.insertWidget(max(0, self.row_layout.count() - 1), row)
        self._advance_defaults(endpoint)

    def _advance_defaults(self, endpoint: str) -> None:
        self._suggested_number = len(self.rows) + 1
        self.name_input.setText(f"Vehicle {self._suggested_number}")
        try:
            prefix, port_text = endpoint.rsplit(":", 1)
            port = int(port_text)
            self.endpoint.setText(f"{prefix}:{port + 1}")
        except Exception:
            pass

    def remove_connection_row(self, connection_id: str) -> None:
        row = self.rows.pop(connection_id, None)
        if row is not None:
            self.row_layout.removeWidget(row)
            row.deleteLater()

    def _toggle_row(self, connection_id: str) -> None:
        row = self.rows.get(connection_id)
        if row is None:
            return
        if row.state in {"connected", "connecting", "warning"}:
            self.disconnect_requested.emit(connection_id)
        else:
            self.reconnect_requested.emit(connection_id)

    def set_connection_state(self, connection_id: str, state: str, text: str) -> None:
        row = self.rows.get(connection_id)
        if row is not None:
            row.set_state(state, text)

    def set_connection_identity(self, connection_id: str, system_id: int, vehicle_type: str) -> None:
        row = self.rows.get(connection_id)
        if row is not None:
            row.set_identity(system_id, vehicle_type)


class ControlPanel(QWidget):
    arm_requested = Signal(bool)
    mode_requested = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self._connected = False
        self._armed = False
        self._mode = "UNKNOWN"
        self.mode_buttons: dict[str, QPushButton] = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(9)

        self.mode_badge = QLabel("UNKNOWN")
        self.mode_badge.setObjectName("modeBadge")
        self.mode_badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self.mode_badge)

        grid = QGridLayout()
        grid.setHorizontalSpacing(7)
        grid.setVerticalSpacing(7)

        self.arm_button = QPushButton("ARM")
        self.disarm_button = QPushButton("DISARM")
        self.arm_button.setProperty("controlButton", True)
        self.disarm_button.setProperty("controlButton", True)
        self.arm_button.clicked.connect(lambda: self.arm_requested.emit(True))
        self.disarm_button.clicked.connect(lambda: self.arm_requested.emit(False))
        grid.addWidget(self.arm_button, 0, 0)
        grid.addWidget(self.disarm_button, 0, 1)

        controls = [
            ("STABILIZE", "STABILIZE", 1, 0, 1, 1),
            ("ALT HOLD", "ALT_HOLD", 1, 1, 1, 1),
            ("LOITER", "LOITER", 2, 0, 1, 1),
            ("GUIDED", "GUIDED", 2, 1, 1, 1),
            ("AUTO", "AUTO", 3, 0, 1, 1),
            ("POSHOLD", "POSHOLD", 3, 1, 1, 1),
            ("BRAKE", "BRAKE", 4, 0, 1, 1),
            ("RTL", "RTL", 4, 1, 1, 1),
            ("LAND", "LAND", 5, 0, 1, 2),
        ]
        for label, mode_name, row, col, row_span, col_span in controls:
            button = QPushButton(label)
            button.setProperty("controlButton", True)
            button.setMinimumHeight(38)
            button.clicked.connect(lambda checked=False, m=mode_name: self.mode_requested.emit(m))
            grid.addWidget(button, row, col, row_span, col_span)
            self.mode_buttons[mode_name] = button
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        layout.addLayout(grid)

        self.command_status = QLabel("Connect to send commands")
        self.command_status.setObjectName("commandStatus")
        self.command_status.setWordWrap(True)
        layout.addWidget(self.command_status)
        layout.addStretch(1)
        self.set_control_state(False, False, "UNKNOWN")

    def set_control_state(self, connected: bool, armed: bool, mode: str) -> None:
        self._connected = connected
        self._armed = armed
        self._mode = mode
        self.mode_badge.setText(mode)

        self.arm_button.setEnabled(connected and not armed)
        self.disarm_button.setEnabled(connected and armed)
        normalized = mode.upper().replace(" ", "_")
        for mode_name, button in self.mode_buttons.items():
            button.setEnabled(connected)
            button.setProperty("modeActive", mode_name == normalized)
            button.style().unpolish(button)
            button.style().polish(button)

    def set_command_status(self, level: str, text: str) -> None:
        self.command_status.setText(text)
        self.command_status.setProperty("commandLevel", level)
        self.command_status.style().unpolish(self.command_status)
        self.command_status.style().polish(self.command_status)

    @staticmethod
    def _mode_category(mode: str) -> str:
        m = mode.upper().replace(" ", "_")
        if m in {"RTL", "QRTL", "RETURN"}:
            return "return"
        if m in {"LAND", "QLAND"}:
            return "land"
        if m in {"LOITER", "POSHOLD", "POSITION", "BRAKE"}:
            return "hold"
        if m in {"GUIDED", "AUTO"}:
            return "guided"
        if m in {"ALT_HOLD", "ALTHOLD"}:
            return "alt"
        if m in {"STABILIZE", "STABILIZED"}:
            return "stabilize"
        return "neutral"


class MessagesPanel(QWidget):
    clear_requested = Signal()

    def __init__(self) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 8)
        layout.setSpacing(5)
        header = QHBoxLayout()
        self.title = QLabel("MESSAGES")
        self.title.setObjectName("messagesTitle")
        clear = QPushButton("CLEAR")
        clear.setObjectName("compactButton")
        clear.setMaximumWidth(70)
        clear.clicked.connect(self.clear_requested.emit)
        header.addWidget(self.title)
        header.addStretch(1)
        header.addWidget(clear)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(110)
        layout.addLayout(header)
        layout.addWidget(self.log)

    def set_context(self, label: str | None) -> None:
        self.title.setText("MESSAGES" if not label else f"MESSAGES · {label}")

    def set_lines(self, lines: list[str]) -> None:
        self.log.setPlainText("\n".join(lines))
        bar = self.log.verticalScrollBar()
        bar.setValue(bar.maximum())

    def append_line(self, line: str) -> None:
        self.log.append(line)

    def clear_view(self) -> None:
        self.log.clear()


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(BRAND_NAME)
        self.setWindowIcon(make_icon("flight", ACCENT, 32))
        self.resize(1500, 900)
        self.setMinimumSize(860, 620)
        self.setDockNestingEnabled(True)

        self.workers: dict[str, TelemetryWorker] = {}
        self.vehicle_states: dict[str, TelemetryState] = {}
        self.connection_meta: dict[str, dict[str, Any]] = {}
        self.vehicle_logs: dict[str, list[str]] = {}
        self._last_map_update: dict[str, float] = {}
        self._connection_counter = 0
        self.selected_vehicle_id: str | None = None
        self.current_state = TelemetryState()
        self._duplicate_sysid_warnings: set[tuple[str, str, int]] = set()

        self.map_view = MapView()
        self.setCentralWidget(self.map_view)

        self.flight_panel = FlightInfoPanel()
        self.control_panel = ControlPanel()
        self.link_panel = LinkPanel()
        self.instrument_panel = InstrumentPanel()
        self.messages_panel = MessagesPanel()
        self.video_workspace = MultiVideoPanel()

        self.flight_dock = self._make_dock(self.flight_panel, "flightDock", 280)
        self.control_dock = self._make_dock(self.control_panel, "controlDock", 310)
        self.link_dock = self._make_dock(self.link_panel, "linkDock", 360)
        self.instrument_dock = self._make_dock(self.instrument_panel, "instrumentDock", 330)
        self.messages_dock = self._make_dock(self.messages_panel, "messagesDock", 330)
        self.video_dock = self._make_dock(self.video_workspace, "videoDock", 650)

        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.flight_dock)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.control_dock)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.link_dock)
        self.tabifyDockWidget(self.flight_dock, self.control_dock)
        self.tabifyDockWidget(self.control_dock, self.link_dock)

        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.instrument_dock)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.messages_dock)
        self.splitDockWidget(self.instrument_dock, self.messages_dock, Qt.Orientation.Vertical)
        self.messages_dock.setAllowedAreas(Qt.DockWidgetArea.RightDockWidgetArea)
        self.messages_dock.setMinimumHeight(180)

        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self.video_dock)
        self.video_dock.setAllowedAreas(Qt.DockWidgetArea.BottomDockWidgetArea)
        self.video_dock.setMinimumHeight(255)
        self.setCorner(Qt.Corner.BottomRightCorner, Qt.DockWidgetArea.RightDockWidgetArea)
        self.setCorner(Qt.Corner.BottomLeftCorner, Qt.DockWidgetArea.LeftDockWidgetArea)

        self.flight_dock.hide()
        self.control_dock.hide()
        self.link_dock.hide()

        self._build_top_bar()
        self._build_nav_rail()
        self.messages_action.setChecked(True)
        self._build_status_bar()
        self._wire_signals()
        self._apply_style()
        self._apply_empty_selected_state()

        # Preserve the old single-vehicle experience: vehicle 1 is created on
        # UDP 14551 automatically. Additional vehicles are added from the left
        # telemetry panel and receive subsequent UDP ports such as 14552.
        QTimer.singleShot(500, lambda: self.add_vehicle_connection("Vehicle 1", DEFAULT_CONNECTION))

    def _make_dock(self, widget: QWidget, name: str, width: int) -> QDockWidget:
        dock = QDockWidget("", self)
        dock.setObjectName(name)
        dock.setWidget(widget)
        dock.setMinimumWidth(width)
        dock.setFeatures(
            QDockWidget.DockWidgetFeature.DockWidgetMovable
            | QDockWidget.DockWidgetFeature.DockWidgetClosable
            | QDockWidget.DockWidgetFeature.DockWidgetFloatable
        )
        blank_title = QWidget()
        blank_title.setFixedHeight(4)
        dock.setTitleBarWidget(blank_title)
        return dock

    def _build_top_bar(self) -> None:
        bar = QToolBar()
        bar.setObjectName("topBar")
        bar.setMovable(False)
        bar.setFloatable(False)
        bar.setIconSize(QPixmap(22, 22).size())
        self.addToolBar(Qt.ToolBarArea.TopToolBarArea, bar)

        bar.addWidget(BrandWidget())
        spacer_small = QWidget()
        spacer_small.setFixedWidth(5)
        bar.addWidget(spacer_small)

        self.link_chip = StatusChip("LINKS", "0/0")
        self.state_chip = StatusChip("STATE", "--")
        self.mode_chip = StatusChip("MODE", "--")
        self.gps_chip = StatusChip("GPS", "--")
        self.battery_chip = StatusChip("BAT", "--")
        for chip in (self.link_chip, self.state_chip, self.mode_chip, self.gps_chip, self.battery_chip):
            bar.addWidget(chip)

        expanding = QWidget()
        expanding.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        bar.addWidget(expanding)

        selector_label = QLabel("VEHICLE")
        selector_label.setObjectName("vehicleSelectorLabel")
        bar.addWidget(selector_label)
        self.vehicle_selector = QComboBox()
        self.vehicle_selector.setObjectName("vehicleSelector")
        self.vehicle_selector.setMinimumWidth(265)
        self.vehicle_selector.setMaximumWidth(360)
        self.vehicle_selector.setToolTip("Select the aircraft whose telemetry, messages and controls are active")
        self.vehicle_selector.currentIndexChanged.connect(self._vehicle_selector_changed)
        bar.addWidget(self.vehicle_selector)

        self.quick_connect = QPushButton("CONNECT ALL")
        self.quick_connect.setObjectName("topConnectButton")
        self.quick_connect.clicked.connect(self._quick_toggle_connection)
        bar.addWidget(self.quick_connect)

    def _build_nav_rail(self) -> None:
        rail = QToolBar()
        rail.setObjectName("navRail")
        rail.setMovable(False)
        rail.setFloatable(False)
        rail.setOrientation(Qt.Orientation.Vertical)
        rail.setIconSize(QPixmap(24, 24).size())
        self.addToolBar(Qt.ToolBarArea.LeftToolBarArea, rail)

        self.flight_action = self._rail_action("flight", "Selected vehicle flight data", self._show_flight)
        self.control_action = self._rail_action("control", "Selected vehicle control", self._show_control)
        self.link_action = self._rail_action("link", "Add / manage vehicle UDP connections", self._show_link)
        self.messages_action = self._rail_action("messages", "Selected vehicle messages", self._toggle_messages)
        center_action = self._rail_action("center", "Center map on selected aircraft", self.map_view.center_on_drone)

        for action in (self.flight_action, self.control_action, self.link_action, self.messages_action):
            action.setCheckable(True)
            rail.addAction(action)
        rail.addSeparator()
        rail.addAction(center_action)

    def _rail_action(self, icon_kind: str, tooltip: str, callback: Callable[[], None]) -> QAction:
        action = QAction(make_icon(icon_kind, "#c8d0d4"), "", self)
        action.setToolTip(tooltip)
        action.triggered.connect(callback)
        return action

    def _build_status_bar(self) -> None:
        bar = QStatusBar()
        bar.setObjectName("statusBar")
        self.setStatusBar(bar)
        self.bottom_link = QLabel("●  NO VEHICLE LINKS")
        self.bottom_link.setObjectName("bottomLink")
        self.bottom_arm = QLabel("NO VEHICLE SELECTED")
        self.bottom_arm.setObjectName("bottomArm")
        bar.addWidget(self.bottom_link)
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        bar.addWidget(spacer, 1)
        bar.addPermanentWidget(self.bottom_arm)

    def _wire_signals(self) -> None:
        self.control_panel.arm_requested.connect(self._handle_arm_request)
        self.control_panel.mode_requested.connect(self._handle_mode_request)
        self.link_panel.add_requested.connect(self.add_vehicle_connection)
        self.link_panel.reconnect_requested.connect(self.connect_vehicle)
        self.link_panel.disconnect_requested.connect(self.disconnect_vehicle)
        self.link_panel.remove_requested.connect(self.remove_vehicle_connection)
        self.link_panel.connect_all_requested.connect(self.connect_all_vehicles)
        self.link_panel.disconnect_all_requested.connect(self.disconnect_all_vehicles)
        self.messages_panel.clear_requested.connect(self._clear_selected_messages)

    def _show_flight(self) -> None:
        self._show_single_left_dock(self.flight_dock, self.flight_action)

    def _show_control(self) -> None:
        self._show_single_left_dock(self.control_dock, self.control_action)

    def _show_link(self) -> None:
        self._show_single_left_dock(self.link_dock, self.link_action)

    def _show_single_left_dock(self, dock: QDockWidget, action: QAction) -> None:
        if dock.isVisible():
            dock.hide()
            action.setChecked(False)
        else:
            for other_dock, other_action in (
                (self.flight_dock, self.flight_action),
                (self.control_dock, self.control_action),
                (self.link_dock, self.link_action),
            ):
                other_dock.hide()
                other_action.setChecked(False)
            dock.show()
            dock.raise_()
            action.setChecked(True)

    def _toggle_messages(self) -> None:
        if self.messages_dock.isVisible():
            self.messages_dock.hide()
            self.messages_action.setChecked(False)
        else:
            self.messages_dock.show()
            self.messages_dock.raise_()
            self.resizeDocks([self.instrument_dock, self.messages_dock], [430, 300], Qt.Orientation.Vertical)
            self.messages_action.setChecked(True)

    def _quick_toggle_connection(self) -> None:
        if any(worker.isRunning() for worker in self.workers.values()):
            self.disconnect_all_vehicles()
        else:
            self.connect_all_vehicles()

    def add_vehicle_connection(self, friendly_name: str, endpoint: str) -> None:
        endpoint = endpoint.strip()
        friendly_name = friendly_name.strip() or "Vehicle"
        if not endpoint:
            return
        if any(str(meta.get("endpoint")) == endpoint for meta in self.connection_meta.values()):
            QMessageBox.warning(self, "Duplicate endpoint", f"{endpoint} is already configured.")
            return

        self._connection_counter += 1
        connection_id = f"vehicle_{self._connection_counter}"
        self.connection_meta[connection_id] = {
            "name": friendly_name,
            "endpoint": endpoint,
            "link_state": "disconnected",
            "link_text": "Disconnected",
        }
        self.vehicle_states[connection_id] = TelemetryState()
        self.vehicle_logs[connection_id] = []
        self._last_map_update[connection_id] = 0.0
        self.link_panel.add_connection_row(connection_id, friendly_name, endpoint)

        self.vehicle_selector.blockSignals(True)
        self.vehicle_selector.addItem(self._vehicle_display_name(connection_id), connection_id)
        self.vehicle_selector.blockSignals(False)
        if self.selected_vehicle_id is None:
            self.select_vehicle(connection_id)
        self.connect_vehicle(connection_id)

    def connect_vehicle(self, connection_id: str) -> None:
        meta = self.connection_meta.get(connection_id)
        if meta is None:
            return
        current = self.workers.get(connection_id)
        if current is not None and current.isRunning():
            return

        worker = TelemetryWorker(str(meta["endpoint"]), self)
        self.workers[connection_id] = worker
        worker.telemetry.connect(lambda data, cid=connection_id: self.update_vehicle_telemetry(cid, data))
        worker.link_status.connect(lambda state, text, cid=connection_id: self.update_vehicle_link_status(cid, state, text))
        worker.log_message.connect(lambda text, cid=connection_id: self._append_vehicle_log(cid, text))
        worker.command_status.connect(lambda level, text, cid=connection_id: self.update_vehicle_command_status(cid, level, text))
        worker.finished.connect(lambda cid=connection_id, w=worker: self._worker_finished(cid, w))
        worker.start()
        self._update_aggregate_link()

    def disconnect_vehicle(self, connection_id: str) -> None:
        worker = self.workers.pop(connection_id, None)
        if worker is not None:
            worker.request_stop()
            worker.wait(2500)
        meta = self.connection_meta.get(connection_id)
        if meta is not None:
            meta["link_state"] = "disconnected"
            meta["link_text"] = "Disconnected"
            self.link_panel.set_connection_state(connection_id, "disconnected", "Disconnected")
        state = self.vehicle_states.get(connection_id)
        if state is not None:
            state.connected = False
        if self.selected_vehicle_id == connection_id:
            self._apply_selected_state(state or TelemetryState())
        self._update_aggregate_link()

    def connect_all_vehicles(self) -> None:
        for connection_id in list(self.connection_meta):
            self.connect_vehicle(connection_id)
        self._update_aggregate_link()

    def disconnect_all_vehicles(self) -> None:
        for connection_id in list(self.connection_meta):
            self.disconnect_vehicle(connection_id)
        self._update_aggregate_link()

    def remove_vehicle_connection(self, connection_id: str) -> None:
        if connection_id not in self.connection_meta:
            return
        self.disconnect_vehicle(connection_id)
        self.connection_meta.pop(connection_id, None)
        self.vehicle_states.pop(connection_id, None)
        self.vehicle_logs.pop(connection_id, None)
        self._last_map_update.pop(connection_id, None)
        self.link_panel.remove_connection_row(connection_id)
        self.map_view.remove_vehicle(connection_id)

        index = self.vehicle_selector.findData(connection_id)
        if index >= 0:
            self.vehicle_selector.removeItem(index)
        if self.selected_vehicle_id == connection_id:
            self.selected_vehicle_id = None
            if self.vehicle_selector.count() > 0:
                self.vehicle_selector.setCurrentIndex(0)
                next_id = self.vehicle_selector.currentData()
                if next_id:
                    self.select_vehicle(str(next_id))
            else:
                self._apply_empty_selected_state()
        self._update_aggregate_link()

    def _worker_finished(self, connection_id: str, worker: TelemetryWorker) -> None:
        current_worker = self.workers.get(connection_id)
        if current_worker is not None and current_worker is not worker:
            try:
                worker.deleteLater()
            except Exception:
                pass
            return
        if current_worker is worker:
            self.workers.pop(connection_id, None)
        try:
            worker.deleteLater()
        except Exception:
            pass
        meta = self.connection_meta.get(connection_id)
        if meta is not None and meta.get("link_state") not in {"error", "warning"}:
            meta["link_state"] = "disconnected"
            meta["link_text"] = "Disconnected"
            self.link_panel.set_connection_state(connection_id, "disconnected", "Disconnected")
        state = self.vehicle_states.get(connection_id)
        if state is not None:
            state.connected = False
        if self.selected_vehicle_id == connection_id:
            self._apply_selected_state(state or TelemetryState())
        self._update_aggregate_link()

    def update_vehicle_link_status(self, connection_id: str, state: str, text: str) -> None:
        meta = self.connection_meta.get(connection_id)
        if meta is None:
            return
        meta["link_state"] = state
        meta["link_text"] = text
        self.link_panel.set_connection_state(connection_id, state, text)
        if state in {"error", "warning"}:
            self._append_vehicle_log(connection_id, text)
        self._update_aggregate_link()

    def update_vehicle_command_status(self, connection_id: str, level: str, text: str) -> None:
        if self.selected_vehicle_id == connection_id:
            self.control_panel.set_command_status(level, text)

    def _append_vehicle_log(self, connection_id: str, text: str) -> None:
        if connection_id not in self.vehicle_logs:
            return
        line = f"[{time.strftime('%H:%M:%S')}] {text}"
        log = self.vehicle_logs[connection_id]
        log.append(line)
        if len(log) > 1000:
            del log[:-1000]
        if self.selected_vehicle_id == connection_id:
            self.messages_panel.append_line(line)

    def _clear_selected_messages(self) -> None:
        if self.selected_vehicle_id and self.selected_vehicle_id in self.vehicle_logs:
            self.vehicle_logs[self.selected_vehicle_id].clear()
        self.messages_panel.clear_view()

    def _vehicle_display_name(self, connection_id: str) -> str:
        meta = self.connection_meta.get(connection_id, {})
        state = self.vehicle_states.get(connection_id, TelemetryState())
        name = str(meta.get("name", "Vehicle"))
        sysid = "--" if state.system_id <= 0 else str(state.system_id)
        vehicle_type = state.vehicle_type or "UNKNOWN"
        return f"{name} · SYS {sysid} · {vehicle_type}"

    def _refresh_selector_item(self, connection_id: str) -> None:
        index = self.vehicle_selector.findData(connection_id)
        if index >= 0:
            self.vehicle_selector.setItemText(index, self._vehicle_display_name(connection_id))

    def _vehicle_selector_changed(self, index: int) -> None:
        if index < 0:
            return
        connection_id = self.vehicle_selector.itemData(index)
        if connection_id:
            self.select_vehicle(str(connection_id))

    def select_vehicle(self, connection_id: str) -> None:
        if connection_id not in self.connection_meta:
            return
        self.selected_vehicle_id = connection_id
        index = self.vehicle_selector.findData(connection_id)
        if index >= 0 and self.vehicle_selector.currentIndex() != index:
            self.vehicle_selector.blockSignals(True)
            self.vehicle_selector.setCurrentIndex(index)
            self.vehicle_selector.blockSignals(False)

        state = self.vehicle_states.get(connection_id, TelemetryState())
        self.current_state = state
        self._apply_selected_state(state)
        label = self._vehicle_display_name(connection_id)
        self.messages_panel.set_context(label)
        self.messages_panel.set_lines(self.vehicle_logs.get(connection_id, []))
        self.map_view.set_selected_vehicle(connection_id)

    def update_vehicle_telemetry(self, connection_id: str, data: dict) -> None:
        if connection_id not in self.connection_meta:
            return
        previous = self.vehicle_states.get(connection_id, TelemetryState())
        state = TelemetryState(**data)
        self.vehicle_states[connection_id] = state

        identity_changed = (
            state.system_id != previous.system_id
            or state.vehicle_type != previous.vehicle_type
        )
        if identity_changed or state.system_id > 0:
            self.link_panel.set_connection_identity(connection_id, state.system_id, state.vehicle_type)
            self._refresh_selector_item(connection_id)
            self._check_duplicate_sysid(connection_id, state.system_id)
            if self.selected_vehicle_id == connection_id:
                self.messages_panel.set_context(self._vehicle_display_name(connection_id))

        meta = self.connection_meta[connection_id]
        map_label = f"{meta['name']} · SYS {state.system_id if state.system_id > 0 else '--'}"
        valid_position = (
            state.latitude is not None
            and state.longitude is not None
            and not (abs(state.latitude) < 1e-7 and abs(state.longitude) < 1e-7)
            and (state.gps_fix is None or state.gps_fix >= 2)
        )
        now = time.monotonic()
        if valid_position and now - self._last_map_update.get(connection_id, 0.0) >= 0.35:
            self.map_view.update_vehicle_position(
                connection_id,
                float(state.latitude),
                float(state.longitude),
                state.heading_deg,
                map_label,
                state.vehicle_type,
                selected=(connection_id == self.selected_vehicle_id),
            )
            self._last_map_update[connection_id] = now
        elif connection_id == self.selected_vehicle_id and not valid_position:
            fix_name = GPS_FIX_NAMES.get(state.gps_fix, "--")
            sats = "--" if state.satellites is None else str(state.satellites)
            self.map_view.set_gps_waiting(f"{meta['name']} · GPS {fix_name} · {sats} sats")

        if state.home_latitude is not None and state.home_longitude is not None:
            self.map_view.update_vehicle_home(
                connection_id,
                float(state.home_latitude),
                float(state.home_longitude),
                map_label,
                state.vehicle_type,
            )

        if self.selected_vehicle_id == connection_id:
            self.current_state = state
            self._apply_selected_state(state)
        self._update_aggregate_link()

    def _check_duplicate_sysid(self, connection_id: str, system_id: int) -> None:
        if system_id <= 0:
            return
        for other_id, other_state in self.vehicle_states.items():
            if other_id == connection_id or other_state.system_id != system_id:
                continue
            key = tuple(sorted((connection_id, other_id))) + (system_id,)
            if key in self._duplicate_sysid_warnings:
                continue
            self._duplicate_sysid_warnings.add(key)
            self._append_vehicle_log(
                connection_id,
                f"WARNING: SYSID {system_id} is also used by another configured vehicle. Unique SYSIDs are recommended.",
            )
            self._append_vehicle_log(
                other_id,
                f"WARNING: SYSID {system_id} is also used by another configured vehicle. Unique SYSIDs are recommended.",
            )

    def _apply_selected_state(self, state: TelemetryState) -> None:
        self.flight_panel.update_state(state)
        self.instrument_panel.update_state(state)
        self.control_panel.set_control_state(state.connected, state.armed, state.mode)

        self.state_chip.set_value("ARMED" if state.armed else ("SAFE" if state.connected else "OFF"))
        self.state_chip.set_state("bad" if state.armed else ("good" if state.connected else "neutral"))

        self.mode_chip.set_value(state.mode if state.connected else "--")
        if state.connected:
            category = ControlPanel._mode_category(state.mode)
            self.mode_chip.set_state(
                "violet" if category == "return" else
                "warn" if category in {"land", "alt"} else
                "good" if category in {"hold", "stabilize"} else
                "info"
            )
        else:
            self.mode_chip.set_state("neutral")

        fix_name = GPS_FIX_NAMES.get(state.gps_fix, "--")
        sats = "--" if state.satellites is None else str(state.satellites)
        self.gps_chip.set_value(f"{fix_name} · {sats}" if state.connected else "--")
        if not state.connected or state.gps_fix is None or state.gps_fix < 2:
            self.gps_chip.set_state("bad" if state.connected else "neutral")
        elif state.gps_fix == 2:
            self.gps_chip.set_state("warn")
        else:
            self.gps_chip.set_state("good")

        if not state.connected or state.battery_voltage_v is None:
            battery_text = "--"
        elif state.battery_remaining_pct is None:
            battery_text = f"{state.battery_voltage_v:.1f}V"
        else:
            battery_text = f"{state.battery_voltage_v:.1f}V · {state.battery_remaining_pct}%"
        self.battery_chip.set_value(battery_text)
        if state.connected and state.battery_remaining_pct is not None:
            if state.battery_remaining_pct <= 15:
                self.battery_chip.set_state("bad")
            elif state.battery_remaining_pct <= 30:
                self.battery_chip.set_state("warn")
            else:
                self.battery_chip.set_state("good")
        else:
            self.battery_chip.set_state("info" if state.connected else "neutral")

        if self.selected_vehicle_id:
            name = str(self.connection_meta.get(self.selected_vehicle_id, {}).get("name", "Vehicle"))
            self.bottom_arm.setText(f"{name} · {'ARMED' if state.armed else 'DISARMED'}")
            self.bottom_arm.setStyleSheet(
                f"color:{DANGER if state.armed else (SUCCESS if state.connected else MUTED)}; font-weight:800;"
            )

    def _apply_empty_selected_state(self) -> None:
        empty = TelemetryState()
        self.current_state = empty
        self.flight_panel.update_state(empty)
        self.instrument_panel.update_state(empty)
        self.control_panel.set_control_state(False, False, "UNKNOWN")
        self.state_chip.set_value("--")
        self.state_chip.set_state("neutral")
        self.mode_chip.set_value("--")
        self.mode_chip.set_state("neutral")
        self.gps_chip.set_value("--")
        self.gps_chip.set_state("neutral")
        self.battery_chip.set_value("--")
        self.battery_chip.set_state("neutral")
        self.bottom_arm.setText("NO VEHICLE SELECTED")
        self.bottom_arm.setStyleSheet(f"color:{MUTED};")
        self.messages_panel.set_context(None)
        self.messages_panel.clear_view()
        self.map_view.set_selected_vehicle(None)
        self._update_aggregate_link()

    def _update_aggregate_link(self) -> None:
        total = len(self.connection_meta)
        live = sum(1 for m in self.connection_meta.values() if m.get("link_state") == "connected")
        active = sum(1 for w in self.workers.values() if w.isRunning())
        warnings = sum(1 for m in self.connection_meta.values() if m.get("link_state") in {"warning", "error"})

        self.link_chip.set_value(f"{live}/{total} LIVE" if total else "0/0")
        if total == 0:
            self.link_chip.set_state("neutral")
            self.bottom_link.setText("●  NO VEHICLE LINKS")
            self.bottom_link.setStyleSheet(f"color:{MUTED};")
        elif live == total:
            self.link_chip.set_state("good")
            self.bottom_link.setText(f"●  {live}/{total} VEHICLES CONNECTED")
            self.bottom_link.setStyleSheet(f"color:{SUCCESS};")
        elif warnings:
            self.link_chip.set_state("warn")
            self.bottom_link.setText(f"●  {live}/{total} VEHICLES LIVE")
            self.bottom_link.setStyleSheet(f"color:{WARNING};")
        elif active:
            self.link_chip.set_state("info")
            self.bottom_link.setText(f"●  {live}/{total} VEHICLES LIVE")
            self.bottom_link.setStyleSheet(f"color:{ACCENT};")
        else:
            self.link_chip.set_state("neutral")
            self.bottom_link.setText(f"●  0/{total} VEHICLES CONNECTED")
            self.bottom_link.setStyleSheet(f"color:{MUTED};")
        self.quick_connect.setText("DISCONNECT ALL" if active else "CONNECT ALL")

    def _selected_worker(self) -> TelemetryWorker | None:
        if not self.selected_vehicle_id:
            return None
        worker = self.workers.get(self.selected_vehicle_id)
        return worker if worker is not None and worker.isRunning() else None

    def _handle_arm_request(self, arm: bool) -> None:
        worker = self._selected_worker()
        if worker is None or not self.current_state.connected:
            QMessageBox.warning(self, "No MAVLink link", "Select a connected aircraft first.")
            return
        worker.queue_arm(arm)

    def _handle_mode_request(self, mode_name: str) -> None:
        worker = self._selected_worker()
        if worker is None or not self.current_state.connected:
            QMessageBox.warning(self, "No MAVLink link", "Select a connected aircraft first.")
            return
        normalized_current = self.current_state.mode.upper().replace(" ", "_")
        if normalized_current == mode_name:
            self.control_panel.set_command_status("success", f"Already in {mode_name}")
            return
        worker.queue_mode(mode_name)

    def resizeEvent(self, event: Any) -> None:
        super().resizeEvent(event)
        if self.width() < 1120:
            if self.instrument_dock.isVisible():
                self.instrument_dock.hide()
            if self.messages_dock.isVisible():
                self.messages_dock.hide()
                self.messages_action.setChecked(False)
        elif self.width() >= 1230:
            if not self.instrument_dock.isVisible():
                self.instrument_dock.show()
            if not self.messages_dock.isVisible():
                self.messages_dock.show()
                self.messages_action.setChecked(True)

    def closeEvent(self, event: Any) -> None:
        try:
            self.video_workspace.shutdown()
        except Exception:
            pass
        for connection_id in list(self.workers):
            worker = self.workers.pop(connection_id, None)
            if worker is not None:
                worker.request_stop()
                worker.wait(2500)
        event.accept()

    def _apply_style(self) -> None:
        self.setStyleSheet(
            f"""
            QMainWindow {{ background:{BG}; }}
            QWidget {{
                background:transparent; color:{TEXT};
                font-family:'Segoe UI'; font-size:10pt;
            }}
            QToolBar#topBar {{
                background:#1c2022; border:none; border-bottom:1px solid {BORDER};
                spacing:7px; padding:5px 8px;
            }}
            QToolBar#navRail {{
                background:{SURFACE}; border:none; border-right:1px solid {BORDER};
                spacing:5px; padding:7px 5px;
            }}
            QToolBar#navRail QToolButton {{
                background:transparent; border:1px solid transparent; border-radius:4px;
                padding:8px;
            }}
            QToolBar#navRail QToolButton:hover {{ background:{SURFACE_2}; border-color:{BORDER}; }}
            QToolBar#navRail QToolButton:checked {{ background:{ACCENT_DARK}; border-color:{ACCENT}; }}
            QDockWidget {{ border:1px solid {BORDER}; }}
            QDockWidget > QWidget {{ background:{SURFACE}; }}
            QFrame#statusChip {{ background:#252a2d; border:1px solid #373e42; border-radius:5px; }}
            QFrame#statusChip QLabel {{ background:transparent; border:none; }}
            QLabel#chipLabel {{ color:{MUTED}; font-size:8pt; font-weight:700; }}
            QLabel#chipValue {{ color:{TEXT}; font-size:9pt; font-weight:750; }}
            QLabel#vehicleSelectorLabel {{ color:{MUTED}; font-size:8pt; font-weight:800; margin-left:5px; }}
            QComboBox#vehicleSelector {{
                background:#252a2d; color:{TEXT}; border:1px solid #465055;
                border-radius:5px; padding:6px 10px; font-weight:700;
            }}
            QComboBox#vehicleSelector QAbstractItemView {{
                background:{SURFACE_2}; color:{TEXT}; selection-background-color:{ACCENT_DARK};
                border:1px solid {BORDER};
            }}
            QFrame#connectionRow {{ background:#171b1d; border:1px solid {BORDER}; border-radius:5px; }}
            QLabel#connectionName {{ color:{TEXT}; font-weight:800; }}
            QLabel#connectionIdentity {{ color:#b9c4c9; font-family:'Consolas'; font-size:8.5pt; }}
            QLabel#connectionEndpoint {{ color:{MUTED}; font-family:'Consolas'; font-size:8pt; }}
            QLabel#connectionStatus {{ color:{MUTED}; font-size:8pt; font-weight:800; }}
            QLabel#connectionStatus[linkState='connected'] {{ color:{SUCCESS}; }}
            QLabel#connectionStatus[linkState='connecting'] {{ color:{WARNING}; }}
            QLabel#connectionStatus[linkState='warning'] {{ color:{WARNING}; }}
            QLabel#connectionStatus[linkState='error'] {{ color:{DANGER}; }}
            QPushButton {{
                background:{SURFACE_2}; color:{TEXT}; border:1px solid {BORDER};
                border-radius:4px; padding:7px 10px; font-weight:650;
            }}
            QPushButton:hover:enabled {{ border-color:#596267; background:#32383b; }}
            QPushButton:pressed:enabled {{ background:#15191b; }}
            QPushButton:disabled {{ color:#657075; background:#1b1f21; border-color:#2a2f31; }}
            QPushButton#topConnectButton {{ min-width:110px; }}
            QPushButton#primaryButton {{ background:{SURFACE_2}; border-color:{ACCENT}; }}
            QFrame#videoPanel {{ background:transparent; border:none; }}
            QFrame#videoWorkspace {{ background:{SURFACE}; border:none; }}
            QFrame#videoFeedCard {{ background:#171b1d; border:1px solid {BORDER}; border-radius:5px; }}
            QWidget#videoFeedContainer {{ background:transparent; }}
            QScrollArea#videoScrollArea {{ background:transparent; border:none; }}
            QScrollArea#videoScrollArea > QWidget > QWidget {{ background:transparent; }}
            QLabel#videoWorkspaceTitle {{ color:{TEXT}; font-size:9pt; font-weight:800; letter-spacing:1px; }}
            QLabel#videoWorkspaceInfo {{ color:{MUTED}; font-size:8pt; }}
            QLabel#videoFeedTitle {{ color:{TEXT}; font-size:8.5pt; font-weight:800; min-width:52px; }}
            QLabel#videoTitle {{ color:{MUTED}; font-size:8pt; font-weight:800; letter-spacing:1px; }}
            QLabel#videoStatus {{ color:{MUTED}; font-size:8pt; padding:0 2px; }}
            QLabel#videoUnavailable {{ background:#080a0b; color:{MUTED}; border:1px solid {BORDER}; }}
            QComboBox#videoDeviceCombo {{
                background:{SURFACE_2}; color:{TEXT}; border:1px solid {BORDER};
                border-radius:4px; padding:4px 7px; min-height:22px;
            }}
            QComboBox#videoDeviceCombo QAbstractItemView {{
                background:{SURFACE_2}; color:{TEXT}; selection-background-color:{ACCENT_DARK};
                border:1px solid {BORDER};
            }}
            QPushButton#videoMiniButton, QPushButton#videoStartButton {{ min-height:22px; padding:3px 6px; font-size:8pt; }}
            QVideoWidget#fpvVideo {{ background:#050607; border:1px solid {BORDER}; border-radius:4px; }}
            QPushButton[controlButton='true'] {{
                background:{SURFACE_2}; color:{TEXT}; border:1px solid #465055;
                min-height:34px;
            }}
            QPushButton[controlButton='true']:hover:enabled {{ background:#30373a; border-color:{TEAL}; }}
            QPushButton[controlButton='true']:pressed:enabled {{ background:#172526; border-color:{TEAL}; }}
            QPushButton[controlButton='true'][modeActive='true'] {{
                background:#203b3a; border:1px solid {TEAL}; color:#dff8f2;
            }}
            QLineEdit {{
                background:{SURFACE_3}; color:{TEXT}; border:1px solid {BORDER};
                border-radius:4px; padding:8px;
            }}
            QLineEdit:focus {{ border-color:{ACCENT}; }}
            QLabel#metricLabel {{ color:{MUTED}; }}
            QLabel#metricValue {{ color:{TEXT}; font-family:'Consolas'; font-weight:700; }}
            QLabel#metricUnit {{ color:{MUTED}; font-size:8pt; }}
            QLabel#miniLabel {{ color:{MUTED}; font-size:7.5pt; font-weight:700; }}
            QLabel#miniValue {{ color:{TEXT}; font-size:18pt; font-weight:750; }}
            QLabel#miniUnit {{ color:{MUTED}; font-size:8pt; }}
            QLabel#modeBadge {{
                background:{SURFACE_2}; border:1px solid #465055; border-radius:4px;
                padding:9px; color:{TEXT}; font-size:13pt; font-weight:800;
            }}
            QLabel#commandStatus {{
                background:{SURFACE_3}; color:{MUTED}; border:1px solid {BORDER};
                border-radius:4px; padding:8px;
            }}
            QLabel#commandStatus[commandLevel='pending'] {{ border-color:{WARNING}; color:#ffd48a; }}
            QLabel#commandStatus[commandLevel='success'] {{ border-color:{SUCCESS}; color:#b6e5a0; }}
            QLabel#commandStatus[commandLevel='error'] {{ border-color:{DANGER}; color:#ffb2b2; }}
            QFrame#separator {{ color:{BORDER}; background:{BORDER}; max-height:1px; }}
            QFrame#mapControls {{
                background:rgba(31,35,37,215); border:1px solid {BORDER}; border-radius:4px;
                margin:10px;
            }}
            QPushButton#mapIconButton {{ padding:4px; }}
            QPushButton#mapTextButton {{ font-size:8pt; padding:5px; min-width:56px; }}
            QPushButton#mapTextButton:checked {{ background:{ACCENT_DARK}; border-color:{ACCENT}; }}
            QLabel#mapPositionStrip {{
                background:rgba(22,25,27,220); color:#c8d0d4; border:1px solid {BORDER};
                border-radius:3px; margin:10px; padding:6px 9px; font-family:'Consolas';
            }}
            QLabel#mapFallback {{ color:{MUTED}; }}
            QLabel#messagesTitle {{ color:{TEXT}; font-weight:800; letter-spacing:1px; }}
            QPushButton#compactButton {{ font-size:8pt; padding:4px 7px; }}
            QTextEdit {{
                background:{SURFACE_3}; color:#cad2d6; border:1px solid {BORDER};
                border-radius:3px; padding:5px; font-family:'Consolas'; font-size:9pt;
            }}
            QLabel#linkText {{ color:{MUTED}; }}
            QStatusBar#statusBar {{
                background:#111416; border-top:1px solid {BORDER}; color:{MUTED};
                font-size:8.5pt;
            }}
            QLabel#bottomArm {{ padding-right:8px; }}
            """
        )


def main() -> None:
    app = QApplication(sys.argv)
    app.setApplicationName(BRAND_NAME)
    app.setOrganizationName(BRAND_NAME)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
