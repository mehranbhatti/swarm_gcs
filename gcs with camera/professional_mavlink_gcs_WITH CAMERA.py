#!/usr/bin/env python3


from __future__ import annotations

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
.drone-shell, .home-shell { background:transparent; border:none; }
.drone { width:46px; height:46px; transform-origin:23px 23px; filter:drop-shadow(0 2px 5px rgba(0,0,0,.85)); }
.waiting {
  position:absolute; z-index:900; left:50%; top:14px; transform:translateX(-50%);
  padding:7px 12px; border-radius:4px; color:#c5d0d5; background:rgba(25,29,31,.88);
  border:1px solid #3a4246; font-size:12px; letter-spacing:.2px; pointer-events:none;
}
</style>
</head>
<body>
<div id="map"></div>
<div id="waiting" class="waiting">Waiting for GPS</div>
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js" crossorigin=""></script>
<script>
const map = L.map('map', { zoomControl:false, preferCanvas:true, attributionControl:true }).setView([24.7136,46.6753], 6);

// High-resolution satellite/aerial imagery from Esri World Imagery.
// A separate reference layer can be enabled to create a hybrid view with
// place names and boundaries while keeping the satellite imagery visible.
const satelliteLayer = L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  {
    maxZoom: 20,
    attribution: 'Imagery &copy; Esri, Vantor, Earthstar Geographics, and the GIS User Community'
  }
);

const labelsLayer = L.tileLayer(
  'https://services.arcgisonline.com/ArcGIS/rest/services/Reference/World_Boundaries_and_Places/MapServer/tile/{z}/{y}/{x}',
  {
    maxZoom: 20,
    attribution: 'Reference &copy; Esri'
  }
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
let droneMarker = null;
let homeMarker = null;
let track = [];
let trackLine = L.polyline([], {color:'#37a8db', weight:3, opacity:.85}).addTo(map);
let firstPosition = true;

function droneSvg(heading) {
  return `<div class="drone" style="transform:rotate(${heading}deg)">
  <svg width="46" height="46" viewBox="0 0 46 46" xmlns="http://www.w3.org/2000/svg">
    <circle cx="10" cy="10" r="6.3" fill="#111416" stroke="#b7c2c7" stroke-width="2"/>
    <circle cx="36" cy="10" r="6.3" fill="#111416" stroke="#b7c2c7" stroke-width="2"/>
    <circle cx="10" cy="36" r="6.3" fill="#111416" stroke="#b7c2c7" stroke-width="2"/>
    <circle cx="36" cy="36" r="6.3" fill="#111416" stroke="#b7c2c7" stroke-width="2"/>
    <path d="M14 14 L32 32 M32 14 L14 32" stroke="#37a8db" stroke-width="3.2" stroke-linecap="round"/>
    <rect x="17" y="18" width="12" height="10" rx="3" fill="#202426" stroke="#edf2f4" stroke-width="1.8"/>
    <path d="M23 2 L18.5 12 L27.5 12 Z" fill="#f5a623"/>
  </svg></div>`;
}
function droneIcon(heading) {
  return L.divIcon({className:'drone-shell', html:droneSvg(heading), iconSize:[46,46], iconAnchor:[23,23]});
}
function homeIcon() {
  return L.divIcon({className:'home-shell', html:`
  <svg width="30" height="30" viewBox="0 0 30 30" xmlns="http://www.w3.org/2000/svg">
    <circle cx="15" cy="15" r="12" fill="#202426" stroke="#59aa29" stroke-width="2"/>
    <path d="M8 15 L15 9 L22 15 V22 H17 V17 H13 V22 H8 Z" fill="#59aa29"/>
  </svg>`, iconSize:[30,30], iconAnchor:[15,15]});
}
function setDroneLocation(lat, lon, heading, follow) {
  const pos=[lat,lon];
  document.getElementById('waiting').style.display='none';
  if (!droneMarker) droneMarker=L.marker(pos,{icon:droneIcon(heading),zIndexOffset:1000}).addTo(map);
  else { droneMarker.setLatLng(pos); droneMarker.setIcon(droneIcon(heading)); }
  const last=track.length ? track[track.length-1] : null;
  if (!last || Math.abs(last[0]-lat)>0.000001 || Math.abs(last[1]-lon)>0.000001) {
    track.push(pos); if(track.length>1500) track.shift(); trackLine.setLatLngs(track);
  }
  if (firstPosition) { map.setView(pos,18); firstPosition=false; }
  else if (follow) map.panTo(pos,{animate:true,duration:.25});
}
function setHome(lat, lon) {
  const pos=[lat,lon];
  if(!homeMarker) homeMarker=L.marker(pos,{icon:homeIcon(),zIndexOffset:800}).addTo(map);
  else homeMarker.setLatLng(pos);
}
function centerDrone() { if(droneMarker) map.setView(droneMarker.getLatLng(),Math.max(map.getZoom(),18)); }
function zoomIn() { map.zoomIn(); }
function zoomOut() { map.zoomOut(); }
function clearTrack() { track=[]; trackLine.setLatLngs([]); }
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
        self._latest_position: tuple[float, float, float] | None = None
        self._latest_home: tuple[float, float] | None = None

        grid = QGridLayout(self)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(0)

        if WEBENGINE_AVAILABLE and QWebEngineView is not None:
            self.web = QWebEngineView()
            self.web.page().profile().setHttpUserAgent(
                "GroundControlStation/1.1 (PySide6 QWebEngine; MAVLink GCS)"
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
        center.setToolTip("Center on aircraft")
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
        clear_track.setToolTip("Clear displayed flight track")
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

        grid.addWidget(
            self.controls,
            0,
            0,
            Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignRight,
        )

        self.position_strip = QLabel("GPS --")
        self.position_strip.setObjectName("mapPositionStrip")
        self.position_strip.setMinimumWidth(260)
        grid.addWidget(
            self.position_strip,
            0,
            0,
            Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignLeft,
        )

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
        if self._latest_home is not None:
            self.update_home(*self._latest_home)
        if self._latest_position is not None:
            lat, lon, hdg = self._latest_position
            self.update_position(lat, lon, hdg)

    def _js(self, script: str) -> None:
        if self.web is not None and self._loaded:
            self.web.page().runJavaScript(script)

    def update_position(self, lat: float, lon: float, heading: float | None) -> None:
        hdg = 0.0 if heading is None else float(heading) % 360.0
        self._latest_position = (lat, lon, hdg)
        self.position_strip.setText(f"{lat:.7f}  {lon:.7f}")
        self._js(
            f"setDroneLocation({lat:.8f},{lon:.8f},{hdg:.2f},{str(self._follow).lower()});"
        )

    def update_home(self, lat: float, lon: float) -> None:
        self._latest_home = (lat, lon)
        self._js(f"setHome({lat:.8f},{lon:.8f});")

    def set_gps_waiting(self, text: str) -> None:
        if self._latest_position is None:
            self.position_strip.setText(text)

    def center_on_drone(self) -> None:
        self._js("centerDrone();")


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
        self.video_panel = VideoPanel()

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

        video_separator = QFrame()
        video_separator.setFrameShape(QFrame.Shape.HLine)
        video_separator.setObjectName("separator")
        layout.addWidget(video_separator)
        layout.addWidget(self.video_panel, 1)

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


class VideoPanel(QFrame):
    """Compact live preview for a USB/UVC FPV receiver.

    The Skydroid receiver presents its decoded analog 5.8 GHz feed to Windows as
    a USB Video Class camera. Qt Multimedia enumerates those Windows camera
    inputs, so the GCS can preview the receiver directly without routing video
    through MAVLink or the Jetson.
    """

    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("videoPanel")
        self._devices: list[Any] = []
        self._camera: Any = None
        self._session: Any = None
        self._media_devices: Any = None
        self._auto_started = False

        outer = QVBoxLayout(self)
        outer.setContentsMargins(0, 4, 0, 0)
        outer.setSpacing(6)

        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        header.setSpacing(5)

        title = QLabel("VIDEO")
        title.setObjectName("videoTitle")
        header.addWidget(title)

        self.device_combo = QComboBox()
        self.device_combo.setObjectName("videoDeviceCombo")
        self.device_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.device_combo.currentIndexChanged.connect(self._selection_changed)
        header.addWidget(self.device_combo, 1)

        self.refresh_button = QPushButton("↻")
        self.refresh_button.setObjectName("videoMiniButton")
        self.refresh_button.setToolTip("Refresh camera devices")
        self.refresh_button.setFixedWidth(34)
        self.refresh_button.clicked.connect(self.refresh_devices)
        header.addWidget(self.refresh_button)

        self.start_button = QPushButton("START")
        self.start_button.setObjectName("videoStartButton")
        self.start_button.setFixedWidth(62)
        self.start_button.clicked.connect(self.toggle_video)
        header.addWidget(self.start_button)
        outer.addLayout(header)

        if MULTIMEDIA_AVAILABLE and QVideoWidget is not None:
            self.video_widget = QVideoWidget()
            self.video_widget.setObjectName("fpvVideo")
            self.video_widget.setMinimumHeight(165)
            self.video_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
            self.video_widget.setAspectRatioMode(Qt.AspectRatioMode.KeepAspectRatio)
            outer.addWidget(self.video_widget, 1)
        else:
            self.video_widget = None
            unavailable = QLabel("Qt Multimedia is unavailable in this PySide6 installation.")
            unavailable.setWordWrap(True)
            unavailable.setAlignment(Qt.AlignmentFlag.AlignCenter)
            unavailable.setMinimumHeight(145)
            unavailable.setObjectName("videoUnavailable")
            outer.addWidget(unavailable)
            self.device_combo.setEnabled(False)
            self.refresh_button.setEnabled(False)
            self.start_button.setEnabled(False)

        self.status = QLabel("Receiver not started")
        self.status.setObjectName("videoStatus")
        self.status.setWordWrap(True)
        outer.addWidget(self.status)

        if MULTIMEDIA_AVAILABLE and QMediaDevices is not None:
            try:
                self._media_devices = QMediaDevices(self)
                self._media_devices.videoInputsChanged.connect(self.refresh_devices)
            except Exception:
                self._media_devices = None
            QTimer.singleShot(250, self.refresh_devices)

    @staticmethod
    def _device_id(device: Any) -> bytes:
        try:
            return bytes(device.id())
        except Exception:
            return str(device.description()).encode("utf-8", errors="ignore")

    def refresh_devices(self) -> None:
        if not MULTIMEDIA_AVAILABLE or QMediaDevices is None:
            return

        previous_id: bytes | None = None
        current = self.device_combo.currentIndex()
        if 0 <= current < len(self._devices):
            previous_id = self._device_id(self._devices[current])

        try:
            devices = list(QMediaDevices.videoInputs())
        except Exception as error:
            self.status.setText(f"Camera scan failed: {error}")
            return

        self._devices = devices
        self.device_combo.blockSignals(True)
        self.device_combo.clear()

        preferred_index = -1
        for index, device in enumerate(devices):
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
            for index, device in enumerate(devices):
                if self._device_id(device) == previous_id:
                    selected = index
                    break
        if selected < 0 and preferred_index >= 0:
            selected = preferred_index
        if selected < 0 and len(devices) == 1:
            selected = 0
        if selected < 0 and devices:
            selected = 0

        if selected >= 0:
            self.device_combo.setCurrentIndex(selected)
        self.device_combo.blockSignals(False)

        if not devices:
            self.stop_video()
            self.status.setText("No USB/UVC camera detected")
            self.start_button.setEnabled(False)
            return

        self.start_button.setEnabled(True)
        try:
            selected_name = str(devices[selected].description()) if selected >= 0 else "camera"
        except Exception:
            selected_name = "camera"
        self.status.setText(f"Ready · {selected_name}")

        # Auto-start a likely FPV/UVC receiver. If the PC exposes only one camera,
        # that camera is also safe to auto-preview; multiple ambiguous cameras
        # remain selectable from the drop-down.
        if not self._auto_started:
            auto_ok = len(devices) == 1
            if selected >= 0:
                try:
                    lower = str(devices[selected].description()).lower()
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
        """Prefer the receiver's typical 640x480 / ~30 fps UVC mode."""
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
            # Prefer >=25 fps when resolution scores are similar.
            low_fps_penalty = 1000.0 if fps < 25.0 else 0.0
            return (resolution_error + low_fps_penalty, fps_error, -fps)

        return min(formats, key=score)

    def start_video(self) -> None:
        if not MULTIMEDIA_AVAILABLE or QCamera is None or QMediaCaptureSession is None:
            return
        index = self.device_combo.currentIndex()
        if index < 0 or index >= len(self._devices):
            self.status.setText("Select a video receiver")
            return

        self.stop_video(clear_status=False)
        device = self._devices[index]

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
            self.start_button.setText("START")
            self.status.setText(f"Video start failed: {error}")

    def stop_video(self, clear_status: bool = True) -> None:
        camera = self._camera
        self._camera = None
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
                name = "receiver"
            self.status.setText(f"Stopped · {name}")

    def toggle_video(self) -> None:
        if self._camera is None:
            self.start_video()
        else:
            self.stop_video()

    def _camera_error(self, _error: Any, error_string: str) -> None:
        self.status.setText(f"VIDEO ERROR · {error_string}")
        self.start_button.setText("START")

    def _camera_active_changed(self, active: bool) -> None:
        if not active and self._camera is not None:
            # Avoid overwriting a more useful explicit error string.
            if not self.status.text().startswith("VIDEO ERROR"):
                self.status.setText("Video receiver stopped")

    def shutdown(self) -> None:
        self.stop_video(clear_status=False)


class LinkPanel(QWidget):
    connect_requested = Signal(str)
    disconnect_requested = Signal()

    def __init__(self) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(9)

        self.endpoint = QLineEdit(DEFAULT_CONNECTION)
        self.endpoint.setPlaceholderText("MAVLink endpoint")
        self.status = QLabel("Disconnected")
        self.status.setWordWrap(True)
        self.status.setObjectName("linkText")
        self.connect_button = QPushButton("CONNECT")
        self.connect_button.setObjectName("primaryButton")
        self.connect_button.clicked.connect(self._toggle)
        self._connected = False

        layout.addWidget(self.endpoint)
        layout.addWidget(self.status)
        layout.addWidget(self.connect_button)
        layout.addStretch(1)

    def _toggle(self) -> None:
        if self._connected:
            self.disconnect_requested.emit()
        else:
            endpoint = self.endpoint.text().strip()
            if endpoint:
                self.connect_requested.emit(endpoint)

    def set_link_state(self, state: str, text: str) -> None:
        self.status.setText(text)
        self._connected = state == "connected"
        self.endpoint.setEnabled(state not in ("connecting", "connected"))
        self.connect_button.setText("DISCONNECT" if self._connected else "CONNECT")
        self.connect_button.setEnabled(state != "connecting")


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
    def __init__(self) -> None:
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 8)
        layout.setSpacing(5)
        header = QHBoxLayout()
        title = QLabel("MESSAGES")
        title.setObjectName("messagesTitle")
        clear = QPushButton("CLEAR")
        clear.setObjectName("compactButton")
        clear.setMaximumWidth(70)
        clear.clicked.connect(lambda: self.log.clear())
        header.addWidget(title)
        header.addStretch(1)
        header.addWidget(clear)
        self.log = QTextEdit()
        self.log.setReadOnly(True)
        self.log.setMinimumHeight(110)
        layout.addLayout(header)
        layout.addWidget(self.log)

    def append(self, text: str) -> None:
        self.log.append(f"[{time.strftime('%H:%M:%S')}] {text}")


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(BRAND_NAME)
        self.setWindowIcon(make_icon("flight", ACCENT, 32))
        self.resize(1500, 900)
        self.setMinimumSize(860, 620)
        self.setDockNestingEnabled(True)

        self.worker: TelemetryWorker | None = None
        self.current_state = TelemetryState()
        self._last_map_update = 0.0

        self.map_view = MapView()
        self.setCentralWidget(self.map_view)

        self.flight_panel = FlightInfoPanel()
        self.control_panel = ControlPanel()
        self.link_panel = LinkPanel()
        self.instrument_panel = InstrumentPanel()
        self.messages_panel = MessagesPanel()

        self.flight_dock = self._make_dock(self.flight_panel, "flightDock", 280)
        self.control_dock = self._make_dock(self.control_panel, "controlDock", 310)
        self.link_dock = self._make_dock(self.link_panel, "linkDock", 310)
        self.instrument_dock = self._make_dock(self.instrument_panel, "instrumentDock", 330)
        self.messages_dock = self._make_dock(self.messages_panel, "messagesDock", 390)

        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.flight_dock)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.control_dock)
        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.link_dock)
        self.tabifyDockWidget(self.flight_dock, self.control_dock)
        self.tabifyDockWidget(self.control_dock, self.link_dock)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self.messages_dock)
        self.messages_dock.setAllowedAreas(Qt.DockWidgetArea.BottomDockWidgetArea)
        self.messages_dock.setMinimumHeight(145)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.instrument_dock)

        # Start clean: map + compact instruments. Other panels open from rail.
        self.flight_dock.hide()
        self.control_dock.hide()
        self.link_dock.hide()
        self.messages_dock.hide()

        self._build_top_bar()
        self._build_nav_rail()
        self._build_status_bar()
        self._wire_signals()
        self._apply_style()

        self._link_state = "disconnected"
        self._link_text = "Disconnected"

        QTimer.singleShot(500, lambda: self.connect_telemetry(DEFAULT_CONNECTION))

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

        self.link_chip = StatusChip("LINK", "OFF")
        self.state_chip = StatusChip("STATE", "DISARMED")
        self.mode_chip = StatusChip("MODE", "--")
        self.gps_chip = StatusChip("GPS", "--")
        self.battery_chip = StatusChip("BAT", "--")

        for chip in (self.link_chip, self.state_chip, self.mode_chip, self.gps_chip, self.battery_chip):
            bar.addWidget(chip)

        expanding = QWidget()
        expanding.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        bar.addWidget(expanding)

        self.quick_connect = QPushButton("DISCONNECT")
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

        self.flight_action = self._rail_action("flight", "Flight data", self._show_flight)
        self.control_action = self._rail_action("control", "Vehicle control", self._show_control)
        self.link_action = self._rail_action("link", "Telemetry connection", self._show_link)
        self.messages_action = self._rail_action("messages", "Messages", self._toggle_messages)
        center_action = self._rail_action("center", "Center map on aircraft", self.map_view.center_on_drone)

        for action in (
            self.flight_action,
            self.control_action,
            self.link_action,
            self.messages_action,
        ):
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
        self.bottom_link = QLabel("●  DISCONNECTED")
        self.bottom_link.setObjectName("bottomLink")
        self.bottom_arm = QLabel("DISARMED")
        self.bottom_arm.setObjectName("bottomArm")
        bar.addWidget(self.bottom_link)
        spacer = QWidget()
        spacer.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        bar.addWidget(spacer, 1)
        bar.addPermanentWidget(self.bottom_arm)

    def _wire_signals(self) -> None:
        self.control_panel.arm_requested.connect(self._handle_arm_request)
        self.control_panel.mode_requested.connect(self._handle_mode_request)
        self.link_panel.connect_requested.connect(self.connect_telemetry)
        self.link_panel.disconnect_requested.connect(self.disconnect_telemetry)

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
            self.resizeDocks([self.messages_dock], [190], Qt.Orientation.Vertical)
            self.messages_action.setChecked(True)

    def _quick_toggle_connection(self) -> None:
        if self.worker is not None and self.worker.isRunning():
            self.disconnect_telemetry()
        else:
            self.connect_telemetry(self.link_panel.endpoint.text().strip() or DEFAULT_CONNECTION)

    def connect_telemetry(self, endpoint: str) -> None:
        if self.worker is not None and self.worker.isRunning():
            return
        if not endpoint:
            return
        self.link_panel.endpoint.setText(endpoint)
        self.worker = TelemetryWorker(endpoint, self)
        self.worker.telemetry.connect(self.update_telemetry)
        self.worker.link_status.connect(self.update_link_status)
        self.worker.log_message.connect(self.messages_panel.append)
        self.worker.command_status.connect(self.update_command_status)
        self.worker.finished.connect(self._worker_finished)
        self.worker.start()

    def disconnect_telemetry(self) -> None:
        if self.worker is not None:
            self.quick_connect.setEnabled(False)
            self.worker.request_stop()
            self.worker.wait(2500)

    def _worker_finished(self) -> None:
        self.quick_connect.setEnabled(True)
        self.quick_connect.setText("CONNECT")
        self.control_panel.set_control_state(False, False, self.current_state.mode)
        if self.worker is not None:
            self.worker.deleteLater()
            self.worker = None

    def update_link_status(self, state: str, text: str) -> None:
        self._link_state = state
        self._link_text = text
        self.link_panel.set_link_state(state, text)
        self.quick_connect.setEnabled(state != "connecting")
        self.quick_connect.setText("DISCONNECT" if state == "connected" else "CONNECT")

        if state == "connected":
            self.link_chip.set_value("LIVE")
            self.link_chip.set_state("good")
            self.bottom_link.setText("●  CONNECTED")
            self.bottom_link.setStyleSheet(f"color:{SUCCESS};")
        elif state == "connecting":
            self.link_chip.set_value("WAIT")
            self.link_chip.set_state("warn")
            self.bottom_link.setText("●  CONNECTING")
            self.bottom_link.setStyleSheet(f"color:{WARNING};")
        elif state == "warning":
            self.link_chip.set_value("STALE")
            self.link_chip.set_state("warn")
            self.bottom_link.setText("●  LINK STALE")
            self.bottom_link.setStyleSheet(f"color:{WARNING};")
        elif state == "error":
            self.link_chip.set_value("ERROR")
            self.link_chip.set_state("bad")
            self.bottom_link.setText("●  ERROR")
            self.bottom_link.setStyleSheet(f"color:{DANGER};")
        else:
            self.link_chip.set_value("OFF")
            self.link_chip.set_state("neutral")
            self.bottom_link.setText("●  DISCONNECTED")
            self.bottom_link.setStyleSheet(f"color:{MUTED};")

    def update_command_status(self, level: str, text: str) -> None:
        self.control_panel.set_command_status(level, text)

    def update_telemetry(self, data: dict) -> None:
        self.current_state = TelemetryState(**data)
        s = self.current_state
        self.flight_panel.update_state(s)
        self.instrument_panel.update_state(s)
        self.control_panel.set_control_state(s.connected, s.armed, s.mode)

        self.state_chip.set_value("ARMED" if s.armed else "SAFE")
        self.state_chip.set_state("bad" if s.armed else "good")
        self.bottom_arm.setText("ARMED" if s.armed else "DISARMED")
        self.bottom_arm.setStyleSheet(f"color:{DANGER if s.armed else SUCCESS}; font-weight:800;")

        self.mode_chip.set_value(s.mode)
        category = ControlPanel._mode_category(s.mode)
        self.mode_chip.set_state(
            "violet" if category == "return" else
            "warn" if category in {"land", "alt"} else
            "good" if category in {"hold", "stabilize"} else
            "info"
        )

        fix_name = GPS_FIX_NAMES.get(s.gps_fix, "--")
        sats = "--" if s.satellites is None else str(s.satellites)
        self.gps_chip.set_value(f"{fix_name} · {sats}")
        if s.gps_fix is None or s.gps_fix < 2:
            self.gps_chip.set_state("bad")
        elif s.gps_fix == 2:
            self.gps_chip.set_state("warn")
        else:
            self.gps_chip.set_state("good")

        if s.battery_voltage_v is None:
            battery_text = "--"
        elif s.battery_remaining_pct is None:
            battery_text = f"{s.battery_voltage_v:.1f}V"
        else:
            battery_text = f"{s.battery_voltage_v:.1f}V · {s.battery_remaining_pct}%"
        self.battery_chip.set_value(battery_text)
        if s.battery_remaining_pct is not None:
            if s.battery_remaining_pct <= 15:
                self.battery_chip.set_state("bad")
            elif s.battery_remaining_pct <= 30:
                self.battery_chip.set_state("warn")
            else:
                self.battery_chip.set_state("good")
        else:
            self.battery_chip.set_state("info")

        valid_position = (
            s.latitude is not None
            and s.longitude is not None
            and not (abs(s.latitude) < 1e-7 and abs(s.longitude) < 1e-7)
            and (s.gps_fix is None or s.gps_fix >= 2)
        )
        now = time.monotonic()
        if valid_position and now - self._last_map_update >= 0.35:
            self.map_view.update_position(float(s.latitude), float(s.longitude), s.heading_deg)
            self._last_map_update = now
        elif not valid_position:
            self.map_view.set_gps_waiting(f"GPS {fix_name} · {sats} sats")

        if s.home_latitude is not None and s.home_longitude is not None:
            self.map_view.update_home(float(s.home_latitude), float(s.home_longitude))


    def _handle_arm_request(self, arm: bool) -> None:
        if not self._require_connection():
            return
        if self.worker is not None:
            self.worker.queue_arm(arm)

    def _handle_mode_request(self, mode_name: str) -> None:
        if not self._require_connection():
            return
        normalized_current = self.current_state.mode.upper().replace(" ", "_")
        if normalized_current == mode_name:
            self.update_command_status("success", f"Already in {mode_name}")
            return

        if self.worker is not None:
            self.worker.queue_mode(mode_name)

    def _require_connection(self) -> bool:
        if self.worker is None or not self.worker.isRunning() or not self.current_state.connected:
            QMessageBox.warning(self, "No MAVLink link", "Connect to the flight controller first.")
            return False
        return True

    def resizeEvent(self, event: Any) -> None:
        super().resizeEvent(event)
        # No absolute positioning is used. On compact windows we hide the right
        # dock automatically so map/control layouts never overlap. Users can
        # re-open it from Qt's dock system by enlarging the window.
        if self.width() < 1120 and self.instrument_dock.isVisible():
            self.instrument_dock.hide()
        elif self.width() >= 1230 and not self.instrument_dock.isVisible():
            self.instrument_dock.show()

    def closeEvent(self, event: Any) -> None:
        try:
            self.instrument_panel.video_panel.shutdown()
        except Exception:
            pass
        if self.worker is not None and self.worker.isRunning():
            self.worker.request_stop()
            self.worker.wait(2500)
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
            QFrame#statusChip {{
                background:#252a2d; border:1px solid #373e42; border-radius:5px;
            }}
            QFrame#statusChip QLabel {{ background:transparent; border:none; }}
            QLabel#chipLabel {{ color:{MUTED}; font-size:8pt; font-weight:700; }}
            QLabel#chipValue {{ color:{TEXT}; font-size:9pt; font-weight:750; }}
            QPushButton {{
                background:{SURFACE_2}; color:{TEXT}; border:1px solid {BORDER};
                border-radius:4px; padding:7px 10px; font-weight:650;
            }}
            QPushButton:hover:enabled {{ border-color:#596267; background:#32383b; }}
            QPushButton:pressed:enabled {{ background:#15191b; }}
            QPushButton:disabled {{ color:#657075; background:#1b1f21; border-color:#2a2f31; }}
            QPushButton#topConnectButton {{ min-width:92px; }}
            QPushButton#primaryButton {{ background:{SURFACE_2}; border-color:{ACCENT}; }}
            QFrame#videoPanel {{ background:transparent; border:none; }}
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
            QPushButton#videoMiniButton, QPushButton#videoStartButton {{
                min-height:22px; padding:3px 6px; font-size:8pt;
            }}
            QVideoWidget#fpvVideo {{
                background:#050607; border:1px solid {BORDER}; border-radius:4px;
            }}
            QPushButton[controlButton='true'] {{
                background:{SURFACE_2}; color:{TEXT}; border:1px solid #465055;
                min-height:34px;
            }}
            QPushButton[controlButton='true']:hover:enabled {{
                background:#30373a; border-color:{TEAL};
            }}
            QPushButton[controlButton='true']:pressed:enabled {{
                background:#172526; border-color:{TEAL};
            }}
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
