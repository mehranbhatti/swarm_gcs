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

try:
    from PySide6.QtSerialPort import QSerialPortInfo
    QT_SERIALPORT_AVAILABLE = True
except ImportError:
    QSerialPortInfo = None  # type: ignore[assignment]
    QT_SERIALPORT_AVAILABLE = False

try:
    from serial.tools import list_ports as pyserial_list_ports
    PYSERIAL_PORTS_AVAILABLE = True
except ImportError:
    pyserial_list_ports = None  # type: ignore[assignment]
    PYSERIAL_PORTS_AVAILABLE = False

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
    QDialog,
    QDialogButtonBox,
    QDockWidget,
    QFormLayout,
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
    QSpinBox,
    QSplitter,
    QStatusBar,
    QTabWidget,
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
    try:
        from PySide6.QtMultimedia import QVideoSink
    except ImportError:
        QVideoSink = None  # type: ignore[assignment]
    from PySide6.QtMultimediaWidgets import QVideoWidget

    MULTIMEDIA_AVAILABLE = True
except ImportError:
    QCamera = None  # type: ignore[assignment]
    QMediaCaptureSession = None  # type: ignore[assignment]
    QMediaDevices = None  # type: ignore[assignment]
    QVideoSink = None  # type: ignore[assignment]
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

    def __init__(
        self,
        connection_string: str,
        baudrate: int = 57600,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.connection_string = connection_string
        self.baudrate = int(baudrate)
        self._stop_requested = False
        self._vehicle: Any = None
        self._state = TelemetryState()
        self._commands: queue.Queue[tuple[str, Any]] = queue.Queue()
        self._pending_commands: dict[int, str] = {}
        # Track which core telemetry messages are actually arriving.  A link can
        # have a healthy HEARTBEAT while GPS/battery/VFR messages are not being
        # streamed yet, especially on a fresh serial/MAVProxy route.
        self._seen_messages: set[str] = set()

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
            connection_kwargs: dict[str, Any] = {
                "source_system": 254,
                "source_component": mavutil.mavlink.MAV_COMP_ID_MISSIONPLANNER,
            }
            # PyMAVLink needs an explicit baud rate for direct COM/serial links.
            if not self.connection_string.lower().startswith(("udp", "tcp")):
                connection_kwargs["baud"] = self.baudrate
            self._vehicle = mavutil.mavlink_connection(
                self.connection_string,
                **connection_kwargs,
            )

            self.link_status.emit("connecting", "Waiting for flight controller")
            heartbeat = self._wait_for_autopilot_heartbeat(timeout=20.0)
            if heartbeat is None:
                raise TimeoutError(
                    "No autopilot heartbeat received for 20 seconds. "
                    "Check the selected transport, address/port, COM port and baud rate."
                )

            if self._stop_requested:
                return

            self._state.connected = True
            self._parse_message(heartbeat)
            # Do not assume the autopilot is already streaming every field the
            # GUI needs. Explicitly request the core MAVLink telemetry used by
            # the PFD, QUICK page, fleet cards and map.
            self._request_telemetry_streams()
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
            last_stream_request = time.monotonic()
            stale_announced = False

            while not self._stop_requested:
                now = time.monotonic()
                self._process_commands()

                if now - last_gcs_heartbeat >= 1.0:
                    self._send_gcs_heartbeat()
                    last_gcs_heartbeat = now

                # If the link is alive but important telemetry is still absent,
                # re-issue the stream request at a low rate.  This fixes the
                # common "connected but values stay --/0" case.
                core_messages = {"ATTITUDE", "GLOBAL_POSITION_INT", "SYS_STATUS", "VFR_HUD"}
                if now - last_stream_request >= 5.0 and not core_messages.issubset(self._seen_messages):
                    self._request_telemetry_streams()
                    last_stream_request = now

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

    def _request_telemetry_streams(self) -> None:
        """Request the telemetry used by every live GCS display.

        HEARTBEAT alone only proves that the MAVLink route is alive.  This method
        requests the actual attitude/GPS/battery/speed messages required by the
        PFD, QUICK parameters, fleet cards and map.  It works through MAVProxy
        as well as on a direct serial connection.
        """
        if self._vehicle is None or self._state.system_id <= 0:
            return
        system_id, component_id = self._target_ids()

        # Modern MAVLink message-interval requests. Values are intentionally
        # moderate so multiple vehicles do not flood a telemetry radio.
        requested = (
            ("ATTITUDE", 10.0),
            ("GLOBAL_POSITION_INT", 5.0),
            ("GPS_RAW_INT", 2.0),
            ("SYS_STATUS", 2.0),
            ("BATTERY_STATUS", 2.0),
            ("VFR_HUD", 5.0),
            ("HOME_POSITION", 1.0),
        )
        for message_name, hz in requested:
            msg_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{message_name}", None)
            if msg_id is None:
                continue
            try:
                interval_us = int(1_000_000.0 / hz)
                self._vehicle.mav.command_long_send(
                    system_id,
                    component_id,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    float(msg_id),
                    float(interval_us),
                    0.0, 0.0, 0.0, 0.0, 0.0,
                )
            except Exception:
                pass

        # Compatibility fallback used by older ArduPilot/MAVLink stacks.
        try:
            self._vehicle.mav.request_data_stream_send(
                system_id,
                component_id,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                10,
                1,
            )
        except Exception:
            pass

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

        # Once this worker has locked onto an FC heartbeat, never mix telemetry
        # from another SYSID into this vehicle's state.  This is important when
        # several FCs are active on the same PC/network.
        try:
            src_system = int(message.get_srcSystem())
        except Exception:
            src_system = 0
        if self._state.system_id > 0 and src_system > 0 and src_system != self._state.system_id:
            return

        self._seen_messages.add(str(message_type))
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

            # GLOBAL_POSITION_INT already carries NED velocity in cm/s. Use it
            # as a robust source for ground speed and vertical speed so those
            # fields continue updating even if VFR_HUD is not streamed.
            try:
                vx = float(getattr(message, "vx", 0.0)) / 100.0
                vy = float(getattr(message, "vy", 0.0)) / 100.0
                vz = float(getattr(message, "vz", 0.0)) / 100.0
                self._state.groundspeed_mps = math.hypot(vx, vy)
                self._state.climb_mps = -vz
            except Exception:
                pass

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


class PrimaryFlightDisplay(QWidget):
    """Integrated Mission-Planner-style primary flight display.

    Combines attitude, heading, airspeed, altitude, armed state, mode,
    battery, GPS and basic motion data in one compact, resizable widget.
    The display is intentionally self-painted with QPainter so it stays
    responsive and does not depend on image assets.
    """

    def __init__(self) -> None:
        super().__init__()
        self._state = TelemetryState()
        self.setMinimumSize(300, 225)
        # The PFD must be able to grow in BOTH directions.  The older build
        # capped it at 285 px and used a vertically Fixed size policy, which
        # prevented the splitter from letting the operator drag it downward.
        self.setMaximumHeight(16777215)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setToolTip("Primary flight display - selected vehicle")

    def set_state(self, state: TelemetryState) -> None:
        self._state = state
        self.update()

    @staticmethod
    def _wrap_heading(value: float) -> float:
        return value % 360.0

    @staticmethod
    def _heading_label(value: int) -> str:
        value %= 360
        cardinal = {0: "N", 90: "E", 180: "S", 270: "W"}
        if value in cardinal:
            return cardinal[value]
        return f"{value // 10:02d}"

    @staticmethod
    def _draw_shadow_text(
        p: QPainter,
        rect: QRectF,
        text: str,
        font: QFont,
        color: QColor,
        alignment: Qt.AlignmentFlag = Qt.AlignmentFlag.AlignCenter,
    ) -> None:
        # Fit the font to the available box.  The PFD can become much taller
        # without becoming wider; scaling fonts from height alone caused words
        # such as DISARMED to be clipped to "ISAR".
        fitted = QFont(font)
        point_size = fitted.pointSizeF()
        if point_size <= 0:
            point_size = 10.0
        min_size = 5.5
        while point_size > min_size:
            fitted.setPointSizeF(point_size)
            p.setFont(fitted)
            fm = p.fontMetrics()
            if fm.horizontalAdvance(text) <= rect.width() * 0.92 and fm.height() <= rect.height() * 0.92:
                break
            point_size -= 0.5
        p.setFont(fitted)
        shadow = QRectF(rect)
        shadow.translate(1.2, 1.2)
        p.setPen(QColor(0, 0, 0, 190))
        p.drawText(shadow, alignment, text)
        p.setPen(color)
        p.drawText(rect, alignment, text)

    def paintEvent(self, event: Any) -> None:
        from PySide6.QtGui import QPainterPath

        s = self._state
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setRenderHint(QPainter.RenderHint.TextAntialiasing)

        outer = QRectF(self.rect()).adjusted(1.5, 1.5, -1.5, -1.5)
        p.setPen(QPen(QColor("#4a5459"), 1.4))
        p.setBrush(QColor("#0a0d0f"))
        p.drawRoundedRect(outer, 7, 7)

        viewport = outer.adjusted(3, 3, -3, -3)
        path = QPainterPath()
        path.addRoundedRect(viewport, 5, 5)
        p.save()
        p.setClipPath(path)

        w = viewport.width()
        h = viewport.height()
        left = viewport.left()
        top = viewport.top()
        right = viewport.right()
        bottom = viewport.bottom()
        cx = viewport.center().x()

        # Typography follows the smaller of width/height, not height alone.
        # Therefore making the PFD taller adds viewing area without exploding
        # text size or pushing labels out of their fixed instrument boxes.
        ui_scale = max(0.82, min(1.18, min(w / 360.0, h / 270.0)))

        def fs(base: float, minimum: int, maximum: int) -> int:
            return max(minimum, min(maximum, int(round(base * ui_scale))))

        heading_h = 30.0
        footer_h = 31.0
        tape_top = top + heading_h + 40.0
        tape_bottom = bottom - footer_h - 5.0
        horizon_center_y = top + heading_h + (h - heading_h - footer_h) * 0.54

        # ------------------------------------------------------------------
        # Moving attitude background
        # ------------------------------------------------------------------
        p.save()
        p.translate(cx, horizon_center_y)
        p.rotate(-float(s.roll_deg))
        pitch_px_per_deg = max(2.3, min(3.2, h / 85.0))
        p.translate(0, float(s.pitch_deg) * pitch_px_per_deg)

        world_w = max(900.0, w * 3.0)
        world_h = max(900.0, h * 4.0)
        p.fillRect(QRectF(-world_w / 2, -world_h / 2, world_w, world_h / 2), QColor("#4d8fc9"))
        p.fillRect(QRectF(-world_w / 2, 0, world_w, world_h / 2), QColor("#667b24"))

        # Soft horizon band for stronger visual separation.
        p.fillRect(QRectF(-world_w / 2, -2.0, world_w, 4.0), QColor("#f0f3f5"))

        # Pitch ladder every 5 degrees, numbers every 10 degrees.
        p.setFont(QFont("Segoe UI", fs(8, 6, 11), QFont.Weight.DemiBold))
        for degree in range(-40, 45, 5):
            if degree == 0:
                continue
            y = -degree * pitch_px_per_deg
            major = degree % 10 == 0
            half = 34.0 if major else 21.0
            inner_gap = 8.0
            p.setPen(QPen(QColor(245, 247, 248, 235), 1.5 if major else 1.0))
            p.drawLine(QPointF(-half, y), QPointF(-inner_gap, y))
            p.drawLine(QPointF(inner_gap, y), QPointF(half, y))
            if major:
                label = str(abs(degree))
                p.setPen(QColor("#f6f7f8"))
                p.drawText(QRectF(-half - 29, y - 9, 25, 18), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, label)
                p.drawText(QRectF(half + 4, y - 9, 25, 18), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, label)
        p.restore()

        # ------------------------------------------------------------------
        # Heading tape
        # ------------------------------------------------------------------
        heading_rect = QRectF(left, top, w, heading_h)
        p.fillRect(heading_rect, QColor(16, 21, 24, 225))
        p.setPen(QPen(QColor("#7e8b91"), 1.0))
        p.drawLine(QPointF(left, top + heading_h), QPointF(right, top + heading_h))

        heading = 0.0 if s.heading_deg is None else self._wrap_heading(float(s.heading_deg))
        px_per_deg = max(2.6, min(4.0, w / 95.0))
        visible_deg = int(w / px_per_deg / 2) + 8
        start_deg = int(math.floor((heading - visible_deg) / 5.0) * 5)
        end_deg = int(math.ceil((heading + visible_deg) / 5.0) * 5)
        p.setFont(QFont("Segoe UI", fs(7, 6, 10), QFont.Weight.DemiBold))
        for unwrapped in range(start_deg, end_deg + 1, 5):
            value = unwrapped % 360
            delta = ((unwrapped - heading + 540.0) % 360.0) - 180.0
            x = cx + delta * px_per_deg
            if x < left - 8 or x > right + 8:
                continue
            major = value % 30 == 0
            tick_h = 10 if major else 6
            p.setPen(QPen(QColor("#d6dde1"), 1.1))
            p.drawLine(QPointF(x, top + heading_h), QPointF(x, top + heading_h - tick_h))
            if major:
                label = self._heading_label(value)
                color = QColor("#ff6b5e") if label == "N" else QColor("#ecf1f3")
                self._draw_shadow_text(
                    p,
                    QRectF(x - 18, top + 2, 36, 16),
                    label,
                    QFont("Segoe UI", fs(8, 7, 11), QFont.Weight.Bold),
                    color,
                )

        # Heading bug and digital readout.
        p.setBrush(QColor("#f6c344"))
        p.setPen(Qt.PenStyle.NoPen)
        p.drawPolygon(QPolygonF([
            QPointF(cx, top + heading_h + 1),
            QPointF(cx - 5.5, top + heading_h - 8),
            QPointF(cx + 5.5, top + heading_h - 8),
        ]))
        p.setBrush(QColor(10, 13, 15, 225))
        p.setPen(QPen(QColor("#6c777c"), 1.0))
        heading_box = QRectF(cx - 25, top + 1.5, 50, 20)
        p.drawRoundedRect(heading_box, 3, 3)
        self._draw_shadow_text(
            p,
            heading_box,
            f"{heading:03.0f}",
            QFont("Consolas", fs(10, 8, 13), QFont.Weight.Bold),
            QColor("#ffffff"),
        )

        # ------------------------------------------------------------------
        # Roll scale
        # ------------------------------------------------------------------
        roll_center = QPointF(cx, top + heading_h + 62)
        roll_radius = min(72.0, w * 0.23)
        for angle in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            rad = math.radians(angle - 90)
            outer_r = roll_radius
            inner_r = roll_radius - (10 if angle in (-60, -30, 0, 30, 60) else 7)
            p1 = QPointF(roll_center.x() + math.cos(rad) * inner_r, roll_center.y() + math.sin(rad) * inner_r)
            p2 = QPointF(roll_center.x() + math.cos(rad) * outer_r, roll_center.y() + math.sin(rad) * outer_r)
            p.setPen(QPen(QColor(242, 245, 246, 220), 1.2))
            p.drawLine(p1, p2)
            if angle != 0 and abs(angle) in (20, 30, 45, 60):
                label_r = roll_radius + 10
                lx = roll_center.x() + math.cos(rad) * label_r - 10
                ly = roll_center.y() + math.sin(rad) * label_r - 8
                p.setFont(QFont("Segoe UI", fs(7, 6, 9), QFont.Weight.DemiBold))
                p.setPen(QColor(235, 239, 241, 215))
                p.drawText(QRectF(lx, ly, 20, 16), Qt.AlignmentFlag.AlignCenter, str(abs(angle)))

        # Fixed roll reference caret.
        p.setBrush(QColor("#ff645e"))
        p.setPen(QPen(QColor("#371515"), 0.7))
        p.drawPolygon(QPolygonF([
            QPointF(cx, roll_center.y() - roll_radius - 4),
            QPointF(cx - 5, roll_center.y() - roll_radius + 5),
            QPointF(cx + 5, roll_center.y() - roll_radius + 5),
        ]))

        # ------------------------------------------------------------------
        # Airspeed and altitude tapes
        # ------------------------------------------------------------------
        tape_w = max(43.0, min(52.0, w * 0.15))
        speed_rect = QRectF(left, tape_top, tape_w, max(60.0, tape_bottom - tape_top))
        alt_rect = QRectF(right - tape_w, tape_top, tape_w, max(60.0, tape_bottom - tape_top))
        # Airspeed/altitude tapes are intentionally COLORLESS / TRANSPARENT.
        # Do not fill the tape rectangles: the sky/ground attitude background must
        # remain fully visible behind the scales.  Keep only a very subtle guide
        # edge so the operator can still recognize the tape boundaries.
        p.setBrush(Qt.BrushStyle.NoBrush)
        for tape in (speed_rect, alt_rect):
            p.setPen(QPen(QColor(255, 255, 255, 52), 0.8))
            p.drawRect(tape)

        airspeed = 0.0 if s.airspeed_mps is None else max(0.0, float(s.airspeed_mps))
        altitude = 0.0 if s.relative_altitude_m is None else float(s.relative_altitude_m)
        tape_center_y = (tape_top + tape_bottom) / 2.0

        # Airspeed ticks: 2 m/s spacing, labels every 4 m/s.
        speed_px = max(9.0, h / 21.0)
        speed_start = int(math.floor(airspeed / 2.0) * 2 - 12)
        speed_end = speed_start + 28
        p.setFont(QFont("Consolas", fs(7, 6, 9), QFont.Weight.DemiBold))
        for value in range(speed_start, speed_end + 1, 2):
            if value < 0:
                continue
            y = tape_center_y - (value - airspeed) / 2.0 * speed_px
            if y < tape_top or y > tape_bottom:
                continue
            major = value % 4 == 0
            p.setPen(QPen(QColor("#eef2f3"), 1.0))
            p.drawLine(QPointF(speed_rect.right() - (15 if major else 9), y), QPointF(speed_rect.right(), y))
            if major:
                p.drawText(QRectF(speed_rect.left() + 2, y - 8, tape_w - 18, 16), Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter, str(value))

        speed_box = QRectF(speed_rect.left() - 1, tape_center_y - 13, tape_w + 9, 26)
        # Current airspeed readout is also transparent: no fill, only a thin
        # dim-white outline so nothing behind it is hidden.
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(QColor(255, 255, 255, 125), 1.1))
        p.drawRoundedRect(speed_box, 2, 2)
        p.setPen(QColor("#ffffff"))
        p.setFont(QFont("Consolas", fs(10, 8, 13), QFont.Weight.Bold))
        p.drawText(speed_box.adjusted(3, 0, -3, 0), Qt.AlignmentFlag.AlignCenter, f"{airspeed:0.1f}")
        p.setFont(QFont("Segoe UI", fs(6, 5, 8), QFont.Weight.Bold))
        p.setPen(QColor("#d7e0e3"))
        p.drawText(QRectF(speed_rect.left() + 2, speed_rect.top() + 3, tape_w - 4, 13), Qt.AlignmentFlag.AlignLeft, "AS")

        # Altitude ticks: adaptive interval.
        alt_step = 5.0 if abs(altitude) < 200 else 10.0
        alt_px = max(9.0, h / 21.0)
        alt_base = math.floor(altitude / alt_step) * alt_step
        for i in range(-8, 9):
            value = alt_base + i * alt_step
            y = tape_center_y - (value - altitude) / alt_step * alt_px
            if y < tape_top or y > tape_bottom:
                continue
            major = i % 2 == 0
            p.setPen(QPen(QColor("#eef2f3"), 1.0))
            p.drawLine(QPointF(alt_rect.left(), y), QPointF(alt_rect.left() + (15 if major else 9), y))
            if major:
                p.drawText(QRectF(alt_rect.left() + 17, y - 8, tape_w - 19, 16), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, f"{value:.0f}")

        alt_box = QRectF(alt_rect.left() - 8, tape_center_y - 13, tape_w + 9, 26)
        # Current altitude readout is transparent as well.
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(QColor(255, 255, 255, 125), 1.1))
        p.drawRoundedRect(alt_box, 2, 2)
        p.setPen(QColor("#ffffff"))
        p.setFont(QFont("Consolas", fs(10, 8, 13), QFont.Weight.Bold))
        p.drawText(alt_box.adjusted(3, 0, -3, 0), Qt.AlignmentFlag.AlignCenter, f"{altitude:0.1f}")
        p.setFont(QFont("Segoe UI", fs(6, 5, 8), QFont.Weight.Bold))
        p.setPen(QColor("#d7e0e3"))
        p.drawText(QRectF(alt_rect.left() + 2, alt_rect.top() + 3, tape_w - 4, 13), Qt.AlignmentFlag.AlignRight, "ALT")

        # ------------------------------------------------------------------
        # Fixed aircraft symbol and arm state
        # ------------------------------------------------------------------
        symbol_y = horizon_center_y + 7
        p.setPen(QPen(QColor("#ffad22"), 3.0, Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap))
        p.drawLine(QPointF(cx - 45, symbol_y), QPointF(cx - 14, symbol_y))
        p.drawLine(QPointF(cx + 14, symbol_y), QPointF(cx + 45, symbol_y))
        p.drawLine(QPointF(cx - 14, symbol_y), QPointF(cx, symbol_y + 7))
        p.drawLine(QPointF(cx + 14, symbol_y), QPointF(cx, symbol_y + 7))
        p.drawLine(QPointF(cx, symbol_y + 7), QPointF(cx, symbol_y + 14))

        state_text = "ARMED" if s.armed else "DISARMED"
        state_color = QColor("#78e05f") if s.armed else QColor("#ff4e4e")
        state_box_w = max(132.0, min(190.0, w - (2.0 * tape_w) - 36.0))
        state_box = QRectF(cx - state_box_w / 2.0, horizon_center_y - 46, state_box_w, 28)
        self._draw_shadow_text(
            p,
            state_box,
            state_text,
            QFont("Segoe UI", fs(15, 10, 18), QFont.Weight.Black),
            state_color,
        )

        # Selected vehicle flight mode.
        if s.connected:
            mode_text = s.mode.replace("_", " ")
            self._draw_shadow_text(
                p,
                QRectF(right - 128, bottom - footer_h - 28, 120, 21),
                mode_text,
                QFont("Segoe UI", fs(9, 7, 12), QFont.Weight.Bold),
                QColor("#f0f3f5"),
                Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            )

        # ------------------------------------------------------------------
        # Footer status strip
        # ------------------------------------------------------------------
        footer = QRectF(left, bottom - footer_h, w, footer_h)
        p.fillRect(footer, QColor(9, 12, 14, 235))
        p.setPen(QPen(QColor("#515b60"), 1.0))
        p.drawLine(QPointF(left, footer.top()), QPointF(right, footer.top()))

        battery = "BAT --"
        if s.battery_voltage_v is not None:
            battery = f"BAT {s.battery_voltage_v:.1f}V"
            if s.battery_remaining_pct is not None:
                battery += f" {s.battery_remaining_pct}%"
        gps_name = GPS_FIX_NAMES.get(s.gps_fix, "--")
        sats = "--" if s.satellites is None else str(s.satellites)
        gs = 0.0 if s.groundspeed_mps is None else float(s.groundspeed_mps)
        climb = 0.0 if s.climb_mps is None else float(s.climb_mps)
        footer_text = f"{battery}     GPS {gps_name} · {sats}     GS {gs:.1f}     V/S {climb:+.1f}"
        p.setPen(QColor("#d4dde1"))
        p.setFont(QFont("Consolas", fs(7, 6, 9), QFont.Weight.DemiBold))
        p.drawText(footer.adjusted(8, 0, -8, 0), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, footer_text)

        # Link-state indicator at upper-left under heading tape.
        link_color = QColor("#67d34e") if s.connected else QColor("#d84a4a")
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(link_color)
        p.drawEllipse(QPointF(left + 11, top + heading_h + 12), 3.5, 3.5)
        p.setPen(QColor("#edf2f4"))
        p.setFont(QFont("Segoe UI", fs(7, 6, 9), QFont.Weight.Bold))
        p.drawText(QRectF(left + 18, top + heading_h + 4, 70, 18), Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter, "LIVE" if s.connected else "OFFLINE")

        p.restore()

        # Crisp outer border above clipped content.
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.setPen(QPen(QColor("#566168"), 1.2))
        p.drawRoundedRect(viewport, 5, 5)


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
        self.pfd = PrimaryFlightDisplay()

        self.altitude = MetricRow("Altitude", unit="m")
        self.latitude = MetricRow("Latitude")
        self.longitude = MetricRow("Longitude")
        self.satellites = MetricRow("Satellites")
        self.home_distance = MetricRow("Home distance", unit="m")
        self.airspeed = MetricRow("Air speed", unit="m/s")
        self.groundspeed = MetricRow("Ground speed", unit="m/s")
        self.climb = MetricRow("Vertical speed", unit="m/s")

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(6)
        layout.addWidget(self.pfd, 0)

        separator = QFrame()
        separator.setFrameShape(QFrame.Shape.HLine)
        separator.setObjectName("separator")
        layout.addWidget(separator)

        parameter_title = QLabel("VEHICLE PARAMETERS")
        parameter_title.setObjectName("panelSectionTitle")
        layout.addWidget(parameter_title)

        for row in (
            self.altitude,
            self.latitude,
            self.longitude,
            self.satellites,
            self.home_distance,
            self.airspeed,
            self.groundspeed,
            self.climb,
        ):
            layout.addWidget(row)

        layout.addStretch(1)

    def update_state(self, s: TelemetryState) -> None:
        self.pfd.set_state(s)
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
        self.groundspeed.set_value("--" if s.groundspeed_mps is None else f"{s.groundspeed_mps:.1f}")
        self.climb.set_value("--" if s.climb_mps is None else f"{s.climb_mps:.1f}")


class VideoDisplayLabel(QLabel):
    """Frame display used by the large monitor and the live feed thumbnails."""

    clicked = Signal()

    def __init__(self, placeholder: str = "NO VIDEO") -> None:
        super().__init__(placeholder)
        self._image: Any = None
        self.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_frame(self, image: Any) -> None:
        self._image = image
        self.setText("")
        self._render_frame()

    def clear_frame(self, text: str = "NO VIDEO") -> None:
        self._image = None
        self.clear()
        self.setText(text)

    def _render_frame(self) -> None:
        if self._image is None:
            return
        try:
            pixmap = QPixmap.fromImage(self._image)
            if pixmap.isNull():
                return
            target = self.size()
            if target.width() > 2 and target.height() > 2:
                pixmap = pixmap.scaled(
                    target,
                    Qt.AspectRatioMode.KeepAspectRatio,
                    Qt.TransformationMode.SmoothTransformation,
                )
            self.setPixmap(pixmap)
        except Exception:
            pass

    def resizeEvent(self, event: Any) -> None:
        super().resizeEvent(event)
        self._render_frame()

    def mousePressEvent(self, event: Any) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit()
        super().mousePressEvent(event)


class VideoFeedCard(QFrame):
    """One live UVC feed shown as a thumbnail in the horizontal feed strip."""

    def __init__(self, owner: "MultiVideoPanel", feed_number: int) -> None:
        super().__init__()
        self.owner = owner
        self.feed_number = feed_number
        self._devices: list[Any] = []
        self._camera: Any = None
        self._session: Any = None
        self._sink: Any = None
        self._active_device_id: bytes | None = None
        self._last_image: Any = None
        self._last_thumb_update = 0.0
        self._selected = False

        self.setObjectName("videoFeedCard")
        self.setProperty("feedSelected", False)
        self.setMinimumWidth(275)
        self.setMaximumWidth(340)
        self.setMinimumHeight(185)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.setSpacing(6)

        header = QHBoxLayout()
        header.setContentsMargins(0, 0, 0, 0)
        header.setSpacing(5)

        self.title = QLabel(f"FEED {feed_number}")
        self.title.setObjectName("videoFeedTitle")
        header.addWidget(self.title)
        header.addStretch(1)

        self.start_button = QPushButton("START")
        self.start_button.setObjectName("videoStartButton")
        self.start_button.setFixedWidth(62)
        self.start_button.clicked.connect(self.toggle_video)
        header.addWidget(self.start_button)

        self.remove_button = QPushButton("×")
        self.remove_button.setObjectName("videoRemoveButton")
        self.remove_button.setToolTip("Remove this feed")
        self.remove_button.setFixedWidth(30)
        self.remove_button.clicked.connect(lambda: self.owner.remove_feed(self))
        header.addWidget(self.remove_button)
        outer.addLayout(header)

        self.device_combo = QComboBox()
        self.device_combo.setObjectName("videoDeviceCombo")
        self.device_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.device_combo.currentIndexChanged.connect(self._selection_changed)
        outer.addWidget(self.device_combo)

        self.thumbnail = VideoDisplayLabel("CLICK TO VIEW")
        self.thumbnail.setObjectName("videoThumbnail")
        self.thumbnail.setMinimumHeight(96)
        self.thumbnail.setMaximumHeight(118)
        self.thumbnail.setCursor(Qt.CursorShape.PointingHandCursor)
        self.thumbnail.clicked.connect(lambda: self.owner.select_feed(self))
        outer.addWidget(self.thumbnail, 1)

        self.status = QLabel("Select a video source")
        self.status.setObjectName("videoCardStatus")
        self.status.setWordWrap(True)
        self.status.setAlignment(Qt.AlignmentFlag.AlignCenter)
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

    @property
    def is_running(self) -> bool:
        return self._camera is not None

    @property
    def last_image(self) -> Any:
        return self._last_image

    def selected_device(self) -> Any | None:
        index = self.device_combo.currentIndex()
        if 0 <= index < len(self._devices):
            return self._devices[index]
        return None

    def selected_device_name(self) -> str:
        device = self.selected_device()
        if device is None:
            return "No video source"
        try:
            return str(device.description())
        except Exception:
            return "Video source"

    def set_feed_number(self, number: int) -> None:
        self.feed_number = number
        self.title.setText(f"FEED {number}")

    def set_remove_enabled(self, enabled: bool) -> None:
        self.remove_button.setEnabled(enabled)

    def set_selected(self, selected: bool) -> None:
        self._selected = selected
        self.setProperty("feedSelected", selected)
        self.style().unpolish(self)
        self.style().polish(self)
        self.update()

    def refresh_devices(self, devices: list[Any]) -> None:
        previous_id: bytes | None = None
        current = self.device_combo.currentIndex()
        if 0 <= current < len(self._devices):
            previous_id = self._device_id(self._devices[current])

        self._devices = list(devices)
        self.device_combo.blockSignals(True)
        self.device_combo.clear()
        for index, device in enumerate(self._devices):
            try:
                name = str(device.description())
            except Exception:
                name = f"Camera {index + 1}"
            self.device_combo.addItem(name)

        selected = -1
        if previous_id is not None:
            for index, device in enumerate(self._devices):
                if self._device_id(device) == previous_id:
                    selected = index
                    break

        if selected < 0 and self._devices:
            selected = min(max(0, self.feed_number - 1), len(self._devices) - 1)
            if self.feed_number == 1:
                for index, device in enumerate(self._devices):
                    try:
                        lower = str(device.description()).lower()
                    except Exception:
                        continue
                    if any(token in lower for token in ("skydroid", "uvc", "fpv", "receiver")):
                        selected = index
                        break

        if selected >= 0:
            self.device_combo.setCurrentIndex(selected)
        self.device_combo.blockSignals(False)

        if not self._devices:
            self.stop_video(clear_status=False)
            self.device_combo.setEnabled(False)
            self.start_button.setEnabled(False)
            self.thumbnail.clear_frame("NO SOURCE")
            self.status.setText("No Windows UVC/video input detected")
            return

        self.device_combo.setEnabled(True)
        self.start_button.setEnabled(True)
        if self._camera is None:
            self.thumbnail.clear_frame("CLICK TO VIEW")
            self.status.setText(f"Ready · {self.selected_device_name()}")

    def _selection_changed(self, _index: int) -> None:
        if self._camera is not None:
            self.start_video()
        else:
            self.status.setText(f"Ready · {self.selected_device_name()}")
        if self._selected:
            self.owner.update_main_context(self)

    def _best_format(self, device: Any) -> Any | None:
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
        if not MULTIMEDIA_AVAILABLE or QCamera is None or QMediaCaptureSession is None or QVideoSink is None:
            self.status.setText("Qt video-sink support is unavailable")
            return

        device = self.selected_device()
        if device is None:
            self.status.setText("Select a video source")
            return

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
            sink = QVideoSink(self)
            session.setCamera(camera)
            session.setVideoSink(sink)
            sink.videoFrameChanged.connect(self._on_video_frame)

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
            self._sink = sink
            self._active_device_id = device_id
            self.start_button.setText("STOP")
            self.thumbnail.clear_frame("STARTING...")

            name = self.selected_device_name()
            if chosen_format is not None:
                try:
                    size = chosen_format.resolution()
                    fps = chosen_format.maxFrameRate()
                    self.status.setText(f"LIVE · {size.width()}×{size.height()} @ {fps:.0f} fps")
                except Exception:
                    self.status.setText(f"LIVE · {name}")
            else:
                self.status.setText(f"LIVE · {name}")
            camera.start()

            if self._selected:
                self.owner.update_main_context(self)
        except Exception as error:
            self._camera = None
            self._session = None
            self._sink = None
            self._active_device_id = None
            self.start_button.setText("START")
            self.thumbnail.clear_frame("VIDEO ERROR")
            self.status.setText(f"Video start failed: {error}")
            if self._selected:
                self.owner.card_stopped(self, f"Video start failed: {error}")

    def _on_video_frame(self, frame: Any) -> None:
        try:
            image = frame.toImage()
            if image is None or image.isNull():
                return
        except Exception:
            return

        self._last_image = image
        now = time.monotonic()
        # Thumbnail previews are intentionally limited to ~10 fps to keep CPU use modest.
        if now - self._last_thumb_update >= 0.10:
            self.thumbnail.set_frame(image)
            self._last_thumb_update = now

        if self._selected:
            self.owner.display_main_frame(self, image)

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

        sink = self._sink
        self._sink = None
        if sink is not None:
            try:
                sink.deleteLater()
            except Exception:
                pass

        self._last_image = None
        self.start_button.setText("START")
        self.thumbnail.clear_frame("CLICK TO VIEW")
        if clear_status and self._devices:
            self.status.setText(f"Stopped · {self.selected_device_name()}")
        if self._selected:
            self.owner.card_stopped(self, "Selected feed is stopped")

    def toggle_video(self) -> None:
        if self._camera is None:
            self.start_video()
        else:
            self.stop_video()

    def _camera_error(self, _error: Any, error_string: str) -> None:
        self._active_device_id = None
        self.status.setText(f"VIDEO ERROR · {error_string}")
        self.thumbnail.clear_frame("VIDEO ERROR")
        self.start_button.setText("START")
        if self._selected:
            self.owner.card_stopped(self, f"VIDEO ERROR · {error_string}")

    def _camera_active_changed(self, active: bool) -> None:
        if not active and self._camera is not None:
            if not self.status.text().startswith("VIDEO ERROR"):
                self.status.setText("Video source stopped")

    def shutdown(self) -> None:
        self.stop_video(clear_status=False)


class MultiVideoPanel(QFrame):
    """Large selected FPV view with live horizontal thumbnail feeds below it."""

    def __init__(self) -> None:
        super().__init__()
        self.setObjectName("videoWorkspace")
        self.cards: list[VideoFeedCard] = []
        self._media_devices: Any = None
        self._active_card: VideoFeedCard | None = None
        self._auto_started = False

        outer = QVBoxLayout(self)
        outer.setContentsMargins(10, 10, 10, 10)
        outer.setSpacing(9)

        # Main selected feed ------------------------------------------------
        self.main_frame = QFrame()
        self.main_frame.setObjectName("mainVideoFrame")
        main_layout = QVBoxLayout(self.main_frame)
        main_layout.setContentsMargins(10, 9, 10, 10)
        main_layout.setSpacing(7)

        main_header = QHBoxLayout()
        self.main_title = QLabel("MAIN FPV VIEW")
        self.main_title.setObjectName("mainVideoTitle")
        main_header.addWidget(self.main_title)

        self.main_source = QLabel("No feed selected")
        self.main_source.setObjectName("mainVideoSource")
        main_header.addWidget(self.main_source)
        main_header.addStretch(1)
        main_layout.addLayout(main_header)

        self.main_display = VideoDisplayLabel("SELECT A FEED BELOW")
        self.main_display.setObjectName("mainFpvVideo")
        self.main_display.setMinimumHeight(360)
        main_layout.addWidget(self.main_display, 1)

        self.main_status = QLabel("Click any feed below to display it here")
        self.main_status.setObjectName("mainVideoStatus")
        main_layout.addWidget(self.main_status)
        outer.addWidget(self.main_frame, 1)

        # Feed slider -------------------------------------------------------
        strip_header = QHBoxLayout()
        strip_header.setContentsMargins(2, 0, 2, 0)
        strip_header.setSpacing(7)

        title = QLabel("VIDEO FEEDS")
        title.setObjectName("videoWorkspaceTitle")
        strip_header.addWidget(title)

        self.source_count = QLabel("0 sources")
        self.source_count.setObjectName("videoWorkspaceInfo")
        strip_header.addWidget(self.source_count)
        strip_header.addStretch(1)

        self.refresh_button = QPushButton("REFRESH SOURCES")
        self.refresh_button.setObjectName("compactButton")
        self.refresh_button.clicked.connect(self.refresh_all_devices)
        strip_header.addWidget(self.refresh_button)

        self.add_button = QPushButton("+ ADD FEED")
        self.add_button.setObjectName("primaryButton")
        self.add_button.clicked.connect(self.add_feed)
        strip_header.addWidget(self.add_button)
        outer.addLayout(strip_header)

        self.scroll = QScrollArea()
        self.scroll.setObjectName("videoScrollArea")
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scroll.setMinimumHeight(205)
        self.scroll.setMaximumHeight(240)

        self.container = QWidget()
        self.container.setObjectName("videoFeedContainer")
        self.feed_layout = QHBoxLayout(self.container)
        self.feed_layout.setContentsMargins(0, 0, 0, 0)
        self.feed_layout.setSpacing(8)
        self.feed_layout.addStretch(1)
        self.scroll.setWidget(self.container)
        outer.addWidget(self.scroll, 0)

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

        if not devices:
            self.main_display.clear_frame("NO VIDEO SOURCES")
            self.main_source.setText("No video source")
            self.main_status.setText("No Windows UVC/video input detected")
            return

        # Start and select FEED 1 when the camera window first opens.
        if not self._auto_started and self.cards:
            self._auto_started = True
            QTimer.singleShot(350, lambda: self.select_feed(self.cards[0]))

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
            card.status.setText("At least one feed is kept open")
            return

        was_active = card is self._active_card
        card.shutdown()
        self.cards.remove(card)
        self.feed_layout.removeWidget(card)
        card.deleteLater()
        self._renumber_cards()

        if was_active:
            self._active_card = None
            if self.cards:
                QTimer.singleShot(0, lambda: self.select_feed(self.cards[0]))

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

    def select_feed(self, card: VideoFeedCard) -> None:
        if card not in self.cards:
            return

        for item in self.cards:
            item.set_selected(item is card)
        self._active_card = card
        self.update_main_context(card)

        if card.last_image is not None:
            self.main_display.set_frame(card.last_image)
        else:
            self.main_display.clear_frame("STARTING FEED...")

        # Selection is also the start action requested by the operator.
        if not card.is_running:
            card.start_video()

    def update_main_context(self, card: VideoFeedCard) -> None:
        if card is not self._active_card:
            return
        self.main_title.setText(f"MAIN FPV VIEW · FEED {card.feed_number}")
        self.main_source.setText(card.selected_device_name())
        if card.is_running:
            self.main_status.setText(f"LIVE · FEED {card.feed_number}")
        else:
            self.main_status.setText(f"FEED {card.feed_number} selected · starting on demand")

    def display_main_frame(self, card: VideoFeedCard, image: Any) -> None:
        if card is not self._active_card:
            return
        self.main_display.set_frame(image)
        self.main_title.setText(f"MAIN FPV VIEW · FEED {card.feed_number}")
        self.main_source.setText(card.selected_device_name())
        self.main_status.setText(f"LIVE · FEED {card.feed_number}")

    def card_stopped(self, card: VideoFeedCard, message: str) -> None:
        if card is not self._active_card:
            return
        self.main_display.clear_frame("FEED STOPPED")
        self.main_source.setText(card.selected_device_name())
        self.main_status.setText(message)

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


class QuickMetricTile(QFrame):
    """Compact Mission-Planner-style telemetry tile for the QUICK page."""

    def __init__(self, label: str, value_color: str = TEXT) -> None:
        super().__init__()
        self.setObjectName("quickMetricTile")
        self.setMinimumSize(120, 78)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 6)
        layout.setSpacing(2)
        self.label = QLabel(label)
        self.label.setObjectName("quickMetricLabel")
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.label.setWordWrap(True)
        self.value = QLabel("--")
        self.value.setObjectName("quickMetricValue")
        self.value.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.value.setStyleSheet(f"color:{value_color};")
        layout.addWidget(self.label)
        layout.addWidget(self.value, 1)

    def set_value(self, value: str) -> None:
        self.value.setText(value)


class QuickParametersPanel(QWidget):
    """Dense 3 x 3 selected-vehicle telemetry grid."""

    def __init__(self) -> None:
        super().__init__()
        grid = QGridLayout(self)
        grid.setContentsMargins(8, 8, 8, 8)
        grid.setHorizontalSpacing(6)
        grid.setVerticalSpacing(6)
        self.altitude = QuickMetricTile("Altitude (m)", "#cda6f7")
        self.groundspeed = QuickMetricTile("Ground Speed (m/s)", "#f2b38a")
        self.airspeed = QuickMetricTile("Air Speed (m/s)", "#9fbbe5")
        self.home_distance = QuickMetricTile("Dist to Home (m)", "#ef9b9b")
        self.heading = QuickMetricTile("Heading (deg)", "#72dfa0")
        self.climb = QuickMetricTile("Vertical Speed (m/s)", "#e58ce4")
        self.battery = QuickMetricTile("Battery", "#e8e77e")
        self.satellites = QuickMetricTile("Sat Count", "#63d9dd")
        self.hdop = QuickMetricTile("GPS HDOP", "#8bbcff")
        tiles = [
            self.altitude, self.groundspeed, self.airspeed,
            self.home_distance, self.heading, self.climb,
            self.battery, self.satellites, self.hdop,
        ]
        for i, tile in enumerate(tiles):
            grid.addWidget(tile, i // 3, i % 3)
        for col in range(3):
            grid.setColumnStretch(col, 1)
        for row in range(3):
            grid.setRowStretch(row, 1)

    def update_state(self, s: TelemetryState) -> None:
        self.altitude.set_value("--" if s.relative_altitude_m is None else f"{s.relative_altitude_m:.2f}")
        self.groundspeed.set_value("--" if s.groundspeed_mps is None else f"{s.groundspeed_mps:.2f}")
        self.airspeed.set_value("--" if s.airspeed_mps is None else f"{s.airspeed_mps:.2f}")
        home_m = distance_m(s.latitude, s.longitude, s.home_latitude, s.home_longitude)
        self.home_distance.set_value("--" if home_m is None else f"{home_m:.1f}")
        self.heading.set_value("--" if s.heading_deg is None else f"{s.heading_deg:.1f}")
        self.climb.set_value("--" if s.climb_mps is None else f"{s.climb_mps:.2f}")
        if s.battery_voltage_v is None and s.battery_remaining_pct is None:
            battery_text = "--"
        elif s.battery_voltage_v is None:
            battery_text = f"{s.battery_remaining_pct}%"
        elif s.battery_remaining_pct is None:
            battery_text = f"{s.battery_voltage_v:.1f} V"
        else:
            battery_text = f"{s.battery_voltage_v:.1f}V  {s.battery_remaining_pct}%"
        self.battery.set_value(battery_text)
        self.satellites.set_value("--" if s.satellites is None else str(s.satellites))
        self.hdop.set_value("--" if s.hdop is None else f"{s.hdop:.2f}")


class LeftFlightOperationsPanel(QWidget):
    """Resizable PFD + QUICK / ACTIONS / MESSAGES left-side operator block."""

    def __init__(self, control_panel: ControlPanel, messages_panel: MessagesPanel) -> None:
        super().__init__()
        self.pfd = PrimaryFlightDisplay()
        self.quick_panel = QuickParametersPanel()
        self.control_panel = control_panel
        self.messages_panel = messages_panel

        self.pfd.setMinimumHeight(190)
        self.quick_panel.setMinimumHeight(110)

        self.tabs = QTabWidget()
        self.tabs.setObjectName("operatorTabs")
        self.tabs.setDocumentMode(True)
        self.tabs.addTab(self.quick_panel, "QUICK")
        self.tabs.addTab(self.control_panel, "ACTIONS")
        self.tabs.addTab(self.messages_panel, "MESSAGES")
        self.tabs.setMinimumHeight(120)

        # This splitter is the vertical resize handle the previous layout lacked.
        # The dock border still provides horizontal resizing; this handle lets the
        # operator independently resize the PFD and lower operator workspace.
        self.vertical_splitter = QSplitter(Qt.Orientation.Vertical)
        self.vertical_splitter.setObjectName("leftOperatorSplitter")
        self.vertical_splitter.setChildrenCollapsible(False)
        self.vertical_splitter.setHandleWidth(10)
        self.vertical_splitter.addWidget(self.pfd)
        self.vertical_splitter.addWidget(self.tabs)
        self.vertical_splitter.setStretchFactor(0, 5)
        self.vertical_splitter.setStretchFactor(1, 4)
        self.vertical_splitter.setSizes([430, 360])

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        layout.setSpacing(0)
        layout.addWidget(self.vertical_splitter, 1)

    def update_state(self, s: TelemetryState) -> None:
        self.pfd.set_state(s)
        self.quick_panel.update_state(s)


class AddConnectionDialog(QDialog):
    """Compact transport-aware dialog for creating a new MAVLink vehicle link."""

    def __init__(
        self,
        suggested_number: int,
        suggested_udp_port: int,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setWindowTitle("Add Vehicle Connection")
        self.setModal(True)
        self.setMinimumWidth(430)

        outer = QVBoxLayout(self)
        outer.setContentsMargins(16, 14, 16, 14)
        outer.setSpacing(12)

        title = QLabel("ADD MAVLINK VEHICLE")
        title.setObjectName("dialogTitle")
        outer.addWidget(title)

        form = QFormLayout()
        form.setHorizontalSpacing(14)
        form.setVerticalSpacing(10)

        self.name_input = QLineEdit(f"Vehicle {suggested_number}")
        self.name_input.setPlaceholderText("e.g. Quad 2 or Survey Wing")
        form.addRow("Vehicle name", self.name_input)

        self.transport = QComboBox()
        self.transport.setObjectName("connectionType")
        self.transport.addItems(["UDP Listen", "TCP Client", "Serial / COM"])
        form.addRow("Connection type", self.transport)

        self.host_label = QLabel("Host / bind address")
        self.host_input = QLineEdit("127.0.0.1")
        self.host_input.setPlaceholderText("127.0.0.1")
        form.addRow(self.host_label, self.host_input)

        self.port_label = QLabel("Network port")
        self.port_input = QSpinBox()
        self.port_input.setRange(1, 65535)
        self.port_input.setValue(int(suggested_udp_port))
        form.addRow(self.port_label, self.port_input)

        self.serial_label = QLabel("COM / serial port")
        self.serial_input = QComboBox()
        self.serial_input.setObjectName("serialPortCombo")
        self.serial_input.setEditable(True)
        self.serial_input.setInsertPolicy(QComboBox.InsertPolicy.NoInsert)
        if self.serial_input.lineEdit() is not None:
            self.serial_input.lineEdit().setPlaceholderText("Select connected COM port or type COM20")

        self.serial_refresh_button = QPushButton("REFRESH")
        self.serial_refresh_button.setObjectName("dialogRefreshButton")
        self.serial_refresh_button.setToolTip("Rescan connected Windows serial/COM ports")
        self.serial_refresh_button.clicked.connect(self._refresh_serial_ports)

        self.serial_row = QWidget()
        serial_row_layout = QHBoxLayout(self.serial_row)
        serial_row_layout.setContentsMargins(0, 0, 0, 0)
        serial_row_layout.setSpacing(6)
        serial_row_layout.addWidget(self.serial_input, 1)
        serial_row_layout.addWidget(self.serial_refresh_button)
        form.addRow(self.serial_label, self.serial_row)

        self.baud_label = QLabel("Baud rate")
        self.baud_input = QComboBox()
        self.baud_input.setObjectName("baudCombo")
        # Put the three rates used most often with telemetry radios first.
        self.baud_input.addItems(["57600", "115200", "9600", "38400", "230400", "460800", "921600"])
        self.baud_input.setCurrentText("57600")
        form.addRow(self.baud_label, self.baud_input)

        outer.addLayout(form)

        self.hint = QLabel()
        self.hint.setObjectName("connectionHint")
        self.hint.setWordWrap(True)
        outer.addWidget(self.hint)

        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Cancel | QDialogButtonBox.StandardButton.Ok
        )
        buttons.button(QDialogButtonBox.StandardButton.Ok).setText("ADD & CONNECT")
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        outer.addWidget(buttons)

        self.transport.currentTextChanged.connect(self._sync_transport_fields)
        self._style_combo_popups()
        self._refresh_serial_ports()
        self._sync_transport_fields(self.transport.currentText())

    def _style_combo_popups(self) -> None:
        """Force dark popup lists on Windows instead of the native white menu."""
        popup_style = f"""
            QAbstractItemView {{
                background:{SURFACE_2};
                color:{TEXT};
                selection-background-color:{ACCENT_DARK};
                selection-color:#ffffff;
                border:1px solid {BORDER};
                outline:0;
                padding:3px;
            }}
            QAbstractItemView::item {{
                min-height:28px;
                padding:3px 7px;
            }}
        """
        for combo in (self.transport, self.serial_input, self.baud_input):
            try:
                combo.view().setStyleSheet(popup_style)
            except Exception:
                pass

    def _refresh_serial_ports(self) -> None:
        """Populate the COM selector with ports Windows currently exposes."""
        current_data = self.serial_input.currentData()
        current_text = self.serial_input.currentText().strip()
        current_port = str(current_data).strip() if current_data else current_text.split(" · ", 1)[0].strip()

        discovered: list[tuple[str, str]] = []
        if QT_SERIALPORT_AVAILABLE and QSerialPortInfo is not None:
            try:
                for info in QSerialPortInfo.availablePorts():
                    port = str(info.portName()).strip()
                    description = str(info.description()).strip()
                    if port:
                        discovered.append((port, description))
            except Exception:
                discovered = []

        if not discovered and PYSERIAL_PORTS_AVAILABLE and pyserial_list_ports is not None:
            try:
                for info in pyserial_list_ports.comports():
                    port = str(getattr(info, "device", "")).strip()
                    description = str(getattr(info, "description", "")).strip()
                    if port:
                        discovered.append((port, description))
            except Exception:
                discovered = []

        # Deduplicate while retaining Windows' scan order.
        unique: list[tuple[str, str]] = []
        seen: set[str] = set()
        for port, description in discovered:
            key = port.upper()
            if key not in seen:
                seen.add(key)
                unique.append((port, description))

        self.serial_input.blockSignals(True)
        self.serial_input.clear()
        for port, description in unique:
            label = port if not description else f"{port} · {description}"
            self.serial_input.addItem(label, port)

        selected_index = self.serial_input.findData(current_port) if current_port else -1
        if selected_index >= 0:
            self.serial_input.setCurrentIndex(selected_index)
        elif unique:
            self.serial_input.setCurrentIndex(0)
        else:
            # Keep the field editable even if Windows reports no port yet.
            self.serial_input.setEditText(current_port or "COM20")
        self.serial_input.blockSignals(False)

    def _sync_transport_fields(self, transport: str) -> None:
        serial = transport == "Serial / COM"
        network = not serial
        self.host_label.setVisible(network)
        self.host_input.setVisible(network)
        self.port_label.setVisible(network)
        self.port_input.setVisible(network)
        self.serial_label.setVisible(serial)
        self.serial_row.setVisible(serial)
        self.baud_label.setVisible(serial)
        self.baud_input.setVisible(serial)

        if transport == "UDP Listen":
            self.hint.setText(
                "Use this for MAVProxy/telemetry forwarded to this PC, for example "
                "udpin:127.0.0.1:14552."
            )
        elif transport == "TCP Client":
            self.hint.setText(
                "Connect to a MAVLink TCP server using the remote host and port."
            )
        else:
            self.hint.setText(
                "Direct serial connection. Select the Windows COM port and the telemetry radio baud rate."
            )

    def connection_config(self) -> dict[str, Any]:
        name = self.name_input.text().strip() or "Vehicle"
        transport = self.transport.currentText()
        baudrate = 57600

        if transport == "Serial / COM":
            selected_port = self.serial_input.currentData()
            if selected_port:
                endpoint = str(selected_port).strip()
            else:
                endpoint = self.serial_input.currentText().split(" · ", 1)[0].strip()
            try:
                baudrate = int(self.baud_input.currentText().strip())
            except Exception:
                baudrate = 57600
        else:
            host = self.host_input.text().strip() or "127.0.0.1"
            port = int(self.port_input.value())
            if transport == "TCP Client":
                endpoint = f"tcp:{host}:{port}"
            else:
                endpoint = f"udpin:{host}:{port}"

        return {
            "friendly_name": name,
            "endpoint": endpoint,
            "baudrate": baudrate,
            "transport": transport,
        }


class VehicleSummaryCard(QFrame):
    """Compact status card for one configured aircraft."""

    clicked = Signal(str)
    remove_requested = Signal(str)

    def __init__(self, connection_id: str, friendly_name: str) -> None:
        super().__init__()
        self.connection_id = connection_id
        self.setObjectName("vehicleSummaryCard")
        self.setProperty("selectedVehicle", False)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setMinimumHeight(205)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 9, 10, 9)
        layout.setSpacing(6)

        top = QHBoxLayout()
        self.name_label = QLabel(friendly_name)
        self.name_label.setObjectName("fleetVehicleName")
        self.link_label = QLabel("OFF")
        self.link_label.setObjectName("fleetLinkState")
        self.link_label.setProperty("linkState", "disconnected")
        self.remove_button = QPushButton("REMOVE")
        self.remove_button.setObjectName("fleetRemoveButton")
        self.remove_button.setToolTip("Remove this vehicle connection from the GCS")
        self.remove_button.setFixedWidth(72)
        self.remove_button.clicked.connect(lambda: self.remove_requested.emit(self.connection_id))
        top.addWidget(self.name_label)
        top.addStretch(1)
        top.addWidget(self.link_label)
        top.addWidget(self.remove_button)
        layout.addLayout(top)

        self.identity_label = QLabel("SYS -- · UNKNOWN")
        self.identity_label.setObjectName("fleetIdentity")
        layout.addWidget(self.identity_label)

        grid = QGridLayout()
        grid.setHorizontalSpacing(12)
        grid.setVerticalSpacing(4)
        self.state_value = self._metric(grid, 0, 0, "STATE")
        self.mode_value = self._metric(grid, 0, 1, "MODE")
        self.battery_value = self._metric(grid, 1, 0, "BATTERY")
        self.altitude_value = self._metric(grid, 1, 1, "ALTITUDE")
        self.latitude_value = self._metric(grid, 2, 0, "LATITUDE")
        self.longitude_value = self._metric(grid, 2, 1, "LONGITUDE")
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        layout.addLayout(grid)

    @staticmethod
    def _metric(grid: QGridLayout, row: int, col: int, title: str) -> QLabel:
        box = QWidget()
        box_layout = QVBoxLayout(box)
        box_layout.setContentsMargins(0, 0, 0, 0)
        box_layout.setSpacing(0)
        label = QLabel(title)
        label.setObjectName("fleetMetricLabel")
        value = QLabel("--")
        value.setObjectName("fleetMetricValue")
        box_layout.addWidget(label)
        box_layout.addWidget(value)
        grid.addWidget(box, row, col)
        return value

    def set_selected(self, selected: bool) -> None:
        self.setProperty("selectedVehicle", bool(selected))
        self.style().unpolish(self)
        self.style().polish(self)

    def set_link_state(self, state: str) -> None:
        labels = {
            "connected": "LIVE",
            "connecting": "WAIT",
            "warning": "STALE",
            "error": "ERROR",
            "disconnected": "OFF",
        }
        self.link_label.setText(labels.get(state, state.upper()))
        self.link_label.setProperty("linkState", state)
        self.link_label.style().unpolish(self.link_label)
        self.link_label.style().polish(self.link_label)

    def update_vehicle(
        self,
        friendly_name: str,
        state: TelemetryState,
        link_state: str,
    ) -> None:
        self.name_label.setText(friendly_name)
        sysid = "--" if state.system_id <= 0 else str(state.system_id)
        self.identity_label.setText(f"SYS {sysid} · {state.vehicle_type or 'UNKNOWN'}")
        self.set_link_state(link_state)

        if state.connected:
            self.state_value.setText("ARMED" if state.armed else "DISARMED")
            self.state_value.setStyleSheet(
                f"color:{DANGER if state.armed else SUCCESS}; font-weight:800;"
            )
            self.mode_value.setText(state.mode or "--")
        else:
            self.state_value.setText("OFFLINE")
            self.state_value.setStyleSheet(f"color:{MUTED}; font-weight:800;")
            self.mode_value.setText("--")

        if state.battery_voltage_v is None:
            battery = "--"
        elif state.battery_remaining_pct is None:
            battery = f"{state.battery_voltage_v:.1f} V"
        else:
            battery = f"{state.battery_voltage_v:.1f} V · {state.battery_remaining_pct}%"
        self.battery_value.setText(battery)
        self.altitude_value.setText(
            "--" if state.relative_altitude_m is None else f"{state.relative_altitude_m:.1f} m"
        )
        self.latitude_value.setText("--" if state.latitude is None else f"{state.latitude:.5f}")
        self.longitude_value.setText("--" if state.longitude is None else f"{state.longitude:.5f}")

    def mousePressEvent(self, event: Any) -> None:
        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit(self.connection_id)
        super().mousePressEvent(event)


class FleetOverviewPanel(QWidget):
    """Scrollable right-side fleet overview; no connection configuration fields."""

    vehicle_selected = Signal(str)
    vehicle_remove_requested = Signal(str)

    def __init__(self) -> None:
        super().__init__()
        self.cards: dict[str, VehicleSummaryCard] = {}

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(7)

        header = QHBoxLayout()
        title = QLabel("VEHICLES")
        title.setObjectName("messagesTitle")
        self.count_label = QLabel("0 configured")
        self.count_label.setObjectName("fleetCount")
        header.addWidget(title)
        header.addStretch(1)
        header.addWidget(self.count_label)
        layout.addLayout(header)

        self.scroll = QScrollArea()
        self.scroll.setObjectName("fleetScroll")
        self.scroll.setWidgetResizable(True)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scroll.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        self.scroll.setFrameShape(QFrame.Shape.NoFrame)

        self.container = QWidget()
        self.card_layout = QVBoxLayout(self.container)
        self.card_layout.setContentsMargins(0, 0, 0, 0)
        self.card_layout.setSpacing(8)
        self.card_layout.addStretch(1)
        self.scroll.setWidget(self.container)
        layout.addWidget(self.scroll, 1)

    def add_vehicle(self, connection_id: str, friendly_name: str) -> None:
        if connection_id in self.cards:
            return
        card = VehicleSummaryCard(connection_id, friendly_name)
        card.clicked.connect(self.vehicle_selected.emit)
        card.remove_requested.connect(self.vehicle_remove_requested.emit)
        self.cards[connection_id] = card
        self.card_layout.insertWidget(max(0, self.card_layout.count() - 1), card)
        self._update_count()

    def remove_vehicle(self, connection_id: str) -> None:
        card = self.cards.pop(connection_id, None)
        if card is not None:
            self.card_layout.removeWidget(card)
            card.deleteLater()
        self._update_count()

    def update_vehicle(
        self,
        connection_id: str,
        friendly_name: str,
        state: TelemetryState,
        link_state: str,
    ) -> None:
        card = self.cards.get(connection_id)
        if card is None:
            self.add_vehicle(connection_id, friendly_name)
            card = self.cards.get(connection_id)
        if card is not None:
            card.update_vehicle(friendly_name, state, link_state)
        self._update_count()

    def set_link_state(self, connection_id: str, state: str) -> None:
        card = self.cards.get(connection_id)
        if card is not None:
            card.set_link_state(state)
        self._update_count()

    def set_selected(self, connection_id: str | None) -> None:
        for cid, card in self.cards.items():
            card.set_selected(cid == connection_id)

    def _update_count(self) -> None:
        live = sum(1 for card in self.cards.values() if card.link_label.text() == "LIVE")
        total = len(self.cards)
        self.count_label.setText(f"{live}/{total} live" if total else "0 configured")


class CameraWindow(QMainWindow):
    """Standalone FPV camera workspace opened alongside the main GCS window."""

    def __init__(self, camera_widget: QWidget, parent: QWidget | None = None) -> None:
        super().__init__(parent)
        self.setWindowFlag(Qt.WindowType.Window, True)
        self.setWindowTitle(f"{BRAND_NAME} — FPV CAMERA MONITOR")
        self.setWindowIcon(make_icon("flight", ACCENT, 32))
        self.resize(1360, 860)
        self.setMinimumSize(940, 650)
        self.setCentralWidget(camera_widget)



class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(BRAND_NAME)
        self.setWindowIcon(make_icon("flight", ACCENT, 32))
        self.resize(1680, 950)
        self.setMinimumSize(1180, 700)
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
        self.video_workspace = MultiVideoPanel()
        self.camera_window: CameraWindow | None = CameraWindow(self.video_workspace, self)

        # Main GCS remains the same professional map + side-panel layout.
        # FPV video is presented in a completely separate top-level window.
        self.setCentralWidget(self.map_view)

        self.flight_panel = FlightInfoPanel()
        self.control_panel = ControlPanel()
        self.messages_panel = MessagesPanel()
        self.instrument_panel = LeftFlightOperationsPanel(self.control_panel, self.messages_panel)
        self.fleet_panel = FleetOverviewPanel()

        # Compact operator layout:
        # LEFT  = resizable PFD + QUICK / ACTIONS / MESSAGES
        # RIGHT = scrollable read-only vehicle summary cards
        # CENTER = satellite multi-vehicle map
        # SEPARATE WINDOW = FPV camera workspace
        self.flight_dock = self._make_dock(self.flight_panel, "flightDock", 280)
        self.instrument_dock = self._make_dock(self.instrument_panel, "instrumentDock", 500)
        self.fleet_dock = self._make_dock(self.fleet_panel, "fleetDock", 360)

        self.addDockWidget(Qt.DockWidgetArea.LeftDockWidgetArea, self.instrument_dock)
        self.instrument_dock.setAllowedAreas(Qt.DockWidgetArea.LeftDockWidgetArea)
        self.instrument_dock.setMinimumWidth(340)

        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.fleet_dock)
        self.fleet_dock.setAllowedAreas(Qt.DockWidgetArea.RightDockWidgetArea)
        self.fleet_dock.setMinimumWidth(300)

        # The old detailed flight-data dock duplicates QUICK telemetry and stays hidden.
        self.flight_dock.hide()

        # No bottom camera dock: cameras open in their own independent window.
        self.setCorner(Qt.Corner.BottomRightCorner, Qt.DockWidgetArea.RightDockWidgetArea)
        self.setCorner(Qt.Corner.BottomLeftCorner, Qt.DockWidgetArea.LeftDockWidgetArea)

        self._build_top_bar()
        self._build_status_bar()
        self._wire_signals()
        self._apply_style()
        self._apply_empty_selected_state()
        QTimer.singleShot(250, self._show_camera_window)

        # Preserve the existing one-click startup behavior for the first MAVProxy link.
        # Additional vehicles are added from the ADD button in the top-right toolbar.
        QTimer.singleShot(
            500,
            lambda: self.add_vehicle_connection(
                "Vehicle 1", DEFAULT_CONNECTION, 57600, "UDP Listen"
            ),
        )

    def _make_dock(self, widget: QWidget, name: str, width: int) -> QDockWidget:
        dock = QDockWidget("", self)
        dock.setObjectName(name)
        dock.setWidget(widget)
        dock.setMinimumWidth(max(260, min(width, 340)))
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
        bar.setIconSize(QPixmap(20, 20).size())
        self.addToolBar(Qt.ToolBarArea.TopToolBarArea, bar)

        bar.addWidget(BrandWidget())

        # Keep telemetry chip objects for the existing update logic, but do not
        # expose them as separate toolbar blocks.
        self.link_chip = StatusChip("LINKS", "0/0")
        self.state_chip = StatusChip("STATE", "--")
        self.mode_chip = StatusChip("MODE", "--")
        self.gps_chip = StatusChip("GPS", "--")
        self.battery_chip = StatusChip("BAT", "--")
        for chip in (self.link_chip, self.state_chip, self.mode_chip, self.gps_chip, self.battery_chip):
            chip.hide()

        expanding = QWidget()
        expanding.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        bar.addWidget(expanding)

        # Only the selected-vehicle block and global connection button remain
        # on the top-right edge of the application.
        self.vehicle_selector = QComboBox()
        self.vehicle_selector.setObjectName("vehicleSelector")
        self.vehicle_selector.setMinimumWidth(300)
        self.vehicle_selector.setMaximumWidth(420)
        self.vehicle_selector.setToolTip(
            "Select the aircraft whose telemetry, messages and controls are active"
        )
        self.vehicle_selector.currentIndexChanged.connect(self._vehicle_selector_changed)
        bar.addWidget(self.vehicle_selector)

        self.connect_all_button = QPushButton("CONNECT ALL")
        self.connect_all_button.setObjectName("topConnectButton")
        self.connect_all_button.clicked.connect(self.connect_all_vehicles)
        bar.addWidget(self.connect_all_button)

        self.add_connection_button = QPushButton("+ ADD")
        self.add_connection_button.setObjectName("topAddButton")
        self.add_connection_button.clicked.connect(self._show_add_connection_dialog)
        bar.addWidget(self.add_connection_button)

        self.disconnect_all_button = QPushButton("DISCONNECT ALL")
        self.disconnect_all_button.setObjectName("topDisconnectButton")
        self.disconnect_all_button.clicked.connect(self.disconnect_all_vehicles)
        bar.addWidget(self.disconnect_all_button)

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
        self.messages_panel.clear_requested.connect(self._clear_selected_messages)
        self.fleet_panel.vehicle_selected.connect(self.select_vehicle)
        self.fleet_panel.vehicle_remove_requested.connect(self._request_remove_vehicle)

    def _request_remove_vehicle(self, connection_id: str) -> None:
        meta = self.connection_meta.get(connection_id)
        if meta is None:
            return
        name = str(meta.get("name", "Vehicle"))
        answer = QMessageBox.question(
            self,
            "Remove Vehicle",
            f"Remove {name} from this GCS?\n\nIts connection will be stopped first.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        )
        if answer == QMessageBox.StandardButton.Yes:
            self.remove_vehicle_connection(connection_id)

    def _quick_toggle_connection(self) -> None:
        if any(worker.isRunning() for worker in self.workers.values()):
            self.disconnect_all_vehicles()
        else:
            self.connect_all_vehicles()

    def _suggested_udp_port(self) -> int:
        used: set[int] = set()
        for meta in self.connection_meta.values():
            endpoint = str(meta.get("endpoint", ""))
            if endpoint.lower().startswith("udpin:"):
                try:
                    used.add(int(endpoint.rsplit(":", 1)[1]))
                except Exception:
                    pass
        port = 14551
        while port in used:
            port += 1
        return port

    def _show_add_connection_dialog(self) -> None:
        dialog = AddConnectionDialog(
            suggested_number=len(self.connection_meta) + 1,
            suggested_udp_port=self._suggested_udp_port(),
            parent=self,
        )
        dialog.setStyleSheet(self.styleSheet())
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        config = dialog.connection_config()
        if not str(config.get("endpoint", "")).strip():
            QMessageBox.warning(self, "Missing connection", "Enter a valid address or COM port.")
            return
        self.add_vehicle_connection(**config)

    def _show_camera_window(self) -> None:
        """Show the FPV workspace as the second application window.

        If a second monitor is connected, place and maximize the camera window
        on that display automatically. Otherwise open it as a normal window.
        """
        if self.camera_window is None:
            self.camera_window = CameraWindow(self.video_workspace, self)

        # Apply the same professional dark theme to the independent camera window.
        self.camera_window.setStyleSheet(self.styleSheet())

        screens = QApplication.screens()
        if len(screens) > 1:
            target = screens[1].availableGeometry()
            self.camera_window.setGeometry(target)
            self.camera_window.showMaximized()
        else:
            self.camera_window.show()

        self.camera_window.raise_()
        self.camera_window.activateWindow()

    def add_vehicle_connection(
        self,
        friendly_name: str,
        endpoint: str,
        baudrate: int = 57600,
        transport: str = "UDP Listen",
    ) -> None:
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
            "baudrate": int(baudrate),
            "transport": transport,
            "link_state": "disconnected",
            "link_text": "Disconnected",
        }
        self.vehicle_states[connection_id] = TelemetryState()
        self.vehicle_logs[connection_id] = []
        self._last_map_update[connection_id] = 0.0
        self.fleet_panel.add_vehicle(connection_id, friendly_name)
        self.fleet_panel.update_vehicle(
            connection_id, friendly_name, self.vehicle_states[connection_id], "disconnected"
        )

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

        worker = TelemetryWorker(
            str(meta["endpoint"]),
            int(meta.get("baudrate", 57600)),
            self,
        )
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
            self.fleet_panel.set_link_state(connection_id, "disconnected")
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
        self.fleet_panel.remove_vehicle(connection_id)
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
            self.fleet_panel.set_link_state(connection_id, "disconnected")
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
        current_state = self.vehicle_states.get(connection_id, TelemetryState())
        current_state.connected = state == "connected"
        self.vehicle_states[connection_id] = current_state
        self.fleet_panel.update_vehicle(
            connection_id,
            str(meta.get("name", "Vehicle")),
            current_state,
            state,
        )
        if self.selected_vehicle_id == connection_id:
            self.current_state = current_state
            self._apply_selected_state(current_state)
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
        self.fleet_panel.set_selected(connection_id)

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

        self.fleet_panel.update_vehicle(
            connection_id,
            str(meta.get("name", "Vehicle")),
            state,
            str(meta.get("link_state", "disconnected")),
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
        self.fleet_panel.set_selected(None)
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
        self.connect_all_button.setEnabled(total > 0 and live < total)
        self.disconnect_all_button.setEnabled(active > 0)

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
        # Permanent side columns are managed by Qt; no automatic dock hiding.

    def closeEvent(self, event: Any) -> None:
        try:
            self.video_workspace.shutdown()
        except Exception:
            pass

        if self.camera_window is not None:
            try:
                self.camera_window.close()
            except Exception:
                pass
            self.camera_window = None

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
            QToolBar#topBar QToolButton {{
                background:transparent; color:#cfd7db; border:1px solid transparent;
                border-radius:5px; padding:6px 11px; margin:0 1px;
                font-size:8.5pt; font-weight:750;
            }}
            QToolBar#topBar QToolButton:hover {{
                background:{SURFACE_2}; border-color:#465055; color:{TEXT};
            }}
            QToolBar#topBar QToolButton:checked {{
                background:#203b3a; border-color:{TEAL}; color:#e3faf5;
            }}
            QDockWidget {{ border:1px solid {BORDER}; }}
            QDockWidget > QWidget {{ background:{SURFACE}; }}
            QFrame#statusChip {{ background:#252a2d; border:1px solid #373e42; border-radius:5px; }}
            QFrame#statusChip QLabel {{ background:transparent; border:none; }}
            QLabel#chipLabel {{ color:{MUTED}; font-size:8pt; font-weight:700; }}
            QLabel#chipValue {{ color:{TEXT}; font-size:9pt; font-weight:750; }}
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
            QPushButton#topConnectButton {{ min-width:104px; border-color:#465055; }}
            QPushButton#topAddButton {{ min-width:74px; border-color:{ACCENT}; color:#dff5ff; }}
            QPushButton#topDisconnectButton {{ min-width:118px; border-color:#5a4646; }}
            QPushButton#primaryButton {{ background:{SURFACE_2}; border-color:{ACCENT}; }}
            QDialog {{ background:{BG}; }}
            QLabel#dialogTitle {{ color:{TEXT}; font-size:11pt; font-weight:850; letter-spacing:1px; }}
            QLabel#connectionHint {{ color:{MUTED}; font-size:8.5pt; padding:6px 2px; }}
            QSpinBox {{
                background:{SURFACE_3}; color:{TEXT}; border:1px solid {BORDER};
                border-radius:4px; padding:7px;
            }}
            QComboBox#connectionType, QComboBox#serialPortCombo, QComboBox#baudCombo {{
                background:{SURFACE_3}; color:{TEXT}; border:1px solid {BORDER};
                border-radius:4px; padding:7px; min-height:22px;
            }}
            QComboBox#connectionType QAbstractItemView,
            QComboBox#serialPortCombo QAbstractItemView,
            QComboBox#baudCombo QAbstractItemView {{
                background:{SURFACE_2}; color:{TEXT};
                selection-background-color:{ACCENT_DARK}; selection-color:#ffffff;
                border:1px solid {BORDER}; outline:0;
            }}
            QPushButton#dialogRefreshButton {{
                min-width:72px; padding:6px 8px; font-size:8pt; border-color:#4b555a;
            }}
            QPushButton#fleetRemoveButton {{
                min-height:20px; padding:3px 6px; font-size:7.5pt;
                background:#2a2020; border-color:#684747; color:#f2b5b5;
            }}
            QPushButton#fleetRemoveButton:hover {{
                background:#3b2424; border-color:{DANGER}; color:#ffd3d3;
            }}
            QSplitter#leftOperatorSplitter::handle {{
                background:#3b4448; height:10px; margin:1px 0;
            }}
            QSplitter#leftOperatorSplitter::handle:hover {{ background:{ACCENT}; }}
            QScrollArea#fleetScroll {{ background:transparent; border:none; }}
            QFrame#vehicleSummaryCard {{
                background:#171b1d; border:1px solid {BORDER}; border-radius:6px;
            }}
            QFrame#vehicleSummaryCard:hover {{ border-color:#596267; background:#1b2022; }}
            QFrame#vehicleSummaryCard[selectedVehicle='true'] {{
                background:#192a30; border:2px solid {ACCENT};
            }}
            QLabel#fleetVehicleName {{ color:{TEXT}; font-size:10pt; font-weight:850; }}
            QLabel#fleetIdentity {{ color:#b9c4c9; font-family:'Consolas'; font-size:8.5pt; }}
            QLabel#fleetCount {{ color:{MUTED}; font-size:8pt; font-weight:700; }}
            QLabel#fleetLinkState {{ color:{MUTED}; font-size:8pt; font-weight:850; }}
            QLabel#fleetLinkState[linkState='connected'] {{ color:{SUCCESS}; }}
            QLabel#fleetLinkState[linkState='connecting'] {{ color:{WARNING}; }}
            QLabel#fleetLinkState[linkState='warning'] {{ color:{WARNING}; }}
            QLabel#fleetLinkState[linkState='error'] {{ color:{DANGER}; }}
            QLabel#fleetMetricLabel {{ color:{MUTED}; font-size:7.3pt; font-weight:750; }}
            QLabel#fleetMetricValue {{ color:{TEXT}; font-family:'Consolas'; font-size:9pt; font-weight:700; }}
            QFrame#videoPanel {{ background:transparent; border:none; }}
            QFrame#videoWorkspace {{ background:{SURFACE}; border:none; }}
            QFrame#mainVideoFrame {{
                background:#101315; border:1px solid #3a4246; border-radius:6px;
            }}
            QLabel#mainVideoTitle {{ color:{TEXT}; font-size:10pt; font-weight:850; letter-spacing:1px; }}
            QLabel#mainVideoSource {{ color:{ACCENT}; font-size:9pt; font-weight:700; }}
            QLabel#mainVideoStatus {{ color:{MUTED}; font-size:8.5pt; padding:1px 2px; }}
            QLabel#mainFpvVideo {{
                background:#030405; color:#657075; border:1px solid #424b50;
                border-radius:5px; font-size:12pt; font-weight:750; letter-spacing:1px;
            }}
            QFrame#videoFeedCard {{
                background:#171b1d; border:1px solid {BORDER}; border-radius:6px;
            }}
            QFrame#videoFeedCard[feedSelected='true'] {{
                background:#18272b; border:2px solid {ACCENT};
            }}
            QWidget#videoFeedContainer {{ background:transparent; }}
            QScrollArea#videoScrollArea {{ background:transparent; border:none; }}
            QScrollArea#videoScrollArea > QWidget > QWidget {{ background:transparent; }}
            QLabel#videoWorkspaceTitle {{ color:{TEXT}; font-size:9pt; font-weight:800; letter-spacing:1px; }}
            QLabel#videoWorkspaceInfo {{ color:{MUTED}; font-size:8pt; }}
            QLabel#videoFeedTitle {{ color:{TEXT}; font-size:8.5pt; font-weight:800; min-width:52px; }}
            QLabel#videoCardStatus {{ color:{MUTED}; font-size:7.5pt; }}
            QLabel#videoThumbnail {{
                background:#050607; color:#667176; border:1px solid #30383c;
                border-radius:4px; font-size:8.5pt; font-weight:800;
            }}
            QFrame#videoFeedCard[feedSelected='true'] QLabel#videoThumbnail {{
                border-color:{ACCENT}; color:#dff7ff;
            }}
            QLabel#videoUnavailable {{ background:#080a0b; color:{MUTED}; border:1px solid {BORDER}; }}
            QComboBox#videoDeviceCombo {{
                background:{SURFACE_2}; color:{TEXT}; border:1px solid {BORDER};
                border-radius:4px; padding:4px 7px; min-height:22px;
            }}
            QComboBox#videoDeviceCombo QAbstractItemView {{
                background:{SURFACE_2}; color:{TEXT}; selection-background-color:{ACCENT_DARK};
                border:1px solid {BORDER};
            }}
            QPushButton#videoStartButton {{ min-height:22px; padding:3px 6px; font-size:8pt; }}
            QPushButton#videoRemoveButton {{ min-height:20px; padding:2px 5px; font-size:9pt; }}
            QPushButton[controlButton='true'] {{
                background:{SURFACE_2}; color:{TEXT}; border:1px solid #465055;
                min-height:34px;
            }}
            QPushButton[controlButton='true']:hover:enabled {{ background:#30373a; border-color:{TEAL}; }}
            QPushButton[controlButton='true']:pressed:enabled {{ background:#172526; border-color:{TEAL}; }}
            QPushButton[controlButton='true'][modeActive='true'] {{
                background:#203b3a; border:1px solid {TEAL}; color:#dff8f2;
            }}
            QTabWidget#operatorTabs::pane {{
                border:1px solid {BORDER}; background:#1a1e20; top:-1px;
            }}
            QTabWidget#operatorTabs QTabBar::tab {{
                background:#24292c; color:#b9c3c8; border:1px solid {BORDER};
                border-bottom:none; padding:8px 16px; min-width:88px;
                font-size:8.5pt; font-weight:800;
            }}
            QTabWidget#operatorTabs QTabBar::tab:selected {{
                background:#1a1e20; color:{TEXT}; border-top:2px solid {ACCENT};
            }}
            QTabWidget#operatorTabs QTabBar::tab:hover:!selected {{
                background:#2d3437; color:{TEXT};
            }}
            QFrame#quickMetricTile {{
                background:#232729; border:1px solid #343a3d; border-radius:3px;
            }}
            QLabel#quickMetricLabel {{
                color:#d5dadd; font-size:8.5pt; font-weight:650;
            }}
            QLabel#quickMetricValue {{
                font-family:'Segoe UI'; font-size:22pt; font-weight:500;
            }}
            QLineEdit {{
                background:{SURFACE_3}; color:{TEXT}; border:1px solid {BORDER};
                border-radius:4px; padding:8px;
            }}
            QLineEdit:focus {{ border-color:{ACCENT}; }}
            QLabel#metricLabel {{ color:{MUTED}; }}
            QLabel#metricValue {{ color:{TEXT}; font-family:'Consolas'; font-weight:700; }}
            QLabel#metricUnit {{ color:{MUTED}; font-size:8pt; }}
            QLabel#panelSectionTitle {{
                color:{MUTED}; font-size:8pt; font-weight:800; letter-spacing:1px;
                padding-top:3px; padding-bottom:2px;
            }}
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
