from __future__ import annotations

from pathlib import Path
from datetime import datetime
import math
import os
import re
import shutil
import subprocess
import tempfile
import time
import uuid

import carb.settings
import cv2
import omni.ext
import omni.kit.app
import omni.ui as ui
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from .stage_objects import (
    configure_front_workbench, create_move_point, create_object_proxy, point_obstacle, rebuild_object_proxy, scene_obstacle_bounds, segment_obstacle,
    set_confirmed, set_world_position, update_object_proxy, update_task_path, world_position,
)
from .task_model import TaskDocument, TaskNode
from .task_compiler import CompilerConfig, TaskCompiler
from .runtime import (
    request_execution, request_preview, request_reset, stop_motion_teaching, stop_preview,
)
from . import runtime


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SAVE_PATH = PROJECT_ROOT / "tasks" / "current_task.json"
D435_PREVIEW_PATH = Path("/tmp/openarm_task_studio_d435_preview.jpg")
TASK_STUDIO_DOCK_RATIO = 0.19
PARAMETER_DOCK_RATIO = 0.236
CENTER_BOTTOM_DOCK_RATIO = 0.273
MOTION_CAPTURE_DOCK_RATIO = 0.42
CURRENT_FRONT_WORKBENCH_BASE_Z = 0.0
LEFT_HORIZONTAL_GRASP_ANGLE = math.pi / 2.0
RIGHT_HORIZONTAL_GRASP_ANGLE = -math.pi / 2.0
MAX_GRIPPER_OPENING = 0.088
NEW_POINT_OFFSET = [0.0, 0.04, 0.0]
FIRST_POINT_FORWARD_DISTANCE = 0.40
FIRST_POINT_TABLE_CLEARANCE = 0.10
ROBOT_PRIM_PATH = "/World/envs/env_0/Robot"
DEFAULT_WORKBENCH_HEIGHT = 0.35
DEFAULT_OBJECT_DIMENSIONS = [0.03, 0.03, 0.03]
MAX_CARTESIAN_TRANSIT_STEP = 0.06
LOCKED_SCENE_ROOTS = (
    "/World/GroundPlane",
    "/World/envs/env_0/FrontWorkbench",
    ROBOT_PRIM_PATH,
)
active_extension = None


class OpenArmTaskStudioExtension(omni.ext.IExt):
    def on_startup(self, ext_id):
        global active_extension
        active_extension = self
        self._document = TaskDocument()
        self._editing_path = None
        self._editing_type = None
        self._shape = "box"
        self._active_arm = "left"
        self._grasp_mode = "vertical"
        self._selected_node_index = None
        self._selected_path_segment = None
        self._editing_insert_before_node_id = None
        self._loading_models = False
        self._joint_override_active = False
        self._stage_event_subscription = None
        self._update_subscription = None
        self._update_counter = 0
        self._applied_joint_limits = None
        self._last_path_positions = None
        self._prim_serial = 0
        self._locked_stage = None
        self._locked_scene_xforms = {}
        self._d435_camera_process = None
        self._motion_camera_source = "rgbd"
        self._d435_tuner_process = None
        self._handled_remote_record_count = 0
        self._d435_preview_mtime_ns = None
        self._workspace_layout_passes = {8, 30, 60}
        self._window = ui.Window("OpenArm Task Studio", width=540, height=850)
        self._build_ui()
        self._parameter_window = ui.Window("Task Point Parameters", width=460, height=720, visible=True)
        self._build_parameter_ui()
        self._d435_window = ui.Window("Motion Capture", width=460, height=480, visible=True)
        self._build_d435_preview_ui()
        self._button_test_window = ui.Window("Button Test", width=640, height=260, visible=True)
        self._build_button_test_ui()
        self._load_window = None
        self._stage_event_subscription = (
            omni.usd.get_context().get_stage_event_stream().create_subscription_to_pop(
                self._on_stage_event, name="OpenArm Task Studio selection"
            )
        )
        self._update_subscription = (
            omni.kit.app.get_app().get_update_event_stream().create_subscription_to_pop(
                self._on_update, name="OpenArm Task Studio path refresh"
            )
        )
        print(f"[OpenArmTaskStudio] Extension ready: {ext_id}")
        if (
            carb.settings.get_settings().get_as_bool("/exts/openarm_task_studio/runSelfTest")
            or os.environ.get("OPENARM_TASK_STUDIO_SELF_TEST") == "1"
        ):
            self._run_self_test()

    def on_shutdown(self):
        global active_extension
        active_extension = None
        stop_preview()
        if self._d435_camera_process and self._d435_camera_process.poll() is None:
            self._d435_camera_process.terminate()
        if self._d435_tuner_process and self._d435_tuner_process.poll() is None:
            self._d435_tuner_process.terminate()
        self._stage_event_subscription = None
        self._update_subscription = None
        self._parameter_window = None
        self._d435_window = None
        self._button_test_window = None
        self._window = None
        print("[OpenArmTaskStudio] Extension stopped")

    @property
    def _stage(self):
        return omni.usd.get_context().get_stage()

    def _apply_default_workspace_layout(self):
        viewport = ui.Workspace.get_window("Viewport")
        if viewport:
            self._window.dock_in(
                viewport, ui.DockPosition.LEFT, TASK_STUDIO_DOCK_RATIO
            )
            self._parameter_window.visible = True
            self._parameter_window.dock_in(
                viewport, ui.DockPosition.RIGHT, PARAMETER_DOCK_RATIO
            )
            content = ui.Workspace.get_window("Content")
            console = ui.Workspace.get_window("Console")
            bottom = content or console
            if bottom:
                bottom.dock_in(
                    viewport, ui.DockPosition.BOTTOM, CENTER_BOTTOM_DOCK_RATIO
                )
                if content and content is not bottom:
                    content.dock_in(bottom, ui.DockPosition.SAME)
                if console and console is not bottom:
                    console.dock_in(bottom, ui.DockPosition.SAME)
                self._button_test_window.visible = True
                self._button_test_window.dock_in(bottom, ui.DockPosition.SAME)
            self._d435_window.visible = True
            self._d435_window.dock_in(
                self._parameter_window,
                ui.DockPosition.BOTTOM,
                MOTION_CAPTURE_DOCK_RATIO,
            )
        hidden_titles = (
            "Stage",
            "Layer",
            "Render Settings",
            "Property",
            "Semantics Schema Editor",
            "Simulation Settings",
        )
        for title in hidden_titles:
            if ui.Workspace.get_window(title):
                ui.Workspace.show_window(title, False)
        print(
            "[OpenArmTaskStudio] Workspace ready: studio=left, parameters=right, "
            "content/console/button-test=center-bottom, default editor panels hidden"
        )

    def _show_and_dock_parameter_window(self):
        self._parameter_window.visible = True

    def _show_empty_parameter_state(self):
        self._selected_node_index = None
        self._parameter_window.visible = True
        self._loading_models = True
        try:
            self._parameter_title.text = "No task point selected"
            self._name_model.set_value("No selection")
            self._order_model.set_value(0)
            self._joint_override_active = False
            self._joint_status.text = "Joint override: disabled until a point is selected"
        finally:
            self._loading_models = False

    def _show_and_dock_d435_window(self):
        self._d435_window.visible = True
        self._parameter_window.visible = True
        self._apply_default_workspace_layout()

    def _update_d435_preview(self):
        if (
            not runtime.d435_teaching_enabled
            or not self._d435_window.visible
            or not D435_PREVIEW_PATH.exists()
        ):
            return
        try:
            mtime_ns = D435_PREVIEW_PATH.stat().st_mtime_ns
            if mtime_ns == self._d435_preview_mtime_ns:
                return
            frame = cv2.imread(str(D435_PREVIEW_PATH), cv2.IMREAD_COLOR)
            if frame is None:
                return
            rgba = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
            height, width = rgba.shape[:2]
            self._d435_image_provider.set_bytes_data(rgba.flatten().data, [width, height])
            self._d435_preview_mtime_ns = mtime_ns
            self._d435_preview_status.text = f"Live tracking | {width} x {height}"
        except OSError:
            pass

    def _clear_motion_capture_preview(self):
        width, height = 640, 480
        black_rgba = bytearray(b"\x00\x00\x00\xff") * (width * height)
        self._d435_image_provider.set_bytes_data(black_rgba, [width, height])
        self._d435_preview_mtime_ns = None
        self._d435_preview_status.text = "Motion capture stopped"

    def _build_ui(self):
        with self._window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=7):
                    ui.Label("OPENARM TASK STUDIO", height=30)
                    with ui.HStack(height=36):
                        ui.Label("Task arm", width=120)
                        ui.Button("LEFT ARM", clicked_fn=lambda: self._set_active_arm("left"))
                        ui.Button("RIGHT ARM", clicked_fn=lambda: self._set_active_arm("right"))
                    self._arm_status = ui.Label("Active task arm: LEFT", height=24)
                    self._status = ui.Label("Scene geometry is used automatically for collision checks", word_wrap=True, height=42)
                    ui.Separator()
                    ui.Label("1. Scene planning", style={"color": 0xFF75C9B7})
                    self._margin = self._float_row("TCP collision margin (m)", 0.02)
                    self._table_enabled = self._bool_row("Front workbench enabled", True)
                    self._table_height = self._text_float_row("Tabletop height, world Z (m)", DEFAULT_WORKBENCH_HEIGHT)
                    self._table_distance = self._text_float_row("Near-edge distance from arm, X (m)", 0.20)
                    ui.Label("Robot and workbench stand directly on the ground.", word_wrap=True, height=26)
                    ui.Separator()
                    ui.Label("2. Task points and teaching", style={"color": 0xFFFFB86B})
                    with ui.HStack(height=34):
                        ui.Button("Move", clicked_fn=self._begin_move)
                        ui.Button("Grasp", clicked_fn=self._begin_grasp)
                        ui.Button("Place", clicked_fn=self._begin_place)
                    ui.Button(
                        "Add Move Point On Selected Line",
                        clicked_fn=self._begin_move_on_selected_line,
                        height=34,
                    )
                    with ui.HStack(height=38):
                        ui.Button("Confirm Current Edit", clicked_fn=self._confirm_edit)
                        ui.Button("Cancel", clicked_fn=self._cancel_edit)
                    with ui.HStack(height=38):
                        ui.Button(
                            "Start Motion Teaching",
                            clicked_fn=self._start_d435_teaching,
                            style={"background_color": 0xFF416E23, "color": 0xFFFFFFFF},
                        )
                        ui.Button("Record Current Point", clicked_fn=self._capture_motion_point)
                    with ui.HStack(height=34):
                        ui.Button("TEACH LEFT", clicked_fn=lambda: self._set_d435_mode("left"))
                        ui.Button("TEACH RIGHT", clicked_fn=lambda: self._set_d435_mode("right"))
                        ui.Button("TEACH BOTH", clicked_fn=lambda: self._set_d435_mode("both"))
                    self._d435_mode_status = ui.Label("Motion-controlled arm(s): LEFT", height=24)
                    with ui.HStack(height=34):
                        ui.Button("Open Gripper", clicked_fn=lambda: self._set_manual_gripper("open"))
                        ui.Button("Close Gripper", clicked_fn=lambda: self._set_manual_gripper("closed"))
                    with ui.HStack(height=38):
                        ui.Button(
                            "Stop Motion Teaching",
                            clicked_fn=self._stop_d435_teaching,
                            style={"background_color": 0xFF323CA0, "color": 0xFFFFFFFF},
                        )
                        ui.Button("Open Motion Mapping Tuner", clicked_fn=self._open_d435_tuner)
                    self._d435_status = ui.Label("Motion teaching: idle; remote button UDP 5011", word_wrap=True, height=34)
                    with ui.HStack(height=34):
                        ui.Button("Delete Selected Point", clicked_fn=self._delete_selected_point)
                        ui.Button("Clear All Points", clicked_fn=self._clear_all_points)
                    ui.Separator()
                    ui.Label("3. Task execution", style={"color": 0xFF8EB8FF})
                    self._execution_speed = self._float_row("Overall speed (0.1x - 3.0x)", 3.0)
                    with ui.HStack(height=36):
                        ui.Button("Reset Robot", clicked_fn=self._reset_robot)
                        ui.Button(
                            "Run Full Task",
                            clicked_fn=self._run_full_task_safe,
                            style={"background_color": 0xFFAA5F2D, "color": 0xFFFFFFFF},
                        )
                        ui.Button("Stop", clicked_fn=self._stop_preview)
                    with ui.HStack(height=34):
                        ui.Button("Preview Selected", clicked_fn=self._preview_selected)
                        ui.Button("Open Point Parameters", clicked_fn=self._show_parameter_window)
                    with ui.HStack(height=34):
                        ui.Button("Validate Scene Path", clicked_fn=self._validate_workspace)
                        ui.Button("Save Task JSON", clicked_fn=self._save)
                        ui.Button("Load Task JSON...", clicked_fn=self._show_load_window)
                    self._task_summary = ui.Label("0 confirmed task nodes", word_wrap=True, height=220)
                    ui.Label("Preview uses position IK and TCP obstacle sampling; whole-arm planning is not connected.", word_wrap=True, height=42)
        self._table_enabled.add_value_changed_fn(lambda _model: self._on_table_config_changed())
        self._table_height.add_value_changed_fn(lambda _model: self._on_table_config_changed())
        self._table_distance.add_value_changed_fn(lambda _model: self._on_table_config_changed())

    def _build_parameter_ui(self):
        with self._parameter_window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=7):
                    self._parameter_title = ui.Label("No task point selected", height=30)
                    with ui.HStack(height=28):
                        ui.Label("Point name", width=190)
                        self._name_model = ui.SimpleStringModel("No selection")
                        ui.StringField(model=self._name_model)
                    with ui.HStack(height=28):
                        ui.Label("Execution order", width=190)
                        self._order_model = ui.SimpleIntModel(0)
                        ui.IntField(model=self._order_model)
                    with ui.HStack(height=32):
                        ui.Button("Convert to Grasp", clicked_fn=lambda: self._convert_selected_move("grasp"))
                        ui.Button("Convert to Place", clicked_fn=lambda: self._convert_selected_move("place"))
                    ui.Label("World position")
                    self._pos_x = self._float_row("X (m)", 0.4)
                    self._pos_y = self._float_row("Y (m)", 0.0)
                    self._pos_z = self._float_row("Z (m)", 1.0)
                    ui.Separator()
                    self._joint_arm_label = ui.Label("LEFT arm motor angles (rad)")
                    self._joint_models = []
                    self._joint_sliders = []
                    for index in range(7):
                        model, slider = self._joint_slider_row(index)
                        self._joint_models.append(model)
                        self._joint_sliders.append(slider)
                    ui.Button("Capture Current Arm Angles", clicked_fn=self._capture_current_joints, height=32)
                    self._joint_status = ui.Label("Joint override: disabled until edited or captured", word_wrap=True, height=34)
                    ui.Separator()
                    ui.Label("Object shape")
                    with ui.HStack(height=32):
                        ui.Button("Box", clicked_fn=lambda: self._set_shape("box"))
                        ui.Button("Cylinder", clicked_fn=lambda: self._set_shape("cylinder"))
                        ui.Button("Sphere", clicked_fn=lambda: self._set_shape("sphere"))
                    self._shape_status = ui.Label("Selected shape: box", height=24)
                    self._dim_x = self._float_row("Length / radius (m)", DEFAULT_OBJECT_DIMENSIONS[0])
                    self._dim_y = self._float_row("Width (m)", DEFAULT_OBJECT_DIMENSIONS[1])
                    self._dim_z = self._float_row("Height (m)", DEFAULT_OBJECT_DIMENSIONS[2])
                    ui.Label("Grasp orientation")
                    with ui.HStack(height=32):
                        ui.Button("Vertical", clicked_fn=lambda: self._set_grasp_mode("vertical"))
                        ui.Button("Horizontal", clicked_fn=lambda: self._set_grasp_mode("horizontal"))
                    self._grasp_status = ui.Label("Selected grasp: vertical", height=24)
                    self._clearance = self._float_row("Finger clearance (m; max opening 0.088)", 0.01)
                    self._approach_height = self._float_row("Approach height (m)", 0.06)
                    self._grasp_height_offset = self._float_row("Grasp height offset (m)", 0.0)
                    self._minimum_wrist_clearance = self._float_row("Minimum wrist/table clearance (m)", 0.0)
                    with ui.HStack(height=36):
                        ui.Button("Preview This Point", clicked_fn=self._preview_selected)
                        ui.Button("Close", clicked_fn=lambda: setattr(self._parameter_window, "visible", False))

        for model in (
            self._dim_x, self._dim_y, self._dim_z, self._clearance, self._approach_height,
            self._grasp_height_offset, self._minimum_wrist_clearance,
        ):
            model.add_value_changed_fn(lambda _model: self._on_object_parameter_changed())
        for model in (self._pos_x, self._pos_y, self._pos_z):
            model.add_value_changed_fn(lambda _model: self._on_position_changed())
        for model in self._joint_models:
            model.add_value_changed_fn(lambda _model: self._on_joint_angles_changed())
        self._name_model.add_value_changed_fn(lambda _model: self._on_name_changed())
        self._order_model.add_value_changed_fn(lambda _model: self._on_order_changed())

    def _build_d435_preview_ui(self):
        self._d435_image_provider = ui.ByteImageProvider()
        with self._d435_window.frame:
            with ui.VStack(spacing=4):
                with ui.HStack(height=32):
                    ui.Label("Camera", width=70)
                    ui.Button("RGB-D (D435)", clicked_fn=lambda: self._set_motion_camera_source("rgbd"))
                    ui.Button("RGB (Webcam)", clicked_fn=lambda: self._set_motion_camera_source("rgb"))
                self._motion_camera_source_status = ui.Label("Selected source: RGB-D (D435)", height=22)
                self._d435_preview_status = ui.Label("Preview starts with Motion Teaching", height=24)
                ui.ImageWithProvider(self._d435_image_provider, height=340)
                self._motion_monitor_status = ui.Label("Monitor: idle", height=22)
                self._motion_monitor_metrics = ui.Label("No samples", height=38, word_wrap=True)
        self._clear_motion_capture_preview()

    def _build_button_test_ui(self):
        with self._button_test_window.frame:
            with ui.VStack(spacing=8):
                ui.Label("ESP32 REMOTE BUTTON LIVE TEST", height=30)
                self._button_connected = ui.Label(
                    "CONNECTED", height=34, style={"color": 0xFF55DD77}
                )
                self._button_disconnected = ui.Label(
                    "WAITING FOR ESP32 HEARTBEAT", height=34, style={"color": 0xFF6666FF}
                )
                self._button_pressed = ui.Label(
                    "BUTTON: PRESSED", height=42, style={"color": 0xFF55DD77}
                )
                self._button_released = ui.Label(
                    "BUTTON: RELEASED", height=42, style={"color": 0xFFBBBBBB}
                )
                self._button_endpoint_status = ui.Label("Device: --", height=26)
                self._button_event_status = ui.Label(
                    "Presses received: 0 | Last sequence: -- | Last press: never",
                    height=26,
                )
                self._button_refresh_status = ui.Label(
                    "Live refresh follows every Kit update; heartbeat interval: 1 s",
                    height=26,
                    style={"color": 0xFFAAAAAA},
                )
        self._button_connected.visible = False
        self._button_pressed.visible = False

    def _update_button_test(self):
        now = time.monotonic()
        packet_age = (
            now - runtime.remote_button_last_packet_time
            if runtime.remote_button_last_packet_time > 0.0 else None
        )
        connected = packet_age is not None and packet_age <= 2.5
        self._button_connected.visible = connected
        self._button_disconnected.visible = not connected
        if connected:
            self._button_connected.text = f"CONNECTED | heartbeat {packet_age:.2f} s ago"
        elif packet_age is not None:
            self._button_disconnected.text = f"DISCONNECTED | last packet {packet_age:.1f} s ago"
        self._button_pressed.visible = connected and runtime.remote_button_is_down
        self._button_released.visible = not self._button_pressed.visible
        address = runtime.remote_button_address
        self._button_endpoint_status.text = (
            f"Device: {address[0]}:{address[1]} | UDP 5011"
            if address else "Device: -- | UDP 5011 waiting"
        )
        press_age = (
            now - runtime.remote_button_last_press_time
            if runtime.remote_button_last_press_time > 0.0 else None
        )
        last_press = f"{press_age:.2f} s ago" if press_age is not None else "never"
        sequence = runtime.remote_button_sequence
        self._button_event_status.text = (
            f"Presses received: {runtime.remote_record_press_count} | "
            f"Last sequence: {sequence if sequence is not None else '--'} | Last press: {last_press}"
        )

    def _float_row(self, label, value):
        with ui.HStack(height=28):
            ui.Label(label, width=190)
            model = ui.SimpleFloatModel(value)
            ui.FloatField(model=model)
        return model

    def _bool_row(self, label, value):
        with ui.HStack(height=28):
            ui.Label(label, width=300)
            model = ui.SimpleBoolModel(value)
            ui.CheckBox(model=model, width=24)
        return model

    def _text_float_row(self, label, value):
        with ui.HStack(height=28):
            ui.Label(label, width=190)
            model = ui.SimpleStringModel(f"{value:.3f}")
            ui.StringField(model=model)
        return model

    @staticmethod
    def _text_float_value(model):
        try:
            return float(model.get_value_as_string().strip())
        except ValueError:
            return None

    def _on_table_config_changed(self):
        if self._loading_models or not self._stage:
            return
        enabled = self._table_enabled.get_value_as_bool()
        raw_height = self._text_float_value(self._table_height)
        raw_distance = self._text_float_value(self._table_distance)
        if raw_height is None or raw_distance is None:
            return
        height = max(0.20, min(2.00, raw_height))
        distance = max(0.0, min(2.00, raw_distance))
        self._loading_models = True
        try:
            self._table_height.set_value(f"{height:.3f}")
            self._table_distance.set_value(f"{distance:.3f}")
        finally:
            self._loading_models = False
        base_z = configure_front_workbench(self._stage, enabled, height, distance)
        self._capture_locked_scene_xforms(force=True)
        self._document.front_workbench_enabled = enabled
        self._document.front_workbench_top_height = height
        self._document.front_workbench_distance = distance
        self._document.front_workbench_base_z = base_z
        self._last_path_positions = None
        self._refresh_summary()
        state = "enabled" if enabled else "disabled"
        self._status.text = f"Front workbench {state}: height={height:.3f} m, distance={distance:.3f} m"

    def _joint_slider_row(self, index):
        with ui.HStack(height=30, spacing=4):
            ui.Label(f"J{index + 1}", width=28)
            model = ui.SimpleFloatModel(0.0)
            ui.Button("-", width=28, clicked_fn=lambda joint=index: self._nudge_joint(joint, -0.01))
            slider = ui.FloatSlider(model=model, min=-3.1416, max=3.1416, step=0.005)
            ui.Button("+", width=28, clicked_fn=lambda joint=index: self._nudge_joint(joint, 0.01))
            value_label = ui.Label("0.000 rad / 0.0 deg", width=136)

        def update_value_label(changed_model):
            value = changed_model.get_value_as_float()
            value_label.text = f"{value:.3f} rad / {math.degrees(value):.1f} deg"

        model.add_value_changed_fn(update_value_label)
        update_value_label(model)
        return model, slider

    def _joint_limit(self, index):
        limits = self._current_arm_limits()
        if limits and len(limits) == 7:
            return limits[index]
        return [-3.1416, 3.1416]

    def _current_arm_joints(self):
        return runtime.current_right_joint_positions if self._active_arm == "right" else runtime.current_left_joint_positions

    def _current_arm_limits(self):
        return runtime.current_right_joint_limits if self._active_arm == "right" else runtime.current_left_joint_limits

    def _nudge_joint(self, index, delta):
        minimum, maximum = self._joint_limit(index)
        value = self._joint_models[index].get_value_as_float() + delta
        self._joint_models[index].set_value(max(minimum, min(maximum, value)))

    def _select(self, path):
        omni.usd.get_context().get_selection().set_selected_prim_paths([path], True)

    def _require_stage(self):
        if self._stage is None:
            self._status.text = "Open or create a USD stage first"
            return False
        return True

    def _default_position(self):
        arm_nodes = [node for node in self._document.nodes if node.arm == self._active_arm]
        if arm_nodes:
            previous = arm_nodes[-1]
            position = previous.parameters.get("position")
            if position:
                return [position[index] + NEW_POINT_OFFSET[index] for index in range(3)]
        robot_prim = self._stage.GetPrimAtPath(ROBOT_PRIM_PATH)
        robot_position = world_position(self._stage, ROBOT_PRIM_PATH) if robot_prim.IsValid() else [0.0, 0.0, 0.0]
        table_height = self._text_float_value(self._table_height)
        if table_height is None:
            table_height = DEFAULT_WORKBENCH_HEIGHT
        return [
            robot_position[0] + FIRST_POINT_FORWARD_DISTANCE,
            robot_position[1],
            table_height + FIRST_POINT_TABLE_CLEARANCE,
        ]

    def _begin_move(self):
        if not self._require_stage(): return
        if not self._can_begin_edit(): return
        self._editing_insert_before_node_id = None
        self._selected_path_segment = None
        self._selected_node_index = None
        path = create_move_point(self._stage, self._next_prim_index(), self._default_position(), self._active_arm)
        self._editing_path, self._editing_type = path, "move"
        self._select(path)
        self._load_editing_models()
        self._show_and_dock_parameter_window()
        self._status.text = "Move point editing: press W, drag the blue sphere, then confirm."

    def _begin_move_on_selected_line(self):
        if not self._require_stage() or not self._can_begin_edit():
            return
        selected = self._selected_path_segment
        if not selected:
            self._status.text = "Select a path line between two task points first"
            return
        segment_path, insert_before_node_id = selected
        segment = self._stage.GetPrimAtPath(segment_path)
        next_node = next(
            (node for node in self._document.nodes if node.node_id == insert_before_node_id),
            None,
        )
        if not segment.IsValid() or next_node is None:
            self._selected_path_segment = None
            self._status.text = "Selected line is no longer available; select it again"
            return
        position = world_position(self._stage, segment_path)
        self._active_arm = next_node.arm
        self._arm_status.text = f"Active task arm: {next_node.arm.upper()}"
        path = create_move_point(
            self._stage, self._next_prim_index(), position, self._active_arm
        )
        self._editing_path, self._editing_type = path, "move"
        self._editing_insert_before_node_id = insert_before_node_id
        self._selected_node_index = None
        self._selected_path_segment = None
        self._select(path)
        self._load_editing_models()
        self._show_and_dock_parameter_window()
        self._status.text = (
            f"Move point inserted before {next_node.name}; drag it if needed, then confirm"
        )

    def _set_shape(self, shape):
        self._shape = shape
        self._shape_status.text = f"Selected shape: {shape}"
        self._update_selected_object(rebuild=True)
        self._status.text = f"Object shape selected: {shape}"

    def _set_grasp_mode(self, mode):
        self._grasp_mode = mode
        self._grasp_status.text = f"Selected grasp: {mode}"
        self._update_selected_object()
        self._status.text = f"Grasp orientation selected: {mode}"

    def _dimensions(self):
        x = max(0.005, self._dim_x.get_value_as_float())
        y = max(0.005, self._dim_y.get_value_as_float())
        z = max(0.005, self._dim_z.get_value_as_float())
        return [x, y, z]

    def _constrain_grasp_dimensions(self, dimensions):
        clearance = max(0.0, min(self._clearance.get_value_as_float(), MAX_GRIPPER_OPENING - 0.005))
        max_object_width = MAX_GRIPPER_OPENING - clearance
        constrained = list(dimensions)
        if self._shape == "sphere":
            constrained[0] = min(constrained[0], max_object_width * 0.5)
        elif self._shape == "cylinder" and self._grasp_mode == "vertical":
            constrained[0] = min(constrained[0], max_object_width * 0.5)
        elif self._shape == "cylinder" or self._grasp_mode == "horizontal":
            constrained[2] = min(constrained[2], max_object_width)
        else:
            constrained[1] = min(constrained[1], max_object_width)
        return constrained, clearance

    def _set_dimension_models(self, dimensions, clearance):
        was_loading = self._loading_models
        self._loading_models = True
        try:
            self._dim_x.set_value(dimensions[0])
            self._dim_y.set_value(dimensions[1])
            self._dim_z.set_value(dimensions[2])
            self._clearance.set_value(clearance)
        finally:
            self._loading_models = was_loading

    def _begin_grasp(self):
        if not self._require_stage(): return
        if not self._can_begin_edit(): return
        self._editing_insert_before_node_id = None
        self._selected_node_index = None
        path = create_object_proxy(self._stage, self._next_prim_index(), "grasp", self._shape, self._default_position(), self._dimensions(), self._active_arm)
        self._editing_path, self._editing_type = path, "grasp"
        self._select(path)
        self._load_editing_models()
        self._show_and_dock_parameter_window()
        self._status.text = "Grasp object editing: drag the orange proxy, set dimensions, then confirm."

    def _on_object_parameter_changed(self):
        if not self._loading_models:
            self._update_selected_object()

    def _update_selected_object(self, rebuild=False):
        path, node = self._current_object()
        if not path:
            return
        try:
            original_dimensions = self._dimensions()
            original_clearance = self._clearance.get_value_as_float()
            dimensions = original_dimensions
            dimensions, clearance = self._constrain_grasp_dimensions(dimensions)
            limited = dimensions != original_dimensions or clearance != original_clearance
            if limited:
                self._set_dimension_models(dimensions, clearance)
            if rebuild:
                rebuild_object_proxy(self._stage, path, self._shape, dimensions, node.arm if node else self._active_arm)
            else:
                update_object_proxy(self._stage, path, self._shape, dimensions)
            if node:
                width = self._grip_width(dimensions)
                node.parameters.update({
                    "shape": self._shape, "dimensions": dimensions,
                    "grasp_orientation": self._grasp_mode, "object_width": width,
                    "preopen_width": width + clearance,
                    "approach_height": max(0.02, self._approach_height.get_value_as_float()),
                    "grasp_height_offset": max(0.0, self._grasp_height_offset.get_value_as_float()),
                    "minimum_wrist_clearance": max(0.0, self._minimum_wrist_clearance.get_value_as_float()),
                    "approach_direction": [0.0, 0.0, -1.0],
                })
            self._status.text = (
                f"Object size limited to gripper opening ({MAX_GRIPPER_OPENING:.3f} m)"
                if limited else "Object parameters updated"
            )
        except Exception as exc:
            self._status.text = f"Object update failed: {exc}"

    def _current_object(self):
        if self._editing_path and self._editing_type in ("grasp", "place"):
            return self._editing_path, None
        if self._selected_node_index is not None:
            node = self._document.nodes[self._selected_node_index]
            if node.task_type in ("grasp", "place"):
                return node.prim_path, node
        return None, None

    def _begin_place(self):
        if not self._require_stage(): return
        if not self._can_begin_edit(): return
        self._editing_insert_before_node_id = None
        self._selected_node_index = None
        path = create_object_proxy(self._stage, self._next_prim_index(), "place", self._shape, self._default_position(), self._dimensions(), self._active_arm)
        self._editing_path, self._editing_type = path, "place"
        self._select(path)
        self._load_editing_models()
        self._show_and_dock_parameter_window()
        self._status.text = "Place editing: drag the object ghost to its final pose, then confirm."

    def _joint_positions(self):
        positions = []
        for index, model in enumerate(self._joint_models):
            minimum, maximum = self._joint_limit(index)
            positions.append(max(minimum, min(maximum, model.get_value_as_float())))
        return positions

    def _set_joint_models(self, positions):
        was_loading = self._loading_models
        self._loading_models = True
        try:
            for index, (model, value) in enumerate(zip(self._joint_models, positions)):
                minimum, maximum = self._joint_limit(index)
                model.set_value(max(minimum, min(maximum, float(value))))
        finally:
            self._loading_models = was_loading

    def _capture_current_joints(self):
        positions = self._current_arm_joints()
        if not positions or len(positions) != 7:
            self._status.text = f"Current {self._active_arm}-arm angles are not available yet"
            return
        self._set_joint_models(positions)
        self._joint_override_active = True
        self._joint_status.text = "Joint override: enabled"
        self._on_joint_angles_changed()
        self._status.text = f"Captured current {self._active_arm}-arm motor angles"

    def _start_d435_teaching(self):
        self._show_and_dock_d435_window()
        if not self._editing_path and self._selected_node_index is None:
            self._begin_move()
            if not self._editing_path:
                self._status.text = "Could not create a Move point for motion teaching"
                return
        if self._selected_node_index is not None:
            self._active_arm = self._document.nodes[self._selected_node_index].arm
        if not self._ensure_motion_camera():
            return
        stop_preview()
        runtime.d435_teaching_enabled = True
        print(
            f"[OpenArmTaskStudio] D435 teaching requested: mode={runtime.d435_teaching_mode}, "
            f"raw_packets={runtime.d435_raw_packet_count}, tracked_packets={runtime.d435_packet_count}"
        )
        self._d435_status.text = f"Motion camera starting: {runtime.d435_teaching_mode.upper()}"
        self._status.text = "Keep shoulders, elbows and wrists visible while motion tracking calibrates"

    def _set_motion_camera_source(self, source):
        if source not in ("rgbd", "rgb") or source == self._motion_camera_source:
            return
        was_running = self._d435_camera_process and self._d435_camera_process.poll() is None
        if was_running:
            self._d435_camera_process.terminate()
            try:
                self._d435_camera_process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._d435_camera_process.kill()
                self._d435_camera_process.wait()
        self._d435_camera_process = None
        self._motion_camera_source = source
        label = "RGB-D (D435)" if source == "rgbd" else "RGB (Webcam)"
        self._motion_camera_source_status.text = f"Selected source: {label}"
        self._clear_motion_capture_preview()
        runtime.d435_tracking_valid["left"] = False
        runtime.d435_tracking_valid["right"] = False
        if was_running or runtime.d435_teaching_enabled:
            self._ensure_motion_camera()
            self._d435_status.text = f"Motion camera switching to {label}; recalibrating"
        self._status.text = f"Motion capture source set to {label}"

    def _set_d435_mode(self, mode):
        runtime.d435_teaching_mode = mode
        self._d435_mode_status.text = f"Motion-controlled arm(s): {mode.upper()}"
        self._status.text = f"Motion teaching control set to {mode.upper()}"

    def _set_manual_gripper(self, state):
        runtime.request_manual_gripper(state)
        controlled = runtime.d435_teaching_mode.upper()
        self._d435_status.text = f"{controlled} gripper: {state}"
        self._status.text = f"Set {controlled} gripper {state}"

    def _stop_d435_teaching(self):
        stop_motion_teaching()
        self._clear_motion_capture_preview()
        self._d435_status.text = "Motion teaching: stopped; robot holding current pose"
        self._status.text = "Motion teaching stopped; task execution control is ready"
        print("[OpenArmTaskStudio] Motion teaching stopped; runtime command reset to idle")

    def _open_d435_tuner(self):
        if self._d435_tuner_process and self._d435_tuner_process.poll() is None:
            self._status.text = "Motion mapping tuner is already open"
            return
        script = str(Path(__file__).resolve().parents[1] / "d435_tuning_gui.py")
        config = Path(__file__).resolve().parents[1] / "config" / "openarm_d435_teleop_visual.yaml"
        environment = os.environ.copy()
        environment.pop("PYTHONNOUSERSITE", None)
        try:
            self._d435_tuner_process = subprocess.Popen(
                ["/usr/bin/python3", script, "--config", str(config)],
                cwd=str(Path(script).parent), env=environment,
            )
            self._status.text = "Motion mapping tuner opened; saved gains reload automatically"
        except OSError as exc:
            self._status.text = f"Could not open motion mapping tuner: {exc}"

    def _ensure_motion_camera(self):
        if self._d435_camera_process and self._d435_camera_process.poll() is None:
            return True
        camera_script = Path(__file__).resolve().parents[1] / "run_d435_teaching_camera.sh"
        try:
            D435_PREVIEW_PATH.unlink(missing_ok=True)
            self._d435_preview_mtime_ns = None
            self._d435_camera_process = subprocess.Popen([
                str(camera_script), "--source", self._motion_camera_source,
            ])
            return True
        except OSError as exc:
            self._status.text = f"Could not start motion camera: {exc}"
            return False

    def _capture_motion_point(self):
        if not runtime.d435_teaching_enabled:
            self._status.text = "Start motion teaching before recording points"
            return False
        arms = (
            ("left", "right") if runtime.d435_teaching_mode == "both"
            else (runtime.d435_teaching_mode,)
        )
        missing = [arm for arm in arms if not runtime.d435_tracking_valid[arm]]
        if missing:
            self._status.text = f"Record blocked: tracking lost for {', '.join(missing)} arm"
            return False
        if self._editing_path and self._stage.GetPrimAtPath(self._editing_path).IsValid():
            self._stage.RemovePrim(self._editing_path)
        self._editing_path = self._editing_type = None
        self._editing_insert_before_node_id = None
        recorded = []
        for arm in arms:
            position = (
                runtime.current_right_tcp_position if arm == "right"
                else runtime.current_left_tcp_position
            )
            joints = (
                runtime.current_right_joint_positions if arm == "right"
                else runtime.current_left_joint_positions
            )
            if not position or not joints:
                continue
            path = create_move_point(self._stage, self._next_prim_index(), position, arm)
            set_confirmed(self._stage, path)
            node = TaskNode(
                node_id=str(uuid.uuid4()), task_type="move", name=self._next_name("move"),
                prim_path=path, parameters={
                    "position": list(position), "joint_positions": list(joints),
                }, arm=arm,
            )
            self._document.nodes.append(node)
            recorded.append(node)
        if not recorded:
            self._status.text = "Record failed: current robot state is unavailable"
            return False
        self._selected_node_index = len(self._document.nodes) - 1
        segment_count = self._refresh_summary()
        names = ", ".join(node.name for node in recorded)
        self._d435_status.text = f"Recorded {names}; teaching continues"
        self._status.text = f"Remote/local record saved {names}; path has {segment_count} segment(s)"
        print(f"[OpenArmTaskStudio] Continuous teaching recorded: {names}")
        return True

    def _on_joint_angles_changed(self):
        if self._loading_models:
            return
        positions = self._joint_positions()
        self._set_joint_models(positions)
        self._joint_override_active = True
        self._joint_status.text = "Joint override: enabled"
        selected_node = None
        if self._selected_node_index is not None:
            selected_node = self._document.nodes[self._selected_node_index]
            selected_node.parameters["joint_positions"] = positions
        preview_arm = selected_node.arm if selected_node is not None else self._active_arm
        path = self._editing_path
        if not path and self._selected_node_index is not None:
            path = self._document.nodes[self._selected_node_index].prim_path
        if path and self._stage.GetPrimAtPath(path).IsValid():
            target = world_position(self._stage, path)
            edited_task_type = (
                self._document.nodes[self._selected_node_index].task_type
                if self._selected_node_index is not None else self._editing_type
            )
            request_preview({
                "position": target,
                "waypoints": [self._waypoint(
                    target, "hold", "live joint edit", joint_positions=positions,
                    arm=preview_arm,
                )],
                "sync_move_point_path": path if edited_task_type == "move" else None,
                "sync_move_point_arm": preview_arm,
                "speed_scale": 3.0,
            })
            self._status.text = "Live joint preview; Move point follows the actual TCP"

    def _grip_width(self, dimensions):
        if self._shape == "sphere": return dimensions[0] * 2.0
        if self._shape == "cylinder": return dimensions[0] * 2.0 if self._grasp_mode == "vertical" else dimensions[2]
        return dimensions[1] if self._grasp_mode == "vertical" else dimensions[2]

    def _confirm_edit(self):
        runtime.d435_teaching_enabled = False
        if not self._editing_path:
            self._status.text = "No point or object is currently being edited"
            return
        try:
            self._ensure_editing_prim()
            position = world_position(self._stage, self._editing_path)
            margin = max(0.0, self._margin.get_value_as_float())
            point_margin = 0.0 if self._editing_type in ("grasp", "place") else margin
            obstacle = point_obstacle(self._stage, position, point_margin)
            if obstacle:
                self._status.text = f"Cannot confirm: TCP overlaps {obstacle}"
                print(
                    f"[OpenArmTaskStudio] Confirm blocked: {self._editing_type} at {position} "
                    f"overlaps {obstacle} with margin={point_margin:.3f}"
                )
                return
            dimensions = self._dimensions()
            parameters = {"position": position}
            if self._joint_override_active:
                parameters["joint_positions"] = self._joint_positions()
            if self._editing_type in ("grasp", "place"):
                dimensions, clearance = self._constrain_grasp_dimensions(dimensions)
                self._set_dimension_models(dimensions, clearance)
                width = self._grip_width(dimensions)
                parameters.update({
                    "shape": self._shape, "dimensions": dimensions,
                    "grasp_orientation": self._grasp_mode,
                    "object_width": width,
                    "preopen_width": width + clearance,
                    "approach_height": max(0.02, self._approach_height.get_value_as_float()),
                    "grasp_height_offset": max(0.0, self._grasp_height_offset.get_value_as_float()),
                    "minimum_wrist_clearance": max(0.0, self._minimum_wrist_clearance.get_value_as_float()),
                    "approach_direction": [0.0, 0.0, -1.0],
                })
            node = TaskNode(
                node_id=str(uuid.uuid4()), task_type=self._editing_type,
                name=self._next_name(self._editing_type),
                prim_path=self._editing_path, parameters=parameters,
                arm=self._active_arm,
            )
            if any(existing.prim_path == node.prim_path for existing in self._document.nodes):
                raise RuntimeError(f"Task point USD path is already in use: {node.prim_path}")
            set_confirmed(self._stage, self._editing_path)
            insert_index = len(self._document.nodes)
            if self._editing_insert_before_node_id:
                insert_index = next(
                    (
                        index for index, existing in enumerate(self._document.nodes)
                        if existing.node_id == self._editing_insert_before_node_id
                    ),
                    insert_index,
                )
            self._document.nodes.insert(insert_index, node)
            self._editing_path = self._editing_type = None
            self._editing_insert_before_node_id = None
            self._selected_node_index = insert_index
            self._load_node(self._selected_node_index)
            segment_count = self._refresh_summary()
            self._status.text = f"Confirmed {node.name}; task path has {segment_count} segment(s)"
            print(
                f"[OpenArmTaskStudio] Confirmed {node.name}: type={node.task_type}, "
                f"position={position}, path_segments={segment_count}"
            )
        except Exception as exc:
            self._status.text = f"Confirm failed: {exc}"
            print(f"[OpenArmTaskStudio] Confirm failed: {exc}")

    def _cancel_edit(self):
        runtime.d435_teaching_enabled = False
        if self._editing_path and self._stage:
            self._stage.RemovePrim(self._editing_path)
        self._editing_path = self._editing_type = None
        self._editing_insert_before_node_id = None
        self._show_empty_parameter_state()
        self._status.text = "Current edit cancelled"

    def _delete_selected_point(self):
        stop_preview()
        if self._editing_path:
            self._cancel_edit()
            return
        if self._selected_node_index is None or not self._document.nodes:
            orphan_path = self._selected_task_object_root()
            if orphan_path:
                self._stage.RemovePrim(orphan_path)
                omni.usd.get_context().get_selection().set_selected_prim_paths([], True)
                self._refresh_summary()
                self._status.text = f"Deleted untracked task object: {orphan_path.rsplit('/', 1)[-1]}"
                return
            self._status.text = "Select a task point to delete"
            return
        index = self._selected_node_index
        node = self._document.nodes.pop(index)
        if self._stage.GetPrimAtPath(node.prim_path).IsValid():
            self._stage.RemovePrim(node.prim_path)
        self._selected_node_index = None
        self._selected_path_segment = None
        self._show_empty_parameter_state()
        omni.usd.get_context().get_selection().set_selected_prim_paths([], True)
        self._refresh_summary()
        self._status.text = f"Deleted task point: {node.name}"

    def _convert_selected_move(self, task_type):
        if task_type not in ("grasp", "place"):
            raise ValueError(f"Unsupported conversion target: {task_type}")
        if self._editing_path:
            self._status.text = "Confirm or Cancel the current point before converting"
            return False
        if self._selected_node_index is None:
            self._status.text = "Select a confirmed Move point to convert"
            return False
        node = self._document.nodes[self._selected_node_index]
        if node.task_type != "move":
            self._status.text = f"Only Move points can be converted; {node.name} is {node.task_type.upper()}"
            return False
        try:
            position = world_position(self._stage, node.prim_path)
            dimensions = list(DEFAULT_OBJECT_DIMENSIONS)
            clearance = 0.01
            width = dimensions[1]
            new_path = create_object_proxy(
                self._stage, self._prim_index_from_path(node.prim_path), task_type,
                "box", position, dimensions, node.arm,
            )
            set_confirmed(self._stage, new_path)
            old_path = node.prim_path
            parameters = {
                "position": position,
                "shape": "box",
                "dimensions": dimensions,
                "grasp_orientation": "vertical",
                "object_width": width,
                "preopen_width": width + clearance,
                "approach_height": max(0.02, self._document.default_approach_height),
                "grasp_height_offset": 0.0,
                "minimum_wrist_clearance": 0.0,
                "approach_direction": [0.0, 0.0, -1.0],
            }
            if node.parameters.get("joint_positions"):
                parameters["joint_positions"] = list(node.parameters["joint_positions"])
            node.task_type = task_type
            node.prim_path = new_path
            node.parameters = parameters
            if old_path != new_path and self._stage.GetPrimAtPath(old_path).IsValid():
                self._stage.RemovePrim(old_path)
            self._load_node(self._selected_node_index)
            self._select(new_path)
            segment_count = self._refresh_summary()
            self._status.text = (
                f"Converted {node.name} to {task_type.upper()}; "
                f"adjust object and grasp parameters as needed ({segment_count} path segments)"
            )
            return True
        except Exception as exc:
            self._status.text = f"Point conversion failed: {exc}"
            print(f"[OpenArmTaskStudio] Point conversion failed: {type(exc).__name__}: {exc}")
            return False

    def _clear_all_points(self):
        stop_preview()
        if self._editing_path and self._stage.GetPrimAtPath(self._editing_path).IsValid():
            self._stage.RemovePrim(self._editing_path)
        self._editing_path = self._editing_type = None
        self._editing_insert_before_node_id = None
        for node in self._document.nodes:
            if self._stage.GetPrimAtPath(node.prim_path).IsValid():
                self._stage.RemovePrim(node.prim_path)
        count = len(self._document.nodes)
        self._document.nodes.clear()
        self._selected_node_index = None
        self._selected_path_segment = None
        self._show_empty_parameter_state()
        omni.usd.get_context().get_selection().set_selected_prim_paths([], True)
        self._refresh_summary()
        self._status.text = f"Cleared {count} task point(s)"

    def _next_name(self, task_type):
        used = {node.name for node in self._document.nodes}
        number = 1
        while f"{task_type}{number}" in used:
            number += 1
        return f"{task_type}{number}"

    def _set_active_arm(self, arm):
        if self._editing_path:
            self._status.text = "Confirm or Cancel the current point before switching arms"
            return
        self._active_arm = arm
        self._arm_status.text = f"Active task arm: {arm.upper()}"
        self._joint_arm_label.text = f"{arm.upper()} arm motor angles (rad)"
        self._status.text = f"New task points will be assigned to the {arm} arm"

    def _can_begin_edit(self):
        if not self._editing_path:
            return True
        self._select(self._editing_path)
        self._show_and_dock_parameter_window()
        self._status.text = "Finish the current point with Confirm or Cancel before creating another"
        return False

    def _selected_task_object_root(self):
        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        task_root = "/World/OpenArmTaskStudio/TaskObjects/"
        if not paths or not paths[0].startswith(task_root):
            return None
        relative = paths[0][len(task_root):]
        if not relative:
            return None
        return task_root + relative.split("/", 1)[0]

    def _next_prim_index(self):
        indices = []
        paths = [node.prim_path for node in self._document.nodes]
        if self._editing_path:
            paths.append(self._editing_path)
        for path in paths:
            match = re.search(r"_(\d+)(?:_|$)", path.rsplit("/", 1)[-1])
            if match:
                indices.append(int(match.group(1)))
        self._prim_serial = max([self._prim_serial, *indices], default=0) + 1
        return self._prim_serial

    @staticmethod
    def _prim_index_from_path(path):
        match = re.search(r"_(\d+)$", path.rsplit("/", 1)[-1])
        if not match:
            raise RuntimeError(f"Task Prim path has no numeric index: {path}")
        return int(match.group(1))

    def _ensure_editing_prim(self):
        if not self._editing_path or self._stage.GetPrimAtPath(self._editing_path).IsValid():
            return
        index = self._prim_index_from_path(self._editing_path)
        position = [
            self._pos_x.get_value_as_float(),
            self._pos_y.get_value_as_float(),
            self._pos_z.get_value_as_float(),
        ]
        expected_path = self._editing_path
        if self._editing_type == "move":
            restored_path = create_move_point(self._stage, index, position, self._active_arm)
        else:
            restored_path = create_object_proxy(
                self._stage, index, self._editing_type, self._shape,
                position, self._dimensions(), self._active_arm,
            )
        if restored_path != expected_path:
            raise RuntimeError(f"Restored editing Prim path changed: {restored_path}")
        print(f"[OpenArmTaskStudio] Restored missing editing Prim: {expected_path}")

    def _ensure_node_prim(self, node):
        if self._stage.GetPrimAtPath(node.prim_path).IsValid():
            return False
        index = self._prim_index_from_path(node.prim_path)
        position = list(node.parameters["position"])
        expected_path = node.prim_path
        if node.task_type == "move":
            restored_path = create_move_point(self._stage, index, position, node.arm)
        else:
            restored_path = create_object_proxy(
                self._stage, index, node.task_type,
                node.parameters.get("shape", "box"), position,
                node.parameters.get("dimensions", DEFAULT_OBJECT_DIMENSIONS), node.arm,
            )
        if restored_path != expected_path:
            raise RuntimeError(f"Restored node Prim path changed: {restored_path}")
        set_confirmed(self._stage, restored_path)
        print(f"[OpenArmTaskStudio] Restored missing node Prim: {node.name} -> {restored_path}")
        return True

    def _ensure_all_node_prims(self):
        restored = 0
        for node in self._document.nodes:
            restored += int(self._ensure_node_prim(node))
        return restored

    def _load_editing_models(self):
        self._loading_models = True
        try:
            name = self._next_name(self._editing_type)
            self._parameter_title.text = f"New {name} | {self._editing_type.upper()}"
            self._name_model.set_value(name)
            self._order_model.set_value(len(self._document.nodes) + 1)
            position = world_position(self._stage, self._editing_path)
            self._pos_x.set_value(position[0])
            self._pos_y.set_value(position[1])
            self._pos_z.set_value(position[2])
            current = self._current_arm_joints() or [0.0] * 7
            for model, value in zip(self._joint_models, current):
                model.set_value(float(value))
            self._joint_override_active = False
            self._joint_status.text = "Joint override: disabled until edited or captured"
            if self._editing_type in ("grasp", "place"):
                self._grasp_height_offset.set_value(0.0)
                self._minimum_wrist_clearance.set_value(0.0)
        finally:
            self._loading_models = False

    def _show_parameter_window(self):
        if self._editing_path:
            self._load_editing_models()
        elif self._selected_node_index is not None:
            self._load_node(self._selected_node_index)
        else:
            self._status.text = "Select or create a task point first"
            return
        self._show_and_dock_parameter_window()

    def _on_position_changed(self):
        if self._loading_models or not self._stage:
            return
        path = self._editing_path
        node = None
        if not path and self._selected_node_index is not None:
            node = self._document.nodes[self._selected_node_index]
            path = node.prim_path
        if not path:
            return
        position = [self._pos_x.get_value_as_float(), self._pos_y.get_value_as_float(), self._pos_z.get_value_as_float()]
        set_world_position(self._stage, path, position)
        if node:
            node.parameters["position"] = position
            arm_nodes = [item for item in self._document.nodes if item.arm == node.arm]
            planned, insert_targets = self._planned_path_with_insert_targets(arm_nodes)
            update_task_path(
                self._stage, arm_nodes, planned, node.arm, insert_targets
            )

    def _reset_robot(self):
        runtime.d435_teaching_enabled = False
        request_reset()
        self._status.text = "Resetting both arms; both grippers open"

    def _run_full_task(self):
        # Run is also a hard control handoff in case the user skips the Stop button.
        stop_motion_teaching()
        self._d435_status.text = "Motion teaching: stopped for task execution"
        if not self._validate_workspace():
            print(
                f"[OpenArmTaskStudio] Run blocked during validation: "
                f"{self._status.text} (confirmed_nodes={len(self._document.nodes)})"
            )
            return
        speed_scale = max(0.1, min(3.0, self._execution_speed.get_value_as_float()))
        self._execution_speed.set_value(speed_scale)
        self._document.execution_speed = speed_scale
        arm_waypoints = {
            arm: self._build_arm_waypoints(
                [node for node in self._document.nodes if node.arm == arm], arm
            )
            for arm in ("left", "right")
        }
        arm_waypoints = {arm: points for arm, points in arm_waypoints.items() if points}
        if not arm_waypoints:
            self._status.text = "Run blocked: confirmed points compiled to no executable waypoints"
            print(f"[OpenArmTaskStudio] {self._status.text}")
            return
        request_execution(arm_waypoints)
        counts = ", ".join(f"{arm}={len(points)}" for arm, points in arm_waypoints.items())
        self._status.text = f"Running at {speed_scale:.2f}x: {counts} waypoint(s)"
        print(
            f"[OpenArmTaskStudio] Full task requested: confirmed_nodes={len(self._document.nodes)}, "
            f"{counts}, generation={runtime.preview_generation}"
        )

    def _build_arm_waypoints(self, nodes, arm):
        current_joints = (
            runtime.current_right_joint_positions if arm == "right"
            else runtime.current_left_joint_positions
        ) or [0.0] * 7
        return self._task_compiler().compile_arm(
            nodes, arm, current_joints, self._document.execution_speed
        )

    def _action_tcp_position(self, object_position, parameters):
        return self._task_compiler().action_tcp_position(object_position, parameters)

    def _task_compiler(self):
        table_height = self._text_float_value(self._table_height)
        if table_height is None:
            table_height = DEFAULT_WORKBENCH_HEIGHT
        config = CompilerConfig(
            collision_margin=max(0.0, self._margin.get_value_as_float()),
            default_approach_height=self._document.default_approach_height,
            max_cartesian_step=MAX_CARTESIAN_TRANSIT_STEP,
            table_height=table_height,
            horizontal_grasp_angles=(
                LEFT_HORIZONTAL_GRASP_ANGLE, RIGHT_HORIZONTAL_GRASP_ANGLE
            ),
        )
        return TaskCompiler(
            config,
            segment_blocked=lambda start, end, margin: bool(
                segment_obstacle(self._stage, start, end, margin)
            ),
            safe_transit_height=self._safe_transit_height,
            joint_limits=lambda arm: (
                runtime.current_right_joint_limits
                if arm == "right" else runtime.current_left_joint_limits
            ),
        )

    def _run_full_task_safe(self):
        try:
            self._run_full_task()
        except Exception as exc:
            self._status.text = f"Run failed: {exc}"
            print(f"[OpenArmTaskStudio] Run failed: {type(exc).__name__}: {exc}")

    def _safe_transit_height(self):
        bounds = scene_obstacle_bounds(self._stage, max(0.0, self._margin.get_value_as_float()))
        obstacle_top = max((maximum[2] for _path, _minimum, maximum in bounds), default=1.20)
        return obstacle_top + 0.06

    def _waypoint(self, position, gripper, label, action=None, object_path=None, wrist_angle=None, object_parameters=None, joint_positions=None, lock_wrist=False, arm=None):
        waypoint = {
            "position": list(position), "gripper": gripper, "label": label,
        }
        if action:
            waypoint["action"] = action
        if object_path:
            waypoint["object_path"] = object_path
        if wrist_angle is not None:
            waypoint["wrist_angle"] = wrist_angle
        if lock_wrist:
            waypoint["lock_wrist"] = True
            waypoint["level_gripper"] = True
        if object_parameters:
            waypoint["shape"] = object_parameters.get("shape", "box")
            waypoint["dimensions"] = list(object_parameters.get("dimensions", DEFAULT_OBJECT_DIMENSIONS))
        if joint_positions:
            waypoint["joint_positions"] = list(joint_positions)
        if arm:
            waypoint["arm"] = arm
        return waypoint

    def _planned_positions(self, nodes):
        arm = nodes[0].arm if nodes else self._active_arm
        return self._task_compiler().planned_positions(nodes, arm)

    def _planned_path_with_insert_targets(self, nodes):
        positions = self._planned_positions(nodes)
        targets = [None] * max(0, len(positions) - 1)
        previous_count = len(self._planned_positions(nodes[:1])) if nodes else 0
        for node_index in range(1, len(nodes)):
            prefix_count = len(self._planned_positions(nodes[:node_index + 1]))
            start_segment = max(0, previous_count - 1)
            for segment_index in range(start_segment, max(0, prefix_count - 1)):
                if segment_index < len(targets):
                    targets[segment_index] = nodes[node_index].node_id
            previous_count = prefix_count
        return positions, targets

    def _validate_workspace(self):
        move_nodes = [node for node in self._document.nodes if "position" in node.parameters]
        if not move_nodes:
            self._status.text = "No confirmed spatial task nodes to validate"
            return False
        try:
            restored = self._ensure_all_node_prims()
            if restored:
                self._status.text = f"Restored {restored} missing task point(s); validating path"
        except Exception as exc:
            self._status.text = f"Task point recovery failed: {exc}"
            print(f"[OpenArmTaskStudio] {self._status.text}")
            return False
        for node in move_nodes:
            node.parameters["position"] = world_position(self._stage, node.prim_path)
            if node.task_type in ("grasp", "place"):
                width = float(node.parameters.get("object_width", 0.0))
                preopen = float(node.parameters.get("preopen_width", width))
                if width > MAX_GRIPPER_OPENING or preopen > MAX_GRIPPER_OPENING:
                    self._status.text = (
                        f"{node.name} exceeds gripper opening: {preopen:.3f} m > {MAX_GRIPPER_OPENING:.3f} m"
                    )
                    print(f"[OpenArmTaskStudio] Validation blocked: {self._status.text}")
                    return False
        self._document.collision_margin = max(0.0, self._margin.get_value_as_float())
        self._document.default_approach_height = max(0.02, self._approach_height.get_value_as_float())
        action_targets = {
            tuple(float(value) for value in node.parameters["position"])
            for node in move_nodes if node.task_type in ("grasp", "place")
        }
        for arm in ("left", "right"):
            arm_nodes = [node for node in move_nodes if node.arm == arm]
            planned_waypoints = self._build_arm_waypoints(arm_nodes, arm)
            path_points = [
                (f"{arm} planned waypoint {index + 1}", list(waypoint["position"]), waypoint)
                for index, waypoint in enumerate(planned_waypoints)
            ]
            for label, point, waypoint in path_points:
                is_recorded_joint_move = (
                    waypoint.get("task_type") == "move"
                    and waypoint.get("control_mode") == "joint"
                )
                margin = (
                    0.0
                    if is_recorded_joint_move
                    or tuple(float(value) for value in point) in action_targets
                    else self._document.collision_margin
                )
                obstacle = point_obstacle(self._stage, point, margin)
                if obstacle:
                    self._status.text = f"Scene collision at {label}: {obstacle}"
                    print(f"[OpenArmTaskStudio] Validation blocked: {self._status.text}")
                    return False
            for (start_label, start, start_waypoint), (end_label, end, end_waypoint) in zip(path_points, path_points[1:]):
                if (
                    start_waypoint.get("control_mode") == "joint"
                    or end_waypoint.get("control_mode") == "joint"
                ):
                    # Joint-space replay does not follow the straight TCP segment.
                    continue
                touches_action = any(
                    tuple(float(value) for value in point) in action_targets for point in (start, end)
                )
                margin = 0.0 if touches_action else self._document.collision_margin
                obstacle = segment_obstacle(self._stage, start, end, margin)
                if obstacle:
                    self._status.text = f"Path collision {start_label} -> {end_label}: {obstacle}"
                    print(f"[OpenArmTaskStudio] Validation blocked: {self._status.text}")
                    return False
        self._status.text = "TCP scene path passed. Whole-arm swept collision still requires a planner."
        return True

    def _save(self):
        try:
            self._document.collision_margin = max(0.0, self._margin.get_value_as_float())
            self._document.default_approach_height = max(0.02, self._approach_height.get_value_as_float())
            self._document.execution_speed = max(0.1, min(3.0, self._execution_speed.get_value_as_float()))
            self._document.front_workbench_enabled = self._table_enabled.get_value_as_bool()
            self._document.front_workbench_top_height = self._text_float_value(self._table_height) or DEFAULT_WORKBENCH_HEIGHT
            distance = self._text_float_value(self._table_distance)
            self._document.front_workbench_distance = 0.20 if distance is None else distance
            self._document.front_workbench_base_z = 0.0
            for node in self._document.nodes:
                if "position" in node.parameters:
                    node.parameters["position"] = world_position(self._stage, node.prim_path)
            if SAVE_PATH.exists():
                backup_dir = SAVE_PATH.parent / "backups"
                backup_dir.mkdir(parents=True, exist_ok=True)
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                shutil.copy2(SAVE_PATH, backup_dir / f"{SAVE_PATH.stem}_{timestamp}.json")
            self._document.save(SAVE_PATH)
            self._status.text = f"Task saved: {SAVE_PATH}"
        except Exception as exc:
            self._status.text = f"Save failed: {exc}"

    def _show_load_window(self):
        task_files = sorted(SAVE_PATH.parent.glob("*.json"))
        self._load_window = ui.Window("Load Task Group", width=480, height=360, visible=True)
        with self._load_window.frame:
            with ui.ScrollingFrame():
                with ui.VStack(spacing=6):
                    ui.Label("Task groups in tasks/", height=28)
                    if not task_files:
                        ui.Label("No task JSON files found")
                    for path in task_files:
                        ui.Button(path.name, clicked_fn=lambda selected=path: self._load_task(selected), height=32)

    def _load_task(self, path):
        try:
            document = TaskDocument.load(path)
            request_reset()
            self._loading_models = True
            legacy_scene = document.front_workbench_top_height is None
            z_offset = -document.robot_base_z
            if legacy_scene:
                document.front_workbench_enabled = True
                document.front_workbench_top_height = 0.89
                document.front_workbench_distance = 0.20
            document.front_workbench_base_z = configure_front_workbench(
                self._stage,
                document.front_workbench_enabled,
                document.front_workbench_top_height,
                document.front_workbench_distance,
            )
            self._capture_locked_scene_xforms(force=True)
            document.robot_base_z = 0.0
            task_root = self._stage.GetPrimAtPath("/World/OpenArmTaskStudio/TaskObjects")
            if task_root.IsValid():
                self._stage.RemovePrim(task_root.GetPath())
            self._document = document
            for index, node in enumerate(document.nodes, start=1):
                position = [float(value) for value in node.parameters.get("position", self._default_position())]
                position[2] += z_offset
                node.parameters["position"] = list(position)
                if node.task_type == "move":
                    node.prim_path = create_move_point(self._stage, index, position, node.arm)
                else:
                    shape = node.parameters.get("shape", "box")
                    dimensions = node.parameters.get("dimensions", DEFAULT_OBJECT_DIMENSIONS)
                    node.prim_path = create_object_proxy(
                        self._stage, index, node.task_type, shape, position, dimensions, node.arm
                    )
                set_confirmed(self._stage, node.prim_path)
                print(f"[OpenArmTaskStudio] Imported {node.name} at {position}")
            self._prim_serial = max(self._prim_serial, len(document.nodes))
            self._selected_node_index = None
            self._selected_path_segment = None
            self._editing_path = self._editing_type = None
            self._editing_insert_before_node_id = None
            self._margin.set_value(document.collision_margin)
            self._approach_height.set_value(document.default_approach_height)
            self._execution_speed.set_value(max(0.1, min(3.0, document.execution_speed)))
            self._table_enabled.set_value(document.front_workbench_enabled)
            self._table_height.set_value(f"{document.front_workbench_top_height:.3f}")
            self._table_distance.set_value(f"{document.front_workbench_distance:.3f}")
            self._refresh_summary()
            if self._load_window:
                self._load_window.visible = False
            self._status.text = f"Loaded task group: {path.name} ({len(document.nodes)} points)"
        except Exception as exc:
            self._status.text = f"Load failed: {exc}"
        finally:
            self._loading_models = False

    def _refresh_summary(self):
        lines = [f"{index + 1}. {node.arm.upper()} | {node.task_type.upper()} - {node.name}" for index, node in enumerate(self._document.nodes)]
        self._task_summary.text = f"{len(lines)} confirmed task nodes\n" + "\n".join(lines)
        if self._stage:
            self._ensure_all_node_prims()
            segment_count = 0
            for arm in ("left", "right"):
                arm_nodes = [node for node in self._document.nodes if node.arm == arm]
                planned, insert_targets = self._planned_path_with_insert_targets(arm_nodes)
                segment_count += update_task_path(
                    self._stage, arm_nodes, planned, arm, insert_targets
                )
            self._last_path_positions = [list(node.parameters.get("position", [])) for node in self._document.nodes]
            return segment_count
        return 0

    def _on_update(self, _event):
        self._restore_locked_scene_xforms()
        self._update_counter += 1
        if self._update_counter in self._workspace_layout_passes:
            self._apply_default_workspace_layout()
        self._update_d435_preview()
        self._update_button_test()
        if runtime.motion_monitor_active:
            self._motion_monitor_status.text = (
                f"Monitor: {runtime.motion_monitor_samples} samples | "
                f"{Path(runtime.motion_monitor_path).name}"
            )
            parts = []
            for arm in ("left", "right"):
                stats = runtime.motion_monitor_stats.get(arm, {})
                if stats.get("samples"):
                    parts.append(
                        f"{arm.upper()} age {stats['packet_age_ms_mean']:.0f}/{stats['packet_age_ms_max']:.0f} ms | "
                        f"err {stats['command_error_rad_mean']:.3f}/{stats['command_error_rad_max']:.3f} rad"
                    )
            self._motion_monitor_metrics.text = "\n".join(parts) if parts else "Waiting for tracked samples"
        elif runtime.motion_monitor_path:
            self._motion_monitor_status.text = f"Monitor saved: {Path(runtime.motion_monitor_path).name}"
        if self._update_counter % 10:
            return
        self._sync_joint_slider_limits()
        while self._stage and self._handled_remote_record_count < runtime.remote_record_press_count:
            self._handled_remote_record_count += 1
            self._capture_motion_point()
        if runtime.d435_teaching_enabled:
            controlled_arms = (
                ("left", "right") if runtime.d435_teaching_mode == "both"
                else (runtime.d435_teaching_mode,)
            )
            lost_arms = [arm.upper() for arm in controlled_arms if not runtime.d435_tracking_valid[arm]]
            if runtime.d435_packet_count and lost_arms:
                self._d435_status.text = (
                    f"TRACKING LOST: {','.join(lost_arms)} holding; mode={runtime.d435_teaching_mode.upper()}"
                )
            elif runtime.d435_packet_count:
                self._d435_status.text = (
                    f"MOTION ACTIVE: {runtime.d435_teaching_mode.upper()}; "
                    f"tracking packets={runtime.d435_packet_count}; "
                    f"filtered spikes={runtime.d435_filtered_spike_count}"
                )
            elif runtime.d435_raw_packet_count:
                self._d435_status.text = (
                    f"Calibrating: packets={runtime.d435_raw_packet_count}; hold neutral pose"
                )
            elif self._d435_camera_process and self._d435_camera_process.poll() is not None:
                self._d435_status.text = "Motion camera exited; check the Task Studio terminal"
            else:
                self._d435_status.text = "Motion camera starting; waiting for tracking data"
        if not self._stage or not self._document.nodes:
            return
        if runtime.command_kind == "execute":
            return
        try:
            request = runtime.preview_request or {}
            sync_path = request.get("sync_move_point_path")
            if sync_path and self._stage.GetPrimAtPath(sync_path).IsValid():
                tcp_position = (
                    runtime.current_right_tcp_position
                    if request.get("sync_move_point_arm") == "right"
                    else runtime.current_left_tcp_position
                )
                if tcp_position:
                    set_world_position(self._stage, sync_path, tcp_position)
                    if self._selected_node_index is not None:
                        node = self._document.nodes[self._selected_node_index]
                        if node.prim_path == sync_path:
                            node.parameters["position"] = list(tcp_position)
                            self._loading_models = True
                            try:
                                self._pos_x.set_value(tcp_position[0])
                                self._pos_y.set_value(tcp_position[1])
                                self._pos_z.set_value(tcp_position[2])
                            finally:
                                self._loading_models = False
            self._ensure_all_node_prims()
            positions = [world_position(self._stage, node.prim_path) for node in self._document.nodes]
            if self._last_path_positions and len(positions) == len(self._last_path_positions):
                unchanged = all(
                    all(abs(current[axis] - previous[axis]) < 1e-5 for axis in range(3))
                    for current, previous in zip(positions, self._last_path_positions)
                )
                if unchanged:
                    return
            for node, position in zip(self._document.nodes, positions):
                node.parameters["position"] = position
            for arm in ("left", "right"):
                arm_nodes = [node for node in self._document.nodes if node.arm == arm]
                planned, insert_targets = self._planned_path_with_insert_targets(arm_nodes)
                update_task_path(
                    self._stage, arm_nodes, planned, arm, insert_targets
                )
            self._last_path_positions = [list(position) for position in positions]
        except RuntimeError:
            pass

    def _capture_locked_scene_xforms(self, force=False):
        stage = self._stage
        if not stage:
            return
        if force or self._locked_stage is not stage:
            self._locked_stage = stage
            self._locked_scene_xforms = {}
        paths = []
        for root_path in LOCKED_SCENE_ROOTS:
            root = stage.GetPrimAtPath(root_path)
            if not root.IsValid():
                continue
            paths.append(root_path)
            if root_path != ROBOT_PRIM_PATH:
                paths.extend(
                    str(prim.GetPath()) for prim in Usd.PrimRange(root)
                    if prim.GetPath() != root.GetPath()
                )
        for path in paths:
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid() or path in self._locked_scene_xforms:
                continue
            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                continue
            operations = [(op.GetName(), op.Get()) for op in xformable.GetOrderedXformOps()]
            order_attribute = prim.GetAttribute("xformOpOrder")
            order = order_attribute.Get() if order_attribute else []
            order = order or []
            self._locked_scene_xforms[path] = (operations, order)

    def _restore_locked_scene_xforms(self):
        self._capture_locked_scene_xforms()
        stage = self._stage
        if not stage:
            return
        for path, (operations, order) in self._locked_scene_xforms.items():
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            for attribute_name, expected in operations:
                attribute = prim.GetAttribute(attribute_name)
                if attribute and attribute.Get() != expected:
                    attribute.Set(expected)
            order_attribute = prim.GetAttribute("xformOpOrder")
            if order_attribute and order_attribute.Get() != order:
                order_attribute.Set(order)

    def _sync_joint_slider_limits(self):
        limits = self._current_arm_limits()
        if not limits or limits == self._applied_joint_limits:
            return
        self._applied_joint_limits = [list(limit) for limit in limits]
        for slider, (minimum, maximum) in zip(self._joint_sliders, limits):
            try:
                slider.min = float(minimum)
                slider.max = float(maximum)
            except AttributeError:
                pass

    def _on_stage_event(self, event):
        if event.type != int(omni.usd.StageEventType.SELECTION_CHANGED):
            return
        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if not paths:
            return
        selected = paths[0]
        path_root = "/World/OpenArmTaskStudio/TaskPathSegments/"
        if selected.startswith(path_root):
            prim = self._stage.GetPrimAtPath(selected)
            attribute = prim.GetAttribute("openarm:insertBeforeNodeId") if prim.IsValid() else None
            insert_before_node_id = attribute.Get() if attribute else ""
            if insert_before_node_id:
                self._selected_path_segment = (selected, insert_before_node_id)
                self._selected_node_index = None
                self._show_empty_parameter_state()
                next_node = next(
                    (
                        node for node in self._document.nodes
                        if node.node_id == insert_before_node_id
                    ),
                    None,
                )
                next_name = next_node.name if next_node else "the next point"
                self._status.text = (
                    f"Path line selected; Add Move Point will insert before {next_name}"
                )
                return
            omni.usd.get_context().get_selection().set_selected_prim_paths([], True)
            self._selected_path_segment = None
            self._status.text = "This line is inside one task action and cannot accept a point"
            return
        if self._editing_path and (
            selected == self._editing_path or selected.startswith(self._editing_path + "/")
        ):
            if selected != self._editing_path:
                self._select(self._editing_path)
            return
        for index, node in enumerate(self._document.nodes):
            if selected == node.prim_path or selected.startswith(node.prim_path + "/"):
                if selected != node.prim_path:
                    self._select(node.prim_path)
                self._selected_node_index = index
                self._selected_path_segment = None
                self._load_node(index)
                self._show_and_dock_parameter_window()
                return
        orphan_path = self._selected_task_object_root()
        if orphan_path and self._stage.GetPrimAtPath(orphan_path).IsValid():
            if selected != orphan_path:
                self._select(orphan_path)
            self._selected_node_index = None
            self._show_empty_parameter_state()
            self._status.text = "Untracked task object selected; use Delete Selected Point to remove it"
            return
        omni.usd.get_context().get_selection().set_selected_prim_paths([], True)
        self._selected_path_segment = None
        self._status.text = "Scene geometry is locked; only task points can be selected and moved"

    def _load_node(self, index):
        node = self._document.nodes[index]
        self._active_arm = node.arm
        self._arm_status.text = f"Active task arm: {node.arm.upper()}"
        self._joint_arm_label.text = f"{node.arm.upper()} arm motor angles (rad)"
        self._loading_models = True
        try:
            self._parameter_title.text = f"{node.name} | {node.task_type.upper()}"
            self._name_model.set_value(node.name)
            self._order_model.set_value(index + 1)
            parameters = node.parameters
            position = world_position(self._stage, node.prim_path)
            parameters["position"] = position
            self._pos_x.set_value(position[0])
            self._pos_y.set_value(position[1])
            self._pos_z.set_value(position[2])
            stored_joints = parameters.get("joint_positions")
            self._set_joint_models(stored_joints or self._current_arm_joints() or [0.0] * 7)
            self._joint_override_active = stored_joints is not None
            self._joint_status.text = "Joint override: enabled" if stored_joints is not None else "Joint override: disabled"
            if node.task_type in ("grasp", "place"):
                self._shape = parameters.get("shape", "box")
                self._grasp_mode = parameters.get("grasp_orientation", "vertical")
                dimensions = parameters.get("dimensions", DEFAULT_OBJECT_DIMENSIONS)
                self._dim_x.set_value(float(dimensions[0]))
                self._dim_y.set_value(float(dimensions[1]))
                self._dim_z.set_value(float(dimensions[2]))
                clearance = float(parameters.get("preopen_width", 0.0)) - float(parameters.get("object_width", 0.0))
                self._clearance.set_value(max(0.0, clearance))
                self._approach_height.set_value(float(parameters.get("approach_height", 0.06)))
                self._grasp_height_offset.set_value(float(parameters.get("grasp_height_offset", 0.0)))
                self._minimum_wrist_clearance.set_value(float(parameters.get("minimum_wrist_clearance", 0.0)))
            self._shape_status.text = f"Selected shape: {self._shape}"
            self._grasp_status.text = f"Selected grasp: {self._grasp_mode}"
            self._status.text = f"Editing {node.name} ({node.task_type}), order {index + 1}"
        finally:
            self._loading_models = False

    def _on_name_changed(self):
        if self._loading_models or self._selected_node_index is None:
            return
        name = self._name_model.get_value_as_string().strip()
        if name:
            self._document.nodes[self._selected_node_index].name = name
            self._refresh_summary()

    def _on_order_changed(self):
        if self._loading_models or self._selected_node_index is None or not self._document.nodes:
            return
        target = max(1, min(len(self._document.nodes), self._order_model.get_value_as_int())) - 1
        current = self._selected_node_index
        if target == current:
            return
        node = self._document.nodes.pop(current)
        self._document.nodes.insert(target, node)
        self._selected_node_index = target
        self._loading_models = True
        self._order_model.set_value(target + 1)
        self._loading_models = False
        self._refresh_summary()
        self._status.text = f"Moved {node.name} to execution order {target + 1}"

    def _preview_selected(self):
        if self._selected_node_index is None:
            self._status.text = "Select a confirmed task point before preview"
            return
        node = self._document.nodes[self._selected_node_index]
        position = world_position(self._stage, node.prim_path)
        node.parameters["position"] = position
        margin = max(0.0, self._margin.get_value_as_float())
        point_margin = 0.0 if node.task_type in ("grasp", "place") else margin
        if point_obstacle(self._stage, position, point_margin):
            self._status.text = "Preview blocked: target overlaps scene geometry"
            return
        current_joints = (
            runtime.current_right_joint_positions
            if node.arm == "right" else runtime.current_left_joint_positions
        )
        waypoints = self._task_compiler().compile_preview(
            node, current_joints, self._document.execution_speed
        )
        request_preview({
            "position": waypoints[-1]["position"],
            "waypoints": waypoints,
            "compiled_stage_count": len(waypoints),
        })
        self._status.text = (
            f"Previewing {node.name}: {len(waypoints)} compiled stage(s), {node.arm} arm"
        )

    def _stop_preview(self):
        runtime.d435_teaching_enabled = False
        stop_preview()
        self._status.text = "Robot preview stopped; holding current pose"

    def _run_self_test(self):
        try:
            self._begin_move()
            first_edit_path = self._editing_path
            self._begin_grasp()
            assert self._editing_path == first_edit_path
            assert self._editing_type == "move"
            self._stage.RemovePrim(first_edit_path)
            self._confirm_edit()
            self._begin_grasp()
            set_world_position(
                self._stage, self._editing_path,
                [0.4, 0.04, DEFAULT_WORKBENCH_HEIGHT + DEFAULT_OBJECT_DIMENSIONS[2] * 0.5],
            )
            self._confirm_edit()
            self._begin_place()
            set_world_position(
                self._stage, self._editing_path,
                [0.4, 0.08, DEFAULT_WORKBENCH_HEIGHT + DEFAULT_OBJECT_DIMENSIONS[2] * 0.5],
            )
            self._confirm_edit()
            assert [node.name for node in self._document.nodes] == ["move1", "grasp1", "place1"]
            self._set_active_arm("right")
            self._begin_move()
            set_world_position(self._stage, self._editing_path, [0.4, -0.20, 0.45])
            self._confirm_edit()
            self._begin_move()
            set_world_position(self._stage, self._editing_path, [0.4, -0.16, 0.45])
            self._confirm_edit()
            assert [node.arm for node in self._document.nodes] == ["left", "left", "left", "right", "right"]
            right_second = self._document.nodes[-1]
            right_root = self._stage.GetPrimAtPath(
                "/World/OpenArmTaskStudio/TaskPathSegments/Right"
            )
            insertion_segment = next(
                child for child in right_root.GetChildren()
                if child.GetAttribute("openarm:insertBeforeNodeId").Get() == right_second.node_id
            )
            self._selected_path_segment = (str(insertion_segment.GetPath()), right_second.node_id)
            self._begin_move_on_selected_line()
            assert self._editing_insert_before_node_id == right_second.node_id
            self._confirm_edit()
            right_nodes = [node for node in self._document.nodes if node.arm == "right"]
            assert [node.task_type for node in right_nodes] == ["move", "move", "move"]
            assert right_nodes[-1].node_id == right_second.node_id
            self._selected_node_index = self._document.nodes.index(right_second)
            self._load_node(self._selected_node_index)
            self._on_joint_angles_changed()
            assert self._joint_arm_label.text == "RIGHT arm motor angles (rad)"
            assert runtime.preview_request["waypoints"][0]["arm"] == "right"
            assert runtime.preview_request["sync_move_point_arm"] == "right"
            preserved_id = right_second.node_id
            preserved_name = right_second.name
            preserved_joints = list(right_second.parameters["joint_positions"])
            self._selected_node_index = self._document.nodes.index(right_nodes[0])
            self._load_node(self._selected_node_index)
            assert self._convert_selected_move("grasp")
            assert right_nodes[0].task_type == "grasp"
            self._selected_node_index = self._document.nodes.index(right_second)
            self._load_node(self._selected_node_index)
            assert self._convert_selected_move("place")
            assert right_second.task_type == "place"
            assert right_second.node_id == preserved_id
            assert right_second.name == preserved_name
            assert right_second.arm == "right"
            assert right_second.parameters["joint_positions"] == preserved_joints
            assert right_second.prim_path.startswith("/World/OpenArmTaskStudio/TaskObjects/Place_")
            left_waypoints = self._build_arm_waypoints(
                [node for node in self._document.nodes if node.arm == "left"], "left"
            )
            left_labels = [waypoint["label"] for waypoint in left_waypoints]
            early_index = left_labels.index("grasp1 early pre-align")
            approach_index = next(
                index for index, label in enumerate(left_labels)
                if index > early_index and label.startswith("grasp1 approach")
            )
            prealign_index = left_labels.index("grasp1 pre-align")
            assert early_index < approach_index
            assert not left_waypoints[early_index].get("lock_wrist")
            assert not left_waypoints[approach_index].get("lock_wrist")
            assert left_waypoints[prealign_index].get("lock_wrist")
            assert left_waypoints[prealign_index].get("approach_direction") == [0.0, 0.0, -1.0]
            missing_node_path = self._document.nodes[0].prim_path
            self._stage.RemovePrim(missing_node_path)
            assert self._ensure_all_node_prims() == 1
            assert self._stage.GetPrimAtPath(missing_node_path).IsValid()
            for node in self._document.nodes:
                assert not self._stage.GetPrimAtPath(f"{node.prim_path}/Geometry").IsValid()
                actual = world_position(self._stage, node.prim_path)
                expected = node.parameters["position"]
                assert all(abs(actual[axis] - expected[axis]) < 1e-6 for axis in range(3))
            visible_segments = {}
            for arm in ("Left", "Right"):
                path_root = self._stage.GetPrimAtPath(f"/World/OpenArmTaskStudio/TaskPathSegments/{arm}")
                visible_segments[arm] = [
                    child for child in path_root.GetChildren()
                    if child.GetName().startswith("Segment_")
                    and UsdGeom.Imageable(child).ComputeVisibility() != UsdGeom.Tokens.invisible
                ]
                assert visible_segments[arm]
            print(f"[OpenArmTaskStudio] Task path visualization: left={len(visible_segments['Left'])}, right={len(visible_segments['Right'])}")
            self._preview_selected()
            self._run_full_task()
            with tempfile.TemporaryDirectory() as directory:
                self._document.save(Path(directory) / "self_test_task.json")
            assert len(self._document.nodes) == 6
            self._show_and_dock_d435_window()
            assert ui.Workspace.get_window("Button Test") is not None
            print(
                "[OpenArmTaskStudio] SELF_TEST_PASS nodes=6 dual_arm_paths=ok "
                "line_insert=ok motion_capture_dock=ok "
                "button_test_dock=ok temporary_save=ok"
            )
        except Exception as exc:
            print(f"[OpenArmTaskStudio] SELF_TEST_FAIL {type(exc).__name__}: {exc}")
