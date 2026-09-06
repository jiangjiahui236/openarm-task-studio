from __future__ import annotations


preview_request: dict | None = None
preview_generation = 0
command_kind = "idle"
current_left_joint_positions: list[float] | None = None
current_left_joint_limits: list[list[float]] | None = None
current_right_joint_positions: list[float] | None = None
current_right_joint_limits: list[list[float]] | None = None
current_left_tcp_position: list[float] | None = None
current_right_tcp_position: list[float] | None = None
d435_teaching_enabled = False
d435_teaching_mode = "left"
d435_packet_count = 0
d435_raw_packet_count = 0
d435_filtered_spike_count = 0
d435_tracking_valid = {"left": False, "right": False}
d435_latest_targets = {"left": None, "right": None}
d435_latest_target_time = 0.0
motion_monitor_active = False
motion_monitor_path = ""
motion_monitor_samples = 0
motion_monitor_stats = {"left": {}, "right": {}}
manual_gripper_generation = 0
manual_gripper_state = "open"
remote_record_press_count = 0
remote_button_last_packet_time = 0.0
remote_button_last_press_time = 0.0
remote_button_address = None
remote_button_is_down = False
remote_button_sequence = None


def request_manual_gripper(state: str) -> None:
    global manual_gripper_generation, manual_gripper_state
    if state not in ("open", "closed"):
        raise ValueError(f"unsupported gripper state: {state}")
    manual_gripper_state = state
    manual_gripper_generation += 1


def execution_needs_initial_transit(waypoints: list[dict]) -> bool:
    return bool(waypoints and waypoints[0].get("control_mode") != "joint")


def request_preview(parameters: dict) -> None:
    global preview_request, preview_generation, command_kind
    preview_request = dict(parameters)
    preview_generation += 1
    command_kind = "preview"


def request_execution(waypoints: list[dict] | dict[str, list[dict]]) -> None:
    global preview_request, preview_generation, command_kind
    if isinstance(waypoints, dict):
        preview_request = {"arm_waypoints": waypoints}
    else:
        preview_request = {"waypoints": waypoints, "position": waypoints[-1]["position"]}
    preview_generation += 1
    command_kind = "execute"


def request_reset() -> None:
    global preview_request, preview_generation, command_kind
    preview_request = None
    preview_generation += 1
    command_kind = "reset"


def stop_preview() -> None:
    global preview_request, preview_generation, command_kind
    preview_request = None
    preview_generation += 1
    command_kind = "idle"


def stop_motion_teaching() -> None:
    """Atomically hand control back from motion teaching to task execution."""
    global d435_teaching_enabled
    d435_teaching_enabled = False
    d435_tracking_valid["left"] = False
    d435_tracking_valid["right"] = False
    stop_preview()
