from __future__ import annotations

import argparse
import os
import time

from isaaclab.app import AppLauncher


parser = argparse.ArgumentParser(description="Launch the OpenArm Task Studio extension.")
parser.add_argument("--duration", type=float, default=0.0, help="Exit after this many seconds; 0 keeps the app open.")
parser.add_argument("--num-envs", type=int, default=1, help="Number of studio scenes; the editor currently supports one.")
parser.add_argument("--workbench-config-test", action="store_true", help="Exercise workbench transforms and collision toggling.")
parser.add_argument("--d435-port", type=int, default=5010, help="UDP port for D435 coarse teaching.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()

launcher = AppLauncher(args)
simulation_app = launcher.app

import isaaclab.sim as sim_utils
import torch
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.scene import InteractiveScene
from omni.isaac.dynamic_control import _dynamic_control

from studio_scene import OpenArmTaskStudioSceneCfg
from openarm_task_studio import runtime
from openarm_task_studio import extension as task_studio_extension
from openarm_task_studio.d435_teaching import D435TeachingReceiver
from openarm_task_studio.motion_monitor import MotionExecutionMonitor
from openarm_task_studio.remote_button import RemoteRecordButtonReceiver
from openarm_task_studio.stage_objects import (
    configure_front_workbench, scene_obstacle_bounds, segment_obstacle, set_world_position,
)


sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.01, device="cpu"))
print("[OpenArmTaskStudio] Physics device: CPU (supports runtime task-object reset); rendering remains GPU")
sim.set_camera_view([2.0, 0.0, 1.10], [0.0, 0.0, 0.65])
scene = InteractiveScene(OpenArmTaskStudioSceneCfg(num_envs=args.num_envs, env_spacing=2.5))
sim.reset()
scene.update(sim.get_physics_dt())
stage = sim.stage
dynamic_control = _dynamic_control.acquire_dynamic_control_interface()
required_paths = (
    "/World/envs/env_0/Robot",
    "/World/envs/env_0/FrontWorkbench",
)
missing_paths = [path for path in required_paths if not stage.GetPrimAtPath(path).IsValid()]
if missing_paths:
    raise RuntimeError(f"Task Studio scene is missing required prims: {missing_paths}")
if stage.GetPrimAtPath("/World/envs/env_0/Workpiece").IsValid():
    raise RuntimeError("Task Studio scene must not contain the drag-teaching workpiece")
print("[OpenArmTaskStudio] Floor scene ready: robot, workbench, no workpiece")
if args.workbench_config_test:
    configure_front_workbench(stage, False, 1.05, 0.35)
    workbench = stage.GetPrimAtPath("/World/envs/env_0/FrontWorkbench")
    if workbench.IsActive():
        raise RuntimeError("Disabled workbench remains active in the stage")
    translation = UsdGeom.Xformable(workbench).ComputeLocalToWorldTransform(0).ExtractTranslation()
    expected = Gf.Vec3d(0.55, 0.0, 0.0)
    if not Gf.IsClose(Gf.Vec3d(translation), expected, 1e-6):
        raise RuntimeError(f"Workbench transform test failed: got {translation}, expected {expected}")
    if any("/FrontWorkbench/" in path for path, _, _ in scene_obstacle_bounds(stage)):
        raise RuntimeError("Disabled workbench remains in collision obstacle bounds")
    configure_front_workbench(stage, True, 1.05, 0.35)
    if not workbench.IsActive():
        raise RuntimeError("Enabled workbench did not reactivate in the stage")
    collision_prims = [
        prim for prim in Usd.PrimRange(workbench)
        if prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    if not collision_prims or any(
        UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False
        for prim in collision_prims
    ):
        raise RuntimeError("Enabled workbench did not restore all physical collisions")
    top = stage.GetPrimAtPath("/World/envs/env_0/FrontWorkbench/Top")
    top_max_z = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_]).ComputeWorldBound(top).ComputeAlignedRange().GetMax()[2]
    if abs(float(top_max_z) - 1.05) > 1e-5:
        raise RuntimeError(f"Workbench height test failed: tabletop top is {top_max_z}")
    leg = stage.GetPrimAtPath("/World/envs/env_0/FrontWorkbench/LegFrontLeft")
    leg_range = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_]).ComputeWorldBound(leg).ComputeAlignedRange()
    top_range = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_]).ComputeWorldBound(top).ComputeAlignedRange()
    print(
        f"[OpenArmTaskStudio] workbench bounds: legZ=({leg_range.GetMin()[2]}, {leg_range.GetMax()[2]}), "
        f"topZ=({top_range.GetMin()[2]}, {top_range.GetMax()[2]})"
    )
    if abs(float(leg_range.GetMin()[2])) > 1e-5:
        raise RuntimeError(f"Workbench legs are not grounded: bottom is {leg_range.GetMin()[2]}")
    if abs(float(leg_range.GetMax()[2]) - float(top_range.GetMin()[2])) > 1e-5:
        raise RuntimeError(
            f"Workbench legs do not meet tabletop: leg top={leg_range.GetMax()[2]}, "
            f"tabletop bottom={top_range.GetMin()[2]}"
        )

robot = scene["robot"]
left_arm = SceneEntityCfg(
    "robot",
    joint_names=[f"openarm_left_joint{index}" for index in range(1, 8)],
    body_names=["openarm_left_ee_tcp"],
)
left_arm.resolve(scene)
right_arm = SceneEntityCfg(
    "robot",
    joint_names=[f"openarm_right_joint{index}" for index in range(1, 8)],
    body_names=["openarm_right_ee_tcp"],
)
right_arm.resolve(scene)
left_gripper = SceneEntityCfg("robot", joint_names=["openarm_left_finger_joint.*"])
right_gripper = SceneEntityCfg("robot", joint_names=["openarm_right_finger_joint.*"])
left_gripper.resolve(scene)
right_gripper.resolve(scene)
arm_entities = {"left": left_arm, "right": right_arm}
jacobian_indices = {
    arm: entity.body_ids[0] - 1 if robot.is_fixed_base else entity.body_ids[0]
    for arm, entity in arm_entities.items()
}
ik_controller = DifferentialIKController(
    DifferentialIKControllerCfg(command_type="position", use_relative_mode=False, ik_method="dls"),
    num_envs=scene.num_envs,
    device=sim.device,
)
pose_ik_controller = DifferentialIKController(
    DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
    num_envs=scene.num_envs,
    device=sim.device,
)
previous_joint_targets = {
    arm: robot.data.joint_pos[:, entity.joint_ids].clone()
    for arm, entity in arm_entities.items()
}
d435_receiver = D435TeachingReceiver(port=args.d435_port)
motion_monitor = MotionExecutionMonitor()
last_d435_command_time = 0.0
remote_button_receiver = RemoteRecordButtonReceiver(port=5011)
print("[OpenArmTaskStudio] Remote record button UDP ready on 0.0.0.0:5011")
handled_manual_gripper_generation = 0
print(f"[OpenArmTaskStudio] D435 coarse teaching UDP ready on 127.0.0.1:{args.d435_port}")
handled_preview_generation = -1
active_waypoints = []
active_waypoint_index = 0
waypoint_hold_time = 0.0
active_gripper_states = {"left": "open", "right": "open"}
attached_object_path = None
attached_object_offset = [0.0, 0.0, 0.0]
task_object_original_positions = {}
action_processed_indices = set()
waypoint_retry_counts = {}
orientation_targets = {}
active_arm_status = "idle"
active_failure_reason = None
needs_initial_transit = False
waypoint_elapsed = 0.0
active_wrist_target = 0.0
ASSISTED_ATTACH_MAX_DISTANCE = 0.04
parallel_states = None
parallel_cursor = 0
last_parallel_succeeded = None


def set_gripper(state: str, arm="left") -> None:
    if state != "hold":
        active_gripper_states[arm] = state
    gripper = left_gripper if arm == "left" else right_gripper
    value = 0.044 if active_gripper_states[arm] == "open" else 0.0
    target = torch.full((scene.num_envs, len(gripper.joint_ids)), value, device=sim.device)
    robot.set_joint_position_target(target, joint_ids=gripper.joint_ids)


set_gripper("open", "left")
set_gripper("open", "right")
if (
    os.environ.get("OPENARM_TASK_STUDIO_SELF_TEST") == "1"
    and runtime.preview_generation == 0
    and task_studio_extension.active_extension is not None
):
    print("[OpenArmTaskStudio] Starting self-test after scene initialization")
    task_studio_extension.active_extension._run_self_test()

run_current_task_test = os.environ.get("OPENARM_TASK_STUDIO_CURRENT_TASK_TEST") == "1"
current_task_test_seen_running = False
if run_current_task_test and task_studio_extension.active_extension is not None:
    extension = task_studio_extension.active_extension
    extension._load_task(task_studio_extension.SAVE_PATH)
    extension._run_full_task_safe()
    print("[OpenArmTaskStudio] CURRENT_TASK_TEST_STARTED")


def set_visible(path: str, visible: bool) -> None:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        imageable = UsdGeom.Imageable(prim)
        imageable.MakeVisible() if visible else imageable.MakeInvisible()


def reset_task_object_pose(path: str, position) -> None:
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return
    handle = dynamic_control.get_rigid_body(path)
    if handle == _dynamic_control.INVALID_HANDLE:
        print(f"[OpenArmTaskStudio] Could not acquire runtime rigid body for reset: {path}")
        return
    dynamic_control.set_rigid_body_pose(
        handle,
        _dynamic_control.Transform(
            (float(position[0]), float(position[1]), float(position[2])),
            (0.0, 0.0, 0.0, 1.0),
        ),
    )
    dynamic_control.set_rigid_body_linear_velocity(handle, (0.0, 0.0, 0.0))
    dynamic_control.set_rigid_body_angular_velocity(handle, (0.0, 0.0, 0.0))


def set_object_collision(path: str, enabled: bool) -> None:
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return
    for descendant in Usd.PrimRange(prim):
        if descendant.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(descendant).CreateCollisionEnabledAttr().Set(bool(enabled))


def set_object_kinematic(path: str, enabled: bool) -> None:
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid() or not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        return
    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr().Set(bool(enabled))


def follow_attached_object(ee_position) -> None:
    if not attached_object_path:
        return
    handle = dynamic_control.get_rigid_body(attached_object_path)
    if handle == _dynamic_control.INVALID_HANDLE:
        return
    position = tuple(
        float(ee_position[index]) + float(attached_object_offset[index])
        for index in range(3)
    )
    dynamic_control.set_rigid_body_pose(
        handle,
        _dynamic_control.Transform(position, (0.0, 0.0, 0.0, 1.0)),
    )


def set_task_editor_visuals(visible: bool) -> None:
    task_root = stage.GetPrimAtPath("/World/OpenArmTaskStudio/TaskObjects")
    if task_root.IsValid():
        for prim in task_root.GetChildren():
            task_type = prim.GetAttribute("openarm:taskType").Get()
            if visible:
                set_visible(str(prim.GetPath()), True)
                if task_type == "grasp":
                    UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(True)
                    set_object_kinematic(str(prim.GetPath()), False)
                continue
            is_grasp_object = task_type == "grasp"
            set_visible(str(prim.GetPath()), is_grasp_object)
            if is_grasp_object:
                UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(True)
    set_visible("/World/OpenArmTaskStudio/TaskPath", visible)
    set_visible("/World/OpenArmTaskStudio/TaskPathSegments", visible)


def reset_visual_objects() -> None:
    global attached_object_path, attached_object_offset
    attached_object_path = None
    attached_object_offset = [0.0, 0.0, 0.0]
    for path, position in task_object_original_positions.items():
        set_object_kinematic(path, False)
        reset_task_object_pose(path, position)
    task_root = stage.GetPrimAtPath("/World/OpenArmTaskStudio/TaskObjects")
    if task_root.IsValid():
        for prim in task_root.GetChildren():
            set_visible(str(prim.GetPath()), True)
    set_task_editor_visuals(True)


def process_waypoint_action(waypoint, ee_position) -> bool:
    global attached_object_path, attached_object_offset
    action = waypoint.get("action")
    object_path = waypoint.get("object_path")
    if action == "attach" and object_path:
        target = waypoint.get("object_position", waypoint["position"])
        distance = sum((float(ee_position[index]) - float(target[index])) ** 2 for index in range(3)) ** 0.5
        distance_limit = float(waypoint.get("attach_distance_limit", ASSISTED_ATTACH_MAX_DISTANCE))
        if distance > distance_limit:
            print(
                f"[OpenArmTaskStudio] Attach blocked: TCP is {distance:.4f} m from object "
                f"(limit {distance_limit:.3f} m)"
            )
            return False
        if stage.GetPrimAtPath(object_path).IsValid():
            attached_object_path = object_path
            attached_object_offset = [
                float(target[index]) - float(ee_position[index]) for index in range(3)
            ]
            set_object_collision(object_path, True)
            set_object_kinematic(object_path, True)
            set_visible(object_path, True)
            print(f"[OpenArmTaskStudio] Physics grasp engaged: {object_path}")
        else:
            print(f"[OpenArmTaskStudio] Attach skipped: object prim is unavailable: {object_path}")
    elif action == "detach" and attached_object_path:
        object_position = waypoint.get("object_position", waypoint["position"])
        handle = dynamic_control.get_rigid_body(attached_object_path)
        if handle != _dynamic_control.INVALID_HANDLE:
            dynamic_control.set_rigid_body_pose(
                handle,
                _dynamic_control.Transform(
                    tuple(float(value) for value in object_position),
                    (0.0, 0.0, 0.0, 1.0),
                ),
            )
        set_object_collision(attached_object_path, True)
        set_object_kinematic(attached_object_path, False)
        attached_object_path = None
        attached_object_offset = [0.0, 0.0, 0.0]
        print(f"[OpenArmTaskStudio] Physics grasp released at: {object_position}")
    return True


def _update_preview_single() -> None:
    global handled_preview_generation, active_waypoints, active_waypoint_index, waypoint_hold_time
    global action_processed_indices, waypoint_retry_counts, orientation_targets
    global active_arm_status, active_failure_reason
    global needs_initial_transit, waypoint_elapsed, active_wrist_target, attached_object_offset
    if handled_preview_generation != runtime.preview_generation:
        handled_preview_generation = runtime.preview_generation
        active_waypoints = []
        active_waypoint_index = 0
        waypoint_hold_time = 0.0
        waypoint_elapsed = 0.0
        action_processed_indices = set()
        waypoint_retry_counts = {}
        orientation_targets = {}
        active_arm_status = "idle"
        active_failure_reason = None
        needs_initial_transit = False
        if runtime.command_kind == "reset":
            defaults = robot.data.default_joint_pos.clone()
            default_velocity = torch.zeros_like(defaults)
            robot.write_joint_state_to_sim(defaults, default_velocity)
            robot.set_joint_position_target(defaults)
            for arm, entity in arm_entities.items():
                previous_joint_targets[arm][:] = defaults[:, entity.joint_ids]
            ik_controller.reset()
            pose_ik_controller.reset()
            set_gripper("open", "left")
            set_gripper("open", "right")
            reset_visual_objects()
            print("[OpenArmTaskStudio] Robot reset immediately; velocities cleared; both grippers open")
        elif runtime.preview_request is not None:
            raw = runtime.preview_request.get("waypoints")
            if raw is None:
                raw = [
                    {"position": position, "gripper": "hold", "label": "preview"}
                    for position in runtime.preview_request.get("preview_path", [runtime.preview_request["position"]])
                ]
            active_waypoints = raw
            active_arm_status = "running"
            active_wrist_target = next((float(item["wrist_angle"]) for item in raw if "wrist_angle" in item), 0.0)
            needs_initial_transit = runtime.command_kind == "execute"
            for waypoint in active_waypoints:
                if waypoint.get("action") == "detach" and waypoint.get("object_path"):
                    set_visible(waypoint["object_path"], False)
            print(f"[OpenArmTaskStudio] {runtime.command_kind} active: {len(active_waypoints)} waypoint(s)")
    runtime.current_left_joint_positions = [float(value) for value in robot.data.joint_pos[0, left_arm.joint_ids]]
    limits = robot.data.soft_joint_pos_limits[0, left_arm.joint_ids]
    runtime.current_left_joint_limits = [[float(row[0]), float(row[1])] for row in limits]
    runtime.current_right_joint_positions = [float(value) for value in robot.data.joint_pos[0, right_arm.joint_ids]]
    right_limits = robot.data.soft_joint_pos_limits[0, right_arm.joint_ids]
    runtime.current_right_joint_limits = [[float(row[0]), float(row[1])] for row in right_limits]
    left_tcp_pose = robot.data.body_pose_w[:, left_arm.body_ids[0]]
    right_tcp_pose = robot.data.body_pose_w[:, right_arm.body_ids[0]]
    runtime.current_left_tcp_position = [float(value) for value in left_tcp_pose[0, 0:3]]
    runtime.current_right_tcp_position = [float(value) for value in right_tcp_pose[0, 0:3]]
    if not active_waypoints:
        return
    root_pose_w = robot.data.root_pose_w
    from isaaclab.utils.math import matrix_from_quat, quat_error_magnitude, quat_from_matrix, subtract_frame_transforms
    if needs_initial_transit:
        first_arm = active_waypoints[0].get("arm", "left")
        first_entity = arm_entities[first_arm]
        ee_pose_w = robot.data.body_pose_w[:, first_entity.body_ids[0]]
        current = [float(value) for value in ee_pose_w[0, 0:3]]
        first = active_waypoints[0]
        if segment_obstacle(stage, current, first["position"], 0.02):
            obstacle_top = max((maximum[2] for _path, _minimum, maximum in scene_obstacle_bounds(stage, 0.02)), default=1.20)
            safe_z = max(obstacle_top + 0.06, current[2], first["position"][2])
            prefix = []
            if abs(current[2] - safe_z) > 0.005:
                prefix.append({
                    "position": [current[0], current[1], safe_z], "gripper": "open",
                    "label": "initial lift", "tolerance": 0.045, "arm": first_arm,
                })
            horizontal = [first["position"][0], first["position"][1], safe_z]
            previous = prefix[-1]["position"] if prefix else current
            if any(abs(horizontal[i] - previous[i]) > 0.005 for i in range(3)):
                prefix.append({
                    "position": horizontal, "gripper": "open",
                    "label": "initial transit", "tolerance": 0.045, "arm": first_arm,
                })
            active_waypoints = prefix + active_waypoints
            if prefix:
                print(f"[OpenArmTaskStudio] Initial path expanded: {len(active_waypoints)} waypoint(s)")
        needs_initial_transit = False
    waypoint = active_waypoints[active_waypoint_index]
    active_arm = waypoint.get("arm", "left")
    arm_entity = arm_entities[active_arm]
    jacobian_index = jacobian_indices[active_arm]
    previous_joint_target = previous_joint_targets[active_arm]
    set_gripper(waypoint.get("gripper", "hold"), active_arm)
    jacobian = robot.root_physx_view.get_jacobians()[:, jacobian_index, :, arm_entity.joint_ids]
    ee_pose_w = robot.data.body_pose_w[:, arm_entity.body_ids[0]]
    joint_pos = robot.data.joint_pos[:, arm_entity.joint_ids]
    ee_pos_b, ee_quat_b = subtract_frame_transforms(
        root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
    )
    follow_attached_object([float(value) for value in ee_pose_w[0, 0:3]])
    target_w = torch.tensor(waypoint["position"], dtype=torch.float32, device=sim.device).unsqueeze(0)
    target_b = target_w - root_pose_w[:, 0:3]
    orientation_error = 0.0
    if waypoint.get("level_gripper") and "target_quaternion" not in waypoint:
        orientation_group = waypoint.get("orientation_group", waypoint.get("stage_id", waypoint["label"]))
        if orientation_group not in orientation_targets:
            current_rotation = matrix_from_quat(ee_pose_w[:, 3:7])
            configured_approach = waypoint.get("approach_direction")
            if configured_approach is not None:
                approach_axis = torch.tensor(
                    configured_approach, dtype=torch.float32, device=sim.device
                ).unsqueeze(0)
                approach_axis = approach_axis / torch.linalg.vector_norm(
                    approach_axis, dim=1, keepdim=True
                ).clamp_min(1.0e-6)
            else:
                approach_axis = current_rotation[:, :, 2]
            world_up = torch.tensor([[0.0, 0.0, 1.0]], dtype=torch.float32, device=sim.device)
            level_axis = torch.linalg.cross(world_up.expand_as(approach_axis), approach_axis, dim=1)
            level_norm = torch.linalg.vector_norm(level_axis, dim=1, keepdim=True)
            current_grip_axis = current_rotation[:, :, 1]
            current_horizontal = current_grip_axis.clone()
            current_horizontal[:, 2] = 0.0
            fallback_norm = torch.linalg.vector_norm(current_horizontal, dim=1, keepdim=True).clamp_min(1.0e-6)
            fallback_axis = current_horizontal / fallback_norm
            level_axis = torch.where(level_norm > 1.0e-5, level_axis / level_norm.clamp_min(1.0e-6), fallback_axis)
            same_direction = torch.sum(level_axis * current_grip_axis, dim=1, keepdim=True) >= 0.0
            level_axis = torch.where(same_direction, level_axis, -level_axis)
            first_axis = torch.linalg.cross(level_axis, approach_axis, dim=1)
            first_axis = first_axis / torch.linalg.vector_norm(first_axis, dim=1, keepdim=True).clamp_min(1.0e-6)
            level_axis = torch.linalg.cross(approach_axis, first_axis, dim=1)
            target_rotation = torch.stack((first_axis, level_axis, approach_axis), dim=2)
            target_quaternion = quat_from_matrix(target_rotation)
            orientation_targets[orientation_group] = [float(value) for value in target_quaternion[0]]
        waypoint["target_quaternion"] = list(orientation_targets[orientation_group])
        print(
            f"[OpenArmTaskStudio] Leveled gripper with approach axis "
            f"{waypoint.get('approach_direction', 'current')}: {waypoint.get('label', '')}"
        )
    if "joint_positions" in waypoint:
        joint_goal = torch.tensor(waypoint["joint_positions"], dtype=torch.float32, device=sim.device).unsqueeze(0)
        limits = robot.data.soft_joint_pos_limits[:, arm_entity.joint_ids]
        joint_goal = torch.maximum(torch.minimum(joint_goal, limits[:, :, 1]), limits[:, :, 0])
        desired = joint_goal
    elif "target_quaternion" in waypoint:
        target_quat_w = torch.tensor(
            waypoint["target_quaternion"], dtype=torch.float32, device=sim.device
        ).unsqueeze(0)
        target_b, target_quat_b = subtract_frame_transforms(
            root_pose_w[:, 0:3], root_pose_w[:, 3:7], target_w, target_quat_w
        )
        pose_ik_controller.set_command(torch.cat((target_b, target_quat_b), dim=-1))
        desired = pose_ik_controller.compute(ee_pos_b, ee_quat_b, jacobian, joint_pos)
        orientation_error = quat_error_magnitude(ee_quat_b, target_quat_b).max().item()
    else:
        ik_controller.set_command(target_b, ee_quat=ee_quat_b)
        desired = ik_controller.compute(ee_pos_b, ee_quat_b, jacobian, joint_pos)
        if "wrist_angle" in waypoint:
            if waypoint.get("lock_wrist"):
                desired[:, -1] = float(waypoint["wrist_angle"])
            else:
                desired[:, -1] = 0.98 * desired[:, -1] + 0.02 * float(waypoint["wrist_angle"])
    speed_scale = max(0.1, min(3.0, float(waypoint.get("speed_scale", 1.0))))
    max_step = 0.75 * speed_scale * sim.get_physics_dt()
    desired = previous_joint_target + torch.clamp(desired - previous_joint_target, -max_step, max_step)
    previous_joint_target[:] = desired
    robot.set_joint_position_target(desired, joint_ids=arm_entity.joint_ids)
    waypoint_elapsed += sim.get_physics_dt()
    position_error = torch.linalg.vector_norm(target_b - ee_pos_b, dim=1).max().item()
    joint_error = torch.max(torch.abs(joint_goal - joint_pos)).item() if "joint_positions" in waypoint else 0.0
    if "joint_positions" in waypoint:
        requires_tcp_match = waypoint.get("task_type") in ("grasp", "place")
        joint_tolerance = float(waypoint.get(
            "joint_tolerance", 0.20 if requires_tcp_match else 0.15
        ))
        if requires_tcp_match:
            tcp_tolerance = float(waypoint.get("tolerance", 0.03))
            reached = joint_error < joint_tolerance and position_error < tcp_tolerance
        else:
            reached = joint_error < joint_tolerance
    else:
        pose_reached = (
            "target_quaternion" not in waypoint
            or orientation_error < float(waypoint.get("orientation_tolerance", 0.04))
        )
        wrist_reached = (
            not waypoint.get("lock_wrist")
            or (
                orientation_error < float(waypoint.get("orientation_tolerance", 0.04))
                if "target_quaternion" in waypoint
                else abs(float(joint_pos[0, -1]) - float(waypoint.get("wrist_angle", joint_pos[0, -1]))) < 0.02
            )
        )
        reached = (
            position_error < float(waypoint.get("tolerance", 0.03))
            and pose_reached
            and wrist_reached
        )
    if reached:
        waypoint_hold_time += sim.get_physics_dt()
        action_hold_time = float(waypoint.get("action_hold_time", 0.35)) / speed_scale
        if waypoint.get("action") and waypoint_hold_time >= action_hold_time and active_waypoint_index not in action_processed_indices:
            if not process_waypoint_action(waypoint, [float(value) for value in ee_pose_w[0, 0:3]]):
                print(f"[OpenArmTaskStudio] Task stopped at {waypoint.get('label', '')}: action precondition failed")
                active_arm_status = "failed"
                active_failure_reason = f"{waypoint.get('stage_id', waypoint.get('label', ''))}: action precondition failed"
                active_waypoints = []
                return
            action_processed_indices.add(active_waypoint_index)
        required_hold_time = float(waypoint.get("hold_time", 0.5)) / speed_scale
        action_ready = not waypoint.get("action") or active_waypoint_index in action_processed_indices
        if waypoint_hold_time >= required_hold_time and action_ready:
            if active_waypoint_index < len(active_waypoints) - 1:
                active_waypoint_index += 1
                waypoint_hold_time = 0.0
                waypoint_elapsed = 0.0
                print(f"[OpenArmTaskStudio] Waypoint {active_waypoint_index + 1}/{len(active_waypoints)}: {active_waypoints[active_waypoint_index].get('label', '')}")
            else:
                print(f"[OpenArmTaskStudio] {waypoint.get('arm', 'left')} arm task completed")
                active_arm_status = "succeeded"
                active_waypoints = []
    elif waypoint_elapsed >= 8.0 / speed_scale:
        retry_key = waypoint.get("stage_id", str(active_waypoint_index))
        retry_count = waypoint_retry_counts.get(retry_key, 0)
        max_retries = int(waypoint.get("max_retries", 0))
        if retry_count < max_retries:
            waypoint_retry_counts[retry_key] = retry_count + 1
            waypoint_elapsed = 0.0
            waypoint_hold_time = 0.0
            waypoint.pop("target_quaternion", None)
            print(
                f"[OpenArmTaskStudio] Retrying stage {waypoint.get('stage_id', retry_key)} "
                f"({retry_count + 1}/{max_retries})"
            )
            return
        if "joint_positions" in waypoint:
            detail = f"TCP error={position_error:.4f} m, joint error={joint_error:.4f} rad"
        elif "target_quaternion" in waypoint:
            detail = f"TCP error={position_error:.4f} m, orientation error={orientation_error:.4f} rad"
        else:
            detail = f"TCP error={position_error:.4f} m"
        active_arm_status = "failed"
        active_failure_reason = f"{waypoint.get('stage_id', waypoint.get('label', ''))}: {detail}"
        print(f"[OpenArmTaskStudio] Task stopped: unreachable waypoint {active_waypoint_index + 1}/{len(active_waypoints)} ({waypoint.get('label', '')}), {detail}")
        active_waypoints = []


def update_preview() -> None:
    global parallel_states, parallel_cursor, handled_preview_generation, last_parallel_succeeded
    global active_waypoints, active_waypoint_index, waypoint_hold_time, action_processed_indices
    global waypoint_retry_counts, orientation_targets, active_arm_status, active_failure_reason
    global needs_initial_transit, waypoint_elapsed, active_wrist_target
    global attached_object_path, attached_object_offset
    request = runtime.preview_request
    if handled_preview_generation != runtime.preview_generation and (
        request is None or "arm_waypoints" not in request
    ):
        parallel_states = None
        set_task_editor_visuals(True)
    if (
        handled_preview_generation != runtime.preview_generation
        and request is not None and "arm_waypoints" in request
    ):
        handled_preview_generation = runtime.preview_generation
        parallel_states = {}
        task_object_original_positions.clear()
        for waypoints in request["arm_waypoints"].values():
            for waypoint in waypoints:
                if waypoint.get("action") != "attach":
                    continue
                object_path = waypoint.get("object_path")
                object_position = waypoint.get("object_position")
                if not object_path or object_position is None:
                    continue
                task_object_original_positions[object_path] = [
                    float(value) for value in object_position
                ]
                reset_task_object_pose(object_path, object_position)
        for arm, waypoints in request["arm_waypoints"].items():
            entity = arm_entities[arm]
            # A stopped teaching controller may leave its last target ahead of the
            # physical joints. Start execution limiting from the measured pose.
            previous_joint_targets[arm][:] = robot.data.joint_pos[:, entity.joint_ids]
            parallel_states[arm] = {
                "active_waypoints": list(waypoints),
                "active_waypoint_index": 0,
                "waypoint_hold_time": 0.0,
                "action_processed_indices": set(),
                "waypoint_retry_counts": {},
                "orientation_targets": {},
                "status": "running",
                "failure_reason": None,
                "needs_initial_transit": runtime.execution_needs_initial_transit(waypoints),
                "waypoint_elapsed": 0.0,
                "active_wrist_target": 0.0,
                "attached_object_path": None,
                "attached_object_offset": [0.0, 0.0, 0.0],
            }
        parallel_cursor = 0
        set_task_editor_visuals(False)
        print(
            "[OpenArmTaskStudio] parallel execute active: "
            + ", ".join(f"{arm}={len(state['active_waypoints'])}" for arm, state in parallel_states.items())
        )
    if not parallel_states:
        _update_preview_single()
        return
    for arm, state in parallel_states.items():
        active_waypoints = state["active_waypoints"]
        active_waypoint_index = state["active_waypoint_index"]
        waypoint_hold_time = state["waypoint_hold_time"]
        action_processed_indices = state["action_processed_indices"]
        waypoint_retry_counts = state["waypoint_retry_counts"]
        orientation_targets = state["orientation_targets"]
        active_arm_status = state["status"]
        active_failure_reason = state["failure_reason"]
        needs_initial_transit = state["needs_initial_transit"]
        waypoint_elapsed = state["waypoint_elapsed"]
        active_wrist_target = state["active_wrist_target"]
        attached_object_path = state["attached_object_path"]
        attached_object_offset = state["attached_object_offset"]
        _update_preview_single()
        state.update({
            "active_waypoints": active_waypoints,
            "active_waypoint_index": active_waypoint_index,
            "waypoint_hold_time": waypoint_hold_time,
            "action_processed_indices": action_processed_indices,
            "waypoint_retry_counts": waypoint_retry_counts,
            "orientation_targets": orientation_targets,
            "status": active_arm_status,
            "failure_reason": active_failure_reason,
            "needs_initial_transit": needs_initial_transit,
            "waypoint_elapsed": waypoint_elapsed,
            "active_wrist_target": active_wrist_target,
            "attached_object_path": attached_object_path,
            "attached_object_offset": attached_object_offset,
        })
    if all(not item["active_waypoints"] for item in parallel_states.values()):
        last_parallel_succeeded = all(
            state["status"] == "succeeded" for state in parallel_states.values()
        )
        summary = ", ".join(
            f"{arm}={state['status']}" + (
                f" ({state['failure_reason']})" if state["failure_reason"] else ""
            )
            for arm, state in parallel_states.items()
        )
        print(f"[OpenArmTaskStudio] Parallel task completed: {summary}")
        set_task_editor_visuals(True)
        parallel_states = None


def update_d435_teaching() -> None:
    global handled_manual_gripper_generation, last_d435_command_time
    if handled_manual_gripper_generation != runtime.manual_gripper_generation:
        handled_manual_gripper_generation = runtime.manual_gripper_generation
        controlled_arms = (
            ("left", "right") if runtime.d435_teaching_mode == "both"
            else (runtime.d435_teaching_mode,)
        )
        for controlled_arm in controlled_arms:
            set_gripper(runtime.manual_gripper_state, controlled_arm)
    targets = d435_receiver.poll()
    runtime.d435_latest_targets = {
        arm: list(values) if values and d435_receiver.target_age(arm) <= 0.25 else None
        for arm, values in targets.items()
    }
    if any(runtime.d435_latest_targets.values()):
        runtime.d435_latest_target_time = time.monotonic()
    runtime.d435_packet_count = d435_receiver.packet_count
    runtime.d435_raw_packet_count = d435_receiver.raw_packet_count
    runtime.d435_filtered_spike_count = d435_receiver.filtered_spike_count
    if not runtime.d435_teaching_enabled:
        if motion_monitor.active:
            summary_path = motion_monitor.stop(d435_receiver.filtered_spike_count)
            print(f"[OpenArmTaskStudio] Motion monitor saved: {summary_path}")
        runtime.motion_monitor_active = False
        last_d435_command_time = 0.0
        runtime.d435_tracking_valid["left"] = False
        runtime.d435_tracking_valid["right"] = False
        for entity in arm_entities.values():
            robot.set_joint_velocity_target(
                torch.zeros((scene.num_envs, len(entity.joint_ids)), device=sim.device),
                joint_ids=entity.joint_ids,
            )
        return
    if not motion_monitor.active:
        monitor_path = motion_monitor.start(
            runtime.d435_teaching_mode, d435_receiver.filtered_spike_count
        )
        runtime.motion_monitor_active = True
        runtime.motion_monitor_path = str(monitor_path)
        runtime.motion_monitor_samples = 0
        print(f"[OpenArmTaskStudio] Motion monitor recording: {monitor_path}")
    controlled_arms = (
        ("left", "right") if runtime.d435_teaching_mode == "both"
        else (runtime.d435_teaching_mode,)
    )
    now = time.monotonic()
    command_period = 1.0 / d435_receiver.command_rate_hz()
    elapsed = command_period if last_d435_command_time <= 0.0 else now - last_d435_command_time
    command_due = last_d435_command_time <= 0.0 or elapsed >= command_period
    for arm in ("left", "right"):
        entity = arm_entities[arm]
        current = robot.data.joint_pos[:, entity.joint_ids]
        values = targets.get(arm)
        if arm not in controlled_arms or not values or d435_receiver.target_age(arm) > 0.5:
            runtime.d435_tracking_valid[arm] = False
            previous_joint_targets[arm][:] = current
            robot.set_joint_position_target(current, joint_ids=entity.joint_ids)
            robot.set_joint_velocity_target(torch.zeros_like(current), joint_ids=entity.joint_ids)
            continue
        runtime.d435_tracking_valid[arm] = True
        # J5/J7 are not mapped from body pose. PhysX can otherwise spin J5 far
        # beyond its limit under fast upstream motion, so keep both passive
        # wrist axes at their bounded mapped state during motion teaching.
        passive_indices = [4, 6]
        passive_joint_ids = [entity.joint_ids[index] for index in passive_indices]
        passive_positions = torch.tensor(
            [[float(values[index]) for index in passive_indices]],
            dtype=torch.float32,
            device=sim.device,
        )
        passive_limits = robot.data.soft_joint_pos_limits[:, passive_joint_ids]
        passive_positions = torch.maximum(
            torch.minimum(passive_positions, passive_limits[:, :, 1]),
            passive_limits[:, :, 0],
        )
        previous_joint_targets[arm][:, passive_indices] = passive_positions
        robot.write_joint_state_to_sim(
            passive_positions,
            torch.zeros_like(passive_positions),
            joint_ids=passive_joint_ids,
        )
        if not command_due:
            continue
        desired = torch.tensor(values, dtype=torch.float32, device=sim.device).unsqueeze(0)
        limits = robot.data.soft_joint_pos_limits[:, entity.joint_ids]
        desired = torch.maximum(torch.minimum(desired, limits[:, :, 1]), limits[:, :, 0])
        max_speeds = torch.tensor(
            d435_receiver.max_joint_speeds(), dtype=torch.float32, device=sim.device
        ).unsqueeze(0)
        command_dt = min(0.1, max(sim.get_physics_dt(), elapsed))
        # A streaming target needs to catch the current pose within the requested
        # horizon, not approach it asymptotically. Two command intervals cover
        # acquisition and application latency; at 30 Hz / 0.05 s this is direct.
        trajectory_fraction = min(1.0, 2.0 * command_dt / d435_receiver.trajectory_time_s())
        delta = (desired - previous_joint_targets[arm]) * trajectory_fraction
        max_step = max_speeds * command_dt
        desired = previous_joint_targets[arm] + torch.clamp(delta, -max_step, max_step)
        command_velocity = (
            (desired - previous_joint_targets[arm])
            / command_dt
            * d435_receiver.velocity_feedforward_gain()
        )
        command_velocity = torch.clamp(command_velocity, -max_speeds, max_speeds)
        command_velocity[:, passive_indices] = 0.0
        previous_joint_targets[arm][:] = desired
        robot.set_joint_position_target(desired, joint_ids=entity.joint_ids)
        robot.set_joint_velocity_target(command_velocity, joint_ids=entity.joint_ids)
        sample = motion_monitor.record(
            arm,
            values,
            desired[0].detach().cpu().tolist(),
            current[0].detach().cpu().tolist(),
            d435_receiver.latest_metadata.get(arm),
            d435_receiver.filtered_spike_count,
        )
        if sample is not None:
            snapshot = motion_monitor.snapshot()
            runtime.motion_monitor_samples = snapshot["samples"]
            runtime.motion_monitor_stats = snapshot["arms"]
    if command_due:
        last_d435_command_time = now

started = time.monotonic()
workbench_test_completed = False
try:
    while simulation_app.is_running():
        runtime.remote_record_press_count = remote_button_receiver.poll()
        remote_button_status = remote_button_receiver.status()
        runtime.remote_button_last_packet_time = remote_button_status["last_packet_time"]
        runtime.remote_button_last_press_time = remote_button_status["last_press_time"]
        runtime.remote_button_address = remote_button_status["address"]
        runtime.remote_button_is_down = remote_button_status["is_down"]
        runtime.remote_button_sequence = remote_button_status["sequence"]
        update_preview()
        if run_current_task_test:
            if parallel_states is not None:
                current_task_test_seen_running = True
            elif current_task_test_seen_running:
                result = "PASS" if last_parallel_succeeded else "FAIL"
                print(f"[OpenArmTaskStudio] CURRENT_TASK_TEST_{result}")
                if not last_parallel_succeeded:
                    raise RuntimeError("current_task.json execution failed")
                break
        update_d435_teaching()
        scene.write_data_to_sim()
        sim.step()
        scene.update(sim.get_physics_dt())
        if args.workbench_config_test and not workbench_test_completed and time.monotonic() - started >= 1.0:
            top = stage.GetPrimAtPath("/World/envs/env_0/FrontWorkbench/Top")
            top_max_z = UsdGeom.BBoxCache(0, [UsdGeom.Tokens.default_]).ComputeWorldBound(top).ComputeAlignedRange().GetMax()[2]
            if abs(float(top_max_z) - 1.05) > 1e-5:
                raise RuntimeError(f"Workbench height did not persist through simulation: tabletop top is {top_max_z}")
            workbench_test_completed = True
            print("[OpenArmTaskStudio] WORKBENCH_CONFIG_TEST_PASS")
        if args.duration > 0.0 and time.monotonic() - started >= args.duration:
            print(f"[OpenArmTaskStudio] Duration reached: {args.duration:.1f}s")
            break
finally:
    motion_monitor.stop(d435_receiver.filtered_spike_count)
    remote_button_receiver.close()
    simulation_app.close(skip_cleanup=True)
