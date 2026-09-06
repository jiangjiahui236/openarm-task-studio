from __future__ import annotations

import isaaclab.sim as sim_utils
from isaacsim.core.utils import prims as prim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim.spawners.spawner_cfg import SpawnerCfg
from isaaclab.sim.utils import clone
from isaaclab.utils import configclass

from openarm.tasks.manager_based.openarm_manipulation.assets.openarm_bimanual import OPEN_ARM_HIGH_PD_CFG


FRONT_WORKBENCH_LEG_HEIGHT = 0.31
FRONT_WORKBENCH_THICKNESS = 0.04
FRONT_WORKBENCH_X_SIZE = 0.40
FRONT_WORKBENCH_Y_SIZE = 0.60
FRONT_WORKBENCH_NEAR_EDGE_X = 0.20
FRONT_WORKBENCH_CENTER_X = FRONT_WORKBENCH_NEAR_EDGE_X + FRONT_WORKBENCH_X_SIZE / 2.0
FRONT_WORKBENCH_BASE_Z = 0.0

NATURAL_HANGING_JOINT_POS = {
    **{f"openarm_left_joint{index}": 0.0 for index in range(1, 8)},
    **{f"openarm_right_joint{index}": 0.0 for index in range(1, 8)},
    "openarm_left_finger_joint.*": 0.044,
    "openarm_right_finger_joint.*": 0.044,
    "openarm_left_hand": 0.0,
    "openarm_right_hand": 0.0,
    "openarm_left_ee_tcp_joint": 0.0,
    "openarm_right_ee_tcp_joint": 0.0,
}


# Keep enough actuator headroom for the 8/10/12 rad/s teaching commands.
# This is a Task Studio-local copy; shared OpenArm assets remain unchanged.
TASK_STUDIO_ROBOT_CFG = OPEN_ARM_HIGH_PD_CFG.copy()
TASK_STUDIO_ROBOT_CFG.spawn.articulation_props.solver_velocity_iteration_count = 2
TASK_STUDIO_ARM_ACTUATOR = TASK_STUDIO_ROBOT_CFG.actuators["openarm_arm"]
TASK_STUDIO_ARM_ACTUATOR.velocity_limit_sim = {
    "openarm_left_joint[1-2]": 10.0,
    "openarm_right_joint[1-2]": 10.0,
    "openarm_left_joint[3-4]": 12.0,
    "openarm_right_joint[3-4]": 12.0,
    "openarm_left_joint[5,7]": 1.5,
    "openarm_right_joint[5,7]": 1.5,
    "openarm_left_joint6": 14.0,
    "openarm_right_joint6": 14.0,
}
TASK_STUDIO_ARM_ACTUATOR.effort_limit_sim = {
    "openarm_left_joint[1,3]": 220.0,
    "openarm_right_joint[1,3]": 220.0,
    "openarm_left_joint[2,4]": 180.0,
    "openarm_right_joint[2,4]": 180.0,
    "openarm_left_joint[5,7]": 80.0,
    "openarm_right_joint[5,7]": 80.0,
    "openarm_left_joint6": 120.0,
    "openarm_right_joint6": 120.0,
}
TASK_STUDIO_ARM_ACTUATOR.stiffness = {
    "openarm_left_joint[1-4]": 800.0,
    "openarm_right_joint[1-4]": 800.0,
    "openarm_left_joint[5,7]": 400.0,
    "openarm_right_joint[5,7]": 400.0,
    "openarm_left_joint6": 800.0,
    "openarm_right_joint6": 800.0,
}
TASK_STUDIO_ARM_ACTUATOR.damping = {
    "openarm_left_joint[1-4]": 30.0,
    "openarm_right_joint[1-4]": 30.0,
    "openarm_left_joint[5,7]": 80.0,
    "openarm_right_joint[5,7]": 80.0,
    "openarm_left_joint6": 24.0,
    "openarm_right_joint6": 24.0,
}


def _static_cuboid(color: tuple[float, float, float], roughness: float = 0.8) -> dict:
    return {
        "collision_props": sim_utils.CollisionPropertiesCfg(),
        "visual_material": sim_utils.PreviewSurfaceCfg(diffuse_color=color, roughness=roughness),
    }


@clone
def spawn_front_workbench(
    prim_path: str,
    cfg: SpawnerCfg,
    translation: tuple[float, float, float] | None = None,
    orientation: tuple[float, float, float, float] | None = None,
    **kwargs,
):
    parent = prim_utils.create_prim(prim_path, prim_type="Xform", translation=translation, orientation=orientation)
    top_cfg = sim_utils.CuboidCfg(
        size=(FRONT_WORKBENCH_X_SIZE, FRONT_WORKBENCH_Y_SIZE, FRONT_WORKBENCH_THICKNESS),
        **_static_cuboid((0.42, 0.35, 0.26), 0.86),
    )
    leg_cfg = sim_utils.CuboidCfg(
        size=(0.04, 0.04, FRONT_WORKBENCH_LEG_HEIGHT),
        **_static_cuboid((0.14, 0.14, 0.14)),
    )
    top_cfg.func(
        f"{prim_path}/Top",
        top_cfg,
        translation=(0.0, 0.0, FRONT_WORKBENCH_LEG_HEIGHT + FRONT_WORKBENCH_THICKNESS / 2.0),
    )
    leg_positions = (
        (FRONT_WORKBENCH_X_SIZE / 2.0 - 0.04, FRONT_WORKBENCH_Y_SIZE / 2.0 - 0.04),
        (FRONT_WORKBENCH_X_SIZE / 2.0 - 0.04, -FRONT_WORKBENCH_Y_SIZE / 2.0 + 0.04),
        (-FRONT_WORKBENCH_X_SIZE / 2.0 + 0.04, FRONT_WORKBENCH_Y_SIZE / 2.0 - 0.04),
        (-FRONT_WORKBENCH_X_SIZE / 2.0 + 0.04, -FRONT_WORKBENCH_Y_SIZE / 2.0 + 0.04),
    )
    for name, (x, y) in zip(("FrontLeft", "FrontRight", "BackLeft", "BackRight"), leg_positions):
        leg_cfg.func(
            f"{prim_path}/Leg{name}",
            leg_cfg,
            translation=(x, y, FRONT_WORKBENCH_LEG_HEIGHT / 2.0),
        )
    return parent


@configclass
class OpenArmTaskStudioSceneCfg(InteractiveSceneCfg):
    ground = AssetBaseCfg(prim_path="/World/GroundPlane", spawn=sim_utils.GroundPlaneCfg())
    dome_light = AssetBaseCfg(
        prim_path="/World/DomeLight",
        spawn=sim_utils.DomeLightCfg(color=(0.78, 0.78, 0.78), intensity=2500.0),
    )
    key_light = AssetBaseCfg(
        prim_path="/World/KeyLight",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[1.2, -1.2, 2.4]),
        spawn=sim_utils.SphereLightCfg(color=(1.0, 0.95, 0.88), intensity=8000.0, radius=0.4),
    )
    front_workbench = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/FrontWorkbench",
        init_state=AssetBaseCfg.InitialStateCfg(pos=[FRONT_WORKBENCH_CENTER_X, 0.0, FRONT_WORKBENCH_BASE_Z]),
        spawn=SpawnerCfg(func=spawn_front_workbench),
    )
    robot = TASK_STUDIO_ROBOT_CFG.replace(
        prim_path="{ENV_REGEX_NS}/Robot",
        init_state=ArticulationCfg.InitialStateCfg(
            pos=[0.0, 0.0, 0.0],
            joint_pos=NATURAL_HANGING_JOINT_POS,
        ),
    )
