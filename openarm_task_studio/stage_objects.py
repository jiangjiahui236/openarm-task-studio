from __future__ import annotations

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


ROOT = "/World/OpenArmTaskStudio"
TASK_ROOT = f"{ROOT}/TaskObjects"
PATH_ROOT = f"{ROOT}/TaskPath"
PATH_SEGMENTS_ROOT = f"{ROOT}/TaskPathSegments"
FRONT_WORKBENCH_PATH = "/World/envs/env_0/FrontWorkbench"
FRONT_WORKBENCH_HALF_X = 0.20
FRONT_WORKBENCH_TOP_THICKNESS = 0.04
TASK_VISUAL_OPACITY = 0.65


def _set_translation_preserving_xform_stack(prim, translation):
    xformable = UsdGeom.Xformable(prim)
    translate_op = next((
        op for op in xformable.GetOrderedXformOps()
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate and not op.IsInverseOp()
    ), None)
    if translate_op is None:
        translate_op = xformable.AddTranslateOp(
            UsdGeom.XformOp.PrecisionDouble, "taskStudio"
        )
    value = (
        Gf.Vec3f(*translation)
        if translate_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat
        else Gf.Vec3d(*translation)
    )
    if not translate_op.Set(value):
        raise RuntimeError(f"Could not update translation for {prim.GetPath()}")


def _set_z_dimension_preserving_xform_stack(prim, z_dimension):
    xformable = UsdGeom.Xformable(prim)
    scale_op = next((
        op for op in xformable.GetOrderedXformOps()
        if op.GetOpType() == UsdGeom.XformOp.TypeScale and not op.IsInverseOp()
    ), None)
    if scale_op is None:
        raise RuntimeError(f"Workbench part has no scale op: {prim.GetPath()}")
    current = scale_op.Get()
    world_range = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]
    ).ComputeWorldBound(prim).ComputeAlignedRange()
    current_height = float(world_range.GetMax()[2] - world_range.GetMin()[2])
    if current_height <= 1e-9:
        raise RuntimeError(f"Workbench part has no measurable height: {prim.GetPath()}")
    z_scale = float(current[2]) * float(z_dimension) / current_height
    value_type = Gf.Vec3f if scale_op.GetPrecision() == UsdGeom.XformOp.PrecisionFloat else Gf.Vec3d
    if not scale_op.Set(value_type(float(current[0]), float(current[1]), float(z_scale))):
        raise RuntimeError(f"Could not resize {prim.GetPath()}")


def configure_front_workbench(stage, enabled, top_height, near_edge_distance):
    parent = stage.GetPrimAtPath(FRONT_WORKBENCH_PATH)
    if not parent.IsValid():
        raise RuntimeError("Front workbench is not available in the stage")
    parent.SetActive(True)
    center_x = float(near_edge_distance) + FRONT_WORKBENCH_HALF_X
    top_height = float(top_height)
    leg_height = max(0.01, top_height - FRONT_WORKBENCH_TOP_THICKNESS)
    _set_translation_preserving_xform_stack(parent, (center_x, 0.0, 0.0))
    top = stage.GetPrimAtPath(f"{FRONT_WORKBENCH_PATH}/Top")
    _set_translation_preserving_xform_stack(
        top, (0.0, 0.0, top_height - FRONT_WORKBENCH_TOP_THICKNESS / 2.0)
    )
    for child in parent.GetChildren():
        if child.GetName().startswith("Leg"):
            current = UsdGeom.Xformable(child).GetLocalTransformation().ExtractTranslation()
            _set_translation_preserving_xform_stack(child, (float(current[0]), float(current[1]), leg_height / 2.0))
            _set_z_dimension_preserving_xform_stack(child, leg_height)
    for prim in Usd.PrimRange(parent):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            collision = UsdPhysics.CollisionAPI(prim)
            attribute = collision.GetCollisionEnabledAttr()
            if not attribute:
                attribute = collision.CreateCollisionEnabledAttr()
            attribute.Set(bool(enabled))
    imageable = UsdGeom.Imageable(parent)
    imageable.MakeVisible() if enabled else imageable.MakeInvisible()
    if not enabled:
        parent.SetActive(False)
    return 0.0


def _set_xform(prim, translation, scale=(1.0, 1.0, 1.0), rotation=(0.0, 0.0, 0.0)):
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation))
    xform.AddScaleOp().Set(Gf.Vec3f(*scale))


def _style(gprim, color, opacity=1.0):
    primvars = UsdGeom.PrimvarsAPI(gprim.GetPrim())
    primvars.CreatePrimvar(
        "displayColor", Sdf.ValueTypeNames.Color3fArray, UsdGeom.Tokens.constant
    ).Set([Gf.Vec3f(*color)])
    primvars.CreatePrimvar(
        "displayOpacity", Sdf.ValueTypeNames.FloatArray, UsdGeom.Tokens.constant
    ).Set([float(opacity)])


def ensure_roots(stage):
    UsdGeom.Xform.Define(stage, ROOT)
    UsdGeom.Xform.Define(stage, TASK_ROOT)
    UsdGeom.Xform.Define(stage, PATH_SEGMENTS_ROOT)


def set_world_position(stage, path, position):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim does not exist: {path}")
    UsdGeom.XformCommonAPI(prim).SetTranslate(Gf.Vec3d(*position))


def update_task_path(
    stage, nodes, planned_positions=None, arm="left", insert_before_node_ids=None
):
    positions = planned_positions or [world_position(stage, node.prim_path) for node in nodes]
    ensure_roots(stage)
    curve = UsdGeom.BasisCurves.Get(stage, PATH_ROOT)
    if curve:
        UsdGeom.Imageable(curve.GetPrim()).MakeInvisible()
    segment_count = max(0, len(positions) - 1)
    arm_root = f"{PATH_SEGMENTS_ROOT}/{arm.title()}"
    parent = UsdGeom.Xform.Define(stage, arm_root).GetPrim()
    for index in range(segment_count):
        start = Gf.Vec3d(*positions[index])
        end = Gf.Vec3d(*positions[index + 1])
        delta = end - start
        length = delta.GetLength()
        path = f"{arm_root}/Segment_{index:03d}"
        capsule = UsdGeom.Capsule.Define(stage, path)
        capsule.CreateAxisAttr("Z")
        capsule.CreateRadiusAttr(0.006)
        capsule.CreateHeightAttr(max(0.001, length))
        color = (0.20, 1.0, 0.35) if arm == "right" else (1.0, 0.72, 0.08)
        _style(capsule, color, TASK_VISUAL_OPACITY)
        prim = capsule.GetPrim()
        insert_before_id = (
            insert_before_node_ids[index]
            if insert_before_node_ids and index < len(insert_before_node_ids) else None
        )
        prim.CreateAttribute(
            "openarm:insertBeforeNodeId", Sdf.ValueTypeNames.String
        ).Set(insert_before_id or "")
        xform = UsdGeom.Xformable(prim)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set((start + end) * 0.5)
        direction = delta / length if length > 1e-9 else Gf.Vec3d(0.0, 0.0, 1.0)
        orientation = Gf.Rotation(Gf.Vec3d(0.0, 0.0, 1.0), direction).GetQuat()
        xform.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(orientation)
        UsdGeom.Imageable(prim).MakeVisible()
    if parent.IsValid():
        for child in parent.GetChildren():
            name = child.GetName()
            if not name.startswith("Segment_"):
                continue
            try:
                index = int(name.rsplit("_", 1)[-1])
            except ValueError:
                continue
            if index >= segment_count:
                UsdGeom.Imageable(child).MakeInvisible()
    return segment_count


def create_move_point(stage, index, position, arm="left"):
    ensure_roots(stage)
    path = f"{TASK_ROOT}/Move_{index:03d}"
    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.CreateRadiusAttr(0.01125)
    _style(
        sphere, (1.0, 0.10, 0.10) if arm == "right" else (0.15, 0.55, 1.0),
        TASK_VISUAL_OPACITY,
    )
    _set_xform(sphere.GetPrim(), position)
    sphere.GetPrim().CreateAttribute("openarm:taskType", Sdf.ValueTypeNames.String).Set("move")
    return path


def create_object_proxy(stage, index, task_type, shape, position, dimensions, arm="left"):
    ensure_roots(stage)
    path = f"{TASK_ROOT}/{task_type.title()}_{index:03d}"
    prim = stage.DefinePrim(path, _proxy_type_name(shape))
    _configure_object_proxy(prim, shape, position, dimensions, arm)
    prim.CreateAttribute("openarm:taskType", Sdf.ValueTypeNames.String).Set(task_type)
    prim.CreateAttribute("openarm:shape", Sdf.ValueTypeNames.String).Set(shape)
    if task_type == "grasp":
        collision = UsdPhysics.CollisionAPI.Apply(prim)
        collision.CreateCollisionEnabledAttr().Set(True)
        rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
        rigid_body.CreateRigidBodyEnabledAttr().Set(True)
        mass = UsdPhysics.MassAPI.Apply(prim)
        mass.CreateMassAttr().Set(0.05)
    return path


def update_object_proxy(stage, path, shape, dimensions):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"Object proxy no longer exists: {path}")
    position = world_position(stage, path)
    if shape == "sphere":
        UsdGeom.Sphere(prim).GetRadiusAttr().Set(dimensions[0])
    elif shape == "cylinder":
        cylinder = UsdGeom.Cylinder(prim)
        cylinder.GetRadiusAttr().Set(dimensions[0])
        cylinder.GetHeightAttr().Set(dimensions[2])
    else:
        _set_xform(prim, position, dimensions)


def rebuild_object_proxy(stage, path, shape, dimensions, arm="left"):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"Object proxy no longer exists: {path}")
    position = world_position(stage, path)
    task_type = prim.GetAttribute("openarm:taskType").Get() or "grasp"
    prim.SetTypeName(_proxy_type_name(shape))
    _configure_object_proxy(prim, shape, position, dimensions, arm)
    prim.CreateAttribute("openarm:taskType", Sdf.ValueTypeNames.String).Set(task_type)
    prim.CreateAttribute("openarm:shape", Sdf.ValueTypeNames.String).Set(shape)


def _proxy_type_name(shape):
    return {"sphere": "Sphere", "cylinder": "Cylinder"}.get(shape, "Cube")


def _configure_object_proxy(prim, shape, position, dimensions, arm="left"):
    if shape == "sphere":
        geom = UsdGeom.Sphere(prim)
        geom.CreateRadiusAttr(dimensions[0])
        _set_xform(prim, position)
    elif shape == "cylinder":
        geom = UsdGeom.Cylinder(prim)
        geom.CreateRadiusAttr(dimensions[0])
        geom.CreateHeightAttr(dimensions[2])
        geom.CreateAxisAttr("Z")
        _set_xform(prim, position)
    else:
        geom = UsdGeom.Cube(prim)
        geom.CreateSizeAttr(1.0)
        _set_xform(prim, position, dimensions)
    _style(
        geom, (1.0, 0.10, 0.10) if arm == "right" else (1.0, 0.55, 0.12),
        TASK_VISUAL_OPACITY,
    )


def world_position(stage, path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        raise RuntimeError(f"Prim no longer exists: {path}")
    value = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(0).ExtractTranslation()
    return [float(value[0]), float(value[1]), float(value[2])]


def set_confirmed(stage, path, confirmed=True):
    prim = stage.GetPrimAtPath(path)
    prim.CreateAttribute("openarm:confirmed", Sdf.ValueTypeNames.Bool).Set(confirmed)


def scene_obstacle_bounds(stage, margin=0.0):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    paths = [
        "/World/envs/env_0/FrontWorkbench/Top",
        "/World/envs/env_0/FrontWorkbench/LegFrontLeft",
        "/World/envs/env_0/FrontWorkbench/LegFrontRight",
        "/World/envs/env_0/FrontWorkbench/LegBackLeft",
        "/World/envs/env_0/FrontWorkbench/LegBackRight",
    ]
    bounds = []
    for path in paths:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        if not prim.IsActive() or not any(
            descendant.HasAPI(UsdPhysics.CollisionAPI)
            and UsdPhysics.CollisionAPI(descendant).GetCollisionEnabledAttr().Get() is not False
            for descendant in Usd.PrimRange(prim)
        ):
            continue
        aligned = cache.ComputeWorldBound(prim).ComputeAlignedRange()
        minimum, maximum = aligned.GetMin(), aligned.GetMax()
        bounds.append((
            path,
            [float(minimum[i]) - margin for i in range(3)],
            [float(maximum[i]) + margin for i in range(3)],
        ))
    return bounds


def point_obstacle(stage, point, margin=0.0):
    for path, minimum, maximum in scene_obstacle_bounds(stage, margin):
        if all(minimum[axis] <= point[axis] <= maximum[axis] for axis in range(3)):
            return path
    return None


def segment_obstacle(stage, start, end, margin=0.0, samples=41):
    for step in range(samples):
        t = step / (samples - 1)
        point = [start[axis] + (end[axis] - start[axis]) * t for axis in range(3)]
        obstacle = point_obstacle(stage, point, margin)
        if obstacle:
            return obstacle
    return None
