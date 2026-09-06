#!/usr/bin/env python3
"""Tkinter slider UI for tuning D435 fake-visual teleop parameters."""

from __future__ import annotations

import argparse
import os
import shutil
import tempfile
import tkinter as tk
from pathlib import Path
from tkinter import ttk
from typing import Any, Callable

import yaml


SIDES = ("left", "right")


def parse_args() -> argparse.Namespace:
    default_config = Path(__file__).resolve().parent / "config" / "openarm_d435_teleop_visual.yaml"
    parser = argparse.ArgumentParser(description="Slider tuner for D435 visual teleop config")
    parser.add_argument("--config", type=Path, default=default_config)
    return parser.parse_args()


def load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def atomic_write_yaml(path: Path, data: dict) -> None:
    fd, tmp_name = tempfile.mkstemp(prefix=path.name, suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=False)
        shutil.copymode(path, tmp_name)
        os.replace(tmp_name, path)
    finally:
        if os.path.exists(tmp_name):
            os.unlink(tmp_name)


def get_path(data: dict, path: list[Any]) -> Any:
    cur: Any = data
    for key in path:
        cur = cur[key]
    return cur


def set_path(data: dict, path: list[Any], value: Any) -> None:
    cur: Any = data
    for key in path[:-1]:
        cur = cur[key]
    cur[path[-1]] = value


class SliderSpec:
    def __init__(
        self,
        label: str,
        minimum: float,
        maximum: float,
        step: float,
        getter: Callable[[dict], float],
        setter: Callable[[dict, float], None],
        hint: str,
    ) -> None:
        self.label = label
        self.minimum = minimum
        self.maximum = maximum
        self.step = step
        self.getter = getter
        self.setter = setter
        self.hint = hint


def scalar(path: list[Any]) -> tuple[Callable[[dict], float], Callable[[dict, float], None]]:
    def getter(data: dict) -> float:
        return float(get_path(data, path))

    def setter(data: dict, value: float) -> None:
        set_path(data, path, round(float(value), 4))

    return getter, setter


def both(path_suffix: list[Any]) -> tuple[Callable[[dict], float], Callable[[dict, float], None]]:
    left_path = ["arms", "left", "joint_pose"] + path_suffix
    right_path = ["arms", "right", "joint_pose"] + path_suffix

    def getter(data: dict) -> float:
        return float(get_path(data, left_path))

    def setter(data: dict, value: float) -> None:
        value = round(float(value), 4)
        set_path(data, left_path, value)
        set_path(data, right_path, value)

    return getter, setter


def signed_pair(
    path_suffix: list[Any], left_sign: float, right_sign: float
) -> tuple[Callable[[dict], float], Callable[[dict, float], None]]:
    left_path = ["arms", "left", "joint_pose"] + path_suffix
    right_path = ["arms", "right", "joint_pose"] + path_suffix

    def getter(data: dict) -> float:
        return abs(float(get_path(data, left_path)))

    def setter(data: dict, value: float) -> None:
        magnitude = abs(round(float(value), 4))
        set_path(data, left_path, round(left_sign * magnitude, 4))
        set_path(data, right_path, round(right_sign * magnitude, 4))

    return getter, setter


def mirrored_pair(
    path_suffix: list[Any], left_sign: float, right_sign: float
) -> tuple[Callable[[dict], float], Callable[[dict, float], None]]:
    left_path = ["arms", "left", "joint_pose"] + path_suffix
    right_path = ["arms", "right", "joint_pose"] + path_suffix

    def getter(data: dict) -> float:
        return float(get_path(data, left_path)) / left_sign

    def setter(data: dict, value: float) -> None:
        value = round(float(value), 4)
        set_path(data, left_path, round(left_sign * value, 4))
        set_path(data, right_path, round(right_sign * value, 4))

    return getter, setter


def joint_limit_pair(
    left_min_path: list[Any],
    right_max_path: list[Any],
    left_sign: float,
    right_sign: float,
) -> tuple[Callable[[dict], float], Callable[[dict, float], None]]:
    def getter(data: dict) -> float:
        return abs(float(get_path(data, left_min_path)))

    def setter(data: dict, value: float) -> None:
        magnitude = abs(round(float(value), 4))
        set_path(data, left_min_path, round(left_sign * magnitude, 4))
        set_path(data, right_max_path, round(right_sign * magnitude, 4))

    return getter, setter


def build_specs() -> list[SliderSpec]:
    specs: list[SliderSpec] = []

    def add(label: str, mn: float, mx: float, step: float, accessors, hint: str) -> None:
        getter, setter = accessors
        specs.append(SliderSpec(label, mn, mx, step, getter, setter, hint))

    add("前伸比例 x", 0.05, 1.20, 0.01, scalar(["mapping", "position_scale", 0]), "手往前伸时机械臂跟随的幅度；越大越灵敏。")
    add("左右比例 y", 0.05, 1.20, 0.01, scalar(["mapping", "position_scale", 1]), "双臂外张/内收的跟随幅度；越大横向动作越明显。")
    add("举手比例 z", 0.05, 1.20, 0.01, scalar(["mapping", "position_scale", 2]), "手往上举时机械臂抬起的幅度；越大越容易举高。")
    add("前伸最大范围", 0.02, 0.35, 0.01, scalar(["mapping", "max_delta_m", 0]), "限制前后方向最大动作，防止一次伸太远。")
    add("左右最大范围", 0.02, 0.25, 0.01, scalar(["mapping", "max_delta_m", 1]), "限制外张/内收最大动作范围。")
    add("举手最大范围", 0.02, 0.35, 0.01, scalar(["mapping", "max_delta_m", 2]), "限制上下方向最大动作范围。")

    add("控制频率 Hz", 1.0, 30.0, 0.1, scalar(["control", "command_rate_hz"]), "每秒发送几次机械臂命令；越高越跟手，太高会抖。")
    add("轨迹时间 s", 0.05, 1.50, 0.01, scalar(["control", "trajectory_time_s"]), "每次动作执行时间；越大越慢越平滑，越小越快。")

    add("上臂举高增益", 0.0, 2.50, 0.01, signed_pair(["joint1_up_gain"], -1.0, 1.0), "控制肩部/上臂抬起；举手不够高就增大。")
    add("上臂前伸增益", 0.0, 1.50, 0.01, signed_pair(["joint1_forward_gain"], -1.0, 1.0), "控制肩部带动上臂向前伸；前伸不明显就增大。")
    add("上臂举高上限", 0.4, 2.40, 0.01, joint_limit_pair(["arms", "left", "joint_pose", "joint1_min"], ["arms", "right", "joint_pose", "joint1_max"], -1.0, 1.0), "限制上臂最多能抬多高；举不到位可适当增大。")

    add("上臂外张增益", 0.0, 1.80, 0.01, signed_pair(["joint2_upper_outward_gain"], -1.0, 1.0), "肩到肘方向控制上臂向两边张开；外张不够就增大。")
    add("手腕外张辅助", 0.0, 0.80, 0.01, signed_pair(["joint2_outward_gain"], -1.0, 1.0), "用手腕左右位置辅助外张/内收；太大可能干扰拥抱动作。")

    add("J3 肘平面旋转增益", -2.0, 2.0, 0.01, mirrored_pair(["joint3_elbow_plane_gain"], -1.0, 1.0), "控制肘平面旋转到 J3 的方向和幅度；左右臂自动使用镜像符号，负数会同时反转方向。")

    add("肘部弯曲增益", 0.0, 2.00, 0.01, both(["joint4_elbow_bend_gain"]), "人体肘弯曲带动机械臂小臂弯曲；小臂夹角不明显就增大。")
    add("举手时肘辅助", 0.0, 0.80, 0.01, both(["joint4_up_gain"]), "举手时额外弯一点肘；太大上臂会不够主导。")
    add("肘部最大弯曲", 0.3, 2.00, 0.01, both(["joint4_max"]), "限制小臂最大弯曲程度；弯不够可增大。")

    add("J5 手掌旋转增益", -2.0, 2.0, 0.01, both(["joint5_hand_rotation_gain"]), "控制手掌绕前臂旋转到 J5 的方向和幅度；负数反向。")

    add("小臂内收增益", 0.0, 1.80, 0.01, signed_pair(["joint6_forearm_inward_gain"], -1.0, 1.0), "拥抱动作中小臂向中间收；内收不够就增大。")
    add("小臂内收起点", 0.0, 0.50, 0.01, both(["forearm_inward_start"]), "越小越早开始内收；太小可能普通动作也触发。")
    add("小臂内收满值", 0.30, 1.00, 0.01, both(["forearm_inward_full"]), "越小越快达到最大内收；太小会过于敏感。")

    return specs


class TunerApp:
    def __init__(self, root: tk.Tk, config_path: Path) -> None:
        self.root = root
        self.config_path = config_path
        self.data = load_yaml(config_path)
        self.specs = build_specs()
        self.pending_save_id: str | None = None
        self.status = tk.StringVar(value=f"Loaded {config_path}")
        root.title("D435 OpenArm 跟手参数调节")
        root.geometry("980x860")
        self._build()

    def _build(self) -> None:
        top = ttk.Frame(self.root, padding=8)
        top.pack(fill="x")
        ttk.Label(top, text=str(self.config_path)).pack(side="left", fill="x", expand=True)
        ttk.Button(top, text="重新加载", command=self.reload).pack(side="right", padx=(8, 0))
        ttk.Button(top, text="立即保存", command=self.save_now).pack(side="right")

        canvas = tk.Canvas(self.root, highlightthickness=0)
        scrollbar = ttk.Scrollbar(self.root, orient="vertical", command=canvas.yview)
        content = ttk.Frame(canvas, padding=8)
        content.bind("<Configure>", lambda _e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=content, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        for row, spec in enumerate(self.specs):
            self._add_slider(content, row * 2, spec)

        bottom = ttk.Frame(self.root, padding=8)
        bottom.pack(side="bottom", fill="x")
        ttk.Label(bottom, textvariable=self.status).pack(side="left", fill="x", expand=True)

    def _add_slider(self, parent: ttk.Frame, row: int, spec: SliderSpec) -> None:
        value = tk.DoubleVar(value=spec.getter(self.data))
        label = ttk.Label(parent, text=spec.label, width=18)
        label.grid(row=row, column=0, sticky="w", padx=(0, 8), pady=(8, 2))
        scale = ttk.Scale(parent, from_=spec.minimum, to=spec.maximum, variable=value, orient="horizontal")
        scale.grid(row=row, column=1, sticky="ew", pady=(8, 2))
        number = ttk.Label(parent, text=f"{value.get():.3f}", width=8)
        number.grid(row=row, column=2, sticky="e", padx=(8, 0), pady=(8, 2))
        hint = ttk.Label(parent, text=spec.hint, wraplength=820, foreground="#555555")
        hint.grid(row=row + 1, column=0, columnspan=3, sticky="w", padx=(0, 0), pady=(0, 6))
        parent.columnconfigure(1, weight=1)

        def on_change(*_args) -> None:
            rounded = round(value.get() / spec.step) * spec.step
            number.configure(text=f"{rounded:.3f}")
            spec.setter(self.data, rounded)
            self.schedule_save()

        value.trace_add("write", on_change)

    def schedule_save(self) -> None:
        if self.pending_save_id is not None:
            self.root.after_cancel(self.pending_save_id)
        self.pending_save_id = self.root.after(250, self.save_now)

    def save_now(self) -> None:
        self.pending_save_id = None
        atomic_write_yaml(self.config_path, self.data)
        self.status.set(f"已保存 {self.config_path}")

    def reload(self) -> None:
        self.data = load_yaml(self.config_path)
        self.status.set(f"已重新加载 {self.config_path}；如果滑条位置没刷新，请重启调参窗口")


def main() -> None:
    args = parse_args()
    root = tk.Tk()
    TunerApp(root, args.config)
    root.mainloop()


if __name__ == "__main__":
    main()
