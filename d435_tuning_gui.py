#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
from pathlib import Path


UPSTREAM_TUNER = Path(__file__).resolve().parent / "d435_tuning_base.py"

MOTOR_EFFECTS = {
    "前伸比例 x": "J1, J4",
    "左右比例 y": "J2",
    "举手比例 z": "J1, J4",
    "前伸最大范围": "J1, J4",
    "左右最大范围": "J2",
    "举手最大范围": "J1, J4",
    "控制频率 Hz": "J1-J7（响应速度）",
    "轨迹时间 s": "J1-J7（运动平滑度）",
    "上臂举高增益": "J1",
    "上臂前伸增益": "J1",
    "上臂后摆增益": "J1",
    "上臂后摆上限": "J1",
    "上臂后收满值": "J1",
    "上臂举高上限": "J1",
    "上臂外张增益": "J2",
    "手腕外张辅助": "J2",
    "J3 肘平面旋转增益": "J3",
    "肘部弯曲增益": "J4",
    "举手时肘辅助": "J4",
    "肘部最大弯曲": "J4",
    "J5 手掌旋转增益": "J5",
    "小臂内收增益": "J6",
    "小臂内收起点": "J6",
    "小臂内收满值": "J6",
}


def load_upstream_tuner():
    spec = importlib.util.spec_from_file_location("openarm_d435_upstream_tuner", UPSTREAM_TUNER)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def task_studio_specs(module):
    specs = module.build_specs()
    elbow_limit = next(spec for spec in specs if spec.label == "肘部最大弯曲")
    elbow_limit.maximum = 2.443
    elbow_limit.hint = "限制小臂最大弯曲程度；OpenArm v1.0 模型上限为 2.443 rad（140 度）。"
    forward_index = next(
        index for index, spec in enumerate(specs) if spec.label == "上臂前伸增益"
    )
    backward_getter, backward_setter = module.signed_pair(
        ["joint1_backward_gain"], 1.0, -1.0
    )
    limit_getter, limit_setter = module.joint_limit_pair(
        ["arms", "left", "joint_pose", "joint1_max"],
        ["arms", "right", "joint_pose", "joint1_min"],
        1.0,
        -1.0,
    )
    specs[forward_index + 1:forward_index + 1] = [
        module.SliderSpec(
            "上臂后摆增益",
            0.0,
            3.0,
            0.01,
            backward_getter,
            backward_setter,
            "控制手腕向中立点后方移动时的 J1 后摆幅度；左右臂自动镜像。",
        ),
        module.SliderSpec(
            "上臂后摆上限",
            0.2,
            2.0,
            0.01,
            limit_getter,
            limit_setter,
            "限制 J1 后摆最大角度；增益调大但动作不再增加时可同步提高。",
        ),
        module.SliderSpec(
            "上臂后收满值",
            0.1,
            1.0,
            0.01,
            *module.both(["upper_backward_full"]),
            "越小越容易由肩到肘的后收方向触发满幅 J1 后摆。",
        ),
    ]
    for spec in specs:
        spec.affected_motors = MOTOR_EFFECTS[spec.label]
    return specs


def task_studio_app_class(module):
    class TaskStudioTunerApp(module.TunerApp):
        def _build(self):
            self.root.geometry("860x820")
            self.root.minsize(720, 560)

            top = module.ttk.Frame(self.root, padding=8)
            top.pack(fill="x")
            module.ttk.Label(top, text=str(self.config_path)).pack(
                side="left", fill="x", expand=True
            )
            module.ttk.Button(top, text="重新加载", command=self.reload).pack(
                side="right", padx=(8, 0)
            )
            module.ttk.Button(top, text="立即保存", command=self.save_now).pack(side="right")

            bottom = module.ttk.Frame(self.root, padding=8)
            bottom.pack(side="bottom", fill="x")
            module.ttk.Label(bottom, textvariable=self.status).pack(
                side="left", fill="x", expand=True
            )

            body = module.ttk.Frame(self.root)
            body.pack(fill="both", expand=True)
            canvas = module.tk.Canvas(body, highlightthickness=0)
            scrollbar = module.ttk.Scrollbar(body, orient="vertical", command=canvas.yview)
            scrollbar.pack(side="right", fill="y")
            canvas.pack(side="left", fill="both", expand=True)

            content = module.ttk.Frame(canvas, padding=8)
            content_window = canvas.create_window((0, 0), window=content, anchor="nw")
            content.bind(
                "<Configure>",
                lambda _event: canvas.configure(scrollregion=canvas.bbox("all")),
            )
            canvas.bind(
                "<Configure>",
                lambda event: canvas.itemconfigure(content_window, width=event.width),
            )
            canvas.configure(yscrollcommand=scrollbar.set)

            for row, spec in enumerate(self.specs):
                self._add_slider(content, row * 2, spec)

        def _add_slider(self, parent, row, spec):
            value = module.tk.DoubleVar(value=spec.getter(self.data))
            value_text = module.tk.StringVar(value=f"{value.get():.3f}")
            module.ttk.Label(parent, text=spec.label, width=18).grid(
                row=row, column=0, sticky="w", padx=(0, 8), pady=(8, 2)
            )
            module.ttk.Scale(
                parent, from_=spec.minimum, to=spec.maximum,
                variable=value, orient="horizontal",
            ).grid(row=row, column=1, sticky="ew", pady=(8, 2))
            module.tk.Label(
                parent, textvariable=value_text, width=9, anchor="e",
                background="#ffffff", foreground="#111111",
                relief="solid", borderwidth=1, padx=5,
            ).grid(row=row, column=2, sticky="e", padx=(8, 0), pady=(8, 2))
            module.ttk.Label(
                parent,
                text=f"影响电机：{spec.affected_motors}  |  {spec.hint}",
                wraplength=760,
                foreground="#444444",
            ).grid(row=row + 1, column=0, columnspan=3, sticky="w", pady=(0, 6))
            parent.columnconfigure(1, weight=1)

            def on_change(*_args):
                rounded = round(value.get() / spec.step) * spec.step
                value_text.set(f"{rounded:.3f}")
                spec.setter(self.data, rounded)
                self.schedule_save()

            value.trace_add("write", on_change)

    return TaskStudioTunerApp


def main():
    module = load_upstream_tuner()
    module.build_specs = lambda: task_studio_specs(load_upstream_tuner())
    module.TunerApp = task_studio_app_class(module)
    module.main()


if __name__ == "__main__":
    main()
