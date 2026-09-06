# OpenArm Task Studio Architecture

## 设计来源

- MoveIt Task Constructor：将操作任务编译为有名称、属性、输入输出和失败信息的 Stage，而不是在 UI 回调中直接拼控制命令。
- FlexBE / BehaviorTree.CPP：每个运行单元拥有明确状态、失败策略和可观察结果；并行分支独立完成或失败。
- ManipArena-Sim：编辑/策略层只产生动作语义，仿真循环负责把动作转换为机器人控制并保持每帧稳定更新。
- Isaac ROS cuMotion：整臂和携带物碰撞规划属于后续运动规划后端，不应伪装成当前 TCP 采样能力。

## 当前分层

```text
TaskDocument / TaskNode
        │
        ▼
TaskCompiler（纯 Python）
  - Move / Grasp / Place 阶段展开
  - TCP 安全高度与桌面间隙
  - 直线细分和 TCP 障碍绕行
  - 左右腕角、姿态组、运行元数据
        │
        ▼
runtime request
  - preview: 单节点编译计划
  - execute: 左右臂独立 Stage 列表
        │
        ▼
launch_task_studio execution engine
  - 每个物理帧同时更新左右臂
  - position / joint / level_pose 控制模式
  - 稳定姿态组、阶段重试、失败汇总
  - 夹爪动作和物体相对位姿保持
```

## 编译阶段

- Move：`move`，必要时自动插入 `lift`、`transit` 和笛卡尔 `step`。
- Grasp：`early-prealign -> approach -> prealign -> align -> grasp -> retreat`。
- Place：`approach -> prealign -> align -> release -> retreat`。
- 每个阶段具有稳定 `stage_id`、`node_id`、`phase`、`control_mode`、`failure_policy`、`max_retries` 和 `speed_scale`。
- 同一 Grasp/Place 的姿态锁定阶段共享 `orientation_group`，只在组入口根据实际 TCP 生成一次校平姿态，后续不再逐航点漂移。

## 双臂执行

- 左右臂拥有独立航点索引、计时器、重试计数、姿态目标、夹持对象、对象偏移、状态和失败原因。
- 每个仿真帧依次计算两臂控制目标，再统一写入仿真；不再采用轮流更新导致的半控制频率。
- 一只手臂完成或失败不会中断另一只手臂；全部结束后输出 `left=...`、`right=...` 汇总。
- `approach` 阶段默认允许一次重新收敛，生产动作阶段默认不自动重试，避免重复闭合或释放。

## 明确边界

- 当前避障仍是 TCP 点/线段对 USD AABB 的采样，不是夹爪、手腕、前臂和携带物的完整碰撞规划。
- 动态校平保持进入姿态组时的工具接近轴，不等于经过标定的固定生产姿态。
- 下一规划后端应实现统一接口，将 TaskCompiler Stage 转换为 MoveIt Task Constructor / cuMotion 轨迹；UI 和 TaskDocument 不需要因此重写。
