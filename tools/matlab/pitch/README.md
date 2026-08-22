# Pitch 离线建模与控制仿真

本目录提供 pitch 轴的离线模型、LQI+ESO 控制器、场景仿真、Monte Carlo
鲁棒性测试和实车日志导入接口。当前参数来自固件中的历史配置，只能作为仿真
初值，不能直接作为上车参数。

## 使用方式

在 MATLAB R2025a 中将当前目录切换到本目录，执行：

```matlab
results = run_all();
```

快速检查可减少 Monte Carlo 次数：

```matlab
options = struct('monte_carlo_runs', 50, 'save_figures', false);
results = run_all(options);
```

结果默认写入 `results/`，包括 MAT 数据、PNG 图和 Markdown 报告。

## 文件分工

日常只需要调用 `run_all.m`。其余文件是函数模块，不是需要逐个运行的脚本。

| 分组 | 文件 | 作用 |
|---|---|---|
| 入口 | `run_all.m` | 串联配置、设计、场景、Monte Carlo、绘图和报告 |
| 配置 | `pitch_default_config.m` | 集中保存采样周期、暂定物理参数、约束和不确定性范围 |
| 对象模型 | `pitch_model_terms.m`, `pitch_linearize_model.m` | 计算非线性模型项，并在工作点建立线性状态空间模型 |
| 控制器 | `pitch_design_lqi.m`, `pitch_design_eso.m` | 生成离散 LQI 和双模式 ESO 增益 |
| 参考轨迹 | `pitch_reference_step.m` | 生成限速度、限加速度的临界阻尼参考轨迹 |
| 仿真 | `pitch_make_scenarios.m`, `pitch_simulate.m` | 定义测试场景并执行控制器/对象闭环仿真 |
| 指标 | `pitch_step_metrics.m`, `pitch_tracking_metrics.m` | 分别评价阶跃与斜坡/正弦跟踪，避免混用指标 |
| 鲁棒验证 | `pitch_run_monte_carlo.m`, `pitch_self_test.m` | 参数随机化以及结构、数值、接口自检 |
| 输出 | `pitch_plot_results.m`, `pitch_write_report.m` | 生成 PNG 图、Markdown 报告和结果摘要 |
| 实车预留 | `pitch_log_schema.m`, `pitch_import_log.m`, `pitch_identify_from_log.m` | 定义日志、导入 CSV，并保存待实测辨识接口 |

这些文件不存在功能重复。可以把小函数合并进一个大文件，但会使单元检查、后续替换
连杆模型以及固件移植更困难，因此目前保留模块化结构更合适。

## 核心方程

真实对象框架使用

```text
M(theta) * theta_ddot + 0.5*M'(theta)*omega^2 + B*omega
  + tau_c*tanh(omega/omega_s) + G(theta)
  = n(theta)*tau_m + d
```

其中 `M(theta)` 是位置相关等效惯量，`n(theta)` 是连杆传动比，`G(theta)` 是
重力广义力矩，`d` 汇总未建模负载和外扰。当前函数只是结构占位，后续可用实测
`phi=f(theta)` 和 CAD/摆动数据替换。

LQI 使用状态

```text
x_a = [theta-theta_ref; omega-omega_ref; integral(theta-theta_ref)]
tau_fb = -K*x_a
```

并由离散代数 Riccati 方程最小化

```text
J = sum(x_a' Q x_a + tau_fb' R tau_fb)
```

`Q/R` 先按 Bryson 规则由允许误差和力矩预算归一化，再在候选尺度上搜索满足
调节时间、过冲和饱和约束的组合。总力矩由模型前馈、LQI 反馈和 ESO 补偿相加，
积分器采用条件积分抗饱和。

ESO 的扩张状态为 `z=[theta; omega; d_a]`，其中 `d_a` 是等效角加速度扰动：

```text
z_dot = [omega;
         -(B/M)*omega + (n/M)*tau_actual + known_model_terms + d_a;
         -lambda_d*d_a]
```

离散观测器使用实际完整力矩作为输入，并把重力、摩擦和科氏项作为已知模型项。
同时生成角度单测量和角度+gyro 双测量增益，实际使用哪一种由
`cfg.eso.use_dual_measurement` 决定。

临界阻尼参考轨迹满足

```text
theta_ref_ddot = wn^2*(theta_raw-theta_ref) - 2*wn*theta_ref_dot
```

随后施加速度和加速度限制。阶跃响应时间仍从原始遥控目标跳变时刻计算，而不是
从平滑后的参考轨迹开始计算。

## MATLAB 模块调用

同一目录下、文件名与主函数名一致的函数可以直接互相调用。`run_all.m` 还会执行
`addpath(root_dir)`，因此从其他工作目录调用时也能找到这些模块。函数之间不会自动
共享变量，本项目统一通过 `cfg`、`designs`、`scenario` 和 `bundle` 结构体显式传递
参数；一个文件末尾的 local function 只对该文件内部可见。

## 模型边界

- `ideal`：名义二阶刚体，不含摩擦、噪声、执行器动态和延迟。
- `realistic`：包含可变传动比/惯量、重力、摩擦、力矩动态、延迟、限幅、
  采样抖动、传感器误差和外部扰动。
- `legacy`：复现当前固件 LQR+ESO 的主要方程，并包含仍然生效的速度 PID
  阻尼，用于对照而不是生成新参数。

连杆几何、真实惯量和噪声尚未辨识。`pitch_default_config` 中所有
`provisional` 参数必须在实车采样后重新估计。

## 实车日志

`pitch_log_schema` 给出 CSV 字段定义，`pitch_import_log` 负责单位和字段检查，
`pitch_identify_from_log` 只在数据包含相应实验阶段时计算候选参数。它不会自动
把结果标记为可上车参数。
