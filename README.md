<div align="center">

# 🎯 SingleTargetTracking

**MATLAB单目标跟踪算法工具箱**

*Detect-Before-Track (DBT) & Track-Before-Detect (TBD)*

[![MATLAB](https://img.shields.io/badge/MATLAB-R2020b+-orange?style=flat-square&logo=mathworks)](https://www.mathworks.com/)
[![License](https://img.shields.io/badge/License-MIT-blue?style=flat-square)](LICENSE)
[![Version](https://img.shields.io/badge/Version-2.0.0-blue?style=flat-square)](startup.m)
[![GitHub](https://img.shields.io/badge/GitHub-yuruofeng/single_target_tracking-black?style=flat-square&logo=github)](https://github.com/yuruofeng/single_target_tracking)

[🚀 快速开始](#-快速开始) · [📖 文档](#-算法详解) · [📊 API](#-api-参考) · [❓ FAQ](#-常见问题-faq)

</div>

---

## 📋 目录

- [✨ 项目特性](#-项目特性)
- [🏗️ 项目架构](#️-项目架构)
- [🚀 快速开始](#-快速开始)
- [📊 算法详解](#-算法详解)
  - [检测后跟踪 (DBT)](#检测后跟踪-dbt)
  - [检测前跟踪 (TBD)](#检测前跟踪-tbd)
  - [运动模型](#运动模型)
- [📖 API 参考](#-api-参考)
- [⚙️ 配置说明](#️-配置说明)
- [🧪 测试](#-测试)
- [📁 目录结构](#-目录结构)
- [❓ 常见问题 (FAQ)](#-常见问题-faq)
- [📄 许可证](#-许可证)

---

## ✨ 项目特性

<table>
<tr>
<td width="50%">

### 🔬 核心算法

- **卡尔曼滤波家族**: EKF, UKF, CKF
- **粒子滤波**: Bootstrap PF (SIR)
- **交互多模型**: IMM算法
- **检测前跟踪**: DP-TBD, PF-TBD

</td>
<td width="50%">

### 🎯 运动模型

- **CV** - 匀速模型
- **CA** - 匀加速模型
- **CT** - 协调转弯模型
- **Singer** - Singer加速度模型
- **CS** - 当前统计模型

</td>
</tr>
<tr>
<td width="50%">

### 🏗️ 架构设计

- 📦 模块化包结构 (`+dbt`, `+tbd`, `+utils`, `+viz`)
- 🏭 工厂模式统一创建对象
- 🎨 策略模式实现算法切换
- 📐 模板方法复用通用流程

</td>
<td width="50%">

### 🛠️ 开发支持

- ✅ 完整的单元测试套件
- 📊 色盲友好的可视化工具
- 📝 详细的中文注释
- 🎮 丰富的演示脚本

</td>
</tr>
</table>

---

## 🏗️ 项目架构

```
┌─────────────────────────────────────────────────────────────────┐
│                   SingleTargetTracking v2.0                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────┐ │
│  │   📦 +dbt   │    │   📦 +tbd   │    │      📦 +utils      │ │
│  │  检测后跟踪  │    │  检测前跟踪  │    │       工具函数       │ │
│  ├─────────────┤    ├─────────────┤    ├─────────────────────┤ │
│  │ • EKF       │    │ • DP-TBD    │    │ • FilterUtils       │ │
│  │ • UKF       │    │ • PF-TBD    │    │ • MeasurementModel  │ │
│  │ • CKF       │    └─────────────┘    └─────────────────────┘ │
│  │ • PF        │                                                 │
│  │ • IMM       │    ┌─────────────────────────────────────────┐│
│  │ • Factory   │    │              📊 +viz                    ││
│  └─────────────┘    │             可视化模块                   ││
│                     ├─────────────────────────────────────────┤│
│  ┌─────────────┐    │ • plotDbtTrajectory                    ││
│  │  🎮 demos   │    │ • plotTbdTrajectory                    ││
│  │   演示脚本   │    │ • plotRmseComparison                   ││
│  ├─────────────┤    │ • plotManeuverComparison               ││
│  │ • demoDbt   │    │ • CUD色盲友好配色                       ││
│  │ • demoTbd   │    └─────────────────────────────────────────┘│
│  │ • demoManeuver │                                              │
│  └─────────────┘    ┌─────────────────────────────────────────┐│
│                     │              🧪 tests                    ││
│                     │              测试套件                    ││
│                     ├─────────────────────────────────────────┤│
│                     │ • TestDbtFilters                        ││
│                     │ • TestTbdAlgorithms                     ││
│                     │ • TestManeuverTracking                  ││
│                     │ • TestVisualization                     ││
│                     └─────────────────────────────────────────┘│
└─────────────────────────────────────────────────────────────────┘
```

---

## 🚀 快速开始

### 📥 环境要求

| 要求 | 说明 |
|:-----|:-----|
| MATLAB | R2020b 或更高版本 |
| 可选工具箱 | Signal Processing Toolbox, Statistics Toolbox |

### ⚡ 安装步骤

```bash
# 1. 克隆项目到本地
git clone https://github.com/yuruofeng/single_target_tracking.git

# 2. 进入项目目录
cd single_target_tracking
```

```matlab
% 3. 在MATLAB中运行初始化脚本
startup()
```

初始化成功后，您将看到：

```
╔════════════════════════════════════════════════════════════╗
║      SingleTargetTracking v2.0.0 - 单目标跟踪工具箱        ║
╠════════════════════════════════════════════════════════════╣
║  模块:                                                     ║
║    +dbt/   检测后跟踪 (EKF, UKF, CKF, IMM, ParticleFilter) ║
║    +tbd/   检测前跟踪 (DP-TBD, PF-TBD)                      ║
║    +utils/ 工具函数                                        ║
║    +viz/   可视化                                          ║
╚════════════════════════════════════════════════════════════╝
```

### 🎮 运行演示

<table>
<tr>
<th>演示脚本</th>
<th>说明</th>
<th>运行命令</th>
</tr>
<tr>
<td><code>demoAll.m</code></td>
<td>🌟 完整演示套件</td>
<td><code>run('demos/demoAll.m')</code></td>
</tr>
<tr>
<td><code>demoDbt.m</code></td>
<td>🎯 DBT滤波器对比</td>
<td><code>run('demos/demoDbt.m')</code></td>
</tr>
<tr>
<td><code>demoTbd.m</code></td>
<td>🔍 TBD算法演示</td>
<td><code>run('demos/demoTbd.m')</code></td>
</tr>
<tr>
<td><code>demoManeuver.m</code></td>
<td>🔄 机动目标跟踪</td>
<td><code>run('demos/demoManeuver.m')</code></td>
</tr>
</table>

### 🧪 运行测试

```matlab
% 运行完整测试套件
run('tests/runAllTests.m')

% 运行单个测试
run('tests/TestDbtFilters.m')
```

---

## 📊 算法详解

### 检测后跟踪 (DBT)

<table>
<tr>
<th width="12%">滤波器</th>
<th width="35%">描述</th>
<th width="18%">状态维度</th>
<th width="18%">复杂度</th>
<th width="17%">特点</th>
</tr>
<tr>
<td><strong>EKF</strong></td>
<td>扩展卡尔曼滤波 (雅可比矩阵)</td>
<td>5 (CT模型)</td>
<td>O(n²)</td>
<td>计算高效</td>
</tr>
<tr>
<td><strong>UKF</strong></td>
<td>无迹卡尔曼滤波 (Sigma点)</td>
<td>5 (CT模型)</td>
<td>O(n³)</td>
<td>精度更高</td>
</tr>
<tr>
<td><strong>CKF</strong></td>
<td>容积卡尔曼滤波 (容积点)</td>
<td>5 (CT模型)</td>
<td>O(n³)</td>
<td>数值稳定</td>
</tr>
<tr>
<td><strong>MotionModelEKF</strong></td>
<td>统一运动模型EKF (CV/CA/CT/Singer/CS)</td>
<td>4-6</td>
<td>O(n²)</td>
<td>多模型支持</td>
</tr>
<tr>
<td><strong>ParticleFilter</strong></td>
<td>Bootstrap粒子滤波 (系统重采样)</td>
<td>5 (CT模型)</td>
<td>O(Np×n)</td>
<td>非线性强</td>
</tr>
<tr>
<td><strong>IMM</strong></td>
<td>交互多模型算法</td>
<td>4-6</td>
<td>O(M×n³)</td>
<td>机动适应</td>
</tr>
</table>

### 检测前跟踪 (TBD)

<table>
<tr>
<th width="15%">算法</th>
<th width="35%">描述</th>
<th width="20%">状态维度</th>
<th width="15%">复杂度</th>
<th width="15%">适用场景</th>
</tr>
<tr>
<td><strong>DP-TBD</strong></td>
<td>动态规划网格搜索</td>
<td>2 (位置)</td>
<td>O(N²×K)</td>
<td>低SNR目标</td>
</tr>
<tr>
<td><strong>PF-TBD</strong></td>
<td>粒子滤波 + 对数似然</td>
<td>5 (位置,速度,幅度)</td>
<td>O(Np×K)</td>
<td>弱目标检测</td>
</tr>
</table>

### 运动模型

`MotionModelEKF` 支持多种运动模型：

| 模型 | 状态向量 | 自由度 | 描述 |
|:-----|:---------|:-------|:-----|
| **CV** | `[x, vx, y, vy]` | 4 | 匀速直线运动 |
| **CA** | `[x, vx, ax, y, vy, ay]` | 6 | 匀加速运动 |
| **CT** | `[x, vx, y, vy, ω]` | 5 | 协调转弯 |
| **Singer** | `[x, vx, ax, y, vy, ay]` | 6 | Singer加速度模型 |
| **CS** | `[x, vx, ax, y, vy, ay]` | 6 | 当前统计模型 |

```matlab
% 创建不同运动模型的EKF
cvEKF = dbt.MotionModelEKF(dbt.MotionModelConfig('CV'));
caEKF = dbt.MotionModelEKF(dbt.MotionModelConfig('CA'));
ctEKF = dbt.MotionModelEKF(dbt.MotionModelConfig('CT'));
```

---

## 📖 API 参考

### 🏭 工厂模式 (推荐)

使用工厂模式统一创建滤波器对象：

```matlab
% 创建DBT滤波器
filter = dbt.FilterFactory.create('EKF-CV', 'dt', 1.0, 'q', 0.1);

% 创建默认IMM (CV + CA + CT)
imm = dbt.FilterFactory.createDefaultIMM({'CV', 'CA', 'CT'});

% 创建TBD算法
dp = tbd.TbdFactory.create('DP');
pf = tbd.TbdFactory.create('PF');
```

### 📦 DBT 模块

```matlab
%% 配置
cfg = dbt.Config('numSteps', 100, 'dt', 1);

%% 场景生成
scenario = dbt.Scenario(cfg);
[truthX, meas] = scenario.generate();

%% 运动模型EKF
mmConfig = dbt.MotionModelConfig('CT', 'dt', 1.0, 'q', 0.1);
ekf = dbt.MotionModelEKF(mmConfig);
[states, covars] = ekf.run(meas, x0, P0);

%% 经典CT模型EKF
ekf = dbt.EKF(cfg);
[states, covars] = ekf.run(meas, x0, P0);

%% 粒子滤波
pf = dbt.ParticleFilter(cfg, 'numParticles', 500);
[states, weights] = pf.run(meas, x0);

%% IMM算法
imm = dbt.IMM(dbt.ConfigIMM({'CV', 'CA', 'CT'}));
[states, modes] = imm.run(meas, x0, P0);
```

### 📦 TBD 模块

```matlab
%% 配置
cfg = tbd.Config('numFrames', 50, 'gridSize', [100, 100]);

%% 场景生成
scenario = tbd.Scenario(cfg);
[~, trueState, measData, psfKernel] = scenario.generate();

%% DP-TBD
dp = tbd.DpTbd(cfg);
dp.run(measData, psfKernel);
[track, score] = dp.getResults();

%% PF-TBD
pf = tbd.PfTbd(cfg);
pf.run(measData, trueState(1,:), psfKernel);
[posRmse, velRmse] = pf.computeRmse(trueState);
```

### 📊 可视化模块

```matlab
%% 色盲友好配色 (CUD色彩)
viz.Visualizer.COLORS.blue    % #0077BB
viz.Visualizer.COLORS.orange  % #EE7733
viz.Visualizer.COLORS.cyan    % #33BBEE
viz.Visualizer.COLORS.green   % #228833

%% 绑图函数
viz.Visualizer.plotDbtTrajectory(truthX, meas, estimates, labels)
viz.Visualizer.plotRmseComparison(rmseData, labels, 'Position RMSE')
viz.Visualizer.plotManeuverComparison(results, configs)
viz.Visualizer.plotTbdTrajectory(trueState, estimates, labels)
```

---

## ⚙️ 配置说明

### demoDbt 参数

| 参数 | 默认值 | 说明 |
|:-----|:-------|:-----|
| `mcRuns` | 1 | Monte Carlo运行次数 |
| `numSteps` | 100 | 时间步数 |
| `dt` | 1 | 采样间隔 [s] |
| `numParticles` | 500 | 粒子数 (PF) |
| `saveResults` | false | 保存结果到.mat文件 |
| `showPlots` | true | 显示绑图 |

### demoTbd 参数

| 参数 | 默认值 | 说明 |
|:-----|:-------|:-----|
| `gridSize` | [100, 100] | 图像尺寸 |
| `numFrames` | 50 | 帧数 |
| `snr` | 9.5 | 目标信噪比 [dB] |
| `numParticles` | 500 | 粒子数 (PF-TBD) |
| `motionModel` | 'CV' | 运动模型类型 |
| `saveResults` | false | 保存结果 |
| `animate` | false | 显示动画 |

---

## 🧪 测试

### 测试套件

| 测试文件 | 覆盖范围 |
|:---------|:---------|
| `TestDbtFilters.m` | EKF, UKF, CKF, PF 滤波器 |
| `TestTbdAlgorithms.m` | DP-TBD, PF-TBD 算法 |
| `TestManeuverTracking.m` | IMM 机动跟踪 |
| `TestVisualization.m` | 可视化函数 |

### 运行测试

```matlab
% 运行所有测试
run('tests/runAllTests.m')

% 单独测试DBT滤波器
run('tests/TestDbtFilters.m')

% 单独测试TBD算法
run('tests/TestTbdAlgorithms.m')
```

---

## 📁 目录结构

```
single_target_tracking/
├── 📂 +dbt/                       # 检测后跟踪模块
│   ├── 📄 BaseFilter.m            # 抽象滤波器接口
│   ├── 📄 KalmanFilterBase.m      # 卡尔曼滤波基类
│   ├── 📄 EKF.m                   # 扩展卡尔曼滤波
│   ├── 📄 UKF.m                   # 无迹卡尔曼滤波
│   ├── 📄 CKF.m                   # 容积卡尔曼滤波
│   ├── 📄 MotionModelEKF.m        # 统一运动模型EKF
│   ├── 📄 ParticleFilter.m        # 粒子滤波器
│   ├── 📄 IMM.m                   # 交互多模型
│   ├── 📄 Config.m                # DBT配置
│   ├── 📄 ConfigIMM.m             # IMM配置
│   ├── 📄 ConfigManeuver.m        # 机动配置
│   ├── 📄 MotionModelConfig.m     # 运动模型配置
│   ├── 📄 Scenario.m              # 轨迹/量测生成
│   ├── 📄 ScenarioManeuver.m      # 机动场景生成
│   └── 📄 FilterFactory.m         # 滤波器工厂
│
├── 📂 +tbd/                       # 检测前跟踪模块
│   ├── 📄 BaseTbd.m               # 抽象TBD接口
│   ├── 📄 DpTbd.m                 # 动态规划TBD
│   ├── 📄 PfTbd.m                 # 粒子滤波TBD
│   ├── 📄 Config.m                # TBD配置
│   ├── 📄 Scenario.m              # 图像域场景
│   └── 📄 TbdFactory.m            # TBD工厂
│
├── 📂 +utils/                     # 共享工具
│   ├── 📄 FilterUtils.m           # Cholesky, Sigma点, 重采样
│   └── 📄 MeasurementModel.m      # CT/CV模型, PSF
│
├── 📂 +viz/                       # 可视化模块
│   └── 📄 Visualizer.m            # 色盲友好绑图
│
├── 📂 demos/                      # 演示脚本
│   ├── 📄 demoDbt.m               # DBT演示
│   ├── 📄 demoTbd.m               # TBD演示
│   ├── 📄 demoManeuver.m          # 机动跟踪演示
│   └── 📄 demoAll.m               # 完整套件演示
│
├── 📂 tests/                      # 单元测试
│   ├── 📄 TestDbtFilters.m        # DBT滤波器测试
│   ├── 📄 TestTbdAlgorithms.m     # TBD算法测试
│   ├── 📄 TestManeuverTracking.m  # 机动跟踪测试
│   ├── 📄 TestVisualization.m     # 可视化测试
│   └── 📄 runAllTests.m           # 测试运行器
│
├── 📂 config/                     # 配置文件
│   └── 📄 ProjectConfig.m         # 项目配置
│
├── 📂 resources/                  # 资源文件
│   └── 📄 README.md               # 资源说明
│
├── 📄 startup.m                   # 项目初始化
├── 📄 LICENSE                     # MIT许可证
├── 📄 .gitignore                  # Git忽略规则
└── 📄 README.md                   # 本文件
```

---

## ❓ 常见问题 (FAQ)

<details>
<summary><strong>🔧 安装与配置</strong></summary>

### Q: 如何验证安装是否成功？

运行初始化脚本后，执行以下命令验证：

```matlab
% 检查模块是否可用
which dbt.EKF
which tbd.DpTbd
which viz.Visualizer

% 运行快速测试
run('tests/TestDbtFilters.m')
```

### Q: 提示"未找到函数"怎么办？

确保已运行 `startup.m` 初始化脚本：

```matlab
cd('path/to/single_target_tracking')
startup()
```

</details>

<details>
<summary><strong>🎯 算法使用</strong></summary>

### Q: 如何选择合适的滤波器？

| 场景 | 推荐滤波器 | 原因 |
|:-----|:-----------|:-----|
| 目标匀速运动 | EKF-CV | 计算高效 |
| 目标转弯机动 | IMM (CV+CA+CT) | 自适应切换 |
| 强非线性 | UKF/CKF | Sigma点逼近 |
| 非高斯噪声 | ParticleFilter | 不依赖高斯假设 |
| 低SNR弱目标 | PF-TBD | 检测前跟踪 |

### Q: 如何调整过程噪声Q和量测噪声R？

```matlab
% 较大的Q：滤波器更信任量测，响应快但可能抖动
cfg = dbt.Config('q', 1.0, 'r', 0.1);

% 较小的Q：滤波器更信任预测，平滑但响应慢
cfg = dbt.Config('q', 0.01, 'r', 0.1);
```

### Q: IMM的模型概率如何解释？

```matlab
% 获取模型概率
[states, ~, modeProbs] = imm.run(meas, x0, P0);

% modeProbs(:, 1) = CV模型概率
% modeProbs(:, 2) = CA模型概率
% modeProbs(:, 3) = CT模型概率
% 概率最高的模型即为当前最可能的运动模式
```

</details>

<details>
<summary><strong>📊 可视化</strong></summary>

### Q: 如何自定义绑图颜色？

```matlab
% 使用CUD色盲友好配色
colors = viz.Visualizer.COLORS;
plot(x, y, 'Color', colors.blue, 'LineWidth', 2);

% 可用颜色: blue, orange, cyan, green, yellow, red
```

### Q: 如何保存高质量的图像？

```matlab
% 保存为矢量图
print('figure1', '-dpdf', '-r300');

% 保存为高清PNG
print('figure1', '-dpng', '-r300');
```

</details>

<details>
<summary><strong>⚡ 性能优化</strong></summary>

### Q: 如何提高粒子滤波的运行速度？

1. **减少粒子数**：从500降到200-300
2. **向量化计算**：避免for循环
3. **并行化**：使用 `parfor` 进行Monte Carlo仿真

```matlab
% 并行Monte Carlo
parfor i = 1:mcRuns
    results(i) = runFilter(cfg);
end
```

### Q: 如何处理长时间序列？

对于超长序列(>1000步)，建议分段处理：

```matlab
segmentLength = 500;
for seg = 1:ceil(numSteps/segmentLength)
    idx = (seg-1)*segmentLength + 1 : min(seg*segmentLength, numSteps);
    [states(:,:,seg), covars(:,:,seg)] = ekf.run(meas(:,idx), x0, P0);
    x0 = states(end,:,seg)';  % 更新初始状态
end
```

</details>

---

## 📄 许可证

本项目采用 [MIT License](LICENSE) 开源许可。

---

<div align="center">

**[⬆ 返回顶部](#-targettracking)**

Made with ❤️ for Target Tracking Research

**[GitHub Repository](https://github.com/yuruofeng/single_target_tracking)**

</div>
