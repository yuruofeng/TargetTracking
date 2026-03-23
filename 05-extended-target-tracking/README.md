# 扩展目标跟踪算法

<div align="center">

[![MATLAB](https://img.shields.io/badge/MATLAB-R2019b%2B-orange?style=for-the-badge&logo=mathworks&logoColor=white)](https://www.mathworks.com/products/matlab.html)
[![License](https://img.shields.io/badge/License-Educational-green?style=for-the-badge&logo=creativecommons&logoColor=white)](../../LICENSE)
[![Code Style](https://img.shields.io/badge/Code%20Style-Standardized-blue?style=for-the-badge&logo=stylelint&logoColor=white)](../MATLAB_Coding_Standards.md)

**基于GGIW-PHD和星凸模型的扩展目标跟踪算法实现**

包含GGIW-PHD滤波器、星凸形状跟踪器、扩展目标PHD滤波器等核心算法

</div>

---

## 📑 目录

- [📖 项目简介](#-项目简介)
- [📁 目录结构](#-目录结构)
- [✨ 核心特性](#-核心特性)
- [⚙️ 环境要求](#️-环境要求)
- [🚀 快速开始](#-快速开始)
- [📦 模块说明](#-模块说明)
- [📋 API参考](#-api参考)
- [🧪 测试说明](#-测试说明)
- [📝 更新日志](#-更新日志)
- [🙏 致谢](#-致谢)

---

## 📖 项目简介

本项目实现了扩展目标跟踪领域的多种经典算法，包括：

- **🔹 GGIW-PHD滤波器**: 基于Gamma-Gaussian-Inverse-Wishart的概率假设密度滤波器，用于多扩展目标跟踪
- **🌟 星凸形状跟踪器**: 使用傅里叶级数描述符结合UKF/CKF的星凸目标形状估计
- **📊 扩展目标PHD滤波器**: 基于Granström 2010算法的扩展目标PHD滤波器

项目采用模块化设计，消除全局变量，统一命名规范，支持完整的日志记录和配置管理。

---

## 📁 目录结构

```
05-extended-target-tracking/
├── 📂 +utils/                           # 🛠️ 通用工具包
│   ├── 📄 Logger.m                      # 📝 日志记录器
│   ├── 📄 ConfigManager.m               # ⚙️ 配置管理器
│   ├── 📄 Constants.m                   # 🔢 常量定义
│   └── 📄 ArrayUtils.m                  # 📊 数组工具
│
├── 📂 +ggiw/                            # 🔹 GGIW-PHD滤波器包
│   ├── 📄 GgiwFilter.m                  # 主滤波器类
│   ├── 📄 generateClutter.m             # 杂波生成函数
│   ├── 📄 generateExtendedMeasurements.m # 扩展测量生成函数
│   └── 📄 plotExtentEllipse.m           # 扩展可视化函数
│
├── 📂 +starconvex/                      # 🌟 星凸目标跟踪包
│   ├── 📄 StarConvexTracker.m           # 主跟踪器类
│   ├── 📄 generateGroundTruth.m         # 真实数据生成函数
│   └── 📄 generateMeasurements.m        # 测量生成函数
│
├── 📂 +phd/                             # 📊 PHD滤波器包
│   ├── 📄 ExtendedTargetPhdFilter.m     # 扩展目标PHD滤波器
│   └── 📄 partitionMeasurementSet.m     # 测量划分函数
│
├── 📂 demos/                            # 🎬 演示脚本
│   ├── 📄 GGIW_PHD_Main.m               # GGIW-PHD演示
│   └── 📄 StarConvex_Main.m             # 星凸目标演示
│
├── 📂 tests/                            # 🧪 测试脚本
│   └── 📄 test_all.m                    # 综合测试
│
└── 📂 docs/                             # 📚 文档
    └── 📄 REFACTORING_REPORT.md         # 重构报告
```

---

## ✨ 核心特性

| 特性 | 描述 |
| :--- | :--- |
| 🔧 **模块化设计** | 使用MATLAB包（package）机制组织代码，便于扩展和维护 |
| ⚙️ **统一配置管理** | 集中式ConfigManager管理所有参数，支持参数验证和类型检查 |
| 📝 **完整日志系统** | 多级别日志记录（DEBUG/INFO/WARNING/ERROR），支持文件输出 |
| 🛡️ **错误处理机制** | 完善的参数验证和异常处理 |
| 📖 **标准化文档** | 中文注释，符合MATLAB编码规范 |
| 🧪 **单元测试** | 完整的测试覆盖，确保代码质量 |

---

## ⚙️ 环境要求

### 🔴 必需环境

| 软件 | 版本要求 | 说明 |
| :--- | :------- | :--- |
| ![MATLAB](https://img.shields.io/badge/MATLAB-R2019b%2B-orange?logo=mathworks) | R2019b 或更高版本 | 核心运行环境 |

### 🟡 可选依赖

| 工具箱 | 用途 |
| :----- | :--- |
| Statistics Toolbox | 部分统计计算功能 |

---

## 🚀 快速开始

### 📥 安装

```matlab
% 方式一：添加项目路径
addpath('path/to/05-extended-target-tracking');

% 方式二：切换到项目目录
cd('path/to/05-extended-target-tracking');
```

### ▶️ 运行演示

```matlab
% GGIW-PHD扩展目标跟踪演示
run('demos/GGIW_PHD_Main.m');

% 星凸形状跟踪演示
run('demos/StarConvex_Main.m');
```

### 🧪 运行测试

```matlab
% 运行综合测试
run('tests/test_all.m');
```

---

## 📦 模块说明

### 🔹 GGIW-PHD滤波器

[![GGIW](https://img.shields.io/badge/模块-ggiw-blue)](+ggiw/)

基于Gamma-Gaussian-Inverse-Wishart的扩展目标PHD滤波器，实现目标状态与扩展形状的联合估计。

**主要功能**:
- 扩展目标状态估计
- 目标扩展（椭圆形状）估计
- 杂波生成与处理
- 多目标跟踪

**使用示例**:

```matlab
% 初始化配置
config = utils.ConfigManager.getInstance();
config.set('tracking.pD', 0.99);  % 检测概率
config.set('tracking.pS', 0.99);  % 存活概率

% 创建滤波器
ggiwFilter = ggiw.GgiwFilter(config);

% 预测步骤
ggiwFilter.predict(motionModel, processNoise);

% 更新步骤
ggiwFilter.update(measurements, measurementModel, measurementNoise);

% 提取状态估计
estimates = ggiwFilter.extractStates();
```

---

### 🌟 星凸形状跟踪器

[![StarConvex](https://img.shields.io/badge/模块-starconvex-green)](+starconvex/)

使用傅里叶级数描述符结合UKF/CKF滤波器的星凸目标形状跟踪器。

**主要功能**:
- 星凸形状建模（傅里叶级数）
- UKF/CKF滤波器选择
- 形状参数估计
- 轨迹生成与测量

**使用示例**:

```matlab
% 创建跟踪器（11个傅里叶系数，使用UKF）
tracker = starconvex.StarConvexTracker(11, 'UKF');

% 初始化状态
tracker.initialize(initialState, initialCovariance);

% 预测
tracker.predict(transitionMatrix, processNoise);

% 更新
tracker.update(measurement, noiseMean, noiseCov);

% 获取形状估计
phiVector = linspace(0, 2*pi, 200);
shape = tracker.getShape(phiVector);
```

---

### 📊 扩展目标PHD滤波器

[![PHD](https://img.shields.io/badge/模块-phd-purple)](+phd/)

基于Granström 2010算法的扩展目标PHD滤波器实现。

**主要功能**:
- 扩展目标PHD滤波
- 测量集划分
- 目标数量估计
- 状态提取

**使用示例**:

```matlab
% 创建滤波器
phdFilter = phd.ExtendedTargetPhdFilter(config);

% 预测
phdFilter.predict(motionModel, processNoise);

% 测量划分
partitions = phd.partitionMeasurementSet(measurements);

% 更新
phdFilter.update(partitions, measurementModel, measurementNoise);

% 提取状态
estimates = phdFilter.extractStates();
```

---

### 🛠️ 工具包

[![Utils](https://img.shields.io/badge/模块-utils-yellow)](+utils/)

通用工具类，提供日志记录、配置管理、常量定义和数组操作功能。

| 类 | 功能 |
| :--- | :--- |
| `Logger` | 多级别日志记录，支持控制台和文件输出 |
| `ConfigManager` | 单例模式配置管理，支持参数验证 |
| `Constants` | 常用常量定义（空间维度、单位矩阵等） |
| `ArrayUtils` | 数组操作工具函数 |

**Logger使用示例**:

```matlab
% 创建日志记录器
logger = utils.Logger('MyModule', 'INFO');

% 记录不同级别日志
logger.debug('调试信息');
logger.info('处理开始');
logger.warning('参数可能不正确');
logger.error('处理失败', exception);

% 函数执行时间记录
logger.logFunctionStart('myFunction');
% ... 函数代码 ...
logger.logFunctionEnd('myFunction', elapsedTime);
```

**ConfigManager使用示例**:

```matlab
% 获取单例实例
config = utils.ConfigManager.getInstance();

% 设置参数
config.set('tracking.pD', 0.99);
config.set('tracking.pS', 0.99);
config.set('measurement.sigmaX', 5);

% 获取参数
pD = config.get('tracking.pD');
sigmaX = config.get('measurement.sigmaX', 5);  % 带默认值

% 从文件加载配置
config.loadFromFile('config.mat');

% 验证参数
config.validate('tracking.pD', @(x) x > 0 && x <= 1);
```

---

## 📋 API参考

### 🔹 GgiwFilter类

| 方法 | 说明 |
| :--- | :--- |
| `GgiwFilter(config)` | 构造函数，创建GGIW滤波器实例 |
| `predict(motionModel, processNoise)` | 预测步骤 |
| `update(measurements, measModel, measNoise)` | 更新步骤 |
| `extractStates()` | 提取状态估计 |
| `prune(threshold)` | 剪枝操作 |
| `merge(threshold)` | 合并操作 |

### 🌟 StarConvexTracker类

| 方法 | 说明 |
| :--- | :--- |
| `StarConvexTracker(numCoeff, filterType, config)` | 构造函数 |
| `initialize(state, covariance)` | 初始化状态和协方差 |
| `predict(transitionMatrix, processNoise)` | 预测步骤 |
| `update(measurement, noiseMean, noiseCov)` | 更新步骤 |
| `getShape(phiVector)` | 获取形状估计 |
| `getState()` | 获取当前状态 |
| `getCovariance()` | 获取当前协方差 |

### 📊 ExtendedTargetPhdFilter类

| 方法 | 说明 |
| :--- | :--- |
| `ExtendedTargetPhdFilter(config)` | 构造函数 |
| `predict(motionModel, processNoise)` | 预测步骤 |
| `update(partitions, measModel, measNoise)` | 更新步骤 |
| `extractStates()` | 提取状态估计 |
| `prune(threshold)` | 剪枝操作 |

---

## 🧪 测试说明

### 测试内容

| 测试项 | 描述 |
| :----- | :--- |
| Logger类测试 | 日志记录、级别设置、格式化输出 |
| ConfigManager类测试 | 参数设置/获取、验证、文件加载 |
| ArrayUtils类测试 | 数组操作功能 |
| GgiwFilter类测试 | GGIW滤波器预测/更新/提取 |
| StarConvexTracker类测试 | 星凸跟踪器预测/更新/形状估计 |
| ExtendedTargetPhdFilter类测试 | PHD滤波器预测/更新/提取 |

### 运行测试

```matlab
% 运行所有测试
run('tests/test_all.m');
```

### 测试输出示例

```
========================================
  MATLAB 项目重构综合测试
  测试时间: 2026-03-01 10:00:00
========================================

【测试1】Logger 日志类测试...
  [通过] Logger类功能正常

【测试2】ConfigManager 配置管理类测试...
  [通过] ConfigManager类功能正常

...

========================================
  测试结果汇总
========================================
  总测试数: 6
  通过数: 6
  失败数: 0
  通过率: 100.0%
  总耗时: 1.23秒
========================================
```

---

## 📝 更新日志

### v1.0.0 (2026-03-01)

**✨ 新功能**:
- 初始版本发布
- 实现GGIW-PHD滤波器
- 实现星凸形状跟踪器
- 实现扩展目标PHD滤波器

**🔧 重构改进**:
- 消除全局变量，使用ConfigManager统一管理
- 面向对象封装，模块化设计
- 统一命名规范，遵循MATLAB编码规范
- 完整中文注释，标准化文档
- 错误处理机制，参数验证和异常处理
- 日志系统，支持多级别日志

---

## 🙏 致谢

本项目参考了以下学术文献：

| 文献 | 内容 |
| :--- | :--- |
| Granström et al. 2010 | 扩展目标PHD滤波器 |
| Baum & Hanebeck 2010 | 星凸形状跟踪 |
| Koch 2008 | GGIW分布扩展目标跟踪 |

---

<div align="center">

**👤 作者: Ruofeng Yu**

**📅 最后更新: 2026-03-01**

![Made with](https://img.shields.io/badge/Made%20with-MATLAB-orange?style=for-the-badge&logo=mathworks)

</div>
