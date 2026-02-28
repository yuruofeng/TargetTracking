% PROJECTCONFIG  项目配置信息
%   定义项目的基本配置信息，包括模块结构、版本等。
%
%   项目结构：
%       TargetTracking/
%       ├── +dbt/           # 检测前跟踪模块 (Detect-Before-Track)
%       │   ├── 滤波器:     BaseFilter, KalmanFilterBase, EKF, UKF, CKF
%       │   ├── 运动模型:   MotionModelEKF, MotionModelConfig
%       │   ├── 高级滤波:   ParticleFilter, IMM
%       │   ├── 配置类:     Config, ConfigIMM, ConfigManeuver
%       │   ├── 场景生成:   Scenario, ScenarioManeuver
%       │   └── 工厂类:     FilterFactory
%       ├── +tbd/           # 检测前跟踪模块 (Track-Before-Detect)
%       │   ├── 算法实现:   BaseTbd, DpTbd, PfTbd
%       │   ├── 配置类:     Config
%       │   ├── 场景生成:   Scenario
%       │   └── 工厂类:     TbdFactory
%       ├── +utils/         # 工具函数模块
%       ├── +viz/           # 可视化模块
%       ├── demos/          # 演示脚本
%       ├── tests/          # 单元测试
%       ├── docs/           # 文档目录
%       ├── resources/      # 资源文件目录
%       └── config/         # 配置文件目录
%
%   使用方法：
%       config = ProjectConfig();
%       config.displayInfo();

classdef ProjectConfig
    properties (Constant)
        PROJECT_NAME = 'TargetTracking'
        PROJECT_VERSION = '2.0.0'
        PROJECT_AUTHOR = 'Target Tracking Team'
        
        MODULES = struct(...
            'DBT', 'dbt', ...
            'TBD', 'tbd', ...
            'Utils', 'utils', ...
            'Viz', 'viz' ...
        )
        
        DBT_FILTERS = {'BaseFilter', 'KalmanFilterBase', 'EKF', 'UKF', 'CKF', ...
                       'MotionModelEKF', 'ParticleFilter', 'IMM'}
        DBT_CONFIGS = {'Config', 'ConfigIMM', 'ConfigManeuver', 'MotionModelConfig'}
        DBT_SCENARIOS = {'Scenario', 'ScenarioManeuver'}
        
        TBD_ALGORITHMS = {'BaseTbd', 'DpTbd', 'PfTbd'}
        TBD_CONFIGS = {'Config'}
        TBD_SCENARIOS = {'Scenario'}
    end
    
    methods (Static)
        function displayInfo()
            fprintf('\n');
            fprintf('====================================================\n');
            fprintf('  %s v%s\n', ProjectConfig.PROJECT_NAME, ProjectConfig.PROJECT_VERSION);
            fprintf('  Author: %s\n', ProjectConfig.PROJECT_AUTHOR);
            fprintf('====================================================\n\n');
            
            fprintf('模块结构:\n');
            fprintf('  +dbt/  - 检测前跟踪 (Detect-Before-Track)\n');
            fprintf('    滤波器: EKF, UKF, CKF, ParticleFilter, IMM\n');
            fprintf('    配置:   Config, ConfigIMM, MotionModelConfig\n');
            fprintf('    场景:   Scenario, ScenarioManeuver\n');
            fprintf('\n');
            fprintf('  +tbd/  - 检测前跟踪 (Track-Before-Detect)\n');
            fprintf('    算法:   DpTbd, PfTbd\n');
            fprintf('    配置:   Config\n');
            fprintf('    场景:   Scenario\n');
            fprintf('\n');
            fprintf('  +utils/ - 工具函数\n');
            fprintf('  +viz/   - 可视化\n');
            fprintf('\n');
            fprintf('快速开始:\n');
            fprintf('  run(''demos/demoDbt.m'')  %% DBT滤波器演示\n');
            fprintf('  run(''demos/demoTbd.m'')  %% TBD算法演示\n');
            fprintf('  run(''tests/runAllTests.m'')  %% 运行测试\n');
            fprintf('====================================================\n\n');
        end
        
        function listFilters()
            fprintf('\n可用滤波器:\n');
            fprintf('  DBT模块:\n');
            for i = 1:length(ProjectConfig.DBT_FILTERS)
                fprintf('    - dbt.%s\n', ProjectConfig.DBT_FILTERS{i});
            end
            fprintf('  TBD模块:\n');
            for i = 1:length(ProjectConfig.TBD_ALGORITHMS)
                fprintf('    - tbd.%s\n', ProjectConfig.TBD_ALGORITHMS{i});
            end
        end
        
        function root = getProjectRoot()
            root = fileparts(mfilename('fullpath'));
            if isempty(root)
                root = pwd;
            end
        end
        
        function setupPath()
            root = ProjectConfig.getProjectRoot();
            addpath(genpath(root));
            fprintf('项目路径已配置: %s\n', root);
        end
    end
end
