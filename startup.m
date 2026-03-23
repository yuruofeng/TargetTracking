function startup()
% STARTUP  初始化SingleTargetTracking项目环境。
%   此脚本将项目路径添加到MATLAB搜索路径，
%   以便正确加载和调用各模块函数。
%
%   使用方法：
%       cd('single-target-tracking');
%       startup();
%
%   初始化后可使用：
%       dbt.EKF, dbt.UKF, dbt.CKF, dbt.IMM, dbt.ParticleFilter
%       phd.ImmPhdFilter, phd.SimmPhdFilter, phd.GaussMixture
%       tbd.DpTbd, tbd.PfTbd
%       utils.FilterUtils, utils.MeasurementModel, utils.Hungarian, utils.OspaMetric
%       viz.Visualizer

    scriptPath = fileparts(mfilename('fullpath'));
    if isempty(scriptPath)
        scriptPath = pwd;
    end
    
    addpath(genpath(scriptPath));
    
    fprintf('\n');
    fprintf('╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║          SingleTargetTracking v2.1.0 - 单目标跟踪工具箱          ║\n');
    fprintf('╠════════════════════════════════════════════════════════════╣\n');
    fprintf('║  模块:                                                      ║\n');
    fprintf('║    +dbt/   检测后跟踪 (EKF, UKF, CKF, IMM, ParticleFilter)   ║\n');
    fprintf('║    +phd/   PHD滤波器 (IMM-PHD, SIMM-PHD, GaussMixture)       ║\n');
    fprintf('║    +tbd/   检测前跟踪 (DP-TBD, PF-TBD)                       ║\n');
    fprintf('║    +utils/ 工具函数 (Hungarian, OspaMetric)                  ║\n');
    fprintf('║    +viz/   可视化                                           ║\n');
    fprintf('╠════════════════════════════════════════════════════════════╣\n');
    fprintf('║  快速开始:                                                  ║\n');
    fprintf('║    run(''demos/demoDbt.m'')      %% DBT滤波器演示            ║\n');
    fprintf('║    run(''demos/demoPhd.m'')      %% PHD滤波器演示            ║\n');
    fprintf('║    run(''demos/demoTbd.m'')      %% TBD算法演示             ║\n');
    fprintf('║    run(''tests/runAllTests.m'')  %% 运行测试套件             ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n');
    fprintf('\n');
    fprintf('项目路径已配置: %s\n\n', scriptPath);
end
