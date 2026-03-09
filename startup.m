function startup()
% STARTUP  初始化TargetTracking项目环境。
%   将项目路径添加到MATLAB搜索路径，并显示项目信息。
%
%   使用方法：
%       cd('TargetTracking');
%       startup();  % 或 run('startup.m')
%
%   初始化后可使用：
%       dbt.EKF, dbt.UKF, dbt.CKF, dbt.IMM, dbt.ParticleFilter
%       tbd.DpTbd, tbd.PfTbd
%       utils.FilterUtils, utils.MeasurementModel
%       viz.Visualizer

    scriptPath = fileparts(mfilename('fullpath'));
    if isempty(scriptPath)
        scriptPath = pwd;
    end
    
    addpath(genpath(scriptPath));
    
    fprintf('\n');
    fprintf('╔════════════════════════════════════════════════════════════╗\n');
    fprintf('║          TargetTracking v2.0.0 - 目标跟踪工具箱              ║\n');
    fprintf('╠════════════════════════════════════════════════════════════╣\n');
    fprintf('║  模块:                                                      ║\n');
    fprintf('║    +dbt/   检测前跟踪 (EKF, UKF, CKF, IMM, ParticleFilter)   ║\n');
    fprintf('║    +tbd/   检测前跟踪 (DP-TBD, PF-TBD)                       ║\n');
    fprintf('║    +utils/ 工具函数                                         ║\n');
    fprintf('║    +viz/   可视化                                           ║\n');
    fprintf('╠════════════════════════════════════════════════════════════╣\n');
    fprintf('║  快速开始:                                                  ║\n');
    fprintf('║    run(''demos/demoDbt.m'')      %% DBT滤波器演示            ║\n');
    fprintf('║    run(''demos/demoTbd.m'')      %% TBD算法演示             ║\n');
    fprintf('║    run(''tests/runAllTests.m'')  %% 运行测试套件             ║\n');
    fprintf('╚════════════════════════════════════════════════════════════╝\n');
    fprintf('\n');
    fprintf('项目路径已配置: %s\n\n', scriptPath);
end
