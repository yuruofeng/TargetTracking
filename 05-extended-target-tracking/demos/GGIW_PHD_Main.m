%% GGIW_PHD_Main.m - GGIW-PHD 扩展目标跟踪演示脚本
%
%  功能说明:
%    演示基于Gamma-Gaussian-Inverse-Wishart的扩展目标PHD滤波器
%    包含目标跟踪、扩展估计和杂波处理
%
%  依赖文件:
%    ggiw.GgiwFilter - GGIW滤波器类
%    ggiw.generateClutter - 杂波生成函数
%    ggiw.generateExtendedMeasurements - 扩展目标测量生成函数
%    utils.ConfigManager - 配置管理器
%    utils.Logger - 日志记录器
%
%  使用方法:
%    直接运行此脚本即可
%
%  作者: 重构版本
%  日期: 2026-03-01

%% ==================== 初始化配置 ====================

close all;
clc;
clear;

scriptPath = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptPath);
addpath(projectRoot);

logger = utils.Logger('GGIW_Main', 'INFO');
logger.info('开始GGIW-PHD扩展目标跟踪演示');

config = utils.ConfigManager.getInstance();

spatialDimension = utils.Constants.SPATIAL_DIMENSION;
identityMatrix = utils.Constants.getIdentityMatrix(spatialDimension);

motionOrder = 3;
samplingTime = 1;
temporalDecayWeight = 25;
temporalDecay = temporalDecayWeight / (temporalDecayWeight - 1);

config.set('measurement.sigmaX', 5);
config.set('measurement.sigmaY', 10);
config.set('measurement.measurementRate', 25);

config.set('tracking.pD', 0.99);
config.set('tracking.pS', 0.99);
config.set('tracking.maxCardinality', 100);
config.set('tracking.temporalDecay', temporalDecay);

config.set('area.xMin', -200);
config.set('area.xMax', 200);
config.set('area.yMin', -200);
config.set('area.yMax', 200);

areaBounds = [config.get('area.xMin'), config.get('area.xMax'), ...
              config.get('area.yMin'), config.get('area.yMax')];
surveillanceVolume = utils.Constants.calculateVolume(areaBounds(1), areaBounds(2), ...
    areaBounds(3), areaBounds(4));

config.set('measurement.clutterRate', 5 / surveillanceVolume);

%% ==================== 运动模型设置 ====================

maneuverTimeConstant = 1;
accelerationRms = 0.1;

stateDimension = 6;
I3 = eye(3);
O3 = zeros(3);
F1 = [1, samplingTime, 0.5 * samplingTime^2; ...
      0, 1, samplingTime; ...
      0, 0, exp(-samplingTime / maneuverTimeConstant)];
transitionMatrix = [F1, O3; O3, F1];
processNoise = accelerationRms^2 * (1 - exp(-2 * samplingTime / maneuverTimeConstant)) * ...
    diag([0, 0, 1, 0, 0, 1]);

measurementModel = [1, 0, 0, 0, 0, 0; 0, 0, 0, 1, 0, 0];
measurementNoise = diag([config.get('measurement.sigmaX'), ...
                         config.get('measurement.sigmaY')])^2;

%% ==================== 真实轨迹生成 ====================

targetVelocity = 10;
simulationTime = 25 + 1;
timeSteps = simulationTime - 1;

truePositionX = linspace(-targetVelocity * timeSteps / 2, ...
    targetVelocity * timeSteps / 2, timeSteps + 1);
trueVelocityX = targetVelocity * ones(1, timeSteps + 1);
trueAccelerationX = zeros(1, timeSteps + 1);
truePositionY = zeros(1, timeSteps + 1);
trueVelocityY = zeros(1, timeSteps + 1);
trueAccelerationY = zeros(1, timeSteps + 1);
trueState = [truePositionX; trueVelocityX; trueAccelerationX; ...
             truePositionY; trueVelocityY; trueAccelerationY];

trueExtent = diag([config.get('measurement.sigmaX'), ...
                   config.get('measurement.sigmaY')])^2;
rotationMatrix = repmat(eye(2), [1, 1, timeSteps + 1]);

trueMeasurementRate = 20;

%% ==================== 测量生成 ====================

logger.info('生成测量数据...');

measurements = ggiw.generateExtendedMeasurements(...
    [trueState(1, :); trueState(4, :)], ...
    trueMeasurementRate, ...
    config.get('tracking.pD'), ...
    trueExtent, ...
    timeSteps + 1);

clutterRate = config.get('measurement.clutterRate');
clutter = ggiw.generateClutter(clutterRate, areaBounds, timeSteps + 1);

allMeasurements = struct('points', cell(1, timeSteps + 1));
for k = 1:(timeSteps + 1)
    if ~isempty(measurements(k).points) && ~isempty(clutter(k).points)
        allMeasurements(k).points = [measurements(k).points, clutter(k).points];
    elseif ~isempty(measurements(k).points)
        allMeasurements(k).points = measurements(k).points;
    elseif ~isempty(clutter(k).points)
        allMeasurements(k).points = clutter(k).points;
    else
        allMeasurements(k).points = [];
    end
end

%% ==================== 滤波器初始化 ====================

logger.info('初始化GGIW滤波器...');

filter = ggiw.GgiwFilter(config);

config.set('tracking.birthWeight', 0.1);
config.set('tracking.useAdaptiveBirth', true);
config.set('tracking.birthMean', trueState(:, 1));
config.set('tracking.birthCov', diag([100, 100, 10, 100, 100, 10].^2));

pruningThreshold = 0.2;
mergingThreshold = 4;
maxGaussians = 10;

%% ==================== 可视化设置 ====================

figure('Name', 'GGIW-PHD 扩展目标跟踪', 'Position', [100, 100, 1200, 600]);

subplot(1, 2, 1);
hold on;
xlabel('X 位置 [m]');
ylabel('Y 位置 [m]');
title('目标跟踪结果');
axis equal;
grid on;

subplot(1, 2, 2);
hold on;
xlabel('时间步');
ylabel('估计目标数');
title('势估计');
grid on;

estimatedCardinality = zeros(1, timeSteps + 1);
estimatedPositions = cell(1, timeSteps + 1);

%% ==================== 主滤波循环 ====================

logger.info('开始滤波循环...');

for k = 1:(timeSteps + 1)
    logger.debug(sprintf('处理时间步: %d', k));
    
    filter.predict(transitionMatrix, processNoise);
    
    if ~isempty(allMeasurements(k).points)
        filter.update({allMeasurements(k)}, measurementModel, measurementNoise);
    end
    
    filter.pruneAndMerge(pruningThreshold, mergingThreshold, maxGaussians);
    
    estimates = filter.extractStates(0.4);
    estimatedCardinality(k) = estimates.numTargets;
    estimatedPositions{k} = estimates.positions;
    
    if mod(k, 5) == 1
        subplot(1, 2, 1);
        cla;
        hold on;
        
        [x, y] = ggiw.plotExtentEllipse(trueState(1, k), trueState(4, k), ...
            rotationMatrix(:, :, k) * trueExtent * rotationMatrix(:, :, k)', 2);
        fill(x, y, [0.7, 0.7, 0.7], 'FaceAlpha', 0.5, 'EdgeColor', 'k');
        
        if ~isempty(allMeasurements(k).points)
            plot(allMeasurements(k).points(1, :), ...
                allMeasurements(k).points(2, :), 'c.', 'MarkerSize', 8);
        end
        
        if estimates.numTargets > 0
            for t = 1:estimates.numTargets
                plot(estimates.positions(1, t), estimates.positions(2, t), ...
                    'ro', 'MarkerSize', 10, 'LineWidth', 2);
                
                if size(estimates.extents, 3) >= t
                    [xEst, yEst] = ggiw.plotExtentEllipse(...
                        estimates.positions(1, t), estimates.positions(2, t), ...
                        estimates.extents(:, :, t), 2);
                    plot(xEst, yEst, 'r-', 'LineWidth', 1.5);
                end
            end
        end
        
        plot(trueState(1, k), trueState(2, k), 'g*', 'MarkerSize', 15, 'LineWidth', 2);
        
        legend('真实扩展', '测量', '估计目标', '估计扩展', '真实位置', ...
            'Location', 'best');
        axis([areaBounds(1), areaBounds(2), areaBounds(3), areaBounds(4)]);
        grid on;
        
        subplot(1, 2, 2);
        plot(1:k, estimatedCardinality(1:k), 'b-', 'LineWidth', 2);
        drawnow;
    end
end

%% ==================== 结果统计 ====================

logger.info('滤波完成，生成统计结果...');

meanCardinality = mean(estimatedCardinality);
stdCardinality = std(estimatedCardinality);

logger.info(sprintf('平均估计目标数: %.2f ± %.2f', meanCardinality, stdCardinality));

figure('Name', '性能统计', 'Position', [150, 150, 800, 400]);

subplot(1, 2, 1);
histogram(estimatedCardinality, 'BinWidth', 0.5);
xlabel('估计目标数');
ylabel('频次');
title('势分布直方图');
grid on;

subplot(1, 2, 2);
plot(1:(timeSteps + 1), estimatedCardinality, 'b-', 'LineWidth', 1.5);
hold on;
yline(1, 'r--', '真实目标数', 'LineWidth', 1.5);
xlabel('时间步');
ylabel('估计目标数');
title('势估计随时间变化');
legend('估计值', '真实值');
grid on;

logger.info('演示完成');
