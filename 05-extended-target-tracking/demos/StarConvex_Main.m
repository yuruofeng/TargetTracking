%% StarConvex_Main.m - 星凸目标跟踪演示脚本
%
%  功能说明:
%    演示基于傅里叶级数描述符的星凸形状扩展目标跟踪
%    使用UKF/CKF滤波器进行状态估计和形状估计
%
%  依赖文件:
%    starconvex.StarConvexTracker - 星凸目标跟踪器类
%    starconvex.generateGroundTruth - 真实数据生成函数
%    starconvex.generateMeasurements - 测量生成函数
%    utils.ConfigManager - 配置管理器
%    utils.Logger - 日志记录器
%
%  使用方法:
%    直接运行此脚本即可
%
%  作者: 重构版本
%  日期: 2026-03-01

%% ==================== 初始化 ====================

close all;
clc;
clear;

scriptPath = fileparts(mfilename('fullpath'));
projectRoot = fileparts(scriptPath);
addpath(projectRoot);

logger = utils.Logger('StarConvex_Main', 'INFO');
logger.info('开始星凸目标跟踪演示');

config = utils.ConfigManager.getInstance();

%% ==================== 参数配置 ====================

motionModel = 'NCV';
poissonLambda = 10;

measurementModel = [1, 0, 0, 0; 0, 1, 0, 0];
velocityModel = [zeros(2, 2), eye(2)];
measurementNoiseCov = diag([0.1, 0.1]);

config.set('starconvex.numFourierCoeff', 11);
config.set('starconvex.scaleMean', 0.7);
config.set('starconvex.scaleVariance', 0.08);

numFourierCoeff = config.get('starconvex.numFourierCoeff', 11);

%% ==================== 真实数据生成 ====================

logger.info('生成真实轨迹和形状数据...');

[groundTruthKinematic, groundTruthParams, objectBounds, sizeObject, ...
    timeSteps, deltaTime] = starconvex.generateGroundTruth();

logger.info(sprintf('时间步数: %d, 时间间隔: %.1f', timeSteps, deltaTime));

%% ==================== 跟踪器初始化 ====================

shapeParams = zeros(numFourierCoeff, 1);
shapeParams(1) = 100;
shapeParamsCov = diag(ones(1, numFourierCoeff) * 0.02);

initialPosition = [100; 100];
initialVelocity = [5; -8];
initialPosCov = 900 * eye(2);
initialVelCov = 16 * eye(2);

initialState = [initialPosition; initialVelocity; shapeParams];
initialCov = blkdiag(initialPosCov, initialVelCov, shapeParamsCov);

scale = struct();
scale.mean = config.get('starconvex.scaleMean', 0.7);
scale.variance = config.get('starconvex.scaleVariance', 0.08);

phiVector = linspace(0, 2*pi, 200);

transitionKinematic = [eye(2), deltaTime * eye(2); zeros(2, 2), eye(2)];
transitionShape = eye(numFourierCoeff);
transitionMatrix = blkdiag(transitionKinematic, transitionShape);

processNoiseKinematic = blkdiag(100 * eye(2), eye(2));
processNoiseShape = 0.5 * eye(numFourierCoeff);
processNoise = blkdiag(processNoiseKinematic, processNoiseShape);

logger.info('创建星凸目标跟踪器...');
tracker = starconvex.StarConvexTracker(numFourierCoeff, 'UKF', config);
tracker.initialize(initialState, initialCov);

%% ==================== 可视化设置 ====================

figure('Name', '星凸目标跟踪', 'Position', [100, 100, 1200, 500]);

subplot(1, 2, 1);
hold on;
xlabel('X 位置 [m]');
ylabel('Y 位置 [m]');
title('星凸目标跟踪结果');
axis equal;
grid on;

subplot(1, 2, 2);
hold on;
xlabel('时间步');
ylabel('位置误差 [m]');
title('跟踪误差');
grid on;

positionErrors = zeros(1, timeSteps);
shapeErrors = zeros(1, timeSteps);

%% ==================== 主跟踪循环 ====================

logger.info('开始跟踪循环...');

for t = 1:timeSteps
    numMeasurements = poissrnd(poissonLambda);
    
    while numMeasurements == 0
        numMeasurements = poissrnd(poissonLambda);
    end
    
    if mod(t, 10) == 0
        logger.debug(sprintf('时间步: %d, 测量数: %d', t, numMeasurements));
    end
    
    currentParams = groundTruthParams(:, t);
    currentKinematic = groundTruthKinematic(:, t);
    currentVelocity = velocityModel * currentKinematic;
    currentRotation = [cos(currentParams(3)), -sin(currentParams(3)); ...
                       sin(currentParams(3)), cos(currentParams(3))];
    
    measurements = starconvex.generateMeasurements(sizeObject, ...
        measurementNoiseCov, currentRotation, measurementModel, ...
        currentKinematic, numMeasurements);
    
    for m = 1:numMeasurements
        tracker.update(measurements(:, m), ...
            [scale.mean; zeros(2, 1)], ...
            blkdiag(scale.variance, measurementNoiseCov));
    end
    
    estimatedPosition = tracker.getPosition();
    truePosition = currentKinematic(1:2);
    positionErrors(t) = norm(estimatedPosition - truePosition);
    
    if mod(t, 3) == 1
        subplot(1, 2, 1);
        cla;
        hold on;
        
        rotatedBounds = currentRotation * objectBounds;
        fill((rotatedBounds(1, :) + currentParams(1)) / 1000, ...
             (rotatedBounds(2, :) + currentParams(2)) / 1000, ...
             [0.7, 0.7, 0.7], 'FaceAlpha', 0.5, 'EdgeColor', 'k', 'LineWidth', 1.5);
        
        plot(measurements(1, :) / 1000, measurements(2, :) / 1000, ...
            '.k', 'MarkerSize', 6, 'Color', [0.3, 0.3, 0.3]);
        
        estimatedShape = tracker.getShape(phiVector);
        plot(estimatedShape(1, :) / 1000, estimatedShape(2, :) / 1000, ...
            'g-', 'LineWidth', 2);
        
        plot(estimatedPosition(1) / 1000, estimatedPosition(2) / 1000, ...
            'r+', 'MarkerSize', 12, 'LineWidth', 2);
        
        plot(truePosition(1) / 1000, truePosition(2) / 1000, ...
            'b*', 'MarkerSize', 12, 'LineWidth', 2);
        
        legend('真实形状', '测量点', '估计形状', '估计位置', '真实位置', ...
            'Location', 'best');
        axis equal;
        grid on;
        ylim([-3.2, 1.2]);
        xlim([-0.2, 8]);
        
        subplot(1, 2, 2);
        plot(1:t, positionErrors(1:t) / 1000, 'b-', 'LineWidth', 1.5);
        drawnow;
    end
    
    tracker.predict(transitionMatrix, processNoise);
end

%% ==================== 结果统计 ====================

logger.info('跟踪完成，生成统计结果...');

meanPositionError = mean(positionErrors);
stdPositionError = std(positionErrors);
maxPositionError = max(positionErrors);

logger.info(sprintf('位置误差统计:'));
logger.info(sprintf('  平均: %.2f m', meanPositionError));
logger.info(sprintf('  标准差: %.2f m', stdPositionError));
logger.info(sprintf('  最大: %.2f m', maxPositionError));

figure('Name', '性能统计', 'Position', [150, 150, 800, 400]);

subplot(1, 2, 1);
histogram(positionErrors / 1000, 20);
xlabel('位置误差 [km]');
ylabel('频次');
title('位置误差分布');
grid on;

subplot(1, 2, 2);
plot(1:timeSteps, positionErrors / 1000, 'b-', 'LineWidth', 1.5);
xlabel('时间步');
ylabel('位置误差 [km]');
title('位置误差随时间变化');
grid on;

logger.info('演示完成');
