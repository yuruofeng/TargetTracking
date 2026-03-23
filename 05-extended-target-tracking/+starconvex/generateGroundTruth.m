%% generateGroundTruth.m - 真实轨迹和形状数据生成函数
%
%  功能说明:
%    生成星凸目标的真实运动轨迹和形状参数
%    包含L形目标的运动状态、方向和尺寸参数
%
%  依赖文件:
%    utils.Logger - 日志记录器
%
%  使用方法:
%    [kin, par, bounds, size, steps, dt] = generateGroundTruth();
%
%  作者: 重构版本
%  日期: 2026-03-01

function [groundTruthKinematic, groundTruthParams, objectBounds, sizeObject, ...
    timeSteps, deltaTime] = generateGroundTruth()
    % GENERATEGROUNDTRUTH 生成真实轨迹和形状数据
    %   [kinematic, params, bounds, sizeObj, steps, dt] = generateGroundTruth()
    %
    %   输出参数:
    %       groundTruthKinematic - 运动状态 [x; y; vx; vy] (double, 4 x timeSteps)
    %       groundTruthParams - 形状参数 [x; y; orientation; length1; length2] (double, 5 x timeSteps)
    %       objectBounds - 形状边界点 (double, 2 x 12)
    %       sizeObject - 目标尺寸 [a; b; c; d] (double, 4 x 1)
    %       timeSteps - 时间步数 (double, 标量)
    %       deltaTime - 时间间隔 (double, 标量)
    %
    %   示例:
    %       [kin, par, bounds, size, steps, dt] = generateGroundTruth();
    
    logger = utils.Logger('GenGroundTruth', 'DEBUG');
    logger.logFunctionStart('generateGroundTruth');
    
    orientation = [repmat(-pi/4, 1, 25), ...
                  (-pi/4 : pi/40 : 0), ...
                  zeros(1, 20), ...
                  (0 : pi/20 : 2*pi/4), ...
                  repmat(2*pi/4, 1, 15), ...
                  (2*pi/4 : pi/20 : pi), ...
                  repmat(pi, 1, 40)];
    
    velocity = [(500/36) * cos(orientation); ...
                (500/36) * sin(orientation)];
    
    lengthParams = repmat([340/2; 80/2], 1, size(velocity, 2));
    
    timeSteps = size(velocity, 2);
    deltaTime = 10;
    
    a = 340;
    b = 50;
    c = 200;
    d = 50;
    
    sizeObject = [a; b; c; d];
    
    objectBounds = [[-d, -c]; [d, -c]; [d, -b]; [a, -b]; [a, b]; [d, b]; ...
                    [d, c]; [-d, c]; [-d, b]; [-a, b]; [-a, -b]; [-d, -b]]' ./ 2;
    
    groundTruthRotation = zeros(2, 2, timeSteps);
    groundTruthCenter = zeros(2, timeSteps);
    
    for t = 1:timeSteps
        groundTruthRotation(:, :, t) = [cos(orientation(t)), -sin(orientation(t)); ...
                                        sin(orientation(t)), cos(orientation(t))];
        if t > 1
            groundTruthCenter(:, t) = groundTruthCenter(:, t-1) + velocity(:, t) * deltaTime;
        end
    end
    
    groundTruthKinematic = [groundTruthCenter; velocity];
    groundTruthParams = [groundTruthCenter; orientation; lengthParams];
    
    logger.debug(sprintf('生成 %d 时间步的真实数据', timeSteps));
    logger.logFunctionEnd('generateGroundTruth', 0);
end
