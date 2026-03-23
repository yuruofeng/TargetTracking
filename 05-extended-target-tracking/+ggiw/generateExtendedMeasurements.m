%% generateExtendedMeasurements.m - 扩展目标测量生成函数
%
%  功能说明:
%    生成扩展目标的测量点，考虑目标扩展形状和检测概率
%    测量点在扩展矩阵定义的椭圆区域内均匀分布
%
%  依赖文件:
%    utils.Logger - 日志记录器
%
%  使用方法:
%    measurements = generateExtendedMeasurements([0;0;5;5], 20, 0.99, diag([100, 50]), 100);
%
%  作者: 重构版本
%  日期: 2026-03-01

function [measurements] = generateExtendedMeasurements(trueState, measurementRate, ...
    detectionProb, extentMatrix, numTimeSteps)
    % GENERATEEXTENDEDMEASUREMENTS 生成扩展目标测量
    %   measurements = generateExtendedMeasurements(trueState, measurementRate, ...
    %       detectionProb, extentMatrix, numTimeSteps)
    %
    %   输入参数:
    %       trueState - 真实目标状态 [x; y; vx; vy; ...] (double, 向量或矩阵)
    %                   如果是矩阵，每列代表一个时间步的状态
    %       measurementRate - 测量率 (double, 标量)
    %       detectionProb - 检测概率 (double, 标量, 0-1)
    %       extentMatrix - 扩展矩阵 (double, 2x2 matrix)
    %       numTimeSteps - 时间步数 (double, 标量)
    %
    %   输出参数:
    %       measurements - 测量结构体数组 (struct array)
    %           .points - 每个时间步的测量点坐标 (2 x N matrix)
    %
    %   异常:
    %       如果参数无效，抛出 MException
    %
    %   示例:
    %       measurements = generateExtendedMeasurements([0;0;5;5], 20, 0.99, diag([100, 50]), 100);
    
    arguments
        trueState (:,:) double
        measurementRate (1,1) double {mustBePositive}
        detectionProb (1,1) double {mustBeInRange(detectionProb, 0, 1)}
        extentMatrix (2,2) double
        numTimeSteps (1,1) double {mustBePositive}
    end
    
    logger = utils.Logger('GenerateExtMeas', 'INFO');
    logger.logFunctionStart('generateExtendedMeasurements');
    
    if size(trueState, 2) == 1
        positionX = trueState(1);
        positionY = trueState(2);
        positions = repmat([positionX; positionY], 1, numTimeSteps);
    else
        positions = trueState(1:2, :);
        if size(positions, 2) < numTimeSteps
            positions = [positions, repmat(positions(:, end), 1, numTimeSteps - size(positions, 2))];
        end
    end
    
    measurements = struct('points', cell(1, numTimeSteps));
    
    extentSqrt = sqrtm(extentMatrix);
    if any(isnan(extentSqrt(:))) || any(isinf(extentSqrt(:)))
        error('GenerateExtMeas:InvalidExtent', ...
            '扩展矩阵不是正定矩阵');
    end
    
    for k = 1:numTimeSteps
        if rand() > detectionProb
            measurements(k).points = [];
            continue;
        end
        
        numMeasurements = poissrnd(measurementRate);
        
        if numMeasurements > 0
            randomPoints = randn(2, numMeasurements);
            unitPoints = randomPoints ./ vecnorm(randomPoints, 2, 1);
            radii = sqrt(rand(1, numMeasurements));
            scaledPoints = unitPoints .* radii;
            
            extentPoints = extentSqrt * scaledPoints;
            
            measurements(k).points = extentPoints + positions(:, k);
        else
            measurements(k).points = [];
        end
    end
    
    logger.logFunctionEnd('generateExtendedMeasurements', 0);
end
