%% generateMeasurements.m - 星凸目标测量生成函数
%
%  功能说明:
%    在星凸目标形状范围内生成测量点
%    支持L形等复杂形状的测量生成
%
%  依赖文件:
%    utils.Logger - 日志记录器
%
%  使用方法:
%    meas = generateMeasurements([340;50;200;50], diag([0.1,0.1]), ...
%        eye(2), [1 0 0 0; 0 1 0 0], [0;0;10;10], 20);
%
%  作者: 重构版本
%  日期: 2026-03-01

function measurements = generateMeasurements(sizeObject, measurementNoiseCov, ...
    rotationMatrix, measurementModel, kinematicState, numMeasurements)
    % GENERATEMEASUREMENTS 生成星凸目标测量
    %   measurements = generateMeasurements(sizeObject, measurementNoiseCov, ...
    %       rotationMatrix, measurementModel, kinematicState, numMeasurements)
    %
    %   输入参数:
    %       sizeObject - 目标尺寸 [a; b; c; d] (double, 4 x 1)
    %       measurementNoiseCov - 测量噪声协方差 (double, 2 x 2)
    %       rotationMatrix - 旋转矩阵 (double, 2 x 2)
    %       measurementModel - 测量模型矩阵 (double, 2 x 4)
    %       kinematicState - 运动状态 (double, 4 x 1)
    %       numMeasurements - 测量数量 (double, 标量)
    %
    %   输出参数:
    %       measurements - 测量点集 (double, 2 x numMeasurements)
    %
    %   示例:
    %       meas = generateMeasurements([340;50;200;50], diag([0.1,0.1]), ...
    %           eye(2), [1 0 0 0; 0 1 0 0], [0;0;10;10], 20);
    
    arguments
        sizeObject (4,1) double
        measurementNoiseCov (2,2) double
        rotationMatrix (2,2) double
        measurementModel (:,:) double
        kinematicState (:,1) double
        numMeasurements (1,1) double {mustBePositive}
    end
    
    logger = utils.Logger('GenMeasurements', 'DEBUG');
    logger.logFunctionStart('generateMeasurements');
    
    a = sizeObject(1);
    b = sizeObject(2);
    c = sizeObject(3);
    d = sizeObject(4);
    
    measurements = zeros(2, numMeasurements);
    
    for m = 1:numMeasurements
        isValidMeasurement = false;
        
        while ~isValidMeasurement
            x = -a/2 + a * rand(1, 1);
            y = -c/2 + c * rand(1, 1);
            
            isOutsideLShape = (y > b/2 && x < -d/2) || ...
                             (y > b/2 && x > d/2) || ...
                             (y < -b/2 && x < -d/2) || ...
                             (y < -b/2 && x > d/2);
            
            if ~isOutsideLShape
                isValidMeasurement = true;
            end
        end
        
        noise = mvnrnd([0, 0], measurementNoiseCov, 1)';
        
        measurements(:, m) = measurementModel * kinematicState + ...
                            rotationMatrix * [x; y] + noise;
    end
    
    logger.logFunctionEnd('generateMeasurements', 0);
end
