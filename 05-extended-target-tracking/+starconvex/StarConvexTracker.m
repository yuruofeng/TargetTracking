classdef StarConvexTracker < handle
    % STARCONVEXTRACKER 星凸目标跟踪器类
    %   使用傅里叶级数描述符和UKF/CKF进行星凸形状目标跟踪
    %
    %   属性说明:
    %       stateVector - 状态向量（运动状态+形状参数）
    %       covarianceMatrix - 协方差矩阵
    %       numFourierCoeff - 傅里叶系数数量
    %       logger - 日志记录器
    %
    %   使用示例:
    %       tracker = starconvex.StarConvexTracker(numFourierCoeff);
    %       tracker.initialize(initialState, initialCovariance);
    %       tracker.predict(transitionMatrix, processNoise);
    %       tracker.update(measurements);
    %       shape = tracker.getShape(phiVector);
    %
    %   参见: utils.ConfigManager, utils.Logger
    
    properties (Access = private)
        logger
        config
        filterType
    end
    
    properties (Access = public)
        stateVector
        covarianceMatrix
        numFourierCoeff
        scale
        kinematicDim
    end
    
    properties (Constant)
        UKF_ALPHA = 1
        UKF_BETA = 0
        UKF_KAPPA = 0
    end
    
    methods
        function obj = StarConvexTracker(numFourierCoeff, filterType, config)
            % STARCONVEXTRACKER 构造函数
            %   tracker = starconvex.StarConvexTracker(numFourierCoeff)
            %   tracker = starconvex.StarConvexTracker(numFourierCoeff, filterType)
            %   tracker = starconvex.StarConvexTracker(numFourierCoeff, filterType, config)
            %
            %   输入参数:
            %       numFourierCoeff - 傅里叶系数数量 (double, 标量)
            %       filterType - 滤波器类型: 'UKF' 或 'CKF' (char, 可选, 默认 'UKF')
            %       config - 配置管理器 (utils.ConfigManager, 可选)
            %
            %   输出参数:
            %       obj - StarConvexTracker实例
            
            arguments
                numFourierCoeff (1,1) double {mustBePositive}
                filterType (1,1) string = 'UKF'
                config = []
            end
            
            if isempty(config)
                obj.config = utils.ConfigManager.getInstance();
            else
                obj.config = config;
            end
            
            obj.logger = utils.Logger('StarConvexTracker', 'INFO');
            obj.numFourierCoeff = numFourierCoeff;
            obj.filterType = char(filterType);
            obj.kinematicDim = 4;
            
            obj.scale = struct();
            obj.scale.mean = 0.7;
            obj.scale.variance = 0.08;
            
            obj.logger.info(sprintf('创建星凸目标跟踪器，滤波器类型: %s, 傅里叶系数数: %d', ...
                obj.filterType, obj.numFourierCoeff));
        end
        
        function initialize(obj, initialState, initialCovariance)
            % INITIALIZE 初始化跟踪器
            %   obj.initialize(initialState, initialCovariance)
            %
            %   输入参数:
            %       initialState - 初始状态向量 (double, 向量)
            %       initialCovariance - 初始协方差矩阵 (double, 矩阵)
            
            obj.logger.logFunctionStart('initialize');
            
            expectedDim = obj.kinematicDim + obj.numFourierCoeff;
            
            if length(initialState) ~= expectedDim
                error('StarConvexTracker:DimensionMismatch', ...
                    '状态向量维度不匹配。期望: %d, 实际: %d', ...
                    expectedDim, length(initialState));
            end
            
            obj.stateVector = initialState(:);
            obj.covarianceMatrix = initialCovariance;
            
            obj.logger.debug(sprintf('初始化完成，状态维度: %d', length(obj.stateVector)));
            obj.logger.logFunctionEnd('initialize', 0);
        end
        
        function predict(obj, transitionMatrix, processNoise)
            % PREDICT 预测步骤
            %   obj.predict(transitionMatrix, processNoise)
            %
            %   输入参数:
            %       transitionMatrix - 状态转移矩阵 (double, 矩阵)
            %       processNoise - 过程噪声协方差 (double, 矩阵)
            
            obj.logger.logFunctionStart('predict');
            
            stateDim = length(obj.stateVector);
            
            if size(transitionMatrix, 1) == stateDim
                fullTransition = transitionMatrix;
            elseif size(transitionMatrix, 1) == obj.kinematicDim
                fullTransition = eye(stateDim);
                fullTransition(1:obj.kinematicDim, 1:obj.kinematicDim) = transitionMatrix;
            else
                error('StarConvexTracker:TransitionDimensionMismatch', ...
                    '转移矩阵维度(%d)不匹配，期望 %d (全状态) 或 %d (运动状态)', ...
                    size(transitionMatrix, 1), stateDim, obj.kinematicDim);
            end
            
            if size(processNoise, 1) == stateDim
                fullProcessNoise = processNoise;
            elseif size(processNoise, 1) == obj.kinematicDim
                fullProcessNoise = zeros(stateDim, stateDim);
                fullProcessNoise(1:obj.kinematicDim, 1:obj.kinematicDim) = processNoise;
            else
                shapeDim = stateDim - obj.kinematicDim;
                fullProcessNoise = zeros(stateDim, stateDim);
                fullProcessNoise(1:size(processNoise, 1), 1:size(processNoise, 2)) = processNoise;
            end
            
            obj.stateVector = fullTransition * obj.stateVector;
            obj.covarianceMatrix = utils.ArrayUtils.makeSymmetric(...
                fullTransition * obj.covarianceMatrix * fullTransition' + fullProcessNoise);
            
            obj.logger.logFunctionEnd('predict', 0);
        end
        
        function update(obj, measurement, measurementNoiseMean, measurementNoiseCov)
            % UPDATE 更新步骤
            %   obj.update(measurement, measurementNoiseMean, measurementNoiseCov)
            %
            %   输入参数:
            %       measurement - 测量值 (double, 2x1向量)
            %       measurementNoiseMean - 测量噪声均值 (double, 向量)
            %       measurementNoiseCov - 测量噪声协方差 (double, 矩阵)
            
            obj.logger.logFunctionStart('update');
            
            extendedState = [obj.stateVector; measurementNoiseMean];
            extendedCov = blkdiag(obj.covarianceMatrix, measurementNoiseCov);
            
            stateDim = length(obj.stateVector);
            
            switch obj.filterType
                case 'UKF'
                    [obj.stateVector, obj.covarianceMatrix] = ...
                        obj.ukfUpdate(extendedState, extendedCov, ...
                        measurement, stateDim);
                case 'CKF'
                    [obj.stateVector, obj.covarianceMatrix] = ...
                        obj.ckfUpdate(extendedState, extendedCov, ...
                        measurement, stateDim);
                otherwise
                    error('StarConvexTracker:InvalidFilterType', ...
                        '不支持的滤波器类型: %s', obj.filterType);
            end
            
            obj.logger.logFunctionEnd('update', 0);
        end
        
        function shape = getShape(obj, phiVector)
            % GETSHAPE 获取形状轮廓
            %   shape = obj.getShape(phiVector)
            %
            %   输入参数:
            %       phiVector - 角度向量 (double, 向量)
            %
            %   输出参数:
            %       shape - 形状轮廓点 [2 x N] (double, 矩阵)
            
            shape = zeros(2, length(phiVector));
            
            for i = 1:length(phiVector)
                phi = phiVector(i);
                fourierMatrix = obj.calcFourierMatrix(phi);
                direction = [cos(phi); sin(phi)];
                shape(:, i) = fourierMatrix * obj.stateVector(5:end) * direction + ...
                    obj.stateVector(1:2);
            end
        end
        
        function position = getPosition(obj)
            % GETPOSITION 获取当前位置估计
            %   position = obj.getPosition()
            %
            %   输出参数:
            %       position - 位置 [x; y] (double, 2x1向量)
            
            position = obj.stateVector(1:2);
        end
        
        function velocity = getVelocity(obj)
            % GETVELOCITY 获取当前速度估计
            %   velocity = obj.getVelocity()
            %
            %   输出参数:
            %       velocity - 速度 [vx; vy] (double, 2x1向量)
            
            velocity = obj.stateVector(3:4);
        end
    end
    
    methods (Access = private)
        function [updatedState, updatedCov] = ukfUpdate(obj, extendedState, ...
                extendedCov, measurement, stateDim)
            % UKFUPDATE UKF更新步骤
            %   [state, cov] = obj.ukfUpdate(extendedState, extendedCov, measurement, stateDim)
            %
            %   输入参数:
            %       extendedState - 扩展状态向量 (double, 向量)
            %       extendedCov - 扩展协方差矩阵 (double, 矩阵)
            %       measurement - 测量值 (double, 向量)
            %       stateDim - 状态维度 (double)
            %
            %   输出参数:
            %       updatedState - 更新后的状态向量 (double, 向量)
            %       updatedCov - 更新后的协方差矩阵 (double, 矩阵)
            %
            %   使用无迹卡尔曼滤波(UKF)进行状态更新
            %   采用UT变换生成sigma点并计算卡尔曼增益
            
            n = length(extendedState);
            lambda = obj.UKF_ALPHA^2 * (n + obj.UKF_KAPPA) - n;
            
            numSigmaPoints = 2*n + 1;
            
            weightMean = zeros(1, numSigmaPoints);
            weightMean(1) = lambda / (n + lambda);
            weightMean(2:end) = 1 / (2 * (n + lambda));
            
            weightCov = zeros(1, numSigmaPoints);
            weightCov(1) = lambda / (n + lambda) + (1 - obj.UKF_ALPHA^2 + obj.UKF_BETA);
            weightCov(2:end) = 1 / (2 * (n + lambda));
            
            try
                sqrtMatrix = sqrt(n + lambda) * chol(extendedCov)';
            catch ME
                obj.logger.warning(sprintf('Cholesky分解失败: %s，使用正则化', ME.message));
                regularizedCov = extendedCov + 1e-6 * eye(n);
                sqrtMatrix = sqrt(n + lambda) * chol(regularizedCov)';
            end
            
            sigmaPoints = [zeros(n, 1) - sqrtMatrix sqrtMatrix];
            sigmaPoints = sigmaPoints + repmat(extendedState, 1, size(sigmaPoints, 2));
            
            actualNumSigmaPoints = size(sigmaPoints, 2);
            
            predictedMeasurements = zeros(3, actualNumSigmaPoints);
            for i = 1:actualNumSigmaPoints
                statePortion = sigmaPoints(1:stateDim, i);
                noisePortion = sigmaPoints(stateDim+1:n, i);
                predictedMeasurements(:, i) = obj.measurementFunction(...
                    statePortion, noisePortion, measurement);
            end
            
            measMean = zeros(3, 1);
            for i = 1:actualNumSigmaPoints
                measMean = measMean + predictedMeasurements(:, i) * weightMean(i);
            end
            
            covYY = zeros(3, 3);
            covXY = zeros(stateDim, 3);
            for i = 1:actualNumSigmaPoints
                innovY = predictedMeasurements(:, i) - measMean;
                innovX = sigmaPoints(1:stateDim, i) - obj.stateVector;
                covYY = covYY + weightCov(i) * (innovY * innovY');
                covXY = covXY + weightCov(i) * (innovX * innovY');
            end
            
            kalmanGain = covXY / covYY;
            innovation = zeros(3, 1) - measMean;
            
            updatedState = obj.stateVector + kalmanGain * innovation;
            updatedCov = obj.covarianceMatrix - kalmanGain * covYY * kalmanGain';
            updatedCov = utils.ArrayUtils.makeSymmetric(updatedCov);
        end
        
        function [updatedState, updatedCov] = ckfUpdate(obj, extendedState, ...
                extendedCov, measurement, stateDim)
            % CKFUPDATE CKF更新步骤
            %   [state, cov] = obj.ckfUpdate(extendedState, extendedCov, measurement, stateDim)
            %
            %   输入参数:
            %       extendedState - 扩展状态向量 (double, 向量)
            %       extendedCov - 扩展协方差矩阵 (double, 矩阵)
            %       measurement - 测量值 (double, 向量)
            %       stateDim - 状态维度 (double)
            %
            %   输出参数:
            %       updatedState - 更新后的状态向量 (double, 向量)
            %       updatedCov - 更新后的协方差矩阵 (double, 矩阵)
            %
            %   使用容积卡尔曼滤波(CKF)进行状态更新
            %   采用容积点进行状态估计
            
            n = length(extendedState);
            
            try
                sqrtCov = chol(extendedCov)';
            catch ME
                obj.logger.warning(sprintf('Cholesky分解失败: %s，使用正则化', ME.message));
                regularizedCov = extendedCov + 1e-6 * eye(n);
                sqrtCov = chol(regularizedCov)';
            end
            
            weight = 1 / (2 * n);
            
            sigmaPoints = [sqrtCov -sqrtCov];
            sigmaPoints = sigmaPoints + repmat(extendedState, 1, 2*n);
            
            actualNumSigmaPoints = size(sigmaPoints, 2);
            
            predictedMeasurements = zeros(3, actualNumSigmaPoints);
            for i = 1:actualNumSigmaPoints
                statePortion = sigmaPoints(1:stateDim, i);
                noisePortion = sigmaPoints(stateDim+1:n, i);
                predictedMeasurements(:, i) = obj.measurementFunction(...
                    statePortion, noisePortion, measurement);
            end
            
            measMean = zeros(3, 1);
            for i = 1:actualNumSigmaPoints
                measMean = measMean + predictedMeasurements(:, i) * weight;
            end
            
            covYY = zeros(3, 3);
            covXY = zeros(stateDim, 3);
            for i = 1:actualNumSigmaPoints
                innovY = predictedMeasurements(:, i) - measMean;
                innovX = sigmaPoints(1:stateDim, i) - obj.stateVector;
                covYY = covYY + weight * (innovY * innovY');
                covXY = covXY + weight * (innovX * innovY');
            end
            
            kalmanGain = covXY / covYY;
            innovation = zeros(3, 1) - measMean;
            
            updatedState = obj.stateVector + kalmanGain * innovation;
            updatedCov = obj.covarianceMatrix - kalmanGain * covYY * kalmanGain';
            updatedCov = utils.ArrayUtils.makeSymmetric(updatedCov);
        end
        
        function measValue = measurementFunction(obj, state, noise, measurement)
            % MEASUREMENTFUNCTION 测量函数
            %   measValue = obj.measurementFunction(state, noise, measurement)
            %
            %   输入参数:
            %       state - 状态向量 (double, 向量)
            %       noise - 噪声向量 (double, 向量)
            %       measurement - 测量值 (double, 向量)
            %
            %   输出参数:
            %       measValue - 测量预测值 (double, 3x1向量)
            %
            %   计算测量预测值，包含位置残差和半径残差
            
            position = state(1:2);
            shapeParams = state(5:end);
            
            if length(measurement) >= 2
                phi = atan2(measurement(2) - position(2), measurement(1) - position(1));
            else
                phi = 0;
            end
            
            fourierMatrix = obj.calcFourierMatrix(phi);
            direction = [cos(phi); sin(phi)];
            radius = fourierMatrix * shapeParams;
            
            predictedPoint = position + radius * direction + noise(1:2);
            
            measValue = [predictedPoint - measurement; radius - obj.scale.mean];
        end
        
        function fourierMatrix = calcFourierMatrix(obj, phi)
            % CALCFURIERMATRIX 计算傅里叶系数矩阵
            %   fourierMatrix = obj.calcFourierMatrix(phi)
            %
            %   输入参数:
            %       phi - 角度值 (double, 标量)
            %
            %   输出参数:
            %       fourierMatrix - 傅里叶系数矩阵 (double, 1 x numFourierCoeff)
            %
            %   根据给定角度计算傅里叶级数系数
            %   第1项为常数1，偶数项为cos，奇数项为sin
            
            fourierMatrix = zeros(1, obj.numFourierCoeff);
            
            for k = 1:obj.numFourierCoeff
                if k == 1
                    fourierMatrix(k) = 1;
                elseif mod(k, 2) == 0
                    fourierMatrix(k) = cos((k/2) * phi);
                else
                    fourierMatrix(k) = sin(((k-1)/2) * phi);
                end
            end
        end
    end
end
