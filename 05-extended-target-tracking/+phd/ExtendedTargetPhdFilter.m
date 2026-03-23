classdef ExtendedTargetPhdFilter < handle
    % EXTENDEDTARGETPHDFILTER 扩展目标PHD滤波器类
    %   实现基于Granstrom 2010的扩展目标PHD滤波器
    %
    %   属性说明:
    %       config - 配置管理器
    %       logger - 日志记录器
    %       components - 高斯分量集合
    %
    %   使用示例:
    %       filter = phd.ExtendedTargetPhdFilter(config);
    %       filter.predict(motionModel, processNoise);
    %       filter.update(measurements, measurementModel, measurementNoise);
    %       estimates = filter.extractStates();
    %
    %   参见: utils.ConfigManager, utils.Logger
    
    properties (Access = private)
        config
        logger
    end
    
    properties (Access = public)
        weights
        means
        covariances
        numComponents
        birthModel
        spawnModel
    end
    
    methods
        function obj = ExtendedTargetPhdFilter(config)
            % EXTENDEDTARGETPHDFILTER 构造函数
            %   filter = phd.ExtendedTargetPhdFilter()
            %   filter = phd.ExtendedTargetPhdFilter(config)
            %
            %   输入参数:
            %       config - 配置管理器实例 (utils.ConfigManager, 可选)
            %
            %   输出参数:
            %       obj - ExtendedTargetPhdFilter实例
            
            if nargin < 1 || isempty(config)
                obj.config = utils.ConfigManager.getInstance();
            else
                obj.config = config;
            end
            
            obj.logger = utils.Logger('ExtTargetPHD', 'INFO');
            obj.initialize();
        end
        
        function initialize(obj)
            % INITIALIZE 初始化滤波器
            %   obj.initialize()
            
            obj.logger.info('初始化扩展目标PHD滤波器');
            
            obj.weights = [];
            obj.means = [];
            obj.covariances = [];
            obj.numComponents = 0;
            
            obj.birthModel = struct();
            obj.birthModel.weight = obj.config.get('tracking.birthWeight', 0.1);
            obj.birthModel.mean = obj.config.get('tracking.birthMean', [250; 250; 0; 0]);
            obj.birthModel.covariance = obj.config.get('tracking.birthCov', ...
                diag([900, 900, 25, 25]));
            obj.birthModel.numComponents = 1;
            
            obj.spawnModel = struct();
            obj.spawnModel.weight = obj.config.get('tracking.spawnWeight', 0.05);
            obj.spawnModel.offset = [0; 0; 0; 0];
            obj.spawnModel.transition = eye(4);
            obj.spawnModel.covariance = diag([100, 100, 400, 400]);
            obj.spawnModel.numComponents = 1;
        end
        
        function predict(obj, motionModel, processNoise)
            % PREDICT 预测步骤
            %   obj.predict(motionModel, processNoise)
            %
            %   输入参数:
            %       motionModel - 运动模型矩阵F (double matrix)
            %       processNoise - 过程噪声协方差Q (double matrix)
            
            obj.logger.logFunctionStart('predict');
            
            pS = obj.config.get('tracking.pS', 0.99);
            
            predictedWeights = [];
            predictedMeans = [];
            predictedCovs = [];
            
            birthWeights = obj.birthModel.weight * ones(1, obj.birthModel.numComponents);
            predictedWeights = [predictedWeights, birthWeights];
            
            for j = 1:obj.birthModel.numComponents
                predictedMeans = [predictedMeans, obj.birthModel.mean];
                predictedCovs = cat(3, predictedCovs, obj.birthModel.covariance);
            end
            
            for j = 1:obj.spawnModel.numComponents
                for l = 1:obj.numComponents
                    spawnWeight = obj.weights(l) * obj.spawnModel.weight;
                    spawnMean = obj.spawnModel.offset + ...
                        obj.spawnModel.transition * obj.means(:, l);
                    spawnCov = obj.spawnModel.covariance + ...
                        obj.spawnModel.transition * ...
                        obj.covariances(:, :, l) * obj.spawnModel.transition';
                    
                    predictedWeights = [predictedWeights, spawnWeight];
                    predictedMeans = [predictedMeans, spawnMean];
                    predictedCovs = cat(3, predictedCovs, spawnCov);
                end
            end
            
            for j = 1:obj.numComponents
                survivingWeight = pS * obj.weights(j);
                survivingMean = motionModel * obj.means(:, j);
                survivingCov = processNoise + ...
                    motionModel * obj.covariances(:, :, j) * motionModel';
                
                predictedWeights = [predictedWeights, survivingWeight];
                predictedMeans = [predictedMeans, survivingMean];
                predictedCovs = cat(3, predictedCovs, survivingCov);
            end
            
            obj.weights = predictedWeights;
            obj.means = predictedMeans;
            obj.covariances = predictedCovs;
            obj.numComponents = length(obj.weights);
            
            obj.logger.debug(sprintf('预测后组件数: %d', obj.numComponents));
            obj.logger.logFunctionEnd('predict', 0);
        end
        
        function update(obj, measurements, measurementModel, measurementNoise)
            % UPDATE 更新步骤
            %   obj.update(measurements, measurementModel, measurementNoise)
            %
            %   输入参数:
            %       measurements - 测量集 (cell array)
            %       measurementModel - 测量模型矩阵H (double matrix)
            %       measurementNoise - 测量噪声协方差R (double matrix)
            
            obj.logger.logFunctionStart('update');
            
            if isempty(measurements)
                obj.logger.warning('测量集为空，跳过更新');
                return;
            end
            
            pD = obj.config.get('tracking.pD', 0.99);
            measurementRate = obj.config.get('measurement.measurementRate', 25);
            
            updatedWeights = [];
            updatedMeans = [];
            updatedCovs = [];
            
            for j = 1:obj.numComponents
                missedDetectionWeight = (1 - (1 - exp(-measurementRate)) * pD) * obj.weights(j);
                updatedWeights = [updatedWeights, missedDetectionWeight];
                updatedMeans = [updatedMeans, obj.means(:, j)];
                updatedCovs = cat(3, updatedCovs, obj.covariances(:, :, j));
            end
            
            numPartitions = length(measurements);
            
            for p = 1:numPartitions
                partition = measurements{p};
                numCells = length(partition.cells);
                
                for w = 1:numCells
                    cellMeas = partition.cells{w};
                    cellSize = size(cellMeas, 2);
                    
                    H_repeated = repmat(measurementModel, cellSize, 1);
                    R_repeated = diag(repmat(diag(measurementNoise), cellSize, 1));
                    
                    for j = 1:obj.numComponents
                        innovationMean = measurementModel * obj.means(:, j);
                        innovationCov = measurementNoise + ...
                            measurementModel * obj.covariances(:, :, j) * measurementModel';
                        
                        likelihoodProduct = 1;
                        for z = 1:cellSize
                            currentMeas = cellMeas(:, z);
                            likelihood = obj.computeGaussianLikelihood(...
                                currentMeas, innovationMean, innovationCov);
                            likelihoodProduct = likelihoodProduct * likelihood;
                        end
                        
                        clutterRate = obj.config.get('measurement.clutterRate', 1e-6); %#ok<NASGU>
                        uniformDensity = 1 / obj.config.get('area.volume', 4e6); %#ok<NASGU>
                        
                        updatedWeight = exp(-measurementRate) * ...
                            (measurementRate ^ cellSize) * pD * ...
                            likelihoodProduct / (uniformDensity ^ cellSize) * ...
                            obj.weights(j);
                        
                        S = R_repeated + H_repeated * obj.covariances(:, :, j) * H_repeated';
                        K = obj.covariances(:, :, j) * H_repeated' / S;
                        
                        measVector = cellMeas(:);
                        predictedMeas = H_repeated * obj.means(:, j);
                        
                        updatedMean = obj.means(:, j) + K * (measVector - predictedMeas);
                        updatedCov = (eye(size(obj.covariances(:, :, j))) - K * H_repeated) * ...
                            obj.covariances(:, :, j);
                        
                        updatedWeights = [updatedWeights, updatedWeight];
                        updatedMeans = [updatedMeans, updatedMean];
                        updatedCovs = cat(3, updatedCovs, updatedCov);
                    end
                end
            end
            
            obj.weights = updatedWeights;
            obj.means = updatedMeans;
            obj.covariances = updatedCovs;
            obj.numComponents = length(obj.weights);
            
            obj.logger.debug(sprintf('更新后组件数: %d', obj.numComponents));
            obj.logger.logFunctionEnd('update', 0);
        end
        
        function pruneAndMerge(obj, pruningThreshold, mergingThreshold, maxComponents)
            % PRUNEANDMERGE 剪枝与合并
            %   obj.pruneAndMerge()
            %   obj.pruneAndMerge(pruningThreshold, mergingThreshold, maxComponents)
            %
            %   输入参数:
            %       pruningThreshold - 剪枝阈值 (double, 可选, 默认1e-5)
            %       mergingThreshold - 合并阈值 (double, 可选, 默认4，保留供未来使用)
            %       maxComponents - 最大组件数 (double, 可选, 默认100)
            %
            %   备注:
            %       当前实现仅包含剪枝功能，合并功能待后续实现
            
            arguments
                obj
                pruningThreshold (1,1) double = 1e-5
                mergingThreshold (1,1) double {mustBeNonnegative} = 4
                maxComponents (1,1) double {mustBePositive} = 100
            end
            
            if mergingThreshold < 0
                return;
            end
            
            obj.logger.debug(sprintf('剪枝前组件数: %d', obj.numComponents));
            
            validIndices = obj.weights > pruningThreshold;
            if ~any(validIndices)
                obj.initialize();
                return;
            end
            
            obj.weights = obj.weights(validIndices);
            obj.means = obj.means(:, validIndices);
            obj.covariances = obj.covariances(:, :, validIndices);
            obj.numComponents = sum(validIndices);
            
            [obj.weights, sortIndices] = sort(obj.weights, 'descend');
            obj.means = obj.means(:, sortIndices);
            obj.covariances = obj.covariances(:, :, sortIndices);
            
            if obj.numComponents > maxComponents
                obj.weights = obj.weights(1:maxComponents);
                obj.means = obj.means(:, 1:maxComponents);
                obj.covariances = obj.covariances(:, :, 1:maxComponents);
                obj.numComponents = maxComponents;
            end
            
            totalWeight = sum(obj.weights);
            obj.weights = obj.weights / totalWeight * totalWeight;
            
            obj.logger.debug(sprintf('剪枝后组件数: %d', obj.numComponents));
        end
        
        function estimates = extractStates(obj, weightThreshold)
            % EXTRACTSTATES 提取状态估计
            %   estimates = obj.extractStates()
            %   estimates = obj.extractStates(weightThreshold)
            %
            %   输入参数:
            %       weightThreshold - 权重阈值 (double, 可选, 默认0.5)
            %
            %   输出参数:
            %       estimates - 状态估计结构体
            
            arguments
                obj
                weightThreshold (1,1) double = 0.5
            end
            
            estimates = struct();
            estimates.numTargets = 0;
            estimates.positions = [];
            estimates.velocities = [];
            
            validIndices = obj.weights > weightThreshold;
            
            if any(validIndices)
                indices = find(validIndices);
                estimates.numTargets = length(indices);
                
                for i = 1:estimates.numTargets
                    idx = indices(i);
                    estimates.positions(:, i) = obj.means(1:2, idx);
                    estimates.velocities(:, i) = obj.means(3:4, idx);
                end
            end
            
            obj.logger.debug(sprintf('提取 %d 个目标状态', estimates.numTargets));
        end
    end
    
    methods (Access = private)
        function likelihood = computeGaussianLikelihood(obj, measurement, mean, covariance)
            % COMPUTEGAUSSIANLIKELIHOOD 计算高斯似然
            %   likelihood = obj.computeGaussianLikelihood(measurement, mean, covariance)
            %
            %   输入参数:
            %       measurement - 测量值 (double, 向量)
            %       mean - 高斯均值 (double, 向量)
            %       covariance - 协方差矩阵 (double, 矩阵)
            %
            %   输出参数:
            %       likelihood - 高斯似然值 (double, 标量)
            %
            %   计算给定测量在高斯分布下的似然值
            %   如果计算失败（如矩阵奇异），返回小正值1e-10
            
            dim = length(measurement);
            diff = measurement - mean;
            
            try
                invCov = inv(covariance);
                detCov = det(covariance);
                
                if detCov <= 0
                    detCov = 1e-10;
                end
                
                exponent = -0.5 * diff' * invCov * diff;
                normalization = 1 / sqrt((2 * pi) ^ dim * detCov);
                
                likelihood = normalization * exp(exponent);
            catch exc
                obj.logger.warning(sprintf('计算高斯似然失败: %s', exc.message));
                likelihood = 1e-10;
            end
        end
    end
end
