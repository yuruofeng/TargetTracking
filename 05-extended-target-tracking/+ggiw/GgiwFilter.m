classdef GgiwFilter < handle
    % GGIWFILTER GGIW-PHD滤波器类
    %   实现基于Gamma-Gaussian-Inverse-Wishart的扩展目标PHD滤波器
    %
    %   属性说明:
    %       config - 配置管理器
    %       logger - 日志记录器
    %       components - GIW组件集合
    %
    %   使用示例:
    %       filter = ggiw.GgiwFilter(config);
    %       filter.predict();
    %       filter.update(measurements);
    %       estimates = filter.extractStates();
    %
    %   参见: utils.ConfigManager, utils.Logger
    
    properties (Access = private)
        config
        logger
        timeStep
    end
    
    properties (Access = public)
        components
        cardinalityDistribution
    end
    
    properties (Dependent)
        numComponents
    end
    
    methods
        function obj = GgiwFilter(config)
            % GGIWFILTER 构造函数
            %   filter = ggiw.GgiwFilter()
            %   filter = ggiw.GgiwFilter(config)
            %
            %   输入参数:
            %       config - 配置管理器实例 (utils.ConfigManager, 可选)
            %
            %   输出参数:
            %       obj - GgiwFilter实例
            
            if nargin < 1 || isempty(config)
                obj.config = utils.ConfigManager.getInstance();
            else
                obj.config = config;
            end
            
            obj.logger = utils.Logger('GGIW_Filter', 'INFO');
            obj.timeStep = 0;
            obj.initialize();
        end
        
        function initialize(obj)
            % INITIALIZE 初始化滤波器
            %   obj.initialize()
            
            obj.logger.info('初始化GGIW-PHD滤波器');
            
            obj.components = struct();
            obj.components.weights = [];
            obj.components.alpha = [];
            obj.components.beta = [];
            obj.components.means = [];
            obj.components.kinematicCov = [];
            obj.components.extentDof = [];
            obj.components.extentMatrix = [];
            obj.components.numGaussians = 0;
            
            maxCard = obj.config.get('tracking.maxCardinality', 100);
            obj.cardinalityDistribution = zeros(1, maxCard + 1);
            obj.cardinalityDistribution(1) = 1.0;
        end
        
        function predict(obj, motionModel, processNoise)
            % PREDICT 预测步骤
            %   obj.predict(motionModel, processNoise)
            %
            %   输入参数:
            %       motionModel - 运动模型矩阵F (double matrix)
            %       processNoise - 过程噪声协方差Q (double matrix)
            
            obj.logger.logFunctionStart('predict');
            tic;
            
            obj.addBirthComponents();
            obj.predictExistingTargets(motionModel, processNoise);
            obj.predictCardinality();
            
            elapsedTime = toc;
            obj.logger.logFunctionEnd('predict', elapsedTime);
        end
        
        function update(obj, measurements, measurementModel, measurementNoise)
            % UPDATE 更新步骤
            %   obj.update(measurements, measurementModel, measurementNoise)
            %
            %   输入参数:
            %       measurements - 测量集 (cell array of structs)
            %       measurementModel - 测量模型矩阵H (double matrix)
            %       measurementNoise - 测量噪声协方差R (double matrix)
            
            obj.logger.logFunctionStart('update');
            tic;
            
            if isempty(measurements)
                obj.logger.warning('测量集为空，跳过更新');
                return;
            end
            
            partitions = obj.partitionMeasurements(measurements);
            obj.updateWithPartitions(partitions, measurementModel, measurementNoise);
            
            elapsedTime = toc;
            obj.logger.logFunctionEnd('update', elapsedTime);
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
            %       estimates - 状态估计结构体 (struct)
            
            arguments
                obj
                weightThreshold (1,1) double = 0.3
            end
            
            obj.logger.logFunctionStart('extractStates');
            
            estimates = struct();
            estimates.numTargets = 0;
            estimates.positions = [];
            estimates.extents = [];
            estimates.measurementRates = [];
            
            validIndices = obj.components.weights > weightThreshold;
            
            obj.logger.debug(sprintf('组件数: %d, 最大权重: %.4f, 阈值: %.4f', ...
                obj.components.numGaussians, ...
                max(obj.components.weights, [], 'all'), ...
                weightThreshold));
            
            if any(validIndices)
                indices = find(validIndices);
                estimates.numTargets = length(indices);
                
                for i = 1:estimates.numTargets
                    idx = indices(i);
                    estimates.positions(:, i) = obj.components.means(:, idx);
                    estimates.extents(:, :, i) = obj.getExtentEstimate(idx);
                    estimates.measurementRates(i) = obj.components.alpha(idx) / obj.components.beta(idx);
                end
            end
            
            obj.logger.info(sprintf('提取了 %d 个目标状态估计', estimates.numTargets));
            obj.logger.logFunctionEnd('extractStates', 0);
        end
        
        function pruneAndMerge(obj, pruningThreshold, mergingThreshold, maxComponents)
            % PRUNEANDMERGE 剪枝与合并
            %   obj.pruneAndMerge()
            %   obj.pruneAndMerge(pruningThreshold, mergingThreshold, maxComponents)
            %
            %   输入参数:
            %       pruningThreshold - 剪枝阈值 (double, 可选, 默认1e-5)
            %       mergingThreshold - 合并阈值 (double, 可选, 默认4，保留供未来使用)
            %       maxComponents - 最大组件数 (double, 可选, 默认100，保留供未来使用)
            %
            %   备注:
            %       当前实现仅包含剪枝功能，合并功能待后续实现
            
            arguments
                obj
                pruningThreshold (1,1) double = 1e-5
                mergingThreshold (1,1) double {mustBeNonnegative} = 4
                maxComponents (1,1) double {mustBePositive} = 100
            end
            
            if mergingThreshold < 0 || maxComponents < 1
                return;
            end
            
            obj.logger.debug(sprintf('剪枝合并: %d 个组件 -> ', obj.components.numGaussians));
            
            validIndices = obj.components.weights > pruningThreshold;
            if ~any(validIndices)
                obj.initialize();
                return;
            end
            
            obj.components.weights = obj.components.weights(validIndices);
            obj.components.alpha = obj.components.alpha(validIndices);
            obj.components.beta = obj.components.beta(validIndices);
            obj.components.means = obj.components.means(:, validIndices);
            obj.components.kinematicCov = obj.components.kinematicCov(:, :, validIndices);
            obj.components.extentDof = obj.components.extentDof(validIndices);
            obj.components.extentMatrix = obj.components.extentMatrix(:, :, validIndices);
            obj.components.numGaussians = sum(validIndices);
            
            if obj.components.numGaussians == 0
                return;
            end
            
            [obj.components.weights, sortIndices] = sort(obj.components.weights, 'descend');
            obj.components.alpha = obj.components.alpha(sortIndices);
            obj.components.beta = obj.components.beta(sortIndices);
            obj.components.means = obj.components.means(:, sortIndices);
            obj.components.kinematicCov = obj.components.kinematicCov(:, :, sortIndices);
            obj.components.extentDof = obj.components.extentDof(sortIndices);
            obj.components.extentMatrix = obj.components.extentMatrix(:, :, sortIndices);
            
            if obj.components.numGaussians > maxComponents
                obj.components.weights = obj.components.weights(1:maxComponents);
                obj.components.alpha = obj.components.alpha(1:maxComponents);
                obj.components.beta = obj.components.beta(1:maxComponents);
                obj.components.means = obj.components.means(:, 1:maxComponents);
                obj.components.kinematicCov = obj.components.kinematicCov(:, :, 1:maxComponents);
                obj.components.extentDof = obj.components.extentDof(1:maxComponents);
                obj.components.extentMatrix = obj.components.extentMatrix(:, :, 1:maxComponents);
                obj.components.numGaussians = maxComponents;
            end
            
            maxWeight = max(obj.components.weights);
            if maxWeight > 100
                obj.components.weights = obj.components.weights / maxWeight;
            end
            
            obj.logger.debug(sprintf('%d 个组件', obj.components.numGaussians));
        end
        
        function extent = getExtentEstimate(obj, componentIndex)
            % GETEXTENTESTIMATE 获取扩展估计
            %   extent = obj.getExtentEstimate(componentIndex)
            %
            %   输入参数:
            %       componentIndex - 组件索引 (double)
            %
            %   输出参数:
            %       extent - 扩展矩阵估计 (double matrix)
            
            d = size(obj.components.extentMatrix, 1);
            nu = obj.components.extentDof(componentIndex);
            V = obj.components.extentMatrix(:, :, componentIndex);
            
            extent = V / max(1, nu - 2 * d - 2);
        end
    end
    
    methods (Access = private)
        function addBirthComponents(obj)
            % ADDBIRTHCOMPONENTS 添加新生目标分量
            %   obj.addBirthComponents()
            %
            %   根据配置参数创建新生目标分量并添加到滤波器组件集合中
            %   使用配置中的birthWeight、birthMean和birthCov参数
            
            birthWeight = obj.config.get('tracking.birthWeight', 0.1);
            birthMean = obj.config.get('tracking.birthMean', [0; 0; 0; 0]);
            birthCov = obj.config.get('tracking.birthCov', diag([100 100 10 10]));
            
            useAdaptiveBirth = obj.config.get('tracking.useAdaptiveBirth', true);
            
            if useAdaptiveBirth && obj.components.numGaussians > 0
                return;
            end
            
            newComponent = struct();
            newComponent.weights = birthWeight;
            newComponent.alpha = 10;
            newComponent.beta = 1;
            newComponent.means = birthMean;
            newComponent.kinematicCov = birthCov;
            newComponent.extentDof = 7;
            newComponent.extentMatrix = diag([100 100]);
            
            if obj.components.numGaussians == 0
                obj.components.weights = newComponent.weights;
                obj.components.alpha = newComponent.alpha;
                obj.components.beta = newComponent.beta;
                obj.components.means = newComponent.means;
                obj.components.kinematicCov = newComponent.kinematicCov;
                obj.components.extentDof = newComponent.extentDof;
                obj.components.extentMatrix = newComponent.extentMatrix;
            else
                obj.components.weights = [obj.components.weights, newComponent.weights];
                obj.components.alpha = [obj.components.alpha, newComponent.alpha];
                obj.components.beta = [obj.components.beta, newComponent.beta];
                obj.components.means = [obj.components.means, newComponent.means];
                obj.components.kinematicCov = cat(3, obj.components.kinematicCov, newComponent.kinematicCov);
                obj.components.extentDof = [obj.components.extentDof, newComponent.extentDof];
                obj.components.extentMatrix = cat(3, obj.components.extentMatrix, newComponent.extentMatrix);
            end
            
            obj.components.numGaussians = obj.components.numGaussians + 1;
        end
        
        function predictExistingTargets(obj, F, Q)
            % PREDICTEXISTINGTARGETS 预测现有目标
            %   obj.predictExistingTargets(F, Q)
            %
            %   输入参数:
            %       F - 状态转移矩阵 (double matrix)
            %       Q - 过程噪声协方差 (double matrix)
            %
            %   对现有目标进行状态预测，更新权重、均值和协方差
            
            if obj.components.numGaussians == 0
                return;
            end
            
            pS = obj.config.get('tracking.pS', 0.99);
            eta = obj.config.get('tracking.temporalDecay', 0.96);
            
            obj.components.weights = pS * obj.components.weights;
            obj.components.alpha = obj.components.alpha / eta;
            obj.components.beta = obj.components.beta / eta;
            
            for j = 1:obj.components.numGaussians
                obj.components.means(:, j) = F * obj.components.means(:, j);
                obj.components.kinematicCov(:, :, j) = ...
                    utils.ArrayUtils.makeSymmetric(...
                    Q + F * obj.components.kinematicCov(:, :, j) * F');
            end
        end
        
        function predictCardinality(obj)
            % PREDICTCARDINALITY 预测势分布
            %   obj.predictCardinality()
            %
            %   预测目标数量的概率分布（势分布）
            %   考虑目标的生存概率和新生概率
            
            birthProb = obj.config.get('tracking.birthProbability', 0.1);
            
            maxCard = length(obj.cardinalityDistribution);
            predictedCD = zeros(1, maxCard);
            
            for n = 0:(maxCard - 1)
                predictedCD(n + 1) = predictedCD(n + 1) + ...
                    (1 - birthProb) * obj.cardinalityDistribution(n + 1);
                if n > 0
                    predictedCD(n) = predictedCD(n) + ...
                        birthProb * obj.cardinalityDistribution(n + 1);
                end
            end
            
            obj.cardinalityDistribution = predictedCD / sum(predictedCD);
        end
        
        function partitions = partitionMeasurements(obj, measurements)
            % PARTITIONMEASUREMENTS 测量集划分
            %   partitions = obj.partitionMeasurements(measurements)
            %
            %   输入参数:
            %       measurements - 测量集 (cell array of structs)
            %
            %   输出参数:
            %       partitions - 划分结果 (cell array)
            %
            %   使用距离聚类方法划分测量集
            %   当前实现为简化版本，将所有测量归为一个单元
            
            allPoints = [];
            for i = 1:length(measurements)
                if isfield(measurements{i}, 'points') && ~isempty(measurements{i}.points)
                    allPoints = [allPoints, measurements{i}.points];
                end
            end
            
            partitions = cell(1, 1);
            partitions{1}.measurements = allPoints;
            partitions{1}.numCells = 1;
        end
        
        function updateWithPartitions(obj, partitions, H, R)
            % UPDATEWITHPARTITIONS 使用划分更新滤波器
            %   obj.updateWithPartitions(partitions, H, R)
            %
            %   输入参数:
            %       partitions - 测量划分 (cell array)
            %       H - 测量模型矩阵 (double matrix)
            %       R - 测量噪声协方差 (double matrix)
            %
            %   遍历所有划分单元并逐一更新滤波器状态
            
            numPartitions = length(partitions);
            
            for p = 1:numPartitions
                partition = partitions{p};
                
                for c = 1:partition.numCells
                    cellMeas = partition.measurements;
                    obj.updateSingleCell(cellMeas, H, R);
                end
            end
        end
        
        function updateSingleCell(obj, cellMeasurements, H, R)
            % UPDATESINGLECELL 更新单个测量单元
            %   obj.updateSingleCell(cellMeasurements, H, R)
            %
            %   输入参数:
            %       cellMeasurements - 单元内测量点 (double matrix, 2 x N)
            %       H - 测量模型矩阵 (double matrix)
            %       R - 测量噪声协方差 (double matrix)
            %
            %   使用卡尔曼滤波更新每个GIW组件的状态
            %   同时更新扩展参数alpha和beta
            
            numMeas = size(cellMeasurements, 2);
            pD = obj.config.get('tracking.pD', 0.99);
            
            for j = 1:obj.components.numGaussians
                totalInnovationSq = 0;
                
                obj.components.alpha(j) = obj.components.alpha(j) + numMeas;
                obj.components.beta(j) = obj.components.beta(j) + 1;
                
                for m = 1:numMeas
                    z = cellMeasurements(:, m);
                    S = H * obj.components.kinematicCov(:, :, j) * H' + R;
                    K = obj.components.kinematicCov(:, :, j) * H' / S;
                    innovation = z - H * obj.components.means(:, j);
                    
                    innovationSq = innovation' / S * innovation;
                    totalInnovationSq = totalInnovationSq + innovationSq;
                    
                    obj.components.means(:, j) = obj.components.means(:, j) + K * innovation;
                    obj.components.kinematicCov(:, :, j) = ...
                        utils.ArrayUtils.makeSymmetric(...
                        obj.components.kinematicCov(:, :, j) - K * S * K');
                end
                
                if numMeas > 0
                    obj.components.weights(j) = obj.components.weights(j) * (1 + numMeas * 0.5);
                end
            end
        end
    end
    
    methods
        function num = get.numComponents(obj)
            num = obj.components.numGaussians;
        end
    end
end
