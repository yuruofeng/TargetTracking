classdef IMM < handle
% DBT.IMM  交互多模型滤波器，用于机动目标跟踪。
%   实现IMM算法，通过在多个运动模型之间自适应切换来跟踪机动目标。
%
%   IMM算法包含四个步骤：
%     1. 输入交互（混合）- 计算各滤波器的混合初始条件
%     2. 模型条件滤波 - 各模型并行执行滤波
%     3. 模型概率更新 - 根据似然更新模型概率
%     4. 输出融合 - 加权融合各模型输出
%
%   使用方法：
%       cfg = dbt.ConfigIMM();                          % 创建IMM配置
%       imm = dbt.IMM(cfg);                             % 创建IMM滤波器
%       [estStates, estCovars, modelProbs] = imm.run(meas, x0, P0);  % 执行滤波
%
%   模型组合示例：
%       cfg = dbt.ConfigIMM('modelTypes', {'CV', 'CA', 'CT'});
%       imm = dbt.IMM(cfg);
%
%   参考文献：
%       Bar-Shalom, Y., et al. "Estimation with Applications to Tracking
%       and Navigation," Wiley, 2001.
%
%   See also: dbt.ConfigIMM, dbt.MotionModelEKF, dbt.MotionModelConfig

    properties
        config       % IMM配置对象 (dbt.ConfigIMM)
        models       % 滤波器模型单元数组 {nModels x 1}
        nModels      % 模型数量
        transProb    % 马尔可夫转移概率矩阵 [nModels x nModels]
        modelProbs   % 当前模型概率向量 [nModels x 1]
        lastStates   % 各模型上一时刻状态 {nModels x 1}
        lastCovars   % 各模型上一时刻协方差 {nModels x 1}
    end

    methods

        function obj = IMM(cfg)
        % IMM  创建IMM滤波器。
        %   obj = IMM(cfg) 使用dbt.ConfigIMM配置对象初始化。
        %   obj = IMM() 使用默认配置（CV+CA+CT模型）。
        %
        %   输入参数：
        %       cfg - (可选) dbt.ConfigIMM配置对象
            if nargin < 1
                cfg = dbt.ConfigIMM();
            end
            obj.config = cfg;
            obj.models = cfg.models;
            obj.nModels = length(obj.models);
            obj.transProb = cfg.transProb;
            obj.modelProbs = cfg.initProbs(:);
        end

        function [estState, estCovar] = init(obj, x0, P0)
        % INIT  初始化IMM滤波器。
        %
        %   输入参数：
        %       x0 - 初始状态向量
        %       P0 - 初始协方差矩阵
        %
        %   输出参数：
        %       estState - 初始融合状态
        %       estCovar - 初始融合协方差
            nModels = obj.nModels;
            obj.lastStates = cell(nModels, 1);
            obj.lastCovars = cell(nModels, 1);
            
            for m = 1:nModels
                stateDim = obj.config.modelStateDims(m);
                obj.lastStates{m} = obj.convertState(x0, stateDim);
                obj.lastCovars{m} = obj.shrinkCovar(P0, stateDim);
            end
            
            estState = x0;
            estCovar = P0;
        end

        function [mixedStates, mixedCovars, cBar] = interact(obj)
        % INTERACT  输入交互步骤。
        %   计算每个滤波器的混合初始条件。
        %
        %   输出参数：
        %       mixedStates - 混合后的状态向量数组 {nModels x 1}
        %       mixedCovars - 混合后的协方差数组 {nModels x 1}
        %       cBar        - 归一化常数向量 [nModels x 1]
            nModels = obj.nModels;
            mixedStates = cell(nModels, 1);
            mixedCovars = cell(nModels, 1);
            
            % 计算归一化常数 c_bar(j) = sum_i(pi_ij * mu_i)
            cBar = zeros(nModels, 1);
            for j = 1:nModels
                cBar(j) = sum(obj.transProb(:,j) .* obj.modelProbs);
            end
            
            % 计算混合权重 mu_i|j = pi_ij * mu_i / c_bar(j)
            weights = zeros(nModels, nModels);
            for i = 1:nModels
                for j = 1:nModels
                    weights(i,j) = obj.transProb(i,j) * obj.modelProbs(i) / cBar(j);
                end
            end
            
            outputDim = obj.config.outputStateDim;
            
            % 计算混合状态和协方差
            for j = 1:nModels
                targetStateDim = obj.config.modelStateDims(j);
                
                % 混合状态: x_0j = sum_i(mu_i|j * x_i)
                mixedState = zeros(outputDim, 1);
                for i = 1:nModels
                    convertedState = obj.convertState(obj.lastStates{i}, outputDim);
                    mixedState = mixedState + weights(i,j) * convertedState;
                end
                
                % 混合协方差: P_0j = sum_i(mu_i|j * [P_i + (x_i-x_0j)*(x_i-x_0j)'])
                mixedCovar = zeros(outputDim, outputDim);
                for i = 1:nModels
                    convertedState = obj.convertState(obj.lastStates{i}, outputDim);
                    diff = convertedState - mixedState;
                    expandedCovar = obj.expandCovar(obj.lastCovars{i}, outputDim);
                    mixedCovar = mixedCovar + weights(i,j) * (expandedCovar + diff * diff');
                end
                
                % 转换到目标状态维度
                mixedStates{j} = obj.convertState(mixedState, targetStateDim);
                mixedCovars{j} = obj.shrinkCovar(mixedCovar, targetStateDim);
            end
        end

        function [states, covars, likelihoods] = filterStep(obj, meas, mixedStates, mixedCovars)
        % FILTERSTEP  模型条件滤波步骤。
        %   各模型并行执行预测和更新。
        %
        %   输入参数：
        %       meas         - 当前测量向量
        %       mixedStates  - 混合状态数组
        %       mixedCovars  - 混合协方差数组
        %
        %   输出参数：
        %       states      - 更新后的状态数组 {nModels x 1}
        %       covars      - 更新后的协方差数组 {nModels x 1}
        %       likelihoods - 各模型的似然值 [nModels x 1]
            nModels = obj.nModels;
            states = cell(nModels, 1);
            covars = cell(nModels, 1);
            likelihoods = zeros(nModels, 1);
            
            for m = 1:nModels
                model = obj.models{m};
                [state, covar] = model.init(mixedStates{m}, mixedCovars{m});
                [state, covar] = model.predict(state, covar);
                [state, covar, innov, innovCov] = model.update(meas, state, covar);
                
                states{m} = state;
                covars{m} = covar;
                
                likelihoods(m) = obj.computeLikelihood(innov, innovCov);
            end
        end

        function lik = computeLikelihood(obj, innov, innovCov)
        % COMPUTELIKELIHOOD  计算高斯似然。
        %   基于新息和新息协方差计算测量似然。
        %
        %   输入参数：
        %       innov    - 新息向量
        %       innovCov - 新息协方差矩阵
        %
        %   输出参数：
        %       lik - 似然值
            d = length(innov);
            detS = det(innovCov);
            if detS <= 0
                detS = 1e-10;  % 防止数值问题
            end
            lik = 1 / sqrt((2*pi)^d * detS) * ...
                  exp(-0.5 * innov' * inv(innovCov) * innov);
        end

        function obj = updateProbs(obj, likelihoods, cBar)
        % UPDATEPROBS  更新模型概率。
        %   mu_j = L_j * c_bar(j) / sum_j(L_j * c_bar(j))
        %
        %   输入参数：
        %       likelihoods - 各模型似然值
        %       cBar        - 归一化常数
            for m = 1:obj.nModels
                obj.modelProbs(m) = likelihoods(m) * cBar(m);
            end
            obj.modelProbs = obj.modelProbs / sum(obj.modelProbs);  % 归一化
        end

        function [estState, estCovar] = fuse(obj, states, covars)
        % FUSE  融合各模型输出。
        %   加权融合所有模型的状态估计和协方差。
        %
        %   输入参数：
        %       states - 各模型状态数组
        %       covars - 各模型协方差数组
        %
        %   输出参数：
        %       estState - 融合状态估计
        %       estCovar - 融合协方差矩阵
            stateDim = obj.config.outputStateDim;
            estState = zeros(stateDim, 1);
            estCovar = zeros(stateDim, stateDim);
            
            % 计算加权状态估计: x_hat = sum_j(mu_j * x_j)
            for m = 1:obj.nModels
                convertedState = obj.convertState(states{m}, stateDim);
                estState = estState + obj.modelProbs(m) * convertedState;
            end
            
            % 计算融合协方差: P = sum_j(mu_j * [P_j + (x_j-x_hat)*(x_j-x_hat)'])
            for m = 1:obj.nModels
                convertedState = obj.convertState(states{m}, stateDim);
                diff = convertedState - estState;
                estCovar = estCovar + obj.modelProbs(m) * ...
                    (obj.expandCovar(covars{m}, stateDim) + diff * diff');
            end
        end

        function convertedState = convertState(obj, state, targetDim)
        % CONVERTSTATE  状态维度转换。
        %   处理CV(4维)、CA(6维)和CT(5维)状态格式之间的转换。
        %
        %   状态格式：
        %       CV: [x, vx, y, vy]
        %       CA: [x, vx, ax, y, vy, ay]
        %       CT: [x, vx, y, vy, omega]
        %
        %   输入参数：
        %       state     - 源状态向量
        %       targetDim - 目标维度
        %
        %   输出参数：
        %       convertedState - 转换后的状态向量
            sourceDim = length(state);
            
            x = state(1); vx = state(2);
            
            if sourceDim == 4
                y = state(3); vy = state(4);
                ax = 0; ay = 0; omega = 0;
            elseif sourceDim == 5
                y = state(3); vy = state(4);
                omega = state(5);
                ax = 0; ay = 0;
            else
                y = state(4); vy = state(5);
                ax = state(3); ay = state(6);
                omega = 0;
            end
            
            if targetDim == 4
                convertedState = [x; vx; y; vy];
            elseif targetDim == 5
                convertedState = [x; vx; y; vy; omega];
            else
                convertedState = [x; vx; ax; y; vy; ay];
            end
        end

        function expandedCovar = expandCovar(obj, covar, targetDim)
        % EXPANDCOVAR  扩展协方差矩阵到目标维度。
        %   将低维协方差扩展到高维，新增维度使用默认值。
        %
        %   输入参数：
        %       covar     - 源协方差矩阵
        %       targetDim - 目标维度
        %
        %   输出参数：
        %       expandedCovar - 扩展后的协方差矩阵
            sourceDim = size(covar, 1);
            if sourceDim == targetDim
                expandedCovar = covar;
                return;
            end
            
            expandedCovar = eye(targetDim) * 100;  % 新增维度使用大初始协方差
            
            if sourceDim == 4 && targetDim == 6
                expandedCovar([1,2,4,5], [1,2,4,5]) = covar;
            elseif sourceDim == 4 && targetDim == 5
                expandedCovar([1,2,3,4], [1,2,3,4]) = covar;
            elseif sourceDim == 5 && targetDim == 6
                expandedCovar([1,2,3,4], [1,2,3,4]) = covar(1:4, 1:4);
            elseif sourceDim == 6 && targetDim == 5
                expandedCovar([1,2,3,4], [1,2,3,4]) = covar([1,2,4,5], [1,2,4,5]);
            else
                expandedCovar(1:sourceDim, 1:sourceDim) = covar;
            end
        end

        function shrunkCovar = shrinkCovar(obj, covar, targetDim)
        % SHRINKCOVAR  收缩协方差矩阵到目标维度。
        %   从高维协方差提取低维子矩阵。
        %
        %   输入参数：
        %       covar     - 源协方差矩阵
        %       targetDim - 目标维度
        %
        %   输出参数：
        %       shrunkCovar - 收缩后的协方差矩阵
            sourceDim = size(covar, 1);
            if sourceDim == targetDim
                shrunkCovar = covar;
                return;
            end
            
            if targetDim == 4 && sourceDim == 6
                shrunkCovar = covar([1,2,4,5], [1,2,4,5]);
            elseif targetDim == 4 && sourceDim == 5
                shrunkCovar = covar(1:4, 1:4);
            elseif targetDim == 5 && sourceDim == 6
                idx = [1,2,4,5];
                shrunkCovar = eye(5) * 100;
                shrunkCovar(idx, idx) = covar(idx, idx);
            else
                shrunkCovar = covar(1:targetDim, 1:targetDim);
            end
        end

        function [estStates, estCovars, modelProbsHist, elapsed] = run(obj, meas, x0, P0)
        % RUN  执行IMM滤波器处理整个测量序列。
        %
        %   输入参数：
        %       meas - 测量序列 [measDim x nSteps]
        %       x0   - 初始状态向量
        %       P0   - 初始协方差矩阵
        %
        %   输出参数：
        %       estStates    - 状态估计序列 [stateDim x nSteps]
        %       estCovars    - 协方差序列 [stateDim x stateDim x nSteps]
        %       modelProbsHist - 模型概率历史 [nModels x nSteps]
        %       elapsed      - 执行时间 [秒]
            nSteps = size(meas, 2);
            stateDim = obj.config.outputStateDim;
            estStates = zeros(stateDim, nSteps);
            estCovars = zeros(stateDim, stateDim, nSteps);
            modelProbsHist = zeros(obj.nModels, nSteps);
            
            obj.init(x0, P0);
            
            tic;
            for k = 1:nSteps
                % 步骤1: 输入交互
                [mixedStates, mixedCovars, cBar] = obj.interact();
                % 步骤2: 模型条件滤波
                [states, covars, likelihoods] = obj.filterStep(meas(:,k), mixedStates, mixedCovars);
                % 步骤3: 模型概率更新
                obj.updateProbs(likelihoods, cBar);
                % 步骤4: 输出融合
                [estState, estCovar] = obj.fuse(states, covars);
                
                obj.lastStates = states;
                obj.lastCovars = covars;
                
                estStates(:,k) = estState;
                estCovars(:,:,k) = estCovar;
                modelProbsHist(:,k) = obj.modelProbs;
            end
            elapsed = toc;
        end
    end
end
