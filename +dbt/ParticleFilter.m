classdef ParticleFilter < dbt.BaseFilter
% DBT.PARTICLEFILTER  粒子滤波器，用于检测后跟踪(DBT)。
%   实现标准SIR(采样重要性重采样)粒子滤波器，用于跟踪具有CT动态特性
%   和距离/方位角测量的目标。
%
%   粒子滤波器使用一组加权粒子表示后验分布，适用于高度非线性系统和
%   非高斯噪声情况。
%
%   状态向量: [x, vx, y, vy, omega]
%       x, y     - 位置坐标 [m]
%       vx, vy   - 速度分量 [m/s]
%       omega    - 转弯角速度 [rad/s]
%
%   测量向量: [方位角, 距离]
%       方位角   - 目标方位角 [rad]
%       距离     - 目标距离 [m]
%
%   算法步骤：
%       1. 初始化   - 从先验分布采样粒子
%       2. 预测     - 通过动力学模型传播粒子
%       3. 更新     - 使用测量似然更新权重
%       4. 重采样   - 检测到退化时重采样粒子
%       5. 估计     - 计算加权状态估计
%
%   使用方法：
%       cfg = dbt.Config();                           % 创建配置
%       pf = dbt.ParticleFilter(cfg);                 % 创建粒子滤波器
%       [weights, particles] = pf.initParticles(x0, P0);  % 初始化粒子
%       for k = 1:numSteps
%           [weights, particles] = pf.predictParticles(weights, particles);  % 预测
%           [weights, particles] = pf.updateParticles(meas(:,k), weights, particles);  % 更新
%           [weights, particles] = pf.resampleParticles(weights, particles);  % 重采样
%       end
%       [state, covar] = pf.estimateState(weights, particles);  % 状态估计
%
%   See also: dbt.BaseFilter, dbt.EKF, dbt.UKF, dbt.CKF, dbt.Config

    properties (Access = private)
        numParticles            % 粒子数量
        processNoise            % 过程噪声协方差矩阵
        measurementNoise        % 测量噪声协方差矩阵
        effectiveParticleThreshold  % 有效粒子数阈值，用于触发重采样
    end

    properties (Dependent)
        particleCount   % 粒子数量（只读属性）
    end

    methods
        function obj = ParticleFilter(cfg)
        % PARTICLEFILTER  创建粒子滤波器。
        %   obj = ParticleFilter(cfg) 使用dbt.Config配置对象初始化。
        %   obj = ParticleFilter() 使用默认配置。
        %
        %   输入参数：
        %       cfg - (可选) dbt.Config配置对象
            obj.setConfig(cfg);
            obj.numParticles = cfg.numParticles;
            obj.processNoise = obj.config.procDriveMat * ...
                               obj.config.procNoiseCov * ...
                               obj.config.procDriveMat';
            obj.measurementNoise = obj.config.getMeasurementNoiseCov();
            obj.effectiveParticleThreshold = obj.numParticles / 2;  % 阈值设为粒子数的一半
        end
        
        function val = get.particleCount(obj)
            val = obj.numParticles;
        end
    end

    methods
        function [weights, particles] = initParticles(obj, x0, P0)
        % INITPARTICLES  从高斯先验初始化粒子。
        %   从初始高斯分布 N(x0, P0) 采样粒子。
        %
        %   输入参数：
        %       x0 - 初始状态向量 [stateDim x 1]
        %       P0 - 初始协方差矩阵 [stateDim x stateDim]
        %
        %   输出参数：
        %       weights   - 初始权重向量 [numParticles x 1]，均匀分布
        %       particles - 粒子矩阵 [stateDim x numParticles]
            particles = mvnrnd(x0', P0, obj.numParticles)';
            weights = ones(obj.numParticles, 1) / obj.numParticles;
        end
        
        function [weightsPre, particlesPre] = predictParticles(obj, weights, particles)
        % PREDICTPARTICLES  通过CT动力学传播粒子。
        %   对每个粒子应用动力学模型并添加过程噪声。
        %
        %   输入参数：
        %       weights   - 当前权重向量
        %       particles - 当前粒子矩阵
        %
        %   输出参数：
        %       weightsPre   - 预测后的权重（保持不变）
        %       particlesPre - 预测后的粒子矩阵
            particlesPre = zeros(obj.stateDim, obj.numParticles);
            noise = mvnrnd(zeros(1, obj.stateDim), obj.processNoise, obj.numParticles)';
            
            for i = 1:obj.numParticles
                F = utils.MeasurementModel.ctDynamicMatrix(obj.config.dt, particles(:, i));
                particlesPre(:, i) = F * particles(:, i) + noise(:, i);
            end
            weightsPre = weights;
        end
        
        function [weightsUpd, particlesUpd] = updateParticles(obj, measZ, weights, particles)
        % UPDATEPARTICLES  使用测量似然更新粒子权重。
        %
        %   输入参数：
        %       measZ     - 测量向量 [方位角; 距离]
        %       weights   - 当前权重向量
        %       particles - 当前粒子矩阵
        %
        %   输出参数：
        %       weightsUpd   - 更新后的权重向量
        %       particlesUpd - 粒子矩阵（保持不变）
            measPre = obj.computePredictedMeasurement(particles);
            innov = measPre - measZ;
            innov(1, :) = utils.FilterUtils.wrapToPi(innov(1, :));  % 方位角归一化
            
            logLikelihood = obj.computeLogLikelihood(innov);
            likelihood = obj.normalizeLogLikelihood(logLikelihood);
            
            weightsUpd = weights .* likelihood;
            weightsUpd = weightsUpd / sum(weightsUpd);  % 归一化权重
            particlesUpd = particles;
        end
        
        function [weightsOut, particlesOut] = resampleParticles(obj, weights, particles)
        % RESAMPLEPARTICLES  粒子系统重采样。
        %   使用低方差系统重采样方法，然后随机打乱粒子顺序。
        %
        %   输入参数：
        %       weights   - 当前权重向量
        %       particles - 当前粒子矩阵
        %
        %   输出参数：
        %       weightsOut   - 重采样后的权重（均匀分布）
        %       particlesOut - 重采样后的粒子矩阵
            idx = utils.FilterUtils.systematicResample(weights, obj.numParticles);
            idx = idx(randperm(obj.numParticles));  % 随机打乱
            weightsOut = ones(obj.numParticles, 1) / obj.numParticles;
            particlesOut = particles(:, idx);
        end
        
        function [state, covar] = estimateState(obj, weights, particles)
        % ESTIMATESTATE  计算加权均值和协方差。
        %
        %   输入参数：
        %       weights   - 权重向量
        %       particles - 粒子矩阵
        %
        %   输出参数：
        %       state - 加权状态估计 [stateDim x 1]
        %       covar - 加权协方差矩阵 [stateDim x stateDim]
            state = particles * weights;
            d = particles - state;
            covar = (d .* weights') * d';
        end
    end

    methods
        function [state, covar] = init(obj, x0, P0)
        % INIT  初始化滤波器（BaseFilter接口）。
        %   返回初始粒子集的加权均值和协方差。
        %
        %   输入参数：
        %       x0 - 初始状态向量
        %       P0 - 初始协方差矩阵
        %
        %   输出参数：
        %       state - 初始状态估计
        %       covar - 初始协方差
            [weights, particles] = obj.initParticles(x0, P0);
            [state, covar] = obj.estimateState(weights, particles);
        end
        
        function [statePre, covarPre] = predict(obj, stateUpd, covarUpd)
        % PREDICT  预测步骤（BaseFilter接口）。
        %   注意：此接口会丢失粒子信息。完整粒子操作请使用predictParticles()。
        %
        %   输入参数：
        %       stateUpd - 更新后的状态向量
        %       covarUpd - 更新后的协方差矩阵
        %
        %   输出参数：
        %       statePre - 预测状态向量
        %       covarPre - 预测协方差矩阵
            particles = mvnrnd(stateUpd', covarUpd, obj.numParticles)';
            weights = ones(obj.numParticles, 1) / obj.numParticles;
            [~, particles] = obj.predictParticles(weights, particles);
            [statePre, covarPre] = obj.estimateState(weights, particles);
        end
        
        function [stateUpd, covarUpd, innov, innovCov] = update(obj, measZ, statePre, covarPre, varargin)
        % UPDATE  更新步骤（BaseFilter接口）。
        %   注意：此接口会丢失粒子信息。完整粒子操作请使用updateParticles()。
        %
        %   输入参数：
        %       measZ    - 测量向量
        %       statePre - 预测状态向量
        %       covarPre - 预测协方差矩阵
        %
        %   输出参数：
        %       stateUpd - 更新后的状态向量
        %       covarUpd - 更新后的协方差矩阵
        %       innov    - 新息向量
        %       innovCov - 新息协方差矩阵
            particles = mvnrnd(statePre', covarPre, obj.numParticles)';
            weights = ones(obj.numParticles, 1) / obj.numParticles;
            [weights, particles] = obj.updateParticles(measZ, weights, particles);
            [stateUpd, covarUpd] = obj.estimateState(weights, particles);
            
            measPre = obj.computePredictedMeasurement(particles);
            innov = measZ - mean(measPre, 2);
            innov(1) = utils.FilterUtils.wrapToPi(innov(1));
            innovCov = obj.measurementNoise;
        end
        
        function [estStates, estCovars, elapsed] = run(obj, meas, x0, P0)
        % RUN  执行粒子滤波器处理整个测量序列。
        %   实现完整的SIR算法，包含自动重采样。
        %
        %   输入参数：
        %       meas - 测量序列 [measDim x nSteps]
        %       x0   - 初始状态向量
        %       P0   - 初始协方差矩阵
        %
        %   输出参数：
        %       estStates - 状态估计序列 [stateDim x nSteps]
        %       estCovars - 协方差序列 [stateDim x stateDim x nSteps]
        %       elapsed   - 执行时间 [秒]
            nSteps = size(meas, 2);
            estStates = zeros(obj.stateDim, nSteps);
            estCovars = zeros(obj.stateDim, obj.stateDim, nSteps);
            
            [weights, particles] = obj.initParticles(x0, P0);
            tic;
            for k = 1:nSteps
                % 预测步骤：传播粒子
                [weights, particles] = obj.predictParticles(weights, particles);
                % 更新步骤：更新权重
                [weights, particles] = obj.updateParticles(meas(:, k), weights, particles);
                
                % 检查是否需要重采样
                nEff = obj.computeEffectiveParticles(weights);
                if nEff < obj.effectiveParticleThreshold
                    [weights, particles] = obj.resampleParticles(weights, particles);
                end
                
                [estStates(:, k), estCovars(:, :, k)] = obj.estimateState(weights, particles);
            end
            elapsed = toc;
        end
    end

    methods (Access = private)
        function measPre = computePredictedMeasurement(obj, particles)
        % COMPUTEPREDICTEDMEASUREMENT  计算每个粒子的预测测量。
        %
        %   输入参数：
        %       particles - 粒子矩阵 [stateDim x numParticles]
        %
        %   输出参数：
        %       measPre - 预测测量矩阵 [2 x numParticles]
            measPre = [atan2(particles(3, :), particles(1, :));
                       sqrt(sum(particles([1 3], :).^2, 1))];
        end
        
        function logLik = computeLogLikelihood(obj, innov)
        % COMPUTELOGLIKELIHOOD  计算每个粒子的对数似然。
        %   使用对数似然避免数值下溢问题。
        %
        %   输入参数：
        %       innov - 新息矩阵 [measDim x numParticles]
        %
        %   输出参数：
        %       logLik - 对数似然向量 [numParticles x 1]
            invR = inv(obj.measurementNoise);
            detR = det(obj.measurementNoise);
            
            logLik = zeros(obj.numParticles, 1);
            for i = 1:obj.numParticles
                logLik(i) = -0.5 * (innov(:, i)' * invR * innov(:, i));
            end
            logLik = logLik - 0.5 * (log(detR) + 2 * log(2*pi));
        end
        
        function likelihood = normalizeLogLikelihood(obj, logLik)
        % NORMALIZELOGLIKELIHOOD  归一化对数似然以避免下溢。
        %   使用log-sum-exp技巧进行数值稳定归一化。
        %
        %   输入参数：
        %       logLik - 对数似然向量
        %
        %   输出参数：
        %       likelihood - 归一化后的似然向量
            maxLogLik = max(logLik);
            likelihood = exp(logLik - maxLogLik);
        end
        
        function nEff = computeEffectiveParticles(obj, weights)
        % COMPUTEEFFECTIVEPARTICLES  计算有效粒子数。
        %   N_eff = 1 / sum(w_i^2)，用于检测粒子退化。
        %
        %   输入参数：
        %       weights - 权重向量
        %
        %   输出参数：
        %       nEff - 有效粒子数
            nEff = 1 / sum(weights.^2);
        end
    end

    methods (Static)
        function name = getFilterType()
            name = 'ParticleFilter';
        end
        
        function description = getFilterDescription()
            description = 'Particle Filter with SIR (Sampling Importance Resampling)';
        end
    end
end
