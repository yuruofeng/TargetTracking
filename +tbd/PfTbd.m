classdef PfTbd < tbd.BaseTbd
% TBD.PFTBD  粒子滤波检测前跟踪算法。
%   实现PF-TBD算法，使用对数似然比加权进行弱目标检测与跟踪。
%
%   状态向量: [行, 列, vRow, vCol, 幅度]
%
%   使用方法：
%       cfg = tbd.Config();
%       pf = tbd.PfTbd(cfg);
%       pf.run(measData, initState, psfKernel);
%       estState = pf.getEstimate();
%       [posRmse, velRmse] = pf.computeRmse(trueState);
%
%   参考文献：
%       Salmond, D. J. (2001). Track-Before-Detect Techniques.
%
%   See also: tbd.BaseTbd, tbd.DpTbd, tbd.Scenario

    properties (Access = protected)
        particles
        logWeights
    end

    properties
        posRmse
        velRmse
    end

    methods

        function obj = PfTbd(cfg)
        % PFTBD  创建PF-TBD算法对象。
        %   obj = PfTbd(cfg) 使用指定配置创建。
        %   obj = PfTbd() 使用默认配置。
        %
        %   输入参数：
        %       cfg - (可选) tbd.Config配置对象
            obj@tbd.BaseTbd(cfg);
        end

        function init(obj, initState)
        % INIT  初始化粒子。
        %   在初始状态周围采样粒子。
        %
        %   输入参数：
        %       initState - 初始状态向量 [1 x 5]
            obj.initializeAlgorithm();
            
            nP = obj.config.numParticles;
            initSpread = [3, 3, 0.5, 0.5, 0.3];
            obj.particles = repmat(initState, nP, 1) + ...
                            randn(nP, 5) .* initSpread;
            obj.particles(:, 5) = abs(obj.particles(:, 5));
            obj.logWeights = zeros(nP, 1);
        end

        function predict(obj)
        % PREDICT  通过CV动力学传播粒子。
            nP = obj.config.numParticles;
            obj.particles(:, 1) = obj.particles(:, 1) + obj.particles(:, 3) * obj.config.dt ...
                                   + obj.config.procNoisePos * randn(nP, 1);
            obj.particles(:, 2) = obj.particles(:, 2) + obj.particles(:, 4) * obj.config.dt ...
                                   + obj.config.procNoisePos * randn(nP, 1);
            obj.particles(:, 3) = obj.particles(:, 3) + obj.config.procNoiseVel * randn(nP, 1);
            obj.particles(:, 4) = obj.particles(:, 4) + obj.config.procNoiseVel * randn(nP, 1);
            obj.particles(:, 5) = abs(obj.particles(:, 5) + obj.config.procNoiseAmp * randn(nP, 1));
        end

        function update(obj, frame, psfKernel)
        % UPDATE  使用对数似然比更新粒子权重。
            nP = obj.config.numParticles;
            gs = obj.config.gridSize;
            noiseStd = obj.config.noiseStd;
            r = obj.config.targetRadius;

            for ip = 1:nP
                cRow = round(obj.particles(ip, 1));
                cCol = round(obj.particles(ip, 2));
                amp  = obj.particles(ip, 5);
                logLR = 0;
                for di = -r:r
                    for dj = -r:r
                        ri = cRow + di;  cj = cCol + dj;
                        if ri >= 1 && ri <= gs(1) && cj >= 1 && cj <= gs(2)
                            hVal = amp * psfKernel(di + r + 1, dj + r + 1);
                            z    = frame(ri, cj);
                            logLR = logLR + (z * hVal - 0.5 * hVal^2) / noiseStd^2;
                        end
                    end
                end
                obj.logWeights(ip) = obj.logWeights(ip) + logLR;
            end
        end

        function resample(obj)
        % RESAMPLE  系统重采样与对数和指数归一化。
            maxLogW = max(obj.logWeights);
            w = exp(obj.logWeights - maxLogW);
            w = w / sum(w);

            nP = obj.config.numParticles;
            idx = utils.FilterUtils.systematicResample(w, nP);
            obj.particles = obj.particles(idx, :);
            obj.logWeights = zeros(nP, 1);
        end

        function state = estimate(obj)
        % ESTIMATE  计算加权均值状态估计。
            maxLogW = max(obj.logWeights);
            w = exp(obj.logWeights - maxLogW);
            w = w / sum(w);
            state = w' * obj.particles;
        end

        function run(obj, measData, initState, psfKernel)
        % RUN  执行PF-TBD算法。
        %
        %   输入参数：
        %       measData  - 测量数据立方体
        %       initState - 初始状态向量
        %       psfKernel - PSF核
            obj.initializeAlgorithm(initState);
            
            nF = obj.config.numFrames;
            obj.estState = zeros(nF, 5);

            obj.init(initState);
            for t = 1:nF
                obj.predict();
                obj.update(measData(:, :, t), psfKernel);
                obj.estState(t, :) = obj.estimate();
                obj.resample();
            end
            
            obj.estTrack = obj.estState(:, 1:2);
            obj.markComputed();
        end

        function [track, state] = getResults(obj)
        % GETRESULTS  返回PF-TBD结果。
        %
        %   输出参数：
        %       track - 估计轨迹 [numFrames x 2]
        %       state - 估计状态 [numFrames x 5]
            obj.checkComputed();
            track = obj.estTrack;
            state = obj.estState;
        end

        function state = getEstimate(obj)
        % GETESTIMATE  获取状态估计序列。
            obj.checkComputed();
            state = obj.estState;
        end

        function [posRmse, velRmse] = computeRmse(obj, trueState)
        % COMPUTERMSE  计算位置和速度RMSE。
        %
        %   输入参数：
        %       trueState - 真实状态 [numFrames x stateDim]
        %
        %   输出参数：
        %       posRmse - 位置RMSE向量
        %       velRmse - 速度RMSE向量
            obj.checkComputed();
            nF = obj.config.numFrames;
            obj.posRmse = zeros(1, nF);
            obj.velRmse = zeros(1, nF);
            for t = 1:nF
                obj.posRmse(t) = sqrt(sum((obj.estState(t, 1:2) - trueState(t, 1:2)).^2));
                obj.velRmse(t) = sqrt(sum((obj.estState(t, 3:4) - trueState(t, 3:4)).^2));
            end
            posRmse = obj.posRmse;
            velRmse = obj.velRmse;
        end

        function [elapsed, posRmse, velRmse] = runWithTiming(obj, measData, initState, psfKernel, trueState)
        % RUNWITHTIMING  运行PF-TBD并计算RMSE和执行时间。
        %
        %   输入参数：
        %       measData   - 测量数据
        %       initState  - 初始状态
        %       psfKernel  - PSF核
        %       trueState  - 真实状态（可选）
        %
        %   输出参数：
        %       elapsed - 执行时间 [秒]
        %       posRmse - 位置RMSE
        %       velRmse - 速度RMSE
            tic;
            obj.run(measData, initState, psfKernel);
            elapsed = toc;

            if nargin > 5
                [posRmse, velRmse] = obj.computeRmse(trueState);
            else
                posRmse = [];
                velRmse = [];
            end
        end

        function plotParticles(obj, frameIdx, trueState)
        % PLOTPARTICLES  可视化粒子分布。
        %
        %   输入参数：
        %       frameIdx   - 帧索引（默认为1）
        %       trueState  - 真实状态（可选）
            if isempty(obj.particles)
                error('PfTbd:NoData', '请先初始化粒子');
            end
            if nargin < 2
                frameIdx = 1;
            end

            figure('Name', 'PF-TBD粒子分布', 'Color', 'w');
            maxLogW = max(obj.logWeights);
            w = exp(obj.logWeights - maxLogW);
            w = w / sum(w);
            scatter(obj.particles(:, 2), obj.particles(:, 1), 20*w*1000, 'filled');
            hold on;
            if nargin > 2
                plot(trueState(frameIdx, 2), trueState(frameIdx, 1), ...
                     'r+', 'MarkerSize', 14, 'LineWidth', 2);
            end
            xlabel('列'); ylabel('行');
            title(sprintf('粒子分布 (帧 %d)', frameIdx));
            axis equal; colorbar; hold off;
        end

    end

    methods (Access = protected)
        function initializeAlgorithm(obj, varargin)
        % INITIALIZEALGORITHM  初始化PF-TBD算法状态。
            obj.particles = [];
            obj.logWeights = [];
            obj.estState = [];
            obj.estTrack = [];
            obj.posRmse = [];
            obj.velRmse = [];
        end
    end

    methods (Static)
        function name = getAlgorithmType()
            name = 'PfTbd';
        end
        
        function description = getAlgorithmDescription()
            description = 'Particle Filter Track-Before-Detect';
        end
    end
end
