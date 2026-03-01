classdef DpTbd < tbd.BaseTbd
% TBD.DPTBD  动态规划检测前跟踪算法。
%   实现DP-TBD算法，用于在图像域测量中进行弱目标检测与跟踪。
%   通过累积似然实现低信噪比目标的有效检测。
%
%   状态向量: [行, 列] (仅位置)
%
%   算法原理：
%       DP-TBD通过动态规划递归累积似然函数，找到使累积似然最大的轨迹。
%       值函数递归：V(x,t) = L(x,t) + max_{x'}[V(x',t-1)]
%       其中L(x,t)是时刻t在状态x的似然，x'是t-1时刻的前驱状态。
%
%   使用方法：
%       cfg = tbd.Config();                           % 创建配置
%       dp = tbd.DpTbd(cfg);                          % 创建DP-TBD
%       dp.run(measData, psfKernel);                  % 执行算法
%       [track, score, valueFunc] = dp.getResults();  % 获取结果
%
%   参考文献：
%       Barniv, Y. (1985). Dynamic Programming Solution for Detection
%       and Tracking of Dim Moving Targets.
%
%   See also: tbd.BaseTbd, tbd.PfTbd, tbd.Scenario

    properties (Access = protected)
        detMap      % 检测图 [gridSize(1) x gridSize(2) x numFrames]
        valueFunc   % 值函数 [gridSize(1) x gridSize(2) x numFrames]
        backPtr     % 回溯指针 [gridSize(1) x gridSize(2) x numFrames x 2]
        maxScore    % 最大累积分数
    end

    methods

        function obj = DpTbd(cfg)
        % DPTBD  创建DP-TBD算法对象。
        %   obj = DpTbd(cfg) 使用指定配置创建。
        %   obj = DpTbd() 使用默认配置。
        %
        %   输入参数：
        %       cfg - (可选) tbd.Config配置对象
            obj@tbd.BaseTbd(cfg);
        end

        function localDetMap = computeDetectionMap(obj, measData, psfKernel)
        % COMPUTEDETECTIONMAP  计算匹配滤波检测图。
        %   对每帧数据应用匹配滤波以增强目标信号。
        %
        %   输入参数：
        %       measData  - 测量数据 [gridSize(1) x gridSize(2) x numFrames]
        %       psfKernel - 点扩散函数核
        %
        %   输出参数：
        %       localDetMap - 检测图，归一化后的匹配滤波响应
            nF = obj.config.numFrames;
            gs = obj.config.gridSize;
            localDetMap = zeros(gs(1), gs(2), nF);
            for t = 1:nF
                localDetMap(:, :, t) = conv2(measData(:, :, t), psfKernel, 'same') ...
                                  / (obj.config.noiseStd^2);
            end
            obj.detMap = localDetMap;
        end

        function run(obj, measData, psfKernel)
        % RUN  执行DP-TBD算法。
        %   执行前向递归和回溯以估计目标轨迹。
        %
        %   输入参数：
        %       measData  - 测量数据 [gridSize(1) x gridSize(2) x numFrames]
        %       psfKernel - 点扩散函数核
        %
        %   算法步骤：
        %       1. 计算检测图（匹配滤波）
        %       2. 前向递归：计算值函数和回溯指针
        %       3. 回溯：从最后一帧反向恢复轨迹
            obj.initializeAlgorithm();
            
            detMap  = obj.computeDetectionMap(measData, psfKernel);
            gs      = obj.config.gridSize;
            nF      = obj.config.numFrames;
            vMax    = obj.config.maxSpeed;

            obj.valueFunc = -inf(gs(1), gs(2), nF);
            obj.backPtr   = zeros(gs(1), gs(2), nF, 2);
            obj.valueFunc(:, :, 1) = detMap(:, :, 1);

            [dR, dC]   = meshgrid(-vMax:vMax, -vMax:vMax);
            nbOffsets  = [dR(:), dC(:)];
            nNb        = size(nbOffsets, 1);

            for t = 2:nF
                for r = 1:gs(1)
                    for c = 1:gs(2)
                        bestScore = -inf;
                        bestPrev  = [0, 0];
                        for k = 1:nNb
                            pr = r - nbOffsets(k, 1);
                            pc = c - nbOffsets(k, 2);
                            if pr >= 1 && pr <= gs(1) && pc >= 1 && pc <= gs(2)
                                cs = obj.valueFunc(pr, pc, t-1);
                                if cs > bestScore
                                    bestScore = cs;
                                    bestPrev  = [pr, pc];
                                end
                            end
                        end
                        if bestScore > -inf
                            obj.valueFunc(r, c, t) = detMap(r, c, t) + bestScore;
                            obj.backPtr(r, c, t, :) = bestPrev;
                        end
                    end
                end
            end

            obj.backtrack();
            obj.markComputed();
        end

        function backtrack(obj)
        % BACKTRACK  从最后一帧回溯恢复轨迹。
        %   找到最后一帧的最大值位置，然后沿回溯指针反向追踪。
            nF = obj.config.numFrames;
            gs = obj.config.gridSize;

            finalVal = obj.valueFunc(:, :, nF);
            [obj.maxScore, maxIdx] = max(finalVal(:));
            [rEnd, cEnd] = ind2sub(gs, maxIdx);

            obj.estTrack = zeros(nF, 2);
            obj.estTrack(nF, :) = [rEnd, cEnd];
            cr = rEnd;  cc = cEnd;
            for t = nF:-1:2
                prev = squeeze(obj.backPtr(cr, cc, t, :))';
                obj.estTrack(t-1, :) = prev;
                cr = prev(1);  cc = prev(2);
            end
        end

        function [track, score, valueFunc] = getResults(obj)
        % GETRESULTS  返回DP-TBD结果。
        %
        %   输出参数：
        %       track     - 估计轨迹 [numFrames x 2]
        %       score     - 最大累积分数
        %       valueFunc - 值函数
            obj.checkComputed();
            track = obj.estTrack;
            score = obj.maxScore;
            valueFunc = obj.valueFunc;
        end

        function [rmse, elapsed] = runWithTiming(obj, measData, psfKernel, trueState)
        % RUNWITHTIMING  运行DP-TBD并计算RMSE和执行时间。
        %
        %   输入参数：
        %       measData  - 测量数据
        %       psfKernel - PSF核
        %       trueState - 真实状态（可选，用于计算RMSE）
        %
        %   输出参数：
        %       rmse    - 均方根误差
        %       elapsed - 执行时间 [秒]
            tic;
            obj.run(measData, psfKernel);
            elapsed = toc;

            if nargin > 4
                rmse = sqrt(sum((obj.estTrack - trueState(:, 1:2)).^2, 2));
            else
                rmse = [];
            end
        end

        function [posRmse, velRmse] = computeRmse(obj, trueState)
        % COMPUTERMSE  计算位置和速度RMSE。
        %
        %   输入参数：
        %       trueState - 真实状态 [numFrames x stateDim]
        %
        %   输出参数：
        %       posRmse - 位置RMSE向量
        %       velRmse - 速度RMSE向量（DP-TBD不估计速度）
            obj.checkComputed();
            nF = obj.config.numFrames;
            posRmse = sqrt(sum((obj.estTrack - trueState(:, 1:2)).^2, 2));
            velRmse = zeros(nF, 1);
        end

        function plotValueFunction(obj, frameIdx)
        % PLOTVALUEFUNCTION  可视化指定帧的值函数。
        %
        %   输入参数：
        %       frameIdx - 帧索引（默认为最后一帧）
            obj.checkComputed();
            if nargin < 2
                frameIdx = obj.config.numFrames;
            end

            figure('Name', 'DP-TBD值函数', 'Color', 'w');
            imagesc(obj.valueFunc(:, :, frameIdx));
            colormap(gca, 'parula'); colorbar;
            hold on;
            if ~isempty(obj.estTrack)
                plot(obj.estTrack(frameIdx, 2), obj.estTrack(frameIdx, 1), ...
                     'w+', 'MarkerSize', 14, 'LineWidth', 2);
            end
            title(sprintf('第%d帧值函数', frameIdx));
            axis equal tight; hold off;
        end

    end

    methods (Access = protected)
        function initializeAlgorithm(obj, varargin)
        % INITIALIZEALGORITHM  初始化DP-TBD算法状态。
            obj.detMap = [];
            obj.valueFunc = [];
            obj.backPtr = [];
            obj.estTrack = [];
            obj.maxScore = -inf;
        end
    end

    methods (Static)
        function name = getAlgorithmType()
            name = 'DpTbd';
        end
        
        function description = getAlgorithmDescription()
            description = 'Dynamic Programming Track-Before-Detect';
        end
    end
end
