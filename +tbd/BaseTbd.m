classdef (Abstract) BaseTbd < handle
% TBD.BASETBD  检测前跟踪(TBD)算法抽象基类。
%   定义所有TBD算法的通用接口，包括DP-TBD和PF-TBD。
%   遵循策略模式，允许不同TBD算法互换使用。
%
%   接口方法（子类必须实现）：
%       run        - 执行TBD算法
%       getResults - 获取估计结果
%       computeRmse - 计算RMSE（可选）
%
%   子类：
%       DpTbd - 动态规划TBD算法
%       PfTbd - 粒子滤波TBD算法
%
%   使用方法：
%       cfg = tbd.Config();
%       tbdAlgo = tbd.DpTbd(cfg);  % 或 tbd.PfTbd(cfg)
%       tbdAlgo.run(measData, psfKernel);
%       [track, score] = tbdAlgo.getResults();
%
%   See also: tbd.DpTbd, tbd.PfTbd, tbd.Config

    properties (Access = protected)
        config
        estTrack
        estState
        isComputed
    end

    properties (Dependent)
        configuration
        estimatedTrack
        isAlgorithmRun
    end

    methods
        function val = get.configuration(obj)
            val = obj.config;
        end
        
        function val = get.estimatedTrack(obj)
            val = obj.estTrack;
        end
        
        function val = get.isAlgorithmRun(obj)
            val = obj.isComputed;
        end
    end

    methods (Abstract)
        run(obj, measData, varargin)
        [track, varargin] = getResults(obj)
    end

    methods (Abstract, Access = protected)
        initializeAlgorithm(obj, varargin)
    end

    methods
        function obj = BaseTbd(cfg)
        % BASETBD  创建TBD算法基类。
        %   obj = BaseTbd(cfg) 使用指定配置初始化。
            if nargin < 1
                cfg = tbd.Config();
            end
            obj.config = cfg;
            obj.isComputed = false;
        end
        
        function [rmse, elapsed] = runWithTiming(obj, measData, varargin)
        % RUNWITHTIMING  运行算法并计算执行时间。
        %
        %   输入参数：
        %       measData - 测量数据立方体
        %       varargin - 额外参数（子类特定）
        %
        %   输出参数：
        %       rmse    - 均方根误差（如果提供真值）
        %       elapsed - 执行时间 [秒]
            tic;
            obj.run(measData, varargin{:});
            elapsed = toc;
            rmse = [];
        end
        
        function reset(obj)
        % RESET  重置算法状态。
            obj.estTrack = [];
            obj.estState = [];
            obj.isComputed = false;
        end
        
        function isValid = hasValidConfig(obj)
        % HASVALIDCONFIG  检查配置是否有效。
            isValid = ~isempty(obj.config) && isa(obj.config, 'tbd.Config');
        end
    end

    methods (Access = protected)
        function validateMeasData(obj, measData)
        % VALIDATEMEASDATA  验证测量数据格式。
            gs = obj.config.gridSize;
            nF = obj.config.numFrames;
            
            if ndims(measData) ~= 3
                error('tbd:InvalidData', '测量数据必须是3D数组');
            end
            if size(measData, 1) ~= gs(1) || size(measData, 2) ~= gs(2)
                error('tbd:InvalidData', ...
                    '测量数据尺寸 [%d x %d] 与配置 [%d x %d] 不匹配', ...
                    size(measData, 1), size(measData, 2), gs(1), gs(2));
            end
            if size(measData, 3) ~= nF
                error('tbd:InvalidData', ...
                    '帧数 %d 与配置 %d 不匹配', size(measData, 3), nF);
            end
        end
        
        function validatePsfKernel(obj, psfKernel)
        % VALIDATEPSFKERNEL  验证PSF核格式。
            r = obj.config.targetRadius;
            expectedSize = 2 * r + 1;
            
            if size(psfKernel, 1) ~= expectedSize || size(psfKernel, 2) ~= expectedSize
                error('tbd:InvalidPsf', ...
                    'PSF核尺寸 [%d x %d] 与期望 [%d x %d] 不匹配', ...
                    size(psfKernel, 1), size(psfKernel, 2), expectedSize, expectedSize);
            end
        end
        
        function markComputed(obj)
        % MARKCOMPUTED  标记算法已执行。
            obj.isComputed = true;
        end
        
        function checkComputed(obj)
        % CHECKCOMPUTED  检查算法是否已执行。
            if ~obj.isComputed
                error('tbd:NotComputed', '请先运行算法');
            end
        end
    end

    methods (Static)
        function name = getAlgorithmType()
            name = 'BaseTbd';
        end
        
        function description = getAlgorithmDescription()
            description = 'Abstract base class for TBD algorithms';
        end
    end
end
