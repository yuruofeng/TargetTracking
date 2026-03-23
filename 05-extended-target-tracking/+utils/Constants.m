classdef Constants
    % CONSTANTS 全局常量定义类
    %   定义项目中使用的所有常量，避免在代码中使用魔法数字
    %
    %   使用示例:
    %       pD = utils.Constants.DEFAULT_DETECTION_PROBABILITY;
    %       dim = utils.Constants.SPATIAL_DIMENSION;
    
    properties (Constant)
        SPATIAL_DIMENSION = 2
        MAX_CARDINALITY = 100
        POLY_LENGTH = 100
    end
    
    properties (Constant)
        DEFAULT_DETECTION_PROBABILITY = 0.99
        DEFAULT_SURVIVAL_PROBABILITY = 0.99
        DEFAULT_SPAWN_WEIGHT = 0.05
        DEFAULT_BIRTH_WEIGHT = 0.1
    end
    
    properties (Constant)
        DEFAULT_MEASUREMENT_RATE = 25
        DEFAULT_SIGMA_X = 5
        DEFAULT_SIGMA_Y = 10
    end
    
    properties (Constant)
        DEFAULT_X_MIN = -200
        DEFAULT_X_MAX = 200
        DEFAULT_Y_MIN = -200
        DEFAULT_Y_MAX = 200
    end
    
    properties (Constant)
        PRUNING_THRESHOLD = 1e-5
        MERGING_THRESHOLD = 4
        MAX_GAUSSIAN_COMPONENTS = 100
    end
    
    properties (Constant)
        NUMERICAL_PRECISION = 1e-10
        INFINITY = 1e100
    end
    
    properties (Constant)
        TEMPORAL_DECAY_FACTOR = 25
        MANEUVER_TIME_CONSTANT = 1
    end
    
    properties (Constant)
        LOG_LEVEL_DEBUG = 'DEBUG'
        LOG_LEVEL_INFO = 'INFO'
        LOG_LEVEL_WARNING = 'WARNING'
        LOG_LEVEL_ERROR = 'ERROR'
        LOG_LEVEL_NONE = 'NONE'
    end
    
    methods (Static)
        function idx = getPIndexMatrix(s, d)
            % GETPINDEXMATRIX 获取用于求解Phat = kron(P,V)的索引矩阵
            %   idx = utils.Constants.getPIndexMatrix(s, d)
            %
            %   输入参数:
            %       s - 运动模型阶数 (double)
            %       d - 空间维度 (double)
            %
            %   输出参数:
            %       idx - 索引向量 (double array)
            
            Pidx = 1:(s*d)^2;
            Pidx = reshape(Pidx, s*d, s*d)';
            Pidxvec = [];
            for r = 1:s
                for c = 1:s
                    Pidxsub = Pidx((1:d)+d*(r-1), (1:d)+d*(c-1))';
                    Pidxvec = [Pidxvec; Pidxsub(:)];
                end
            end
            idx = Pidxvec;
        end
        
        function R = getIdentityMatrix(d)
            % GETIDENTITYMATRIX 获取指定维度的单位矩阵
            %   R = utils.Constants.getIdentityMatrix(d)
            %
            %   输入参数:
            %       d - 维度 (double)
            %
            %   输出参数:
            %       R - 单位矩阵 (double matrix)
            
            R = eye(d);
        end
        
        function vol = calculateVolume(xMin, xMax, yMin, yMax)
            % CALCULATEVOLUME 计算监视区域体积
            %   vol = utils.Constants.calculateVolume(xMin, xMax, yMin, yMax)
            %
            %   输入参数:
            %       xMin, xMax - X轴范围 (double)
            %       yMin, yMax - Y轴范围 (double)
            %
            %   输出参数:
            %       vol - 区域面积 (double)
            
            vol = (xMax - xMin) * (yMax - yMin);
        end
    end
end
