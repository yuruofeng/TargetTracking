classdef MotionModelEKF < handle
% DBT.MOTIONMODELEKF  运动模型统一扩展卡尔曼滤波器。
%   实现统一的EKF接口，支持CV、CA、CT、Singer和CS等运动模型。
%   通过配置类选择不同的运动模型，简化滤波器使用流程。
%
%   支持的运动模型：
%       'CV'     - 匀速运动模型 [x, vx, y, vy] (4维)
%       'CA'     - 匀加速运动模型 [x, vx, ax, y, vy, ay] (6维)
%       'CT'     - 协调转弯模型 [x, vx, y, vy, omega] (5维)
%       'Singer' - Singer加速度模型 [x, vx, ax, y, vy, ay] (6维)
%       'CS'     - 当前统计模型 [x, vx, ax, y, vy, ay] (6维)
%
%   使用方法：
%       cfg = dbt.MotionModelConfig('CT');           % 创建CT模型配置
%       ekf = dbt.MotionModelEKF(cfg);               % 创建EKF滤波器
%       [state, covar] = ekf.init(x0, P0);           % 初始化
%       for k = 1:numSteps
%           [state, covar] = ekf.predict(state, covar);      % 预测步
%           [state, covar] = ekf.update(meas(:,k), state, covar);  % 更新步
%       end
%
%   状态向量说明：
%       CV:  [x, vx, y, vy]           - 位置和速度
%       CA:  [x, vx, ax, y, vy, ay]   - 位置、速度和加速度
%       CT:  [x, vx, y, vy, omega]    - 位置、速度和转弯角速度
%
%   See also: dbt.MotionModelConfig, dbt.EKF, dbt.KalmanFilterBase, dbt.IMM

    properties
        config      % 配置对象 (dbt.MotionModelConfig)
        modelName   % 模型名称字符串
        stateDim    % 状态向量维度
        measDim     % 测量向量维度
    end

    properties (Access = private)
        tau         % Singer/CS模型的时间常数
        sigma_a     % Singer模型的加速度标准差
        sigma_a_max % CS模型的最大加速度标准差
        p_accel     % CS模型的加速度参数
    end

    methods
        function obj = MotionModelEKF(cfg)
        % MOTIONMODELEKF  创建运动模型EKF滤波器。
        %   obj = MotionModelEKF(cfg) 使用指定配置创建EKF。
        %   obj = MotionModelEKF() 使用默认CV模型。
        %
        %   输入参数：
        %       cfg - (可选) dbt.MotionModelConfig配置对象
        %
        %   示例：
        %       ekf = dbt.MotionModelEKF(dbt.MotionModelConfig('CT'));
            if nargin < 1
                cfg = dbt.MotionModelConfig('CV');
            end
            obj.config = cfg;
            obj.modelName = cfg.modelName;
            obj.stateDim = cfg.stateDim;
            obj.measDim = cfg.measDim;
            
            obj.tau = cfg.tau;
            obj.sigma_a = cfg.sigma_a;
            obj.sigma_a_max = cfg.sigma_a_max;
            obj.p_accel = cfg.p_accel;
        end
    end

    methods
        function [state, covar] = init(obj, x0, P0)
        % INIT  初始化滤波器状态和协方差。
        %
        %   输入参数：
        %       x0 - 初始状态向量 [stateDim x 1]
        %       P0 - 初始协方差矩阵 [stateDim x stateDim]
        %
        %   输出参数：
        %       state - 初始化后的状态向量
        %       covar - 初始化后的协方差矩阵
            state = x0(:);
            covar = P0;
        end
        
        function [statePre, covarPre] = predict(obj, stateUpd, covarUpd)
        % PREDICT  时间更新（预测）步骤。
        %   根据运动模型预测下一时刻的状态和协方差。
        %
        %   输入参数：
        %       stateUpd - 更新后的状态向量
        %       covarUpd - 更新后的协方差矩阵
        %
        %   输出参数：
        %       statePre - 预测状态向量
        %       covarPre - 预测协方差矩阵
            T = obj.config.dt;
            modelType = obj.config.modelType;
            
            if strcmp(modelType, 'CS')
                % 当前统计模型：使用自适应控制输入
                alpha = 1 / obj.tau;
                beta = exp(-alpha * T);
                
                F = obj.getSingerTransitionMatrix(T, beta);
                u = obj.config.getControlInput(stateUpd, T, alpha, beta);
                sigma_a = obj.config.getAdaptiveSigma(stateUpd(3), stateUpd(6));
                Qd = obj.getSingerProcessNoise(T, alpha, beta, sigma_a);
                
                statePre = F * stateUpd + u;
                
            elseif strcmp(modelType, 'Singer')
                % Singer模型：指数相关加速度模型
                alpha = 1 / obj.tau;
                beta = exp(-alpha * T);
                
                F = obj.getSingerTransitionMatrix(T, beta);
                Qd = obj.getSingerProcessNoise(T, alpha, beta, obj.sigma_a);
                
                statePre = F * stateUpd;
                
            else
                % CV/CA/CT模型：使用标准状态转移矩阵
                F = obj.config.getStateTransitionMatrix(stateUpd);
                Qd = obj.config.getProcessNoiseCov();
                statePre = F * stateUpd;
            end
            
            covarPre = F * covarUpd * F' + Qd;
        end
        
        function [stateUpd, covarUpd, innov, innovCov] = update(obj, measZ, statePre, covarPre)
        % UPDATE  测量更新步骤。
        %   使用测量值校正状态估计。
        %
        %   输入参数：
        %       measZ    - 测量向量 [方位角; 距离]
        %       statePre - 预测状态向量
        %       covarPre - 预测协方差矩阵
        %
        %   输出参数：
        %       stateUpd - 更新后的状态向量
        %       covarUpd - 更新后的协方差矩阵
        %       innov    - 新息（测量残差）
        %       innovCov - 新息协方差矩阵
            H = obj.config.getMeasurementJacobian(statePre);
            measPre = obj.config.measurementFunction(statePre);
            R = obj.config.getMeasurementNoiseCov();
            
            innov = measZ - measPre;
            innov(1) = utils.FilterUtils.wrapToPi(innov(1));  % 方位角归一化到[-pi, pi]
            innovCov = H * covarPre * H' + R;
            K = covarPre * H' / innovCov;  % 卡尔曼增益
            stateUpd = statePre + K * innov;
            covarUpd = (eye(obj.stateDim) - K * H) * covarPre;
        end
        
        function [estStates, estCovars, elapsed] = run(obj, meas, x0, P0)
        % RUN  执行滤波器处理整个测量序列。
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
            
            [state, covar] = obj.init(x0, P0);
            tic;
            for k = 1:nSteps
                [state, covar] = obj.predict(state, covar);
                [state, covar] = obj.update(meas(:, k), state, covar);
                estStates(:, k) = state;
                estCovars(:, :, k) = covar;
            end
            elapsed = toc;
        end
    end

    methods (Access = private)
        function F = getSingerTransitionMatrix(obj, T, beta)
        % GETSINGERTRANSITIONMATRIX  获取Singer/CS模型的状态转移矩阵。
        %
        %   输入参数：
        %       T    - 采样间隔
        %       beta - exp(-alpha*T)，用于Singer模型
        %
        %   输出参数：
        %       F - 状态转移矩阵 [6 x 6]
            alpha = 1 / obj.tau;
            Fx = [1, T, (T - (1-beta)/alpha)/alpha;
                  0, 1, (1-beta)/alpha;
                  0, 0, beta];
            F = blkdiag(Fx, Fx);
        end
        
        function Qd = getSingerProcessNoise(obj, T, alpha, beta, sigma_a)
        % GETSINGERPROCESSNOISE  获取Singer/CS模型的过程噪声协方差。
        %   基于Singer一阶时间相关模型的离散化过程噪声。
        %
        %   输入参数：
        %       T       - 采样间隔
        %       alpha   - 1/tau，时间常数的倒数
        %       beta    - exp(-alpha*T)
        %       sigma_a - 加速度标准差
        %
        %   输出参数：
        %       Qd - 离散过程噪声协方差矩阵 [6 x 6]
            a = alpha;
            b = beta;
            sa2 = sigma_a^2;
            
            % Singer模型离散化过程噪声的解析解
            q11 = (2*a*T - 3 + 4*b - b^2 + 2*b^2*T*a) / (2*a^5);
            q12 = (1 - 2*b + b^2) / (2*a^4);
            q13 = (1 - b) / (a^3);
            q22 = (4*b - 3 - b^2 + 2*a*T) / (2*a^3);
            q23 = (1 - b)^2 / (a^2);
            q33 = (1 - b^2) / (2*a);
            
            Qx = sa2 * [q11, q12, q13; q12, q22, q23; q13, q23, q33];
            Qd = blkdiag(Qx, Qx);
        end
    end

    methods (Static)
        function name = getFilterType()
            name = 'MotionModelEKF';
        end
        
        function description = getFilterDescription()
            description = 'Unified Extended Kalman Filter for motion models';
        end
    end
end
