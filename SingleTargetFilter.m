classdef SingleTargetFilter
% ================ code information =================
% This is a matlab value class for single target Bayesian filter, consisting of
% Kalman Filter, Extended Kalman Filter, Unscented Kalman Filter, Cubature Kalman Filter
% and Particle Filter. Constant Turn Rate Model is used as the dynamic
% model and the sensor can provide radial distance and
% corresponding azimuth of the target.
% P.S. the resampling algorithm for particle filter is a copy from a open source project.
% 
% ==================================================
% Version 0.1 ---> 2022.04.29
% Version 0.2 ---> 2022.05.22
% Version 0.3 ---> 2022.05.31
% coded by YU Ruofeng
% ============ reference ===========================
% [1] 何友. 雷达数据处理及应用[M]
% 
    properties
        K; % 观测总帧数
        T; % 滤波间隔
        % ====== 动态模型参数 =======
        targetStateDim; % 状态维数
        sigma_process;  % 过程噪声协方差   
        processNoiseDim;% 过程噪声维数
        Q;              % 过程噪声驱动矩阵
        % ====== 测量模型参数 =======
        sigma_measNoise;% 测量噪声协方差
        MeasNoiseDim;   % 测量噪声维数
        MeasDim;        % 测量数据维数
        % ====== 交互参数 =======
        truth_X;        % 真实航迹
        meas;           % 观测数据
        % ====== UKF参数 ========
        alpha = 1e-3;   % 0 <  alpha <= 1     (1e-3)
        beta = 2;       % 0 <= beta           (2)
        kappa = 0;      % 0 <= kappa <= 3     (0)
        % ====== PF参数 =========
        particle_number = 500;  % 粒子数
    end
    % 方法
    methods
        function obj = SingleTargetFilter
            % 构造函数
            % ============== 参数初始化 ===============
            obj.K = 100;
            obj.T = 1;
            obj.targetStateDim = 5;
            obj.processNoiseDim = 3;
            obj.MeasNoiseDim = 2;
            obj.MeasDim = 2;
            obj.sigma_process = diag([1, 1, 4e-4]);
            obj.sigma_measNoise = diag([pi/90;5]);
            obj.Q = [obj.T^2/2 0 0; obj.T 0 0; 0 obj.T^2/2 0; 0 obj.T 0; 0 0 1];
        end
        % ============== 生成二维航迹和观测 ===============
        function obj = gen_model(obj)
            target_state  = [ 0; 6; 0; 1; 0.02 ];
            for k = 1:obj.K
                target_state = obj.CT_dynamin_model(obj.T,target_state)*target_state;
                obj.truth_X(:,k) = target_state;
                % 测量
                obj.meas(:,k) = [atan2(target_state(3,:),target_state(1,:)); 
                                sqrt(sum(target_state([1 3],:).^2,1))]...
                    +obj.sigma_measNoise*randn(obj.MeasNoiseDim,size(target_state,2));
            end
        end      

        %% ============== 扩展卡尔曼滤波EKF ===============
        % 预测
        function [statePre,covarPre] = EKFpredict(obj,statePrior,covarPrior)
            % 将目标动态模型在先验点处进行Taylor展开
            % 等价于求解非线性变换函数的Jacobian矩阵
            F = obj.CT_dynamin_model(obj.T,statePrior);
            % 状态预测
            statePre = F*statePrior;
            % 协方差预测
            covarPre = F*covarPrior*F'+obj.Q*sqrtm(obj.sigma_process)*obj.Q';
        end
        
        % 更新
        function [stateUpd,covarUpd] = EKFupdate(obj,measZ,statePre,covarPre)
            % 将观测模型在预测点处进行Taylor展开
            % 等价于求解非线性测量函数的Jacobian矩阵
            H = obj.CT_measurement_model(statePre);
            % 测量预测（非线性）
            mea_Pre = [atan2(statePre(3,:),statePre(1,:)); sqrt(sum(statePre([1 3],:).^2,1))];
            % 卡尔曼增益
            K_gain = covarPre*H'*inv(H*covarPre*H'+obj.sigma_measNoise*obj.sigma_measNoise'); %#ok<MINV> 
            % 状态更新
            stateUpd = statePre+K_gain*(measZ-mea_Pre);
            % 协方差更新
            covarUpd = (eye(obj.targetStateDim)-K_gain*H)*covarPre;
        end

        %% ============== 无迹卡尔曼滤波UKF ===============
        % 生成sigma点
        function [weightState_SP,state_SP] = gen_sigmaPoint(obj,statePrior_SP,covarPrior_SP)     
            lambda = obj.alpha^2*(obj.targetStateDim+obj.kappa)-obj.targetStateDim;
            % 协方差矩阵方根
            cholesky_decom = obj.cholPSD(covarPrior_SP*(obj.targetStateDim+lambda));
            % sigma点状态
            state_SP = zeros(obj.targetStateDim,2*obj.targetStateDim);
            for iSigmaPoint = 1:obj.targetStateDim
                state_SP(:,iSigmaPoint) = statePrior_SP+cholesky_decom(:,iSigmaPoint);
                state_SP(:,iSigmaPoint+obj.targetStateDim) = statePrior_SP-cholesky_decom(:,iSigmaPoint);
            end
            state_SP = cat(2,statePrior_SP,state_SP);
            % sigma点权重
            weightState_SP(1,:) = cat(2,lambda/(obj.targetStateDim+lambda),1/(2*(obj.targetStateDim+lambda)).*ones(1,2*obj.targetStateDim));
            weightState_SP(2,:) = cat(2,lambda/(obj.targetStateDim+lambda)+(1-obj.alpha^2+obj.beta),1/(2*(obj.targetStateDim+lambda)).*ones(1,2*obj.targetStateDim));
        end
        % 预测
        function [weight_SP,statePre,covarPre] = UKFpredict(obj,statePrior,covarPrior)
            % 获取先验状态的sigma点
            [weight_SP,state_SP] = obj.gen_sigmaPoint(statePrior,covarPrior);
            % sigma点状态预测
            sigmaPoint_num = size(state_SP,2);
            statePre_SP = zeros(obj.targetStateDim,sigmaPoint_num);
            % 逐点预测
            for i = 1:sigmaPoint_num
                statePrior = state_SP(:,i);
                statePre_SP(:,i) = obj.CT_dynamin_model(obj.T,statePrior)*statePrior;
            end
            % 加权求和
            statePre = statePre_SP*weight_SP(1,:)';
            % 预测协方差
            covarPre = zeros(obj.targetStateDim,obj.targetStateDim);
            for i = 1:sigmaPoint_num
                covarPre = covarPre+weight_SP(2,i)*(statePre-statePre_SP(:,i))*(statePre-statePre_SP(:,i))';
            end
            covarPre = covarPre+obj.Q*sqrtm(obj.sigma_process)*obj.Q';
%             state_weightedBias = (statePre_SP-statePre).*(ones(obj.targetStateDim,1)*sqrt(weightCovar_SP));
%             covarPre = state_weightedBias*state_weightedBias'+obj.Q*sqrtm(obj.sigma_process)*obj.Q';
        end
        % 更新
        function [stateUpd,covarUpd] = UKFupdate(obj,measZ,statePre,covarPre,weight_SP)
            % 获取先验状态的sigma点
            [~,state_SP] = obj.gen_sigmaPoint(statePre,covarPre);
            % 观测预测
            measPre_SP = [atan2(state_SP(3,:),state_SP(1,:)); sqrt(sum(state_SP([1 3],:).^2,1))];
            % 观测均值
            measPre = measPre_SP*weight_SP(1,:)';
            % 观测预测协方差
            covarMeas = zeros(obj.MeasDim,obj.MeasDim);
            sigmaPoint_num = size(state_SP,2);
            for i = 1:sigmaPoint_num
                covarMeas = covarMeas+weight_SP(2,i)*(measPre-measPre_SP(:,i))*(measPre-measPre_SP(:,i))';
            end
            covarMeas = covarMeas+obj.sigma_measNoise*obj.sigma_measNoise';

            covar_StateMeas = zeros(obj.targetStateDim,obj.MeasDim);
            for i = 1:sigmaPoint_num
                covar_StateMeas = covar_StateMeas+weight_SP(2,i)*(statePre-state_SP(:,i))*(measPre-measPre_SP(:,i))';
            end
            % 卡尔曼增益
            K_gain = covar_StateMeas*inv(covarMeas); %#ok<MINV> 
            % 状态更新
            stateUpd = statePre+K_gain*(measZ-measPre);
            % 协方差更新
            covarUpd = covarPre-K_gain*covarMeas*K_gain';
        end

        %% ============== 容积卡尔曼滤波CKF ===============
        % 生成cubature点
        function [weight_CP,state_CP] = gen_cubaturePoint(obj,statePrior,covarPrior)     
            cubaturePointNum = 2*obj.targetStateDim;
            varphi = sqrt(obj.targetStateDim)*cat(2,eye(obj.targetStateDim),-1*eye(obj.targetStateDim));
            % 协方差矩阵方根
            choleskyDecompose = obj.cholPSD(covarPrior);
            % cubature点状态
            state_CP = zeros(obj.targetStateDim,cubaturePointNum);
            for iCubaturePoint = 1:cubaturePointNum
                state_CP(:,iCubaturePoint) = statePrior+choleskyDecompose*varphi(:,iCubaturePoint);
            end
            % cubature点权重
            weight_CP = 1/cubaturePointNum.*ones(1,cubaturePointNum);
        end
        % 预测
        function [statePre,covarPre] = CKFpredict(obj,statePrior,covarPrior)
            [weight_CP,state_CP] = obj.gen_cubaturePoint(statePrior,covarPrior);
            % cubature点状态预测
            cubaturePoint_num = size(state_CP,2);
            statePre_CP = zeros(obj.targetStateDim,cubaturePoint_num);
            % 逐点预测
            for i = 1:cubaturePoint_num
                statePre_CP(:,i) = obj.CT_dynamin_model(obj.T,state_CP(:,i))*state_CP(:,i);
            end
            % 加权求和
            statePre = statePre_CP*weight_CP';
            % 预测协方差
            covarPre = zeros(obj.targetStateDim,obj.targetStateDim);
            for i = 1:cubaturePoint_num
                covarPre = covarPre+weight_CP(1,i)*statePre_CP(:,i)*statePre_CP(:,i)';
            end
            covarPre = covarPre+obj.Q*sqrtm(obj.sigma_process)*obj.Q'-statePre*statePre';
        end
        % 更新
        function [stateUpd,covarUpd] = CKFupdate(obj,measZ,statePre,covarPre)
            % 获取先验状态的cubature点
            [weight_CP,state_CP] = obj.gen_cubaturePoint(statePre,covarPre);
            % 观测预测
            measPre_CP = [atan2(state_CP(3,:),state_CP(1,:)); sqrt(sum(state_CP([1 3],:).^2,1))];
            % 观测均值
            measPre = measPre_CP*weight_CP';
            % 观测预测协方差
            covarMeas = zeros(obj.MeasDim,obj.MeasDim);
            sigmaPoint_num = size(state_CP,2);
            for i = 1:sigmaPoint_num
                covarMeas = covarMeas+weight_CP(1,i)*measPre_CP(:,i)*measPre_CP(:,i)';
            end
            covarMeas = covarMeas+obj.sigma_measNoise*obj.sigma_measNoise'-measPre*measPre';

            covar_StateMeas = zeros(obj.targetStateDim,obj.MeasDim);
            for i = 1:sigmaPoint_num
                covar_StateMeas = covar_StateMeas+weight_CP(1,i)*state_CP(:,i)*measPre_CP(:,i)';
            end
            covar_StateMeas = covar_StateMeas-statePre*measPre';
            % 卡尔曼增益
            K_gain = covar_StateMeas/covarMeas;
            % 状态更新
            stateUpd = statePre+K_gain*(measZ-measPre);
            % 协方差更新
            covarUpd = covarPre-K_gain*covarMeas*K_gain';
        end

        %% ============== 粒子滤波PF ===============
        function [weight_p, state_p] = particles_init(obj,statePrior,covarPrior)
            state_p = mvnrnd(statePrior,covarPrior,obj.particle_number)';
            weight_p = 1/obj.particle_number*ones(obj.particle_number,1);
        end

        function [weightPre_p,statePre_p] = PFpredict(obj,weightPrior_p,statePrior_p)
            % 根据目标动态模型对粒子进行预测
            statePre_p = zeros(obj.targetStateDim,obj.particle_number);
            for ipart = 1:obj.particle_number
                statePre_p(:,ipart) = obj.CT_dynamin_model(obj.T,statePrior_p(:,ipart))*statePrior_p(:,ipart)...
                    +mvnrnd(zeros(1,obj.targetStateDim),obj.Q*sqrtm(obj.sigma_process)*obj.Q')';
            end
            weightPre_p = weightPrior_p; % 预测步不改变权值
        end

        function [weightUpd_p,stateUpd_p] = PFupdate(obj,measZ,weightPre_p,statePre_p)
            % 观测预测
            measPre_p = [atan2(statePre_p(3,:),statePre_p(1,:)); sqrt(sum(statePre_p([1 3],:).^2,1))];
            % 权值更新（需要根据模型进行调整）
            % 
            weightUpd_p = weightPre_p.*mvnpdf(measPre_p',measZ',obj.sigma_measNoise*obj.sigma_measNoise');
            weightUpd_p = weightUpd_p./sum(weightUpd_p);

            stateUpd_p = statePre_p;
        end

        function [weightUpd_r,stateUpd_r] = resampling(obj,weightUpd_p,stateUpd_p)
            idx = obj.systematicR(1:obj.particle_number,weightUpd_p);
            idx = idx(randperm(numel(idx),obj.particle_number));
            weightUpd_r = ones(obj.particle_number,1)/obj.particle_number;
            stateUpd_r = stateUpd_p(:,idx);
        end
    end

    methods(Static)
        % 动态模型
        % ============== 匀速转弯模型（CT model） ===================
        function F = CT_dynamin_model(T,x_prior)
            omega = x_prior(5);
            F = [1 sin(omega*T)/omega 0 -((1-cos(omega*T))/omega) 0;
                0 cos(omega*T) 0 -sin(omega*T) 0;
                0 (1-cos(omega*T))/omega 1 sin(omega*T)/omega 0;
                0 sin(omega*T) 0 cos(omega*T) 0;
                0 0 0 0 1];
        end
        
        % 观测模型
        % ============== 距离/角度观测 ===================
        % 
        % Jacobian矩阵
        function H = CT_measurement_model(x)
            p = x([1 3],:);
            mag = p(1)^2 + p(2)^2;
            sqrt_mag = sqrt(mag);
            H = [-p(2)/mag  0  p(1)/mag 0  0 ; ...
                 p(1)/sqrt_mag  0  p(2)/sqrt_mag  0  0];
        end

        % ============== 矩阵cholesky分解 ==============
        function Xi = cholPSD(A)
            [~, flag] = chol(A);
            if (flag == 0)
                Xi = (chol(A)).';
            else
                [~,S,V] = svd(A);
                Ss = sqrt(S);
                Xi = V*Ss;
            end
        end

        % =============== 重采样 =====================
        function outIndex = systematicR(inIndex,wn)
            wn=wn';
            [~,N] = size(wn);  % N = Number of particles.
            % SYSTEMATIC RESAMPLING:
            % ====================
            N_children=zeros(1,N);           label=1:1:N;
            s=1/N;            li=0;   % Label of the current point
            % Initialisation
            T=s*rand(1);            j=1;            Q=0;
            % Sampling before
            u=rand(1,N);
            while (T<1)
                if (Q>T)
                    T=T+s;N_children(1,li)=N_children(1,li)+1;
                else
                    % select i uniformly between j and N
                    i=fix((N-j+1)*u(1,j))+j;
                    % save the associate characteristic
                    auxw=wn(1,i);
                    li=label(1,i);
                    % update the cfd
                    Q=Q+auxw;
                    % swap
                    wn(1,i)=wn(1,j);label(1,i)=label(1,j);
                    %wn(1,j)=auxw;
                    %label(1,j)=li;
                    j=j+1;
                end
            end
            index=1;
            for i=1:N
                if (N_children(1,i)>0)
                    for j=index:index+N_children(1,i)-1
                        outIndex(j) = inIndex(i);
                    end
                end
                index= index+N_children(1,i);
            end
        end
    end
end

