classdef MotionModelConfig
% DBT.MOTIONMODELCONFIG  Unified configuration for motion models.
%   Provides a single configuration class supporting CV, CA, CT, Singer,
%   and CS motion models through a modelType parameter.
%
%   Supported Models:
%       'CV'     - Constant Velocity [x, vx, y, vy] (4D)
%       'CA'     - Constant Acceleration [x, vx, ax, y, vy, ay] (6D)
%       'CT'     - Coordinated Turn [x, vx, y, vy, omega] (5D)
%       'Singer' - Singer acceleration [x, vx, ax, y, vy, ay] (6D)
%       'CS'     - Current Statistics [x, vx, ax, y, vy, ay] (6D)
%
%   Usage:
%       cfg = dbt.MotionModelConfig('CV');
%       cfg = dbt.MotionModelConfig('CA', 'dt', 0.5);
%       cfg = dbt.MotionModelConfig('Singer', 'tau', 10, 'sigma_a', 5);

    properties
        modelType
        numSteps = 100
        dt = 1
        stateDim
        measDim = 2
        procNoiseStd = 1
        measNoiseCov
        initState
        tau = 5
        sigma_a = 3
        sigma_a_max = 10
        p_accel = 4
        procDriveMat
        procNoiseCov
    end

    properties (Dependent)
        modelName
    end

    methods
        function obj = MotionModelConfig(modelType, varargin)
        % MOTIONMODELCONFIG  Create motion model configuration.
        %   obj = MotionModelConfig(modelType) creates configuration for
        %   specified model type.
        %   obj = MotionModelConfig(modelType, 'param', value, ...) sets
        %   additional parameters.
            if nargin < 1
                modelType = 'CV';
            end
            
            obj.modelType = upper(modelType);
            obj = obj.initializeDefaults();
            
            for i = 1:2:length(varargin)
                if isprop(obj, varargin{i})
                    obj.(varargin{i}) = varargin{i+1};
                end
            end
            obj = obj.updateDerivedParams();
        end
        
        function name = get.modelName(obj)
            name = obj.modelType;
        end
    end

    methods (Access = private)
        function obj = initializeDefaults(obj)
        % INITIALIZEDEFAULTS  Set default values based on model type.
            obj.measNoiseCov = diag([pi/90, 5]);
            
            switch obj.modelType
                case 'CV'
                    obj.stateDim = 4;
                    obj.initState = [0; 10; 0; 5];
                    
                case 'CA'
                    obj.stateDim = 6;
                    obj.procNoiseStd = 0.5;
                    obj.initState = [0; 10; 0; 0; 5; 0];
                    
                case 'CT'
                    obj.stateDim = 5;
                    obj.initState = [0; 6; 0; 1; 0.02];
                    T = obj.dt;
                    obj.procDriveMat = [T^2/2 0 0; T 0 0; ...
                                        0 T^2/2 0; 0 T 0; ...
                                        0 0 1];
                    obj.procNoiseCov = diag([1, 1, 4e-4]);
                    
                case 'SINGER'
                    obj.stateDim = 6;
                    obj.initState = [0; 10; 0; 0; 5; 0];
                    
                case 'CS'
                    obj.stateDim = 6;
                    obj.initState = [0; 10; 0; 0; 5; 0];
                    
                otherwise
                    error('dbt:UnknownModel', 'Unknown motion model: %s', obj.modelType);
            end
        end
    end

    methods
        function obj = updateDerivedParams(obj)
        % UPDATEDERIVEDPARAMS  Update derived parameters after changes.
            T = obj.dt;
            
            switch obj.modelType
                case 'CT'
                    obj.procDriveMat = [T^2/2 0 0; T 0 0; ...
                                        0 T^2/2 0; 0 T 0; ...
                                        0 0 1];
            end
        end
        
        function Qd = getProcessNoiseCov(obj)
        % GETPROCESSNOISECOV  Get discrete process noise covariance.
            T = obj.dt;
            
            switch obj.modelType
                case 'CV'
                    q = obj.procNoiseStd^2;
                    Qd = q * [T^4/4, T^3/2, 0, 0;
                              T^3/2, T^2,   0, 0;
                              0,     0,   T^4/4, T^3/2;
                              0,     0,   T^3/2, T^2];
                              
                case 'CA'
                    q = obj.procNoiseStd^2;
                    Qx = q * [T^5/20, T^4/8, T^3/6;
                              T^4/8,  T^3/3, T^2/2;
                              T^3/6,  T^2/2, T];
                    Qd = blkdiag(Qx, Qx);
                    
                case 'CT'
                    Qd = obj.procDriveMat * obj.procNoiseCov * obj.procDriveMat';
                    
                case {'SINGER', 'CS'}
                    alpha = 1 / obj.tau;
                    beta = exp(-alpha * T);
                    sa2 = obj.sigma_a^2;
                    
                    q11 = (2*alpha*T - 3 + 4*beta - beta^2 + 2*beta^2*T*alpha) / (2*alpha^5);
                    q12 = (1 - 2*beta + beta^2) / (2*alpha^4);
                    q13 = (1 - beta) / (alpha^3);
                    q22 = (4*beta - 3 - beta^2 + 2*alpha*T) / (2*alpha^3);
                    q23 = (1 - beta)^2 / (alpha^2);
                    q33 = (1 - beta^2) / (2*alpha);
                    
                    Qx = sa2 * [q11, q12, q13; q12, q22, q23; q13, q23, q33];
                    Qd = blkdiag(Qx, Qx);
                    
                otherwise
                    Qd = [];
            end
        end
        
        function R = getMeasurementNoiseCov(obj)
        % GETMEASUREMENTNOISECOV  Get measurement noise covariance.
            R = obj.measNoiseCov * obj.measNoiseCov';
        end
        
        function F = getStateTransitionMatrix(obj, state)
        % GETSTATETRANSITIONMATRIX  Get state transition matrix.
            T = obj.dt;
            
            switch obj.modelType
                case 'CV'
                    F = kron(eye(2), [1, T; 0, 1]);
                    
                case {'CA', 'SINGER', 'CS'}
                    Fx = [1, T, T^2/2; 0, 1, T; 0, 0, 1];
                    F = blkdiag(Fx, Fx);
                    
                case 'CT'
                    F = utils.MeasurementModel.ctDynamicMatrix(T, state);
                    
                otherwise
                    F = [];
            end
        end
        
        function H = getMeasurementJacobian(obj, state)
        % GETMEASUREMENTJACOBIAN  Get measurement Jacobian matrix.
            switch obj.modelType
                case 'CV'
                    px = state(1);  py = state(3);
                    r2 = px^2 + py^2;
                    r = sqrt(r2);
                    H = [-py/r2, 0, px/r2, 0;
                          px/r,  0, py/r,  0];
                          
                case {'CA', 'SINGER', 'CS'}
                    px = state(1);  py = state(4);
                    r2 = px^2 + py^2;
                    r = sqrt(r2);
                    H = [-py/r2, 0, 0, px/r2, 0, 0;
                          px/r,  0, 0, py/r,  0, 0];
                          
                case 'CT'
                    H = utils.MeasurementModel.ctMeasJacobian(state);
                    
                otherwise
                    H = [];
            end
        end
        
        function z = measurementFunction(obj, state)
        % MEASUREMENTFUNCTION  Compute predicted measurement.
            switch obj.modelType
                case 'CV'
                    z = [atan2(state(3), state(1)); norm(state([1 3]))];
                case {'CA', 'SINGER', 'CS'}
                    z = [atan2(state(4), state(1)); norm(state([1 4]))];
                case 'CT'
                    z = utils.MeasurementModel.ctMeasFunc(state);
                otherwise
                    z = [];
            end
        end
        
        function u = getControlInput(obj, state, T, alpha, beta)
        % GETCONTROLINPUT  Get control input for CS model.
            if ~strcmp(obj.modelType, 'CS')
                u = zeros(obj.stateDim, 1);
                return;
            end
            
            ax_bar = state(3);
            ay_bar = state(6);
            
            ux = [((1-beta)/alpha - T + alpha*T^2/2), T - (1-beta)/alpha, 1-beta]' * ax_bar;
            uy = [((1-beta)/alpha - T + alpha*T^2/2), T - (1-beta)/alpha, 1-beta]' * ay_bar;
            u = [ux; uy];
        end
        
        function sigma_a = getAdaptiveSigma(obj, ax, ay)
        % GETADAPTIVESIGMA  Get adaptive sigma for CS model.
            if ~strcmp(obj.modelType, 'CS')
                sigma_a = obj.sigma_a;
                return;
            end
            
            a_mag = sqrt(ax^2 + ay^2);
            sigma_a = (obj.p_accel - 1) / obj.p_accel * a_mag + obj.sigma_a_max / obj.p_accel;
        end
    end

    methods (Static)
        function models = getAvailableModels()
        % GETAVAILABLEMODELS  List available motion models.
            models = {'CV', 'CA', 'CT', 'SINGER', 'CS'};
        end
    end
end
