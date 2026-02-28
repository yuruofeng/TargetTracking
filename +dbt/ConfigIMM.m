classdef ConfigIMM
% DBT.CONFIGIMM  Configuration for Interacting Multiple Model (IMM) filter.
%   Provides configuration for IMM algorithm including model selection,
%   transition probabilities, and initial model probabilities.
%
%   Default configuration uses CV, CA, and CT models.
%
%   Usage:
%       cfg = dbt.ConfigIMM();
%       cfg = dbt.ConfigIMM('dt', 0.5);
%       cfg = dbt.ConfigIMM('models', {'CV', 'CA', 'CT'});

    properties
        numSteps = 100
        dt = 1
        outputStateDim = 6
        models
        modelConfigs
        modelStateDims
        transProb
        initProbs
        measNoiseCov
        initState
    end

    methods
        function obj = ConfigIMM(varargin)
        % CONFIGIMM  Create IMM configuration.
        %   obj = ConfigIMM() creates default CV-CA-CT IMM configuration.
        %   obj = ConfigIMM('param', value, ...) sets additional parameters.
            obj.initState = [0; 10; 0; 0; 5; 0];
            obj.measNoiseCov = diag([pi/90, 5]);
            
            obj.modelConfigs = {
                dbt.MotionModelConfig('CV', 'dt', obj.dt),
                dbt.MotionModelConfig('CA', 'dt', obj.dt),
                dbt.MotionModelConfig('CT', 'dt', obj.dt)
            };
            
            obj.models = {
                dbt.MotionModelEKF(obj.modelConfigs{1}),
                dbt.MotionModelEKF(obj.modelConfigs{2}),
                dbt.MotionModelEKF(obj.modelConfigs{3})
            };
            
            obj.modelStateDims = [4, 6, 5];
            
            obj.transProb = [0.8, 0.1, 0.1;
                             0.1, 0.8, 0.1;
                             0.1, 0.1, 0.8];
            
            obj.initProbs = [1/3, 1/3, 1/3];
            
            for i = 1:2:length(varargin)
                if isprop(obj, varargin{i})
                    obj.(varargin{i}) = varargin{i+1};
                end
            end
            obj = obj.updateDerivedParams();
        end
        
        function obj = updateDerivedParams(obj)
        % UPDATEDERIVEDPARAMS  Update derived parameters after changes.
            for i = 1:length(obj.modelConfigs)
                obj.modelConfigs{i}.dt = obj.dt;
            end
        end
        
        function R = getMeasurementNoiseCov(obj)
        % GETMEASUREMENTNOISECOV  Get measurement noise covariance.
            R = obj.measNoiseCov * obj.measNoiseCov';
        end
    end

    methods (Static)
        function cfg = createCustom(models, transProb, initProbs, dt)
        % CREATECUSTOM  Create custom IMM configuration.
        %   cfg = ConfigIMM.createCustom(models, transProb, initProbs, dt)
        %   creates IMM configuration with specified models.
            if nargin < 4
                dt = 1;
            end
            if nargin < 3
                nModels = length(models);
                initProbs = ones(1, nModels) / nModels;
            end
            if nargin < 2
                nModels = length(models);
                transProb = 0.9 * eye(nModels) + 0.1 / nModels * ones(nModels);
                transProb = transProb ./ sum(transProb, 2);
            end
            
            nModels = length(models);
            cfg = dbt.ConfigIMM();
            cfg.dt = dt;
            cfg.transProb = transProb;
            cfg.initProbs = initProbs;
            
            cfg.modelConfigs = cell(nModels, 1);
            cfg.models = cell(nModels, 1);
            cfg.modelStateDims = zeros(nModels, 1);
            
            for i = 1:nModels
                cfg.modelConfigs{i} = dbt.MotionModelConfig(models{i}, 'dt', dt);
                cfg.models{i} = dbt.MotionModelEKF(cfg.modelConfigs{i});
                cfg.modelStateDims(i) = cfg.modelConfigs{i}.stateDim;
            end
        end
    end
end
