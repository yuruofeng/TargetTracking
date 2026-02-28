classdef (Abstract) BaseFilter < handle
% DBT.BASEFILTER  Abstract base class for all tracking filters.
%   Defines the common interface for all filter implementations including
%   Kalman filters (EKF, UKF, CKF) and Particle Filters.
%
%   This class follows the Strategy pattern, allowing different filter
%   implementations to be used interchangeably.
%
%   Interface Methods (must be implemented by subclasses):
%       init    - Initialize filter state
%       predict - Time update (prediction) step
%       update  - Measurement update step
%       run     - Execute filter on measurement sequence
%
%   Subclasses:
%       KalmanFilterBase - Base for EKF, UKF, CKF
%       ParticleFilter   - Particle filter implementation
%
%   See also: dbt.KalmanFilterBase, dbt.EKF, dbt.UKF, dbt.CKF

    properties (Access = protected)
        config
        stateDim
        measDim
    end

    properties (Dependent)
        configuration
        stateDimension
        measurementDimension
    end

    methods
        function val = get.configuration(obj)
            val = obj.config;
        end
        
        function val = get.stateDimension(obj)
            val = obj.stateDim;
        end
        
        function val = get.measurementDimension(obj)
            val = obj.measDim;
        end
    end

    methods (Abstract)
        [state, covar] = init(obj, x0, P0)
        [statePre, covarPre] = predict(obj, stateUpd, covarUpd)
        [stateUpd, covarUpd, innov, innovCov] = update(obj, measZ, statePre, covarPre, varargin)
        [estStates, estCovars, elapsed] = run(obj, meas, x0, P0)
    end

    methods (Access = protected)
        function obj = setConfig(obj, cfg)
            if nargin < 2 || isempty(cfg)
                cfg = dbt.Config();
            end
            obj.config = cfg;
            obj.stateDim = cfg.stateDim;
            obj.measDim = cfg.measDim;
        end
        
        function validateState(obj, state, name)
            if size(state, 1) ~= obj.stateDim
                error('dbt:InvalidState', ...
                    '%s: Expected state dimension %d, got %d', ...
                    name, obj.stateDim, size(state, 1));
            end
        end
        
        function validateCovariance(obj, covar, name)
            if size(covar, 1) ~= obj.stateDim || size(covar, 2) ~= obj.stateDim
                error('dbt:InvalidCovariance', ...
                    '%s: Expected covariance size [%d x %d], got [%d x %d]', ...
                    name, obj.stateDim, obj.stateDim, size(covar, 1), size(covar, 2));
            end
        end
        
        function validateMeasurement(obj, meas, name)
            if size(meas, 1) ~= obj.measDim
                error('dbt:InvalidMeasurement', ...
                    '%s: Expected measurement dimension %d, got %d', ...
                    name, obj.measDim, size(meas, 1));
            end
        end
    end

    methods (Static)
        function name = getFilterType()
            name = 'BaseFilter';
        end
        
        function description = getFilterDescription()
            description = 'Abstract base filter class';
        end
    end
end
