classdef (Abstract) KalmanFilterBase < dbt.BaseFilter
% DBT.KALMANFILTERBASE  Abstract base class for Kalman-type filters.
%   Provides common functionality for Extended Kalman Filter (EKF),
%   Unscented Kalman Filter (UKF), and Cubature Kalman Filter (CKF).
%
%   This class implements the standard Kalman filter interface with
%   abstract methods for generating sigma/cubature points and computing
%   predicted measurements.
%
%   Subclasses:
%       EKF  - Extended Kalman Filter (linearization via Jacobian)
%       UKF  - Unscented Kalman Filter (sigma point transformation)
%       CKF  - Cubature Kalman Filter (cubature point transformation)
%
%   See also: dbt.BaseFilter, dbt.EKF, dbt.UKF, dbt.CKF

    properties (Access = protected)
        processNoise
        measurementNoise
    end

    properties (Constant)
        MIN_COVARIANCE_EIGENVALUE = 1e-10
    end

    methods
        function obj = KalmanFilterBase(cfg)
        % KALMANFILTERBASE  Create base Kalman filter.
        %   obj = KalmanFilterBase(cfg) initializes with configuration.
            obj.setConfig(cfg);
            obj.processNoise = obj.config.procDriveMat * ...
                               obj.config.procNoiseCov * ...
                               obj.config.procDriveMat';
            obj.measurementNoise = obj.config.getMeasurementNoiseCov();
        end
    end

    methods
        function [state, covar] = init(obj, x0, P0)
        % INIT  Initialize filter state and covariance.
            state = x0(:);
            covar = obj.ensurePositiveDefinite(P0);
        end
        
        function [statePre, covarPre] = predict(obj, stateUpd, covarUpd)
        % PREDICT  Kalman filter prediction step.
            [statePre, covarPre] = obj.predictStep(stateUpd, covarUpd);
            covarPre = obj.ensurePositiveDefinite(covarPre);
        end
        
        function [stateUpd, covarUpd, innov, innovCov] = update(obj, measZ, statePre, covarPre, varargin)
        % UPDATE  Kalman filter update step.
            [stateUpd, covarUpd, innov, innovCov] = obj.updateStep(measZ, statePre, covarPre);
            covarUpd = obj.ensurePositiveDefinite(covarUpd);
        end
        
        function [estStates, estCovars, elapsed] = run(obj, meas, x0, P0)
        % RUN  Execute filter on measurement sequence.
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

    methods (Abstract, Access = protected)
        [statePre, covarPre] = predictStep(obj, stateUpd, covarUpd)
        [stateUpd, covarUpd, innov, innovCov] = updateStep(obj, measZ, statePre, covarPre)
    end

    methods (Access = protected)
        function covar = ensurePositiveDefinite(obj, covar)
        % ENSUREPOSITIVEDEFINITE  Ensure covariance matrix is positive definite.
            eigvals = eig(covar);
            if any(eigvals <= 0)
                minEig = abs(min(eigvals)) + obj.MIN_COVARIANCE_EIGENVALUE;
                covar = covar + minEig * eye(size(covar));
            end
        end
        
        function innov = wrapAngleInnovation(obj, innov, angleIdx)
        % WRAPANGLEINNOVATION  Wrap angle innovation to [-pi, pi].
            if nargin < 3
                angleIdx = 1;
            end
            innov(angleIdx) = utils.FilterUtils.wrapToPi(innov(angleIdx));
        end
        
        function K = computeKalmanGain(obj, covarPre, H, innovCov)
        % COMPUTEKALMANGAIN  Compute Kalman gain matrix.
            K = covarPre * H' / innovCov;
        end
    end

    methods (Static)
        function name = getFilterType()
            name = 'KalmanFilterBase';
        end
        
        function description = getFilterDescription()
            description = 'Base class for Kalman-type filters';
        end
    end
end
