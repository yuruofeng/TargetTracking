classdef UKF < dbt.KalmanFilterBase
% DBT.UKF  Unscented Kalman Filter for Detect-Before-Track.
%   Implements the UKF using Van der Merwe sigma points for tracking
%   targets with CT dynamics and range/azimuth measurements.
%
%   The UKF uses the unscented transformation to propagate mean and
%   covariance through nonlinear functions, avoiding Jacobian computation.
%
%   State vector: [x, vx, y, vy, omega]
%       x, y     - Position coordinates [m]
%       vx, vy   - Velocity components [m/s]
%       omega    - Turn rate [rad/s]
%
%   Measurement: [azimuth, range]
%       azimuth  - Bearing angle [rad]
%       range    - Distance to target [m]
%
%   UKF Parameters:
%       alpha - Spread of sigma points (typical: 1e-3)
%       beta  - Prior knowledge of distribution (2 for Gaussian)
%       kappa - Secondary scaling parameter (typical: 0)
%
%   Usage:
%       cfg = dbt.Config();
%       ukf = dbt.UKF(cfg);
%       [state, covar] = ukf.init(x0, P0);
%       for k = 1:numSteps
%           [state, covar] = ukf.predict(state, covar);
%           [state, covar, innov, innovCov] = ukf.update(meas(:,k), state, covar);
%       end
%
%   Reference:
%       Van der Merwe, R. (2004). Sigma-Point Kalman Filters.
%
%   See also: dbt.KalmanFilterBase, dbt.EKF, dbt.CKF, dbt.Config

    properties (Access = private)
        alpha
        beta
        kappa
    end

    methods
        function obj = UKF(cfg)
        % UKF  Create Unscented Kalman Filter with configuration.
        %   obj = UKF(cfg) initializes with dbt.Config object.
        %   obj = UKF() uses default configuration.
            obj@dbt.KalmanFilterBase(cfg);
            obj.alpha = obj.config.ukfAlpha;
            obj.beta = obj.config.ukfBeta;
            obj.kappa = obj.config.ukfKappa;
        end
    end

    methods
        function [state, covar, weights] = init(obj, x0, P0)
        % INIT  Initialize filter state and covariance.
        %   Returns weighted mean and covariance. Optional third output
        %   returns sigma weights structure for advanced usage.
            state = x0(:);
            covar = obj.ensurePositiveDefinite(P0);
            if nargout > 2
                weights = obj.generateSigmaWeights(state, covar);
            end
        end
        
        function [estStates, estCovars, elapsed] = run(obj, meas, x0, P0)
        % RUN  Execute UKF on measurement sequence.
            nSteps = size(meas, 2);
            estStates = zeros(obj.stateDim, nSteps);
            estCovars = zeros(obj.stateDim, obj.stateDim, nSteps);
            
            [state, covar, ~] = obj.init(x0, P0);
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

    methods (Access = protected)
        function [statePre, covarPre] = predictStep(obj, stateUpd, covarUpd)
        % PREDICTSTEP  UKF prediction using sigma point transformation.
            weights = obj.generateSigmaWeights(stateUpd, covarUpd);
            spPts = weights.spPts;
            nSp = size(spPts, 2);
            
            preSp = zeros(obj.stateDim, nSp);
            for i = 1:nSp
                F = utils.MeasurementModel.ctDynamicMatrix(obj.config.dt, spPts(:, i));
                preSp(:, i) = F * spPts(:, i);
            end
            
            statePre = preSp * weights.wm';
            covarPre = obj.processNoise;
            for i = 1:nSp
                d = preSp(:, i) - statePre;
                covarPre = covarPre + weights.wc(i) * (d * d');
            end
        end
        
        function [stateUpd, covarUpd, innov, innovCov] = updateStep(obj, measZ, statePre, covarPre)
        % UPDATESTEP  UKF update using sigma point transformation.
            weights = obj.generateSigmaWeights(statePre, covarPre);
            spPts = weights.spPts;
            nSp = size(spPts, 2);
            
            measSp = zeros(obj.measDim, nSp);
            for i = 1:nSp
                measSp(:, i) = utils.MeasurementModel.ctMeasFunc(spPts(:, i));
            end
            measPre = measSp * weights.wm';
            
            innovCov = obj.measurementNoise;
            Cxz = zeros(obj.stateDim, obj.measDim);
            for i = 1:nSp
                dz = measSp(:, i) - measPre;
                dx = spPts(:, i) - statePre;
                innovCov = innovCov + weights.wc(i) * (dz * dz');
                Cxz = Cxz + weights.wc(i) * (dx * dz');
            end
            
            innov = measZ - measPre;
            innov = obj.wrapAngleInnovation(innov, 1);
            
            K = Cxz / innovCov;
            stateUpd = statePre + K * innov;
            covarUpd = covarPre - K * innovCov * K';
        end
    end

    methods (Access = private)
        function weights = generateSigmaWeights(obj, state, covar)
        % GENERATESIGMAWEIGHTS  Generate sigma points and weights.
            [w, spPts] = ...
                utils.FilterUtils.generateSigmaPoints(state, covar, obj.stateDim, ...
                    obj.alpha, obj.beta, obj.kappa);
            weights.wm = w(1, :);
            weights.wc = w(2, :);
            weights.spPts = spPts;
        end
    end

    methods (Static)
        function name = getFilterType()
            name = 'UKF';
        end
        
        function description = getFilterDescription()
            description = 'Unscented Kalman Filter with sigma point transformation';
        end
    end
end
