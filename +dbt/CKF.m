classdef CKF < dbt.KalmanFilterBase
% DBT.CKF  Cubature Kalman Filter for Detect-Before-Track.
%   Implements the CKF using third-degree cubature points for tracking
%   targets with CT dynamics and range/azimuth measurements.
%
%   The CKF uses cubature points (2n points for n-dimensional state) to
%   propagate mean and covariance through nonlinear functions. It is
%   numerically more stable than UKF for high-dimensional systems.
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
%   Usage:
%       cfg = dbt.Config();
%       ckf = dbt.CKF(cfg);
%       [state, covar] = ckf.init(x0, P0);
%       for k = 1:numSteps
%           [state, covar] = ckf.predict(state, covar);
%           [state, covar, innov, innovCov] = ckf.update(meas(:,k), state, covar);
%       end
%
%   Reference:
%       Arasaratnam, I. (2009). Cubature Kalman Filters.
%       IEEE Transactions on Automatic Control, 54(6), 1254-1269.
%
%   See also: dbt.KalmanFilterBase, dbt.EKF, dbt.UKF, dbt.Config

    methods
        function obj = CKF(cfg)
        % CKF  Create Cubature Kalman Filter with configuration.
        %   obj = CKF(cfg) initializes with dbt.Config object.
        %   obj = CKF() uses default configuration.
            obj@dbt.KalmanFilterBase(cfg);
        end
    end

    methods (Access = protected)
        function [statePre, covarPre] = predictStep(obj, stateUpd, covarUpd)
        % PREDICTSTEP  CKF prediction using cubature point transformation.
            [wCp, cpPts] = obj.generateCubaturePoints(stateUpd, covarUpd);
            nCp = size(cpPts, 2);
            
            preCp = zeros(obj.stateDim, nCp);
            for i = 1:nCp
                F = utils.MeasurementModel.ctDynamicMatrix(obj.config.dt, cpPts(:, i));
                preCp(:, i) = F * cpPts(:, i);
            end
            
            statePre = preCp * wCp';
            covarPre = obj.processNoise - statePre * statePre';
            for i = 1:nCp
                covarPre = covarPre + wCp(i) * (preCp(:, i) * preCp(:, i)');
            end
        end
        
        function [stateUpd, covarUpd, innov, innovCov] = updateStep(obj, measZ, statePre, covarPre)
        % UPDATESTEP  CKF update using cubature point transformation.
            [wCp, cpPts] = obj.generateCubaturePoints(statePre, covarPre);
            nCp = size(cpPts, 2);
            
            measCp = zeros(obj.measDim, nCp);
            for i = 1:nCp
                measCp(:, i) = utils.MeasurementModel.ctMeasFunc(cpPts(:, i));
            end
            measPre = measCp * wCp';
            
            innovCov = obj.measurementNoise - measPre * measPre';
            Cxz = -statePre * measPre';
            for i = 1:nCp
                innovCov = innovCov + wCp(i) * (measCp(:, i) * measCp(:, i)');
                Cxz = Cxz + wCp(i) * (cpPts(:, i) * measCp(:, i)');
            end
            
            innov = measZ - measPre;
            innov = obj.wrapAngleInnovation(innov, 1);
            
            K = Cxz / innovCov;
            stateUpd = statePre + K * innov;
            covarUpd = covarPre - K * innovCov * K';
        end
    end

    methods (Access = private)
        function [wCp, cpPts] = generateCubaturePoints(obj, state, covar)
        % GENERATECUBATUREPOINTS  Generate cubature points and weights.
            [wCp, cpPts] = utils.FilterUtils.generateCubaturePoints(state, covar, obj.stateDim);
        end
    end

    methods (Static)
        function name = getFilterType()
            name = 'CKF';
        end
        
        function description = getFilterDescription()
            description = 'Cubature Kalman Filter with cubature point transformation';
        end
    end
end
