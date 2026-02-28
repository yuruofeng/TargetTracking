classdef Config
% DBT.CONFIG  Configuration parameters for Detect-Before-Track algorithms.
%   Provides default simulation parameters for DBT filters including
%   model dimensions, noise covariances, and filter tuning parameters.
%
%   Usage:
%       cfg = dbt.Config();           % Default configuration
%       cfg = dbt.Config('numSteps', 200, 'dt', 0.5);  % Custom parameters
%
%   Properties:
%       numSteps     - Total observation frames (default: 100)
%       dt           - Sampling interval [s] (default: 1)
%       stateDim     - State dimension (default: 5 for CT model)
%       procNoiseDim - Process noise dimension (default: 3)
%       measDim      - Measurement dimension (default: 2)
%       procDriveMat - Process noise driving matrix
%       procNoiseCov - Process noise covariance
%       measNoiseCov - Measurement noise covariance
%       ukfAlpha     - UKF primary scaling parameter
%       ukfBeta      - UKF Gaussian prior parameter
%       ukfKappa     - UKF secondary scaling parameter
%       numParticles - Number of particles for PF
%       initState    - Initial state for trajectory generation

    properties
        numSteps = 100
        dt = 1
        stateDim = 5
        procNoiseDim = 3
        measDim = 2
        procDriveMat
        procNoiseCov
        measNoiseCov
        ukfAlpha = 1e-3
        ukfBeta = 2
        ukfKappa = 0
        numParticles = 500
        initState
    end

    methods

        function obj = Config(varargin)
        % CONFIG  Create DBT configuration with optional parameters.
        %   obj = Config() creates default configuration.
        %   obj = Config('param1', value1, ...) creates configuration with
        %   specified parameters.
            obj.initState = [0; 6; 0; 1; 0.02];
            T = obj.dt;
            obj.procDriveMat = [T^2/2 0 0; T 0 0; ...
                                0 T^2/2 0; 0 T 0; ...
                                0 0 1];
            obj.procNoiseCov = diag([1, 1, 4e-4]);
            obj.measNoiseCov = diag([pi/90, 5]);

            if nargin > 0
                for i = 1:2:length(varargin)
                    if isprop(obj, varargin{i})
                        obj.(varargin{i}) = varargin{i+1};
                    end
                end
            end
            obj = obj.updateDerivedParams();
        end

        function obj = updateDerivedParams(obj)
        % UPDATEDERIVEDPARAMS  Update derived parameters after changes.
            T = obj.dt;
            obj.procDriveMat = [T^2/2 0 0; T 0 0; ...
                                0 T^2/2 0; 0 T 0; ...
                                0 0 1];
        end

        function Qd = getProcessNoiseCov(obj)
        % GETPROCESSNOISECOV  Get discrete process noise covariance.
            Qd = obj.procDriveMat * obj.procNoiseCov * obj.procDriveMat';
        end

        function R = getMeasurementNoiseCov(obj)
        % GETMEASUREMENTNOISECOV  Get measurement noise covariance.
            R = obj.measNoiseCov * obj.measNoiseCov';
        end

        function display(obj)
        % DISPLAY  Display configuration summary.
            fprintf('=== DBT Configuration ===\n');
            fprintf('  Steps: %d  |  dt: %.2f s\n', obj.numSteps, obj.dt);
            fprintf('  State dim: %d  |  Meas dim: %d\n', obj.stateDim, obj.measDim);
            fprintf('  Particles (PF): %d\n', obj.numParticles);
            fprintf('  UKF params: alpha=%.1e, beta=%.1f, kappa=%.1f\n', ...
                    obj.ukfAlpha, obj.ukfBeta, obj.ukfKappa);
        end

    end
end
