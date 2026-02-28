classdef Config
% TBD.CONFIG  Configuration parameters for Track-Before-Detect algorithms.
%   Provides simulation parameters for TBD algorithms including
%   image-domain scene parameters, target model, and filter tuning.
%
%   Motion Models:
%       'CV' - Constant Velocity (default)
%       'CA' - Constant Acceleration
%       'CT' - Coordinated Turn
%       'Maneuver' - Segmented maneuver trajectory
%
%   Usage:
%       cfg = tbd.Config();           % Default configuration (CV model)
%       cfg = tbd.Config('motionModel', 'CA');  % Constant Acceleration
%       cfg = tbd.Config('numFrames', 100, 'gridSize', [200, 200]);

    properties
        gridSize = [100, 100]
        numFrames = 50
        dt = 1
        motionModel = 'CV'
        initPos = [15, 20]
        initVel = [1.2, 0.8]
        initAccel = [0, 0]
        turnRate = 0.05
        targetRadius = 3
        amplitude = 3.0
        noiseStd = 1.0
        maxSpeed = 3
        numParticles = 500
        procNoisePos = 2.0
        procNoiseVel = 0.5
        procNoiseAmp = 0.3
        rngSeed = 2024
        maneuverSegments
    end

    methods

        function obj = Config(varargin)
        % CONFIG  Create TBD configuration with optional parameters.
            if nargin > 0
                for i = 1:2:length(varargin)
                    if isprop(obj, varargin{i})
                        obj.(varargin{i}) = varargin{i+1};
                    end
                end
            end
            
            obj.motionModel = upper(obj.motionModel);
            
            if isempty(obj.maneuverSegments) && strcmp(obj.motionModel, 'MANEUVER')
                obj.maneuverSegments = obj.getDefaultManeuverSegments();
            end
        end

        function snr = getSnr(obj)
        % GETSNR  Compute approximate SNR in dB.
            snr = 20 * log10(obj.amplitude / obj.noiseStd);
        end

        function display(obj)
        % DISPLAY  Display configuration summary.
            fprintf('=== TBD Configuration ===\n');
            fprintf('  Grid: %dx%d  |  Frames: %d  |  dt: %.1f\n', ...
                    obj.gridSize(1), obj.gridSize(2), obj.numFrames, obj.dt);
            fprintf('  Motion Model: %s\n', obj.motionModel);
            fprintf('  SNR: %.1f dB\n', obj.getSnr());
            fprintf('  Target radius: %d  |  Amplitude: %.1f\n', ...
                    obj.targetRadius, obj.amplitude);
            fprintf('  Initial pos: (%.1f, %.1f)  |  Vel: (%.1f, %.1f)\n', ...
                    obj.initPos(1), obj.initPos(2), obj.initVel(1), obj.initVel(2));
            if strcmp(obj.motionModel, 'CA') || strcmp(obj.motionModel, 'MANEUVER')
                fprintf('  Initial accel: (%.1f, %.1f)\n', obj.initAccel(1), obj.initAccel(2));
            end
            if strcmp(obj.motionModel, 'CT')
                fprintf('  Turn rate: %.3f rad/frame\n', obj.turnRate);
            end
            fprintf('  Particles (PF): %d  |  Max speed (DP): %d\n', ...
                    obj.numParticles, obj.maxSpeed);
        end
        
        function segments = getDefaultManeuverSegments(obj)
        % GETDEFAULTMANEUVERSEGMENTS  Get default maneuver segment configuration.
            nF = obj.numFrames;
            segmentLen = floor(nF / 4);
            segments = struct();
            
            segments(1).startFrame = 1;
            segments(1).endFrame = segmentLen;
            segments(1).type = 'CV';
            segments(1).accel = [0, 0];
            
            segments(2).startFrame = segmentLen + 1;
            segments(2).endFrame = 2 * segmentLen;
            segments(2).type = 'CA';
            segments(2).accel = [0.1, 0.05];
            
            segments(3).startFrame = 2 * segmentLen + 1;
            segments(3).endFrame = 3 * segmentLen;
            segments(3).type = 'CT';
            segments(3).turnRate = obj.turnRate;
            
            segments(4).startFrame = 3 * segmentLen + 1;
            segments(4).endFrame = nF;
            segments(4).type = 'CV';
            segments(4).accel = [0, 0];
        end
        
        function stateDim = getStateDim(obj)
        % GETSTATEDIM  Get state dimension based on motion model.
            switch obj.motionModel
                case 'CV'
                    stateDim = 4;
                case {'CA', 'CT', 'Maneuver'}
                    stateDim = 5;
                otherwise
                    stateDim = 4;
            end
        end
    end

    methods (Static)
        function models = getAvailableModels()
        % GETAVAILABLEMODELS  List available motion models.
            models = {'CV', 'CA', 'CT', 'MANEUVER'};
        end
    end
end
