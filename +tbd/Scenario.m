classdef Scenario
% TBD.SCENARIO  Scenario generation for Track-Before-Detect simulations.
%   Generates target trajectories and image-domain measurements
%   with Gaussian PSF for testing TBD algorithms.
%
%   Supported Motion Models:
%       'CV'       - Constant Velocity [row, col, vRow, vCol, amplitude]
%       'CA'       - Constant Acceleration [row, col, vRow, vCol, aRow, aCol, amplitude]
%       'CT'       - Coordinated Turn [row, col, vRow, vCol, omega, amplitude]
%       'Maneuver' - Segmented maneuver trajectory
%
%   Usage:
%       cfg = tbd.Config();
%       scenario = tbd.Scenario(cfg);
%       [scenario, trueState, measData, psfKernel, maneuverLabels] = scenario.generate();
%       scenario.plotTrajectory(trueState, maneuverLabels);

    properties
        config
        trueState
        measData
        psfKernel
        maneuverLabels
    end

    methods

        function obj = Scenario(cfg)
        % SCENARIO  Create scenario generator with configuration.
            if nargin < 1
                cfg = tbd.Config();
            end
            obj.config = cfg;
        end

        function [obj, trueState, measData, psfKernel, maneuverLabels] = generate(obj)
        % GENERATE  Generate trajectory and measurements together.
        %   Returns updated object for value class semantics.
            trueState = obj.generateTrajectory();
            obj.trueState = trueState;
            [measData, psfKernel] = obj.generateMeasurements(trueState);
            obj.measData = measData;
            obj.psfKernel = psfKernel;
            maneuverLabels = obj.maneuverLabels;
        end

        function trueState = generateTrajectory(obj)
        % GENERATETRAJECTORY  Generate target trajectory based on motion model.
            model = obj.config.motionModel;
            
            switch model
                case 'CV'
                    trueState = obj.generateCvTrajectory();
                case 'CA'
                    trueState = obj.generateCaTrajectory();
                case 'CT'
                    trueState = obj.generateCtTrajectory();
                case 'MANEUVER'
                    trueState = obj.generateManeuverTrajectory();
                otherwise
                    trueState = obj.generateCvTrajectory();
            end
            
            obj.trueState = trueState;
        end

        function [measData, psfKernel] = generateMeasurements(obj, trueState)
        % GENERATEMEASUREMENTS  Simulate image-domain measurement frames.
            gs = obj.config.gridSize;
            nF = obj.config.numFrames;
            r  = obj.config.targetRadius;

            psfKernel = utils.MeasurementModel.createPsfKernel(r);

            measData = zeros(gs(1), gs(2), nF);
            for t = 1:nF
                signal = zeros(gs);
                cRow = round(trueState(t, 1));
                cCol = round(trueState(t, 2));
                amp  = trueState(t, end);
                for di = -r:r
                    for dj = -r:r
                        ri = cRow + di;  cj = cCol + dj;
                        if ri >= 1 && ri <= gs(1) && cj >= 1 && cj <= gs(2)
                            signal(ri, cj) = signal(ri, cj) + ...
                                amp * psfKernel(di + r + 1, dj + r + 1);
                        end
                    end
                end
                measData(:, :, t) = signal + obj.config.noiseStd * randn(gs);
            end
            obj.measData = measData;
            obj.psfKernel = psfKernel;
        end
    end

    methods (Access = private)
        function trueState = generateCvTrajectory(obj)
        % GENERATECVTRAJECTORY  Generate constant velocity trajectory.
            nF = obj.config.numFrames;
            trueState = zeros(nF, 5);
            trueState(1, :) = [obj.config.initPos, obj.config.initVel, obj.config.amplitude];
            
            for t = 2:nF
                trueState(t, 1:2) = trueState(t-1, 1:2) + trueState(t-1, 3:4) * obj.config.dt;
                trueState(t, 3:4) = trueState(t-1, 3:4);
                trueState(t, 5)   = obj.config.amplitude;
            end
            
            obj.maneuverLabels = repmat({'CV'}, nF, 1);
        end

        function trueState = generateCaTrajectory(obj)
        % GENERATECATRAJECTORY  Generate constant acceleration trajectory.
            nF = obj.config.numFrames;
            dt = obj.config.dt;
            trueState = zeros(nF, 6);
            
            pos = obj.config.initPos;
            vel = obj.config.initVel;
            acc = obj.config.initAccel;
            
            for t = 1:nF
                trueState(t, :) = [pos, vel, acc, obj.config.amplitude];
                pos = pos + vel * dt + 0.5 * acc * dt^2;
                vel = vel + acc * dt;
            end
            
            obj.maneuverLabels = repmat({'CA'}, nF, 1);
        end

        function trueState = generateCtTrajectory(obj)
        % GENERATECTTRAJECTORY  Generate coordinated turn trajectory.
            nF = obj.config.numFrames;
            dt = obj.config.dt;
            omega = obj.config.turnRate;
            trueState = zeros(nF, 5);
            
            pos = obj.config.initPos(:);
            vel = obj.config.initVel(:);
            
            for t = 1:nF
                trueState(t, :) = [pos', vel', obj.config.amplitude];
                
                cosW = cos(omega * dt);
                sinW = sin(omega * dt);
                
                newVel = [cosW, -sinW; sinW, cosW] * vel;
                newPos = pos + (dt / omega) * [sinW, cosW-1; -(cosW-1), sinW] * vel;
                
                pos = newPos;
                vel = newVel;
            end
            
            obj.maneuverLabels = repmat({'CT'}, nF, 1);
        end

        function trueState = generateManeuverTrajectory(obj)
        % GENERATEMANEUVERTRAJECTORY  Generate segmented maneuver trajectory.
            nF = obj.config.numFrames;
            segments = obj.config.maneuverSegments;
            trueState = zeros(nF, 5);
            labels = cell(nF, 1);
            
            pos = obj.config.initPos(:);
            vel = obj.config.initVel(:);
            velMag = norm(vel);
            
            rowIdx = 1;
            for s = 1:length(segments)
                seg = segments(s);
                segLen = seg.endFrame - seg.startFrame + 1;
                
                for t = 1:segLen
                    frameIdx = seg.startFrame + t - 1;
                    if frameIdx > nF
                        break;
                    end
                    
                    trueState(frameIdx, :) = [pos', vel', obj.config.amplitude];
                    labels{frameIdx} = seg.type;
                    
                    switch seg.type
                        case 'CV'
                            pos = pos + vel * obj.config.dt;
                            
                        case 'CA'
                            acc = seg.accel(:);
                            pos = pos + vel * obj.config.dt + 0.5 * acc * obj.config.dt^2;
                            vel = vel + acc * obj.config.dt;
                            
                        case 'CT'
                            omega = seg.turnRate;
                            cosW = cos(omega * obj.config.dt);
                            sinW = sin(omega * obj.config.dt);
                            
                            rotMat = [cosW, -sinW; sinW, cosW];
                            newVel = rotMat * vel;
                            
                            if abs(omega) > 1e-6
                                deltaPos = (obj.config.dt / omega) * ...
                                    [sinW, cosW-1; -(cosW-1), sinW] * vel;
                            else
                                deltaPos = vel * obj.config.dt;
                            end
                            
                            pos = pos + deltaPos;
                            vel = newVel;
                    end
                end
            end
            
            obj.maneuverLabels = labels;
        end
    end

    methods
        function plotSnapshot(obj, frameIdx)
        % PLOTSNAPSHOT  Visualize a single measurement frame.
            if isempty(obj.measData)
                error('Scenario:NoData', 'Generate measurements first');
            end
            if nargin < 2
                frameIdx = 1;
            end

            figure('Name', 'TBD Measurement Snapshot', 'Color', 'w');
            imagesc(obj.measData(:, :, frameIdx));
            colormap(gca, 'parula'); colorbar;
            hold on;
            if ~isempty(obj.trueState)
                plot(obj.trueState(frameIdx, 2), obj.trueState(frameIdx, 1), ...
                     'w+', 'MarkerSize', 14, 'LineWidth', 2);
            end
            title(sprintf('Frame %d', frameIdx));
            axis equal tight; hold off;
        end

        function plotAnimation(obj, varargin)
        % PLOTANIMATION  Animate the measurement sequence.
            if isempty(obj.measData)
                error('Scenario:NoData', 'Generate measurements first');
            end

            p = inputParser;
            addParameter(p, 'interval', 0.1, @isscalar);
            addParameter(p, 'showTrue', true, @islogical);
            parse(p, varargin{:});

            figure('Name', 'TBD Animation', 'Color', 'w');
            for t = 1:obj.config.numFrames
                imagesc(obj.measData(:, :, t));
                colormap(gca, 'parula'); colorbar;
                hold on;
                if p.Results.showTrue && ~isempty(obj.trueState)
                    plot(obj.trueState(t, 2), obj.trueState(t, 1), ...
                         'w+', 'MarkerSize', 14, 'LineWidth', 2);
                end
                title(sprintf('Frame %d / %d', t, obj.config.numFrames));
                axis equal tight; drawnow;
                hold off;
                pause(p.Results.interval);
            end
        end
        
        function plotTrajectory(obj, trueState, maneuverLabels)
        % PLOTTRAJECTORY  Plot the generated trajectory with maneuver segments.
        %   plotTrajectory() uses obj.trueState (must call generate first)
        %   plotTrajectory(trueState) uses provided trajectory
        %   plotTrajectory(trueState, maneuverLabels) with segment labels
            if nargin < 2
                trueState = obj.trueState;
                if nargin < 3
                    maneuverLabels = obj.maneuverLabels;
                end
            end
            
            if isempty(trueState)
                error('Scenario:NoData', 'Generate trajectory first');
            end
            
            figure('Name', 'TBD Trajectory', 'Color', 'w');
            hold on;
            
            if ~isempty(maneuverLabels)
                uniqueLabels = unique(maneuverLabels);
                colors = lines(length(uniqueLabels));
                
                for i = 1:length(uniqueLabels)
                    label = uniqueLabels{i};
                    idx = strcmp(maneuverLabels, label);
                    segIndices = find(idx);
                    
                    if ~isempty(segIndices)
                        for j = 1:length(segIndices)
                            k = segIndices(j);
                            if k > 1 && strcmp(maneuverLabels{k-1}, label)
                                plot(trueState(k-1:k, 2), trueState(k-1:k, 1), ...
                                     '-', 'Color', colors(i,:), 'LineWidth', 2);
                            end
                        end
                    end
                end
                legend(uniqueLabels, 'Location', 'best');
            else
                plot(trueState(:, 2), trueState(:, 1), 'b-', 'LineWidth', 2);
            end
            
            plot(trueState(1, 2), trueState(1, 1), 'go', ...
                 'MarkerSize', 10, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
            plot(trueState(end, 2), trueState(end, 1), 'rs', ...
                 'MarkerSize', 10, 'MarkerFaceColor', 'r', 'DisplayName', 'End');
            
            xlabel('Column'); ylabel('Row');
            title(sprintf('Target Trajectory (%s Model)', obj.config.motionModel));
            axis equal; grid on; hold off;
        end
    end
end
