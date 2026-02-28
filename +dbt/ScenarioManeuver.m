classdef ScenarioManeuver < handle
% DBT.SCENARIOMANEUVER  Scenario generation for maneuvering target tracking.
%   Generates trajectories with various maneuvers including acceleration,
%   deceleration, coordinated turns, and sudden direction changes.
%
%   Maneuver segments:
%     - CV: Constant velocity
%     - CA: Constant acceleration
%     - CT: Coordinated turn
%     - Stop: Brief stop
%
%   Usage:
%       cfg = dbt.ConfigManeuver();
%       scenario = dbt.ScenarioManeuver(cfg);
%       [truthX, meas] = scenario.generate();

    properties
        config
        truthX
        meas
        maneuverLabels
    end

    methods

        function obj = ScenarioManeuver(cfg)
            if nargin < 1
                cfg = dbt.ConfigManeuver();
            end
            obj.config = cfg;
        end

        function [truthX, meas] = generate(obj)
            nSteps = obj.config.numSteps;
            dt = obj.config.dt;
            
            truthX = zeros(6, nSteps);
            meas = zeros(2, nSteps);
            obj.maneuverLabels = repmat({'CV'}, 1, nSteps);
            
            segments = obj.config.maneuverSegments;
            
            state = obj.config.initState(:);
            currentStep = 1;
            
            for seg = 1:length(segments)
                segType = segments{seg}.type;
                segLen = segments{seg}.length;
                segParams = segments{seg}.params;
                
                for k = 1:segLen
                    if currentStep > nSteps
                        break;
                    end
                    
                    switch segType
                        case 'CV'
                            state = obj.cvStep(state, dt);
                        case 'CA'
                            state = obj.caStep(state, dt, segParams.accel);
                        case 'CT'
                            state = obj.ctStep(state, dt, segParams.omega);
                        case 'Stop'
                            state = obj.stopStep(state, dt);
                    end
                    
                    obj.maneuverLabels{currentStep} = segType;
                    truthX(:, currentStep) = state;
                    
                    R = obj.config.measNoiseCov * obj.config.measNoiseCov';
                    measNoise = sqrtm(R) * randn(2, 1);
                    meas(:, currentStep) = [atan2(state(4), state(1)); ...
                                           norm(state([1 4]))] + measNoise;
                    
                    currentStep = currentStep + 1;
                end
            end
            
            while currentStep <= nSteps
                state = obj.cvStep(state, dt);
                truthX(:, currentStep) = state;
                
                R = obj.config.measNoiseCov * obj.config.measNoiseCov';
                measNoise = sqrtm(R) * randn(2, 1);
                meas(:, currentStep) = [atan2(state(4), state(1)); ...
                                       norm(state([1 4]))] + measNoise;
                currentStep = currentStep + 1;
            end
            
            obj.truthX = truthX;
            obj.meas = meas;
        end

        function state = cvStep(obj, state, dt)
            F = [1, dt, 0, 0, 0, 0;
                 0, 1,  0, 0, 0, 0;
                 0, 0,  1, 0, 0, 0;
                 0, 0,  0, 1, dt, 0;
                 0, 0,  0, 0, 1, 0;
                 0, 0,  0, 0, 0, 1];
            state = F * state;
        end

        function state = caStep(obj, state, dt, accel)
            ax = accel(1);
            ay = accel(2);
            
            state(1) = state(1) + state(2)*dt + 0.5*ax*dt^2;
            state(2) = state(2) + ax*dt;
            state(3) = ax;
            
            state(4) = state(4) + state(5)*dt + 0.5*ay*dt^2;
            state(5) = state(5) + ay*dt;
            state(6) = ay;
        end

        function state = ctStep(obj, state, dt, omega)
            x = state(1); vx = state(2);
            y = state(4); vy = state(5);
            
            if abs(omega) < 1e-8
                state(1) = x + vx*dt;
                state(4) = y + vy*dt;
            else
                state(1) = x + vx*sin(omega*dt)/omega - y*(1-cos(omega*dt))/omega;
                state(2) = vx*cos(omega*dt) - vy*sin(omega*dt);
                state(4) = y + vx*(1-cos(omega*dt))/omega + vy*sin(omega*dt)/omega;
                state(5) = vx*sin(omega*dt) + vy*cos(omega*dt);
            end
            state(3) = 0;
            state(6) = 0;
        end

        function state = stopStep(obj, state, dt)
            state(2) = 0;
            state(3) = 0;
            state(5) = 0;
            state(6) = 0;
        end

        function plotTrajectory(obj)
            if isempty(obj.truthX)
                error('ScenarioManeuver:NoData', 'Generate trajectory first');
            end
            
            figure('Name', 'Maneuver Scenario', 'Color', 'w', 'Position', [100, 100, 1200, 500]);
            
            subplot(1, 2, 1);
            xTrue = obj.truthX(1, :);
            yTrue = obj.truthX(4, :);
            xMeas = obj.meas(2, :) .* cos(obj.meas(1, :));
            yMeas = obj.meas(2, :) .* sin(obj.meas(1, :));
            
            plot(xTrue, yTrue, 'b-', 'LineWidth', 2, 'DisplayName', 'True'); hold on;
            plot(xMeas, yMeas, 'ro', 'MarkerSize', 4, 'DisplayName', 'Measurements');
            plot(xTrue(1), yTrue(1), 'g^', 'MarkerSize', 10, ...
                 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
            plot(xTrue(end), yTrue(end), 'mv', 'MarkerSize', 10, ...
                 'MarkerFaceColor', 'm', 'DisplayName', 'End');
            
            xlabel('x [m]'); ylabel('y [m]');
            title('Maneuvering Target Trajectory');
            legend('Location', 'best');
            grid on; axis equal; hold off;
            
            subplot(1, 2, 2);
            colors = containers.Map({'CV', 'CA', 'CT', 'Stop'}, ...
                                    {'b', 'r', 'g', 'k'});
            
            hold on;
            for k = 1:length(obj.maneuverLabels)-1
                label = obj.maneuverLabels{k};
                plot(k:k+1, obj.truthX(2, k:k+1), '-', ...
                     'Color', colors(label), 'LineWidth', 2);
            end
            
            xlabel('Time Step'); ylabel('Velocity X [m/s]');
            title('Velocity with Maneuver Segments');
            
            legendEntries = {};
            for key = {'CV', 'CA', 'CT', 'Stop'}
                if any(strcmp(obj.maneuverLabels, key{1}))
                    legendEntries{end+1} = plot(NaN, NaN, '-', 'Color', colors(key{1}), ...
                                               'LineWidth', 2, 'DisplayName', key{1});
                end
            end
            legend([legendEntries{:}], 'Location', 'best');
            grid on; hold off;
        end
    end
end
