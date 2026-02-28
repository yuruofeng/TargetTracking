classdef Scenario
% DBT.SCENARIO  Scenario generation for Detect-Before-Track simulations.
%   Generates CT-model target trajectories and radar measurements
%   for testing and evaluating DBT filters.
%
%   Usage:
%       cfg = dbt.Config();
%       scenario = dbt.Scenario(cfg);
%       [truthX, meas] = scenario.generate();
%
%   Methods:
%       generate - Generate trajectory and measurements
%       addClutter - Add false alarms to measurements (optional)

    properties
        config
        truthX
        meas
    end

    methods

        function obj = Scenario(cfg)
        % SCENARIO  Create scenario generator with configuration.
        %   obj = Scenario(cfg) initializes with dbt.Config object.
            if nargin < 1
                cfg = dbt.Config();
            end
            obj.config = cfg;
        end

        function [truthX, meas] = generate(obj)
        % GENERATE  Generate CT-model trajectory and radar measurements.
        %   [truthX, meas] = generate() produces:
        %     truthX - True state trajectory [stateDim x numSteps]
        %     meas   - Radar measurements [measDim x numSteps]
            nSteps = obj.config.numSteps;
            state  = obj.config.initState;

            truthX = zeros(obj.config.stateDim, nSteps);
            meas   = zeros(obj.config.measDim, nSteps);

            R = obj.config.measNoiseCov * obj.config.measNoiseCov';
            for k = 1:nSteps
                F     = utils.MeasurementModel.ctDynamicMatrix(obj.config.dt, state);
                state = F * state;
                truthX(:, k) = state;

                measNoise  = sqrtm(R) * randn(obj.config.measDim, 1);
                meas(:, k) = [atan2(state(3), state(1)); ...
                              norm(state([1 3]))] + measNoise;
            end
            obj.truthX = truthX;
            obj.meas = meas;
        end

        function obj = reset(obj)
        % RESET  Clear stored trajectory and measurements.
            obj.truthX = [];
            obj.meas = [];
        end

        function plotTrajectory(obj)
        % PLOTTRAJECTORY  Visualize the generated trajectory.
            if isempty(obj.truthX)
                error('Scenario:NoData', 'Generate trajectory first');
            end

            figure('Name', 'DBT Scenario', 'Color', 'w');
            xTrue = obj.truthX(1, :);
            yTrue = obj.truthX(3, :);
            xMeas = obj.meas(2, :) .* cos(obj.meas(1, :));
            yMeas = obj.meas(2, :) .* sin(obj.meas(1, :));

            plot(xTrue, yTrue, 'b-', 'LineWidth', 2, 'DisplayName', 'True'); hold on;
            plot(xMeas, yMeas, 'ro', 'MarkerSize', 4, 'DisplayName', 'Measurements');
            plot(xTrue(1), yTrue(1), 'g^', 'MarkerSize', 10, ...
                 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
            plot(xTrue(end), yTrue(end), 'mv', 'MarkerSize', 10, ...
                 'MarkerFaceColor', 'm', 'DisplayName', 'End');

            xlabel('x [m]'); ylabel('y [m]');
            title(sprintf('CT Trajectory (%d steps)', obj.config.numSteps));
            legend('Location', 'best');
            grid on; axis equal; hold off;
        end

    end
end
