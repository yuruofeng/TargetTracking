classdef ConfigManeuver
% DBT.CONFIGMANEUVER  Configuration for maneuvering target scenarios.
%   Defines maneuver segments for generating complex trajectories.
%
%   Default scenario:
%     - CV (30 steps) -> CA (20 steps) -> CT (25 steps) -> CA (20 steps) -> CV (5 steps)
%
%   Usage:
%       cfg = dbt.ConfigManeuver();
%       cfg = dbt.ConfigManeuver('numSteps', 150);

    properties
        numSteps = 100
        dt = 1
        stateDim = 6
        measDim = 2
        measNoiseCov
        initState
        maneuverSegments
    end

    methods

        function obj = ConfigManeuver(varargin)
            obj.initState = [0; 10; 0; 0; 5; 0];
            obj.measNoiseCov = diag([pi/90, 5]);
            
            obj.maneuverSegments = {
                struct('type', 'CV', 'length', 30, 'params', struct());
                struct('type', 'CA', 'length', 20, 'params', struct('accel', [2; 1]));
                struct('type', 'CT', 'length', 25, 'params', struct('omega', 0.05));
                struct('type', 'CA', 'length', 20, 'params', struct('accel', [-2; -1]));
                struct('type', 'CV', 'length', 5, 'params', struct());
            };
            
            if nargin > 0
                for i = 1:2:length(varargin)
                    if isprop(obj, varargin{i})
                        obj.(varargin{i}) = varargin{i+1};
                    end
                end
            end
        end

        function R = getMeasurementNoiseCov(obj)
            R = obj.measNoiseCov * obj.measNoiseCov';
        end

        function seg = getSegment(obj, stepIdx)
        % GETSEGMENT  Get maneuver segment for a given step.
            currentStep = 1;
            for i = 1:length(obj.maneuverSegments)
                segLen = obj.maneuverSegments{i}.length;
                if stepIdx >= currentStep && stepIdx < currentStep + segLen
                    seg = obj.maneuverSegments{i};
                    return;
                end
                currentStep = currentStep + segLen;
            end
            seg = struct('type', 'CV', 'length', 1, 'params', struct());
        end
    end
end
