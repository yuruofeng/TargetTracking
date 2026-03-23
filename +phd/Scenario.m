classdef Scenario
% PHD.SCENARIO  PHD Filter Simulation Scenario Generator
%   Generates target ground truth trajectories and cluttered measurements.
%
%   Properties:
%       K            - Total time steps
%       X            - True state sequence (cell array)
%       N            - Target count sequence
%       track_list   - Target ID list
%       total_tracks - Total number of targets
%       Z            - Measurement sequence (cell array)
%
%   Usage:
%       model = phd.Config();
%       scenario = phd.Scenario(model);
%       truth = scenario.generateTruth();
%       meas = scenario.generateMeasurements(truth);
%
%   See also: phd.Config, phd.MultiModelFilter

    properties
        K             % Total time steps
        X             % True state sequence
        N             % Target count sequence
        L             % Gaussian component count sequence
        track_list    % Target ID list
        total_tracks  % Total number of targets
        Z             % Measurement sequence
    end

    properties (Access = private)
        model         % Model configuration
    end

    methods

        function obj = Scenario(model)
        % SCENARIO  Create PHD scenario generator.
        %
        %   Inputs:
        %       model - phd.Config configuration object

            if nargin < 1
                model = phd.Config();
            end
            obj.model = model;
            obj.K = 100;
        end

        function obj = generateTruth(obj)
        % GENERATETRUTH  Generate target ground truth trajectories.
        %   Creates target trajectories with CV->CT->CA mode switching based on maneuver intervals.
        %
        %   Outputs:
        %       obj - Updated scenario object (with X, N, track_list)

            obj.X = cell(obj.K, 1);
            obj.N = zeros(obj.K, 1);
            obj.L = cell(obj.K, 1);
            obj.track_list = cell(obj.K, 1);
            obj.total_tracks = 0;

            nBirths = 1;
            xStart(:, 1) = [600; 7; 0; 800; -5; 0];

            targetState = xStart;

            for targetNum = 1:nBirths
                for k = 1:obj.K
                    if any(obj.model.maneuvers(1, :) == k)
                        mIdx = obj.model.maneuvers(2, obj.model.maneuvers(1, :) == k);

                        switch mIdx
                            case 1
                                G = obj.model.G{1};
                                Q = obj.model.Q{1};
                                targetState = xStart;
                                for kk = obj.model.tbirth(mIdx):min(obj.model.tdeath(mIdx), obj.K)
                                    targetState = obj.genNewState(targetState, G, Q, 'noiseless');
                                    obj.X{kk} = [obj.X{kk}, targetState];
                                    obj.track_list{kk} = [obj.track_list{kk}, targetNum];
                                    obj.N(kk) = obj.N(kk) + 1;
                                end

                            case 2
                                G = obj.model.G{3};
                                Q = obj.model.Q{3};
                                for kk = obj.model.tbirth(mIdx):min(obj.model.tdeath(mIdx), obj.K)
                                    targetState = obj.genNewState(targetState, G, Q, 'noiseless');
                                    obj.X{kk} = [obj.X{kk}, targetState];
                                    obj.track_list{kk} = [obj.track_list{kk}, targetNum];
                                    obj.N(kk) = obj.N(kk) + 1;
                                end

                            case 3
                                G = obj.model.G{2};
                                Q = obj.model.Q{2};
                                targetState = targetState + obj.model.a;
                                for kk = obj.model.tbirth(mIdx):min(obj.model.tdeath(mIdx), obj.K)
                                    targetState = obj.genNewState(targetState, G, Q, 'noiseless');
                                    obj.X{kk} = [obj.X{kk}, targetState];
                                    obj.track_list{kk} = [obj.track_list{kk}, targetNum];
                                    obj.N(kk) = obj.N(kk) + 1;
                                end
                        end
                    end
                end
            end
            obj.total_tracks = nBirths;
        end

        function obj = generateMeasurements(obj, truth)
        % GENERATEMEASUREMENTS  Generate cluttered measurements.
        %   For each time step, determines target detection with probability P_D and adds Poisson clutter.
        %
        %   Inputs:
        %       truth - Truth structure (with X, N)
        %
        %   Outputs:
        %       obj - Updated scenario object (with Z)

            obj.Z = cell(truth.K, 1);
            measNoise = obj.model.D_CV * randn(2, truth.K);

            for k = 1:truth.K
                if truth.N(k) > 0
                    detIdx = find(rand(truth.N(k), 1) <= obj.model.P_D);
                    obj.Z{k} = obj.genObservation(truth.X{k}(:, detIdx), k, measNoise);
                else
                    obj.Z{k} = [];
                end

                nClutter = poissrnd(obj.model.lambda_c);
                if nClutter > 0
                    clutter = repmat(obj.model.range_c(:, 1), [1, nClutter]) ...
                            + diag(obj.model.range_c * [-1; 1]) * rand(obj.model.z_dim, nClutter);
                    obj.Z{k} = [obj.Z{k}, clutter];
                end
            end
        end

        function [xTrack, kBirth, kDeath] = extractTracks(obj)
        % EXTRACTTRACKS  Extract individual target tracks from truth.
        %
        %   Outputs:
        %       xTrack - Track matrix [dim x K x nTargets]
        %       kBirth - Birth time for each target
        %       kDeath - Death time for each target

            xDim = 6;
            kk = obj.K;
            while xDim == 6 && kk > 0
                if ~isempty(obj.X{kk})
                    xDim = size(obj.X{kk}, 1);
                    break;
                end
                kk = kk - 1;
            end

            xTrack = zeros(xDim, obj.K, obj.total_tracks);
            kBirth = zeros(obj.total_tracks, 1);
            kDeath = zeros(obj.total_tracks, 1);
            maxIdx = 0;

            for k = 1:obj.K
                if ~isempty(obj.X{k})
                    xTrack(:, k, obj.track_list{k}) = obj.X{k};
                end
                if ~isempty(obj.track_list{k}) && max(obj.track_list{k}) > maxIdx
                    idx = find(obj.track_list{k} > maxIdx);
                    kBirth(obj.track_list{k}(idx)) = k;
                end
                if ~isempty(obj.track_list{k})
                    maxIdx = max(obj.track_list{k});
                    kDeath(obj.track_list{k}) = k;
                end
            end
        end

    end

    methods (Access = private)

        function X = genNewState(obj, Xd, B, Q, V)
        % GENNEWSTATE  Generic state transition function.

            if isequal(B, obj.model.G_CV)
                x4 = [Xd([1,2], :); Xd([4,5], :)];

                if ~isnumeric(V)
                    if strcmp(V, 'noise')
                        V = obj.model.sigma_cv * obj.model.G_CV * randn(size(B, 2), size(Xd, 2));
                    else
                        V = zeros(size(B, 1), size(Xd, 2));
                    end
                end

                if isempty(Xd)
                    X = [];
                else
                    x4new = obj.model.F_CV * x4 + V;
                    X = phd.MathUtils.expandState4to6(x4new);
                end

            elseif isequal(B, obj.model.G_CA)
                if ~isnumeric(V)
                    if strcmp(V, 'noise')
                        V = B * randn(size(B, 2), size(Xd, 2));
                    else
                        V = zeros(size(B, 1), size(Xd, 2));
                    end
                end

                if isempty(Xd)
                    X = [];
                else
                    X = obj.model.F_CA * Xd + V;
                end

            elseif isequal(B, obj.model.G_CT)
                nCols = size(Xd, 2);
                x5 = [Xd([1,2,4,5], :); obj.model.omega * ones(1, nCols)];

                if ~isnumeric(V)
                    if strcmp(V, 'noise')
                        V = obj.model.B_CT * randn(size(obj.model.B_CT, 2), nCols);
                    else
                        V = zeros(size(obj.model.B_CT, 1), nCols);
                    end
                end

                if isempty(Xd)
                    X = [];
                else
                    x5new = zeros(size(x5));
                    L = nCols;
                    T = obj.model.T;
                    omega = obj.model.omega;
                    tol = 1e-10;

                    sinOT = sin(omega * T);
                    cosOT = cos(omega * T);
                    a = T * ones(1, L);
                    b = zeros(1, L);
                    idx = find(abs(omega) > tol);
                    a(idx) = sinOT / omega;
                    b(idx) = (1 - cosOT) / omega;

                    x5new(1, :) = x5(1, :) + a .* x5(2, :) - b .* x5(4, :);
                    x5new(2, :) = cosOT .* x5(2, :) - sinOT .* x5(4, :);
                    x5new(3, :) = b .* x5(2, :) + x5(3, :) + a .* x5(4, :);
                    x5new(4, :) = sinOT .* x5(2, :) + cosOT .* x5(4, :);
                    x5new(5, :) = x5(5, :);
                    x5new = x5new + obj.model.G_CT * V;

                    x4ct = x5new([1,2,3,4], :);
                    X = phd.MathUtils.expandState4to6(x4ct);
                end

            else
                error('genNewState: Unrecognized noise matrix B');
            end
        end

        function Z = genObservation(obj, x, k, measNoise)
        % GENOBSERVATION  Generic observation function.

            Z = x([1, 4], :) + measNoise(:, k);
        end

    end
end
