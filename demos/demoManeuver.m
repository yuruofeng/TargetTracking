function varargout = demoManeuver(varargin)
% DEMOMANEUVER  Maneuvering target tracking demonstration.
%   Demonstrates various algorithms for tracking maneuvering targets
%   including CV, CA, CT, Singer, CS models and IMM algorithm.
%
%   Usage:
%       demoManeuver()                    % Default: compare all algorithms
%       demoManeuver('algorithms', {'CV', 'IMM'})
%       demoManeuver('mcRuns', 10)         % 10 Monte Carlo runs
%       demoManeuver('showPlots', false)   % Disable visualization
%       demoManeuver('saveResults', true)  % Save results to file
%
%   Algorithms:
%       'CV'     - Constant Velocity model
%       'CA'     - Constant Acceleration model
%       'CT'     - Coordinated Turn model
%       'Singer' - Singer acceleration model
%       'CS'     - Current Statistics model
%       'IMM'    - Interacting Multiple Model (CV+CA+CT)
%
%   Outputs:
%       results - Struct with algorithm estimates and RMSE

    testFolder = fileparts(mfilename('fullpath'));
    if isempty(testFolder)
        testFolder = pwd;
    end
    addpath(genpath(fileparts(testFolder)));

    opts = parseArgs(varargin);

    printHeader();

    cfgManeuver = dbt.ConfigManeuver('numSteps', opts.numSteps, 'dt', opts.dt);
    scenario = dbt.ScenarioManeuver(cfgManeuver);

    algorithms = opts.algorithms;
    nAlgo = length(algorithms);

    allPosRmse = cell(nAlgo, 1);
    allVelRmse = cell(nAlgo, 1);
    allEstStates = cell(nAlgo, 1);
    allElapsed = zeros(nAlgo, 1);

    for run = 1:opts.mcRuns
        [truthX, meas] = scenario.generate();
        
        for a = 1:nAlgo
            algoName = algorithms{a};
            [estState, posRmse, velRmse, elapsed] = runAlgorithm(algoName, truthX, meas, cfgManeuver);
            
            if run == 1
                allEstStates{a} = estState;
                allPosRmse{a} = posRmse;
                allVelRmse{a} = velRmse;
            else
                allPosRmse{a} = allPosRmse{a} + posRmse;
                allVelRmse{a} = allVelRmse{a} + velRmse;
            end
            allElapsed(a) = allElapsed(a) + elapsed;
        end
    end

    for a = 1:nAlgo
        allPosRmse{a} = allPosRmse{a} / opts.mcRuns;
        allVelRmse{a} = allVelRmse{a} / opts.mcRuns;
    end
    allElapsed = allElapsed / opts.mcRuns;

    printResults(algorithms, allPosRmse, allVelRmse, allElapsed);

    if opts.showPlots
        fprintf('Generating visualizations...\n');
        viz.Visualizer.plotManeuverComparison(truthX, meas, allEstStates, ...
            allPosRmse, allVelRmse, algorithms, scenario.maneuverLabels);
    end

    results = struct();
    results.truthX = truthX;
    results.meas = meas;
    results.maneuverLabels = scenario.maneuverLabels;
    for a = 1:nAlgo
        algoName = algorithms{a};
        results.(['est_' algoName]) = allEstStates{a};
        results.(['posRmse_' algoName]) = allPosRmse{a};
        results.(['velRmse_' algoName]) = allVelRmse{a};
        results.(['time_' algoName]) = allElapsed(a);
    end

    if opts.saveResults
        save(opts.saveFile, '-struct', 'results');
        fprintf('Results saved to %s\n', opts.saveFile);
    end

    fprintf('=== Maneuver Demo Complete ===\n\n');

    if nargout > 0
        varargout{1} = results;
    end
end

function opts = parseArgs(args)
    opts.numSteps = 100;
    opts.dt = 1;
    opts.mcRuns = 1;
    opts.algorithms = {'CV', 'CA', 'CT', 'Singer', 'CS', 'IMM'};
    opts.showPlots = true;
    opts.saveResults = false;
    opts.saveFile = 'maneuver_results.mat';

    for i = 1:2:length(args)
        if isfield(opts, args{i})
            opts.(args{i}) = args{i+1};
        end
    end
end

function printHeader()
    fprintf('\n');
    fprintf('============================================================\n');
    fprintf('   Maneuvering Target Tracking Demonstration\n');
    fprintf('============================================================\n\n');
end

function [estState, posRmse, velRmse, elapsed] = runAlgorithm(algoName, truthX, meas, cfgManeuver)
    dt = cfgManeuver.dt;
    
    x0 = [truthX(1,1); truthX(2,1); truthX(4,1); truthX(5,1)];
    P0_cv = diag([100, 25, 100, 25]);
    P0_ca = diag([100, 25, 100, 25, 25, 25]);
    P0_ct = diag([100, 25, 100, 25, 0.01]);
    
    switch algoName
        case 'CV'
            cfg = dbt.MotionModelConfig('CV', 'dt', dt);
            ekf = dbt.MotionModelEKF(cfg);
            [estState, ~, elapsed] = ekf.run(meas, x0, P0_cv);
            
        case 'CA'
            cfg = dbt.MotionModelConfig('CA', 'dt', dt);
            ekf = dbt.MotionModelEKF(cfg);
            x0_ca = [x0; 0; 0];
            [estState, ~, elapsed] = ekf.run(meas, x0_ca, P0_ca);
            
        case 'CT'
            cfg = dbt.MotionModelConfig('CT', 'dt', dt);
            ekf = dbt.MotionModelEKF(cfg);
            x0_ct = [x0; 0.01];
            [estState, ~, elapsed] = ekf.run(meas, x0_ct, P0_ct);
            
        case 'Singer'
            cfg = dbt.MotionModelConfig('Singer', 'dt', dt);
            ekf = dbt.MotionModelEKF(cfg);
            x0_ca = [x0; 0; 0];
            [estState, ~, elapsed] = ekf.run(meas, x0_ca, P0_ca);
            
        case 'CS'
            cfg = dbt.MotionModelConfig('CS', 'dt', dt);
            ekf = dbt.MotionModelEKF(cfg);
            x0_ca = [x0; 0; 0];
            [estState, ~, elapsed] = ekf.run(meas, x0_ca, P0_ca);
            
        case 'IMM'
            cfg = dbt.ConfigIMM('dt', dt);
            imm = dbt.IMM(cfg);
            x0_ca = [x0; 0; 0];
            [estState, ~, ~, elapsed] = imm.run(meas, x0_ca, P0_ca);
            
        otherwise
            error('Unknown algorithm: %s', algoName);
    end
    
    nSteps = size(truthX, 2);
    posRmse = zeros(nSteps, 1);
    velRmse = zeros(nSteps, 1);
    
    for k = 1:nSteps
        estPos = getEstPos(estState, k, algoName);
        truePos = truthX([1 4], k);
        posRmse(k) = norm(estPos - truePos);
        
        estVel = getEstVel(estState, k, algoName);
        trueVel = truthX([2 5], k);
        velRmse(k) = norm(estVel - trueVel);
    end
end

function pos = getEstPos(estState, k, algoName)
    switch algoName
        case {'CV'}
            pos = estState([1 3], k);
        case {'CA', 'Singer', 'CS', 'IMM'}
            pos = estState([1 4], k);
        case {'CT'}
            pos = estState([1 3], k);
    end
end

function vel = getEstVel(estState, k, algoName)
    switch algoName
        case {'CV'}
            vel = estState([2 4], k);
        case {'CA', 'Singer', 'CS', 'IMM'}
            vel = estState([2 5], k);
        case {'CT'}
            vel = estState([2 4], k);
    end
end

function printResults(algorithms, posRmse, velRmse, elapsed)
    fprintf('\n=== Results Summary ===\n');
    fprintf('Algorithm | Mean Pos RMSE | Mean Vel RMSE | Time [s]\n');
    fprintf('----------|---------------|---------------|---------\n');
    
    for a = 1:length(algorithms)
        meanPos = mean(posRmse{a});
        meanVel = mean(velRmse{a});
        fprintf('%-9s | %13.2f | %13.2f | %7.3f\n', ...
                algorithms{a}, meanPos, meanVel, elapsed(a));
    end
    fprintf('\n');
end
