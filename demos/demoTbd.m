function varargout = demoTbd(varargin)
% DEMOTBD  Track-Before-Detect algorithm demonstration.
%   Demonstrates DP-TBD and PF-TBD algorithms on an image-domain
%   weak target tracking scenario with low SNR.
%
%   Usage:
%       demoTbd()                         % Run with default settings (CV model)
%       demoTbd('motionModel', 'CT')      % Use coordinated turn model
%       demoTbd('motionModel', 'Maneuver')% Use segmented maneuver trajectory
%       demoTbd('numFrames', 100)         % Use 100 frames
%       demoTbd('snr', 6)                 % Set SNR to 6 dB
%       demoTbd('saveResults', true)      % Save results to file
%
%   Parameters (name-value pairs):
%       motionModel  - Motion model: 'CV', 'CA', 'CT', 'Maneuver' (default: 'CV')
%       gridSize     - Image size [rows, cols] (default: [100, 100])
%       numFrames    - Number of frames (default: 50)
%       snr          - Target SNR in dB (default: ~9.5)
%       numParticles - Number of PF particles (default: 500)
%       maxSpeed     - Max speed for DP (default: 3)
%       turnRate     - Turn rate for CT model [rad/frame] (default: 0.05)
%       initAccel    - Initial acceleration for CA model (default: [0, 0])
%       saveResults  - Save results to .mat file (default: false)
%       saveFile     - Output filename (default: 'tbd_results.mat')
%       showPlots    - Display plots (default: true)
%       animate      - Show animation (default: false)
%
%   Outputs:
%       results - Struct containing all estimates and RMSE data
%
%   Motion Models:
%       CV       - Constant Velocity (straight-line motion)
%       CA       - Constant Acceleration (accelerating target)
%       CT       - Coordinated Turn (circular motion)
%       Maneuver - Segmented trajectory with multiple motion types
%
%   Algorithms:
%       DP-TBD - Dynamic Programming Track-Before-Detect
%       PF-TBD - Particle Filter Track-Before-Detect
%
%   Example:
%       results = demoTbd('motionModel', 'Maneuver', 'numFrames', 100);
%
%   See also: tbd.Config, tbd.Scenario, tbd.DpTbd, tbd.PfTbd

    close all; clc;

    fprintf('\n');
    fprintf('============================================================\n');
    fprintf('   Track-Before-Detect (TBD) Algorithm Demonstration\n');
    fprintf('============================================================\n\n');

    testFolder = fileparts(mfilename('fullpath'));
    if isempty(testFolder)
        testFolder = pwd;
    end
    addpath(genpath(fileparts(testFolder)));

    p = inputParser;
    addParameter(p, 'gridSize', [100, 100], @isvector);
    addParameter(p, 'numFrames', 50, @isscalar);
    addParameter(p, 'motionModel', 'CV', @ischar);
    addParameter(p, 'snr', 9.5, @isscalar);
    addParameter(p, 'numParticles', 500, @isscalar);
    addParameter(p, 'maxSpeed', 3, @isscalar);
    addParameter(p, 'turnRate', 0.05, @isscalar);
    addParameter(p, 'initAccel', [0, 0], @isvector);
    addParameter(p, 'saveResults', false, @islogical);
    addParameter(p, 'saveFile', 'tbd_results.mat', @ischar);
    addParameter(p, 'showPlots', true, @islogical);
    addParameter(p, 'animate', false, @islogical);
    addParameter(p, 'rngSeed', 2024, @isscalar);
    parse(p, varargin{:});
    opts = p.Results;

    noiseStd = 1.0;
    amplitude = noiseStd * 10^(opts.snr/20);

    cfg = tbd.Config('gridSize', opts.gridSize, 'numFrames', opts.numFrames, ...
                     'motionModel', opts.motionModel, ...
                     'amplitude', amplitude, 'noiseStd', noiseStd, ...
                     'numParticles', opts.numParticles, 'maxSpeed', opts.maxSpeed, ...
                     'turnRate', opts.turnRate, 'initAccel', opts.initAccel, ...
                     'rngSeed', opts.rngSeed);
    cfg.display();
    fprintf('\n');

    rng(cfg.rngSeed);

    scenario = tbd.Scenario(cfg);
    [scenario, trueState, measData, psfKernel, maneuverLabels] = scenario.generate();

    fprintf('Start: (%.1f, %.1f)  Vel: (%.1f, %.1f)\n', ...
            trueState(1,1), trueState(1,2), trueState(1,3), trueState(1,4));
    if strcmpi(opts.motionModel, 'CA') || strcmpi(opts.motionModel, 'MANEUVER')
        fprintf('Initial acceleration: (%.2f, %.2f)\n', cfg.initAccel(1), cfg.initAccel(2));
    end
    fprintf('\n');

    fprintf('Running DP-TBD...\n');
    dp = tbd.DpTbd(cfg);
    [dpPosRmse, dpTime] = dp.runWithTiming(measData, psfKernel, trueState);
    [dpTrack, dpScore] = dp.getResults();
    fprintf('  Time: %.2f s | Mean pos RMSE: %.2f px\n', dpTime, mean(dpPosRmse));

    fprintf('Running PF-TBD (%d particles)...\n', cfg.numParticles);
    pf = tbd.PfTbd(cfg);
    [pfTime, pfPosRmse, pfVelRmse] = pf.runWithTiming(measData, trueState(1,:), ...
                                                       psfKernel, trueState);
    pfState = pf.estState;
    fprintf('  Time: %.2f s | Mean pos RMSE: %.2f px | Mean vel RMSE: %.2f\n', ...
            pfTime, mean(pfPosRmse), mean(pfVelRmse));

    dpVelEst = diff(dpTrack, 1, 1) / cfg.dt;
    dpVelRmse = sqrt(sum((dpVelEst - trueState(2:end, 3:4)).^2, 2));
    dpVelRmse = [nan; dpVelRmse];

    fprintf('\n=== Results Summary ===\n');
    fprintf('Algorithm | Mean Pos RMSE | Mean Vel RMSE | Time [s]\n');
    fprintf('----------|---------------|---------------|---------\n');
    fprintf('DP-TBD    | %13.2f | %13.2f | %7.2f\n', mean(dpPosRmse), mean(dpVelRmse, 'omitnan'), dpTime);
    fprintf('PF-TBD    | %13.2f | %13.2f | %7.2f\n', mean(pfPosRmse), mean(pfVelRmse), pfTime);
    fprintf('\nDP score: %.1f\n\n', dpScore);

    if opts.animate
        fprintf('Playing animation...\n');
        scenario.plotAnimation('interval', 0.05, 'showTrue', true);
    end

    if opts.showPlots
        fprintf('Generating visualizations...\n');
        
        viz.Visualizer.plotComprehensiveTbd(trueState, dpTrack, pfState, measData, ...
            dpPosRmse, pfPosRmse, dpVelRmse, pfVelRmse, cfg);
        
        scenario.plotTrajectory(trueState, maneuverLabels);

        viz.Visualizer.plotTimingComparison([dpTime, pfTime], ...
            {'DP-TBD', 'PF-TBD'}, 'Computation Time');
    end

    results = struct(...
        'trueState', trueState, 'measData', measData, 'psfKernel', psfKernel, ...
        'dpTrack', dpTrack, 'dpScore', dpScore, 'dpPosRmse', dpPosRmse, ...
        'pfState', pfState, 'pfPosRmse', pfPosRmse, 'pfVelRmse', pfVelRmse, ...
        'dpTime', dpTime, 'pfTime', pfTime, 'config', cfg, ...
        'maneuverLabels', maneuverLabels);

    if opts.saveResults
        save(opts.saveFile, '-struct', 'results');
        fprintf('Results saved to %s\n', opts.saveFile);
    end

    fprintf('=== TBD Demo Complete ===\n\n');

    if nargout > 0
        varargout{1} = results;
    end
end
