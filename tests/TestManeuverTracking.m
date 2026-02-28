classdef TestManeuverTracking < matlab.unittest.TestCase
% TESTMANEUVERTRACKING  Unit tests for maneuvering target tracking.
%   Tests all motion models (CV, CA, CT, Singer, CS) and IMM algorithm
%   using the unified MotionModelConfig and MotionModelEKF interfaces.

    properties
        configCV
        configCA
        configCT
        configSinger
        configCS
        configIMM
        configManeuver
        numSteps = 50
        dt = 1
    end

    methods (TestMethodSetup)

        function setupConfigs(obj)
            testFolder = fileparts(mfilename('fullpath'));
            if isempty(testFolder)
                testFolder = pwd;
            end
            addpath(genpath(fileparts(testFolder)));
            
            obj.configCV = dbt.MotionModelConfig('CV', 'numSteps', obj.numSteps, 'dt', obj.dt);
            obj.configCA = dbt.MotionModelConfig('CA', 'numSteps', obj.numSteps, 'dt', obj.dt);
            obj.configCT = dbt.MotionModelConfig('CT', 'numSteps', obj.numSteps, 'dt', obj.dt);
            obj.configSinger = dbt.MotionModelConfig('Singer', 'numSteps', obj.numSteps, 'dt', obj.dt);
            obj.configCS = dbt.MotionModelConfig('CS', 'numSteps', obj.numSteps, 'dt', obj.dt);
            obj.configIMM = dbt.ConfigIMM('numSteps', obj.numSteps, 'dt', obj.dt);
            obj.configManeuver = dbt.ConfigManeuver('numSteps', obj.numSteps, 'dt', obj.dt);
        end
    end

    methods (Test)

        function testUnifiedCVFilterCreation(obj)
            ekf = dbt.MotionModelEKF(obj.configCV);
            verifyEqual(obj, ekf.modelName, 'CV');
            verifyEqual(obj, ekf.stateDim, 4);
        end

        function testUnifiedCVFilterRun(obj)
            ekf = dbt.MotionModelEKF(obj.configCV);
            scenario = dbt.ScenarioManeuver(obj.configManeuver);
            [truthX, meas] = scenario.generate();
            
            x0 = [truthX(1,1); truthX(2,1); truthX(4,1); truthX(5,1)];
            P0 = diag([100, 25, 100, 25]);
            
            [estStates, estCovars, elapsed] = ekf.run(meas, x0, P0);
            
            verifySize(obj, estStates, [4, obj.numSteps]);
            verifySize(obj, estCovars, [4, 4, obj.numSteps]);
            verifyGreaterThan(obj, elapsed, 0);
        end

        function testUnifiedCAFilterCreation(obj)
            ekf = dbt.MotionModelEKF(obj.configCA);
            verifyEqual(obj, ekf.modelName, 'CA');
            verifyEqual(obj, ekf.stateDim, 6);
        end

        function testUnifiedCAFilterRun(obj)
            ekf = dbt.MotionModelEKF(obj.configCA);
            scenario = dbt.ScenarioManeuver(obj.configManeuver);
            [truthX, meas] = scenario.generate();
            
            x0 = [truthX(1,1); truthX(2,1); 0; truthX(4,1); truthX(5,1); 0];
            P0 = diag([100, 25, 25, 100, 25, 25]);
            
            [estStates, ~, elapsed] = ekf.run(meas, x0, P0);
            
            verifySize(obj, estStates, [6, obj.numSteps]);
            verifyGreaterThan(obj, elapsed, 0);
        end

        function testUnifiedCTFilterCreation(obj)
            ekf = dbt.MotionModelEKF(obj.configCT);
            verifyEqual(obj, ekf.modelName, 'CT');
            verifyEqual(obj, ekf.stateDim, 5);
        end

        function testUnifiedCTFilterRun(obj)
            ekf = dbt.MotionModelEKF(obj.configCT);
            scenario = dbt.ScenarioManeuver(obj.configManeuver);
            [truthX, meas] = scenario.generate();
            
            x0 = [truthX(1,1); truthX(2,1); truthX(4,1); truthX(5,1); 0.01];
            P0 = diag([100, 25, 100, 25, 0.01]);
            
            [estStates, ~, elapsed] = ekf.run(meas, x0, P0);
            
            verifySize(obj, estStates, [5, obj.numSteps]);
            verifyGreaterThan(obj, elapsed, 0);
        end

        function testUnifiedSingerFilterCreation(obj)
            ekf = dbt.MotionModelEKF(obj.configSinger);
            verifyEqual(obj, ekf.modelName, 'SINGER');
            verifyEqual(obj, ekf.stateDim, 6);
        end

        function testUnifiedSingerFilterRun(obj)
            ekf = dbt.MotionModelEKF(obj.configSinger);
            scenario = dbt.ScenarioManeuver(obj.configManeuver);
            [truthX, meas] = scenario.generate();
            
            x0 = [truthX(1,1); truthX(2,1); 0; truthX(4,1); truthX(5,1); 0];
            P0 = diag([100, 25, 25, 100, 25, 25]);
            
            [estStates, ~, elapsed] = ekf.run(meas, x0, P0);
            
            verifySize(obj, estStates, [6, obj.numSteps]);
            verifyGreaterThan(obj, elapsed, 0);
        end

        function testUnifiedCSFilterCreation(obj)
            ekf = dbt.MotionModelEKF(obj.configCS);
            verifyEqual(obj, ekf.modelName, 'CS');
            verifyEqual(obj, ekf.stateDim, 6);
        end

        function testUnifiedCSFilterRun(obj)
            ekf = dbt.MotionModelEKF(obj.configCS);
            scenario = dbt.ScenarioManeuver(obj.configManeuver);
            [truthX, meas] = scenario.generate();
            
            x0 = [truthX(1,1); truthX(2,1); 0; truthX(4,1); truthX(5,1); 0];
            P0 = diag([100, 25, 25, 100, 25, 25]);
            
            [estStates, ~, elapsed] = ekf.run(meas, x0, P0);
            
            verifySize(obj, estStates, [6, obj.numSteps]);
            verifyGreaterThan(obj, elapsed, 0);
        end

        function testIMMFilterCreation(obj)
            imm = dbt.IMM(obj.configIMM);
            verifyEqual(obj, imm.nModels, 3);
            verifyEqual(obj, length(imm.modelProbs), 3);
        end

        function testIMMFilterRun(obj)
            imm = dbt.IMM(obj.configIMM);
            scenario = dbt.ScenarioManeuver(obj.configManeuver);
            [truthX, meas] = scenario.generate();
            
            x0 = [truthX(1,1); truthX(2,1); 0; truthX(4,1); truthX(5,1); 0];
            P0 = diag([100, 25, 25, 100, 25, 25]);
            
            [estStates, estCovars, modelProbs, elapsed] = imm.run(meas, x0, P0);
            
            verifySize(obj, estStates, [6, obj.numSteps]);
            verifySize(obj, estCovars, [6, 6, obj.numSteps]);
            verifySize(obj, modelProbs, [3, obj.numSteps]);
            verifyGreaterThan(obj, elapsed, 0);
            
            probSum = sum(modelProbs, 1);
            verifyEqual(obj, probSum, ones(1, obj.numSteps), 'AbsTol', 1e-10);
        end

        function testScenarioManeuverGeneration(obj)
            scenario = dbt.ScenarioManeuver(obj.configManeuver);
            [truthX, meas] = scenario.generate();
            
            verifySize(obj, truthX, [6, obj.numSteps]);
            verifySize(obj, meas, [2, obj.numSteps]);
            verifyLength(obj, scenario.maneuverLabels, obj.numSteps);
        end

        function testManeuverSegmentTypes(obj)
            scenario = dbt.ScenarioManeuver(obj.configManeuver);
            scenario.generate();
            
            uniqueLabels = unique(scenario.maneuverLabels);
            validLabels = {'CV', 'CA', 'CT', 'Stop'};
            
            for i = 1:length(uniqueLabels)
                verifyTrue(obj, any(strcmp(validLabels, uniqueLabels{i})));
            end
        end

        function testConfigIMMTransitionMatrix(obj)
            T = obj.configIMM.transProb;
            verifySize(obj, T, [3, 3]);
            
            rowSums = sum(T, 2);
            verifyEqual(obj, rowSums, ones(3, 1), 'AbsTol', 1e-10);
        end

        function testFilterAccuracy(obj)
            ekf = dbt.MotionModelEKF(obj.configCV);
            scenario = dbt.ScenarioManeuver(obj.configManeuver);
            [truthX, meas] = scenario.generate();
            
            x0 = [truthX(1,1); truthX(2,1); truthX(4,1); truthX(5,1)];
            P0 = diag([100, 25, 100, 25]);
            
            [estStates, ~, ~] = ekf.run(meas, x0, P0);
            
            posError = sqrt(mean((estStates(1,:) - truthX(1,:)).^2 + ...
                                  (estStates(3,:) - truthX(4,:)).^2));
            
            verifyLessThan(obj, posError, 50);
        end
        
        function testMotionModelConfigCreation(obj)
            cfg = dbt.MotionModelConfig('CV');
            verifyEqual(obj, cfg.modelType, 'CV');
            verifyEqual(obj, cfg.stateDim, 4);
            
            cfg = dbt.MotionModelConfig('CA');
            verifyEqual(obj, cfg.modelType, 'CA');
            verifyEqual(obj, cfg.stateDim, 6);
            
            cfg = dbt.MotionModelConfig('CT');
            verifyEqual(obj, cfg.modelType, 'CT');
            verifyEqual(obj, cfg.stateDim, 5);
        end
        
        function testMotionModelConfigCustomParams(obj)
            cfg = dbt.MotionModelConfig('Singer', 'tau', 10, 'sigma_a', 5);
            verifyEqual(obj, cfg.tau, 10);
            verifyEqual(obj, cfg.sigma_a, 5);
        end
        
        function testAvailableModels(obj)
            models = dbt.MotionModelConfig.getAvailableModels();
            verifyTrue(obj, ismember('CV', models));
            verifyTrue(obj, ismember('CA', models));
            verifyTrue(obj, ismember('CT', models));
            verifyTrue(obj, ismember('SINGER', models));
            verifyTrue(obj, ismember('CS', models));
        end
    end
end
