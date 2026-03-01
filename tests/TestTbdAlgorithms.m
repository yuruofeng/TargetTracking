classdef TestTbdAlgorithms < matlab.unittest.TestCase
% TESTTBDALGORITHMS  TBD算法单元测试。
%   测试DP-TBD和PF-TBD算法的正确性。
%
%   运行测试：
%       results = runtests('TestTbdAlgorithms')
%       results = TestTbdAlgorithms.run()

    properties
        config
        trueState
        measData
        psfKernel
    end

    methods (TestMethodSetup)

        function setupScenario(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            if isempty(testFolder)
                testFolder = pwd;
            end
            addpath(genpath(fileparts(testFolder)));
            
            testCase.config = tbd.Config('numFrames', 20, 'numParticles', 100, ...
                                         'gridSize', [50, 50]);
            rng(testCase.config.rngSeed);
            scenario = tbd.Scenario(testCase.config);
            [~, testCase.trueState, testCase.measData, testCase.psfKernel] = scenario.generate();
        end

    end

    methods (Test)

        function testDpTbdRun(testCase)
            dp = tbd.DpTbd(testCase.config);
            dp.run(testCase.measData, testCase.psfKernel);
            [track, score, ~] = dp.getResults();

            testCase.assertEqual(size(track, 1), testCase.config.numFrames);
            testCase.assertEqual(size(track, 2), 2);
            testCase.verifyGreaterThan(score, -inf);

            posRmse = mean(sqrt(sum((track - testCase.trueState(:, 1:2)).^2, 2)));
            testCase.verifyLessThan(posRmse, 10, 'DP-TBD位置RMSE应合理');
        end

        function testDpTbdInheritance(testCase)
            dp = tbd.DpTbd(testCase.config);
            testCase.verifyTrue(isa(dp, 'tbd.BaseTbd'), 'DpTbd应继承自BaseTbd');
        end

        function testDpTbdComputeRmse(testCase)
            dp = tbd.DpTbd(testCase.config);
            dp.run(testCase.measData, testCase.psfKernel);
            [posRmse, velRmse] = dp.computeRmse(testCase.trueState);
            
            testCase.assertEqual(length(posRmse), testCase.config.numFrames);
            testCase.assertEqual(length(velRmse), testCase.config.numFrames);
        end

        function testPfTbdRun(testCase)
            pf = tbd.PfTbd(testCase.config);
            pf.run(testCase.measData, testCase.trueState(1, :), testCase.psfKernel);
            
            state = pf.getEstimate();
            testCase.assertEqual(size(state, 1), testCase.config.numFrames);
            testCase.assertEqual(size(state, 2), 5);

            [posRmse, ~] = pf.computeRmse(testCase.trueState);
            testCase.verifyLessThan(mean(posRmse), 10, 'PF-TBD位置RMSE应合理');
        end

        function testPfTbdInheritance(testCase)
            pf = tbd.PfTbd(testCase.config);
            testCase.verifyTrue(isa(pf, 'tbd.BaseTbd'), 'PfTbd应继承自BaseTbd');
        end

        function testPfTbdGetResults(testCase)
            pf = tbd.PfTbd(testCase.config);
            pf.run(testCase.measData, testCase.trueState(1, :), testCase.psfKernel);
            [track, state] = pf.getResults();
            
            testCase.assertEqual(size(track, 1), testCase.config.numFrames);
            testCase.assertEqual(size(track, 2), 2);
            testCase.assertEqual(size(state, 2), 5);
        end

        function testTbdConfig(testCase)
            cfg = tbd.Config();
            testCase.assertEqual(cfg.numFrames, 50);
            testCase.assertEqual(cfg.gridSize, [100, 100]);
            testCase.assertEqual(cfg.numParticles, 500);
        end

        function testTbdConfigSnr(testCase)
            cfg = tbd.Config('amplitude', 3, 'noiseStd', 1);
            snr = cfg.getSnr();
            expectedSnr = 20 * log10(3);
            testCase.verifyEqual(snr, expectedSnr, 'AbsTol', 0.1);
        end

        function testTbdScenarioTrajectory(testCase)
            scenario = tbd.Scenario(testCase.config);
            localTrueState = scenario.generateTrajectory();

            testCase.assertEqual(size(localTrueState, 1), testCase.config.numFrames);
            testCase.assertEqual(size(localTrueState, 2), 5);

            velDiff = diff(localTrueState(:, 3:4));
            testCase.verifyTrue(all(abs(velDiff(:)) < 1e-10), '速度应恒定');
        end

        function testTbdScenarioMeasurements(testCase)
            scenario = tbd.Scenario(testCase.config);
            localTrueState = scenario.generateTrajectory();
            [localMeasData, localPsfKernel] = scenario.generateMeasurements(localTrueState);

            testCase.assertEqual(size(localMeasData, 3), testCase.config.numFrames);
            testCase.assertEqual(size(localMeasData, 1), testCase.config.gridSize(1));
            testCase.assertEqual(size(localMeasData, 2), testCase.config.gridSize(2));
            testCase.assertEqual(size(localPsfKernel, 1), 2*testCase.config.targetRadius + 1);
        end

        function testPsfKernel(testCase)
            [psfKernel, ~] = utils.MeasurementModel.createPsfKernel(3);
            
            testCase.assertEqual(size(psfKernel, 1), 7);
            testCase.assertEqual(size(psfKernel, 2), 7);
            testCase.verifyEqual(max(psfKernel(:)), 1, 'AbsTol', 1e-10);
        end

        function testDpTbdDetectionMap(testCase)
            dp = tbd.DpTbd(testCase.config);
            localDetMap = dp.computeDetectionMap(testCase.measData, testCase.psfKernel);

            testCase.assertEqual(size(localDetMap, 1), testCase.config.gridSize(1));
            testCase.assertEqual(size(localDetMap, 2), testCase.config.gridSize(2));
            testCase.assertEqual(size(localDetMap, 3), testCase.config.numFrames);
        end

        function testPfTbdParticleInit(testCase)
            pf = tbd.PfTbd(testCase.config);
            initState = [25, 30, 1, 0.5, 3];
            pf.init(initState);
            
            testCase.verifyTrue(pf.hasValidConfig(), '配置应有效');
        end

        function testTbdFactory(testCase)
            dp = tbd.TbdFactory.create('DP');
            pf = tbd.TbdFactory.create('PF');
            
            testCase.verifyTrue(isa(dp, 'tbd.DpTbd'));
            testCase.verifyTrue(isa(pf, 'tbd.PfTbd'));
        end

        function testTbdFactoryCreateAll(testCase)
            [dp, pf] = tbd.TbdFactory.createAll();
            
            testCase.verifyTrue(isa(dp, 'tbd.DpTbd'));
            testCase.verifyTrue(isa(pf, 'tbd.PfTbd'));
        end

        function testBaseTbdInterface(testCase)
            dp = tbd.DpTbd(testCase.config);
            
            testCase.verifyTrue(dp.hasValidConfig());
            testCase.verifyFalse(dp.isAlgorithmRun);
            
            dp.run(testCase.measData, testCase.psfKernel);
            testCase.verifyTrue(dp.isAlgorithmRun);
        end

    end
end
