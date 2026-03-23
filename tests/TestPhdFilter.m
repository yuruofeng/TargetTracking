classdef TestPhdFilter < matlab.unittest.TestCase
% TESTPHDFILTER  PHD Filter Unit Tests
%   Tests the correctness of each component in the +phd package.

    properties
        Model
        Tolerance = 1e-6
    end

    methods (TestMethodSetup)
        function setupModel(testCase)
            testCase.Model = phd.Config();
        end
    end

    methods (Test)

        function testGaussMixturePrune(testCase)
            w = [0.3; 0.5; 0.1; 0.05];
            x = [1 2 3 4; 1 2 3 4];
            P = repmat(eye(2), [1, 1, 4]);
            [wP, xP, PP] = phd.GaussMixture.prune(w, x, P, 0.1);
            testCase.assertEqual(numel(wP), 2, 'Prune should keep 2 components');
        end

        function testGaussMixtureCap(testCase)
            w = [0.3; 0.5; 0.1; 0.05];
            x = [1 2 3 4; 1 2 3 4];
            P = repmat(eye(2), [1, 1, 4]);
            [wC, xC, PC] = phd.GaussMixture.cap(w, x, P, 2);
            testCase.assertEqual(numel(wC), 2, 'Cap should limit to 2 components');
        end

        function testGaussMixtureExtractTargets(testCase)
            w = [0.3; 0.6; 0.1; 0.05];
            nT = phd.GaussMixture.extractTargets(w, 0.5);
            testCase.assertEqual(nT, 1, 'Should extract 1 target');
        end

        function testMathUtilsEnsurePositiveDefinite(testCase)
            A = [1 0.5; 0.5 1];
            AP = phd.MathUtils.ensurePositiveDefinite(A);
            eigVals = eig(AP);
            testCase.assertTrue(all(eigVals > 0), 'Matrix should be positive definite');
        end

        function testMathUtilsExpandState4to6(testCase)
            m4 = [1; 2; 3; 4];
            m6 = phd.MathUtils.expandState4to6(m4);
            testCase.assertEqual(size(m6, 1), 6, 'State should be 6D');
            testCase.assertEqual(m6(3), 0, 'Acceleration x should be 0');
            testCase.assertEqual(m6(6), 0, 'Acceleration y should be 0');
        end

        function testMathUtilsCoordinateConversion(testCase)
            cart = [1; 1];
            polar = phd.MathUtils.cartToPolar(cart);
            cartBack = phd.MathUtils.polarToCart(polar);
            testCase.assertEqual(cartBack, cart, 'RelTol', 1e-10, 'Coordinate conversion should be reversible');
        end

        function testCvFilter(testCase)
            model = testCase.Model;
            m = [100; 10; 200; 20; 0; 0];
            P = repmat(eye(6), [1, 1, 1]);
            [mPred, PPred] = phd.CvFilter.predict(model, m, P);
            testCase.assertEqual(size(mPred, 1), 4, 'Predicted state should be 4D');
            z = [100; 200];
            [qz, mUpd, PUpd] = phd.CvFilter.update(z, model, mPred, PPred);
            testCase.assertEqual(size(mUpd, 1), 4, 'Updated state should be 4D');
        end

        function testCtFilter(testCase)
            model = testCase.Model;
            m = [100; 10; 200; 20; 0; 0];
            P = repmat(eye(6), [1, 1, 1]);
            [mPred, PPred] = phd.CtFilter.predict(model, m, P);
            testCase.assertEqual(size(mPred, 1), 5, 'Predicted state should be 5D');
            xd = [100; 10; 200; 20; model.omega];
            xNew = phd.CtFilter.genNewState(model, xd, 'noiseless');
            testCase.assertEqual(size(xNew, 1), 5, 'New state should be 5D');
        end

        function testCaFilter(testCase)
            model = testCase.Model;
            m = [100; 10; 0; 200; 20; 0];
            P = repmat(eye(6), [1, 1, 1]);
            [mPred, PPred] = phd.CaFilter.predict(model, m, P);
            testCase.assertEqual(size(mPred, 1), 6, 'Predicted state should be 6D');
            z = [100; 200];
            [qz, mUpd, PUpd] = phd.CaFilter.update(z, model, mPred, PPred);
            testCase.assertEqual(size(mUpd, 1), 6, 'Updated state should be 6D');
        end

        function testSimmUtils(testCase)
            model = testCase.Model;
            w = 0.5;
            m = [100; 10; 0; 200; 20; 0];
            P = eye(6);
            L = 1;
            miu = [1; 1; 1];
            est = struct('W', cell(100,3), 'X', cell(100,3), 'P', cell(100,3));
            [w0, m0, P0, cNorm] = phd.SimmUtils.inputInteraction(1, model, est, w, m, P, L, miu);
            testCase.assertEqual(length(w0), 3, 'Should have 3 model inputs');
        end

        function testHungarian(testCase)
            costMatrix = [4 1 3; 2 0 5; 3 2 2];
            [matching, cost] = utils.Hungarian.solve(costMatrix);
            testCase.assertEqual(cost, 5, 'Optimal cost should be 5');
            testCase.assertEqual(sum(matching(:)), 3, 'Should have 3 assignments');
            costMatrixInf = [4 Inf 3; 2 0 Inf; Inf 2 2];
            [matchingInf, costInf] = utils.Hungarian.solve(costMatrixInf);
            testCase.assertTrue(costInf < Inf, 'Should find finite cost');
        end

        function testOspaMetric(testCase)
            X = [0 10 20; 0 0 0];
            Y = [1 11 21; 0 0 0];
            [dist, locErr, cardErr] = utils.OspaMetric.compute(X, Y, 30, 1);
            testCase.assertEqual(dist, locErr, 'Cardinality error should be 0');
            testCase.assertTrue(locErr > 0, 'Localization error should be positive');
            [distEmpty, ~, ~] = utils.OspaMetric.compute([], [], 30, 1);
            testCase.assertEqual(distEmpty, 0, 'Distance between empty sets should be 0');
            [distOne, ~, cardErrOne] = utils.OspaMetric.compute(X, [], 30, 1);
            testCase.assertEqual(distOne, 30, 'Distance to empty set should be c');
            testCase.assertEqual(cardErrOne, 30, 'Cardinality error should be c');
        end

        function testConfig(testCase)
            model = phd.Config();
            testCase.assertEqual(model.M, 3, 'Should have 3 models');
            testCase.assertEqual(model.x_dimCV, 4, 'CV state dimension should be 4');
            testCase.assertEqual(model.x_dimCA, 6, 'CA state dimension should be 6');
            testCase.assertEqual(model.x_dimCT, 5, 'CT state dimension should be 5');
            testCase.assertEqual(model.P_D + model.Q_D, 1, 'Detection probability should sum to 1');
            testCase.assertEqual(model.P_S + model.Q_S, 1, 'Survival probability should sum to 1');
        end

        function testScenario(testCase)
            model = phd.Config();
            scenario = phd.Scenario(model);
            scenario = scenario.generateTruth();
            testCase.assertEqual(scenario.K, 100, 'Should have 100 time steps');
            testCase.assertTrue(scenario.total_tracks >= 1, 'Should have at least 1 target');
            scenario = scenario.generateMeasurements(scenario);
            testCase.assertEqual(length(scenario.Z), scenario.K, 'Should have measurements for all times');
        end

        function testImmPhdFilter(testCase)
            model = phd.Config();
            scenario = phd.Scenario(model);
            scenario = scenario.generateTruth();
            truth.X = scenario.X;
            truth.N = scenario.N;
            truth.K = scenario.K;
            truth.track_list = scenario.track_list;
            truth.total_tracks = scenario.total_tracks;
            scenario = scenario.generateMeasurements(scenario);
            meas.Z = scenario.Z;
            meas.K = scenario.K;
            filter = phd.ImmPhdFilter(model);
            est = filter.run(meas, truth);
            testCase.assertEqual(length(est.IMMX), meas.K, 'Should have estimates for all times');
            testCase.assertEqual(size(est.miu, 1), 3, 'Should have 3 model probabilities');
        end

        function testSimmPhdFilter(testCase)
            model = phd.Config();
            scenario = phd.Scenario(model);
            scenario = scenario.generateTruth();
            truth.X = scenario.X;
            truth.N = scenario.N;
            truth.K = scenario.K;
            truth.track_list = scenario.track_list;
            truth.total_tracks = scenario.total_tracks;
            scenario = scenario.generateMeasurements(scenario);
            meas.Z = scenario.Z;
            meas.K = scenario.K;
            filter = phd.SimmPhdFilter(model);
            est = filter.run(meas, truth);
            testCase.assertEqual(length(est.IMMX), meas.K, 'Should have estimates for all times');
        end

    end
end
