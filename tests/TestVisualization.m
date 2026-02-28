classdef TestVisualization < matlab.unittest.TestCase
% TESTVISUALIZATION  Unit tests for visualization utilities.
%   Tests the viz.Visualizer class methods.

    properties
        tempFigFolder
    end

    methods (TestMethodSetup)

        function setup(testCase)
            testFolder = fileparts(mfilename('fullpath'));
            if isempty(testFolder)
                testFolder = pwd;
            end
            addpath(genpath(fileparts(testFolder)));
            
            testCase.tempFigFolder = tempdir;
        end

    end

    methods (Test)

        function testCreateFigure(testCase)
            fig = viz.Visualizer.createFigure('Test', 800, 600);
            
            testCase.verifyTrue(ishandle(fig));
            testCase.assertEqual(fig.Name, 'Test');
            close(fig);
        end

        function testColorPalette(testCase)
            colors = viz.Visualizer.COLORS;
            
            testCase.verifyTrue(isfield(colors, 'blue'));
            testCase.verifyTrue(isfield(colors, 'orange'));
            testCase.verifyTrue(isfield(colors, 'cyan'));
            testCase.verifyTrue(isfield(colors, 'green'));
            
            testCase.assertEqual(length(colors.blue), 3);
            testCase.verifyTrue(all(colors.blue >= 0 & colors.blue <= 1));
        end

        function testPlotDbtTrajectory(testCase)
            nSteps = 50;
            truthX = zeros(5, nSteps);
            truthX(1, :) = linspace(0, 100, nSteps);
            truthX(3, :) = linspace(0, 50, nSteps);
            meas = [atan2(truthX(3,:), truthX(1,:)); sqrt(truthX(1,:).^2 + truthX(3,:).^2)];
            estimates = {truthX, truthX};
            labels = {'Alg1', 'Alg2'};

            fig = viz.Visualizer.createFigure('Test', 600, 400);
            viz.Visualizer.plotDbtTrajectory(truthX, meas, estimates, labels);
            
            testCase.verifyTrue(ishandle(fig));
            close(fig);
        end

        function testPlotRmseComparison(testCase)
            rmseData = {rand(1, 50), rand(1, 50)};
            labels = {'Alg1', 'Alg2'};

            fig = viz.Visualizer.createFigure('Test', 600, 400);
            viz.Visualizer.plotRmseComparison(rmseData, labels, 'Test RMSE');
            
            testCase.verifyTrue(ishandle(fig));
            close(fig);
        end

        function testPlotRmseBarChart(testCase)
            meanRmse = [1.5, 2.0; 0.5, 0.8];
            labels = {'Alg1', 'Alg2'};
            groups = {'Pos', 'Vel'};

            fig = viz.Visualizer.createFigure('Test', 600, 400);
            viz.Visualizer.plotRmseBarChart(meanRmse, labels, groups, 'Test');
            
            testCase.verifyTrue(ishandle(fig));
            close(fig);
        end

        function testPlotHeatmap(testCase)
            data = rand(20, 20);

            fig = viz.Visualizer.createFigure('Test', 600, 400);
            viz.Visualizer.plotHeatmap(data, 'X', 'Y', 'Test Heatmap');
            
            testCase.verifyTrue(ishandle(fig));
            close(fig);
        end

        function testPlotTimingComparison(testCase)
            timingData = [0.5, 0.8, 0.3, 1.2];
            labels = {'EKF', 'UKF', 'CKF', 'PF'};

            fig = viz.Visualizer.createFigure('Test', 600, 400);
            viz.Visualizer.plotTimingComparison(timingData, labels);
            
            testCase.verifyTrue(ishandle(fig));
            close(fig);
        end

    end
end
