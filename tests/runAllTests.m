function runAllTests()
% RUNALLTESTS  Execute all unit tests for the Target Tracking suite.
%   Runs tests for DBT filters, TBD algorithms, and visualization.
%
%   Usage:
%       runAllTests()
%       results = runAllTests()
%
%   Output:
%       results - Test results array (optional)

    fprintf('\n');
    fprintf('============================================================\n');
    fprintf('   Target Tracking Suite - Unit Test Runner\n');
    fprintf('============================================================\n\n');

    testFolder = fileparts(mfilename('fullpath'));
    if isempty(testFolder)
        testFolder = pwd;
    end

    addpath(genpath(fileparts(testFolder)));

    fprintf('Running DBT Filter Tests...\n');
    try
        dbtResults = runtests('TestDbtFilters');
        dbtPassed = sum([dbtResults.Passed]);
        dbtFailed = sum([dbtResults.Failed]);
        fprintf('  Passed: %d | Failed: %d\n\n', dbtPassed, dbtFailed);
    catch ME
        fprintf('  Error: %s\n\n', ME.message);
        dbtResults = [];
    end

    fprintf('Running TBD Algorithm Tests...\n');
    try
        tbdResults = runtests('TestTbdAlgorithms');
        tbdPassed = sum([tbdResults.Passed]);
        tbdFailed = sum([tbdResults.Failed]);
        fprintf('  Passed: %d | Failed: %d\n\n', tbdPassed, tbdFailed);
    catch ME
        fprintf('  Error: %s\n\n', ME.message);
        tbdResults = [];
    end

    fprintf('Running Visualization Tests...\n');
    try
        vizResults = runtests('TestVisualization');
        vizPassed = sum([vizResults.Passed]);
        vizFailed = sum([vizResults.Failed]);
        fprintf('  Passed: %d | Failed: %d\n\n', vizPassed, vizFailed);
    catch ME
        fprintf('  Error: %s\n\n', ME.message);
        vizResults = [];
    end

    fprintf('Running Maneuver Tracking Tests...\n');
    try
        maneuverResults = runtests('TestManeuverTracking');
        maneuverPassed = sum([maneuverResults.Passed]);
        maneuverFailed = sum([maneuverResults.Failed]);
        fprintf('  Passed: %d | Failed: %d\n\n', maneuverPassed, maneuverFailed);
    catch ME
        fprintf('  Error: %s\n\n', ME.message);
        maneuverResults = [];
    end

    allResults = [dbtResults, tbdResults, vizResults, maneuverResults];
    totalPassed = sum([allResults.Passed]);
    totalFailed = sum([allResults.Failed]);
    totalTests = length(allResults);

    fprintf('============================================================\n');
    fprintf('   Test Summary\n');
    fprintf('============================================================\n');
    fprintf('Total Tests: %d\n', totalTests);
    fprintf('Passed: %d (%.1f%%)\n', totalPassed, 100*totalPassed/totalTests);
    fprintf('Failed: %d (%.1f%%)\n', totalFailed, 100*totalFailed/totalTests);
    fprintf('============================================================\n\n');

    if totalFailed > 0
        fprintf('Failed Tests:\n');
        for i = 1:length(allResults)
            if ~allResults(i).Passed
                fprintf('  - %s\n', allResults(i).Name);
            end
        end
        fprintf('\n');
    end

    if nargout > 0
        varargout{1} = allResults;
    end
end
