function demoAll(varargin)
% DEMOALL  Complete target tracking algorithm demonstration.
%   Runs both DBT and TBD algorithm demonstrations with comprehensive
%   visualization and performance comparison.
%
%   Usage:
%       demoAll()                    % Run both demos with defaults
%       demoAll('mode', 'dbt')       % Run only DBT demo
%       demoAll('mode', 'tbd')       % Run only TBD demo
%       demoAll('mcRuns', 5)         % 5 MC runs for DBT
%
%   Parameters (name-value pairs):
%       mode        - 'both', 'dbt', or 'tbd' (default: 'both')
%       mcRuns      - MC runs for DBT (default: 1)
%       saveResults - Save results to files (default: false)
%       showPlots   - Display plots (default: true)
%
%   Outputs:
%       results - Struct containing DBT and TBD results
%
%   Example:
%       results = demoAll('mode', 'both', 'mcRuns', 3);
%
%   See also: demoDbt, demoTbd

    close all; clc;

    fprintf('\n');
    fprintf('================================================================\n');
    fprintf('    Target Tracking Algorithm Suite - Complete Demonstration\n');
    fprintf('================================================================\n\n');

    p = inputParser;
    addParameter(p, 'mode', 'both', @(x) ismember(lower(x), {'both', 'dbt', 'tbd'}));
    addParameter(p, 'mcRuns', 1, @isscalar);
    addParameter(p, 'saveResults', false, @islogical);
    addParameter(p, 'showPlots', true, @islogical);
    parse(p, varargin{:});
    opts = p.Results;

    results = struct();
    mode = lower(opts.mode);

    if ismember(mode, {'both', 'dbt'})
        fprintf('\n>>> Running DBT Demo <<<\n');
        dbtResults = demoDbt('mcRuns', opts.mcRuns, ...
                            'saveResults', opts.saveResults, ...
                            'saveFile', 'dbt_results.mat', ...
                            'showPlots', opts.showPlots);
        results.dbt = dbtResults;
    end

    if ismember(mode, {'both', 'tbd'})
        fprintf('\n>>> Running TBD Demo <<<\n');
        tbdResults = demoTbd('saveResults', opts.saveResults, ...
                            'saveFile', 'tbd_results.mat', ...
                            'showPlots', opts.showPlots);
        results.tbd = tbdResults;
    end

    fprintf('\n================================================================\n');
    fprintf('    All Demonstrations Complete\n');
    fprintf('================================================================\n\n');

    if nargout > 0
        varargout{1} = results;
    end
end
