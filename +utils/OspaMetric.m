classdef OspaMetric
% UTILS.OSPAMETRIC  OSPA Distance Metric Utilities
%   Implements Optimal Sub-Pattern Assignment (OSPA) distance for
%   multi-target tracking performance evaluation.
%
%   OSPA Distance Components:
%     - Localization: Position error
%     - Cardinality: Target count error
%     - Total: Weighted combination
%
%   Methods:
%       compute     - Compute OSPA distance between two finite sets
%       computeSeq  - Compute OSPA distance for time sequences
%       computeMean - Compute mean OSPA distance over sequences
%
%   Reference:
%       Schuhmacher, D., Vo, B.-T., & Vo, B.-N. (2008).
%       A consistent metric for performance evaluation in
%       multi-object filtering. IEEE TSP.
%
%   See also: utils.Hungarian, phd.GaussMixture

    properties (Constant)
        DEFAULT_C = 100     % Default cutoff parameter
        DEFAULT_P = 1       % Default order parameter
    end

    methods (Static)

        function [dist, locErr, cardErr] = compute(X, Y, c, p)
        % COMPUTE  Compute OSPA distance between two finite sets.
        %
        %   Inputs:
        %       X - Ground truth matrix [dim x n]
        %       Y - Estimate matrix [dim x m]
        %       c - Cutoff parameter (default: 100)
        %       p - Order parameter (default: 1)
        %
        %   Outputs:
        %       dist    - Total OSPA distance
        %       locErr  - Localization error component
        %       cardErr - Cardinality error component

            if nargin < 3 || isempty(c)
                c = utils.OspaMetric.DEFAULT_C;
            end
            if nargin < 4 || isempty(p)
                p = utils.OspaMetric.DEFAULT_P;
            end

            if isempty(X) && isempty(Y)
                dist = 0;
                locErr = 0;
                cardErr = 0;
                return;
            end

            if isempty(X) || isempty(Y)
                dist = c;
                locErr = 0;
                cardErr = c;
                return;
            end

            n = size(X, 2);
            m = size(Y, 2);

            XX = repmat(X, [1, m]);
            YY = reshape(repmat(Y, [n, 1]), [size(Y, 1), n * m]);
            D = reshape(sqrt(sum((XX - YY).^2)), [n, m]);
            D = min(c, D).^p;

            [assignment, cost] = utils.Hungarian.solve(D);

            dist = (1 / max(m, n) * (c^p * abs(m - n) + cost))^(1/p);

            if nargout > 1
                locErr = (1 / max(m, n) * cost)^(1/p);
                cardErr = (1 / max(m, n) * c^p * abs(m - n))^(1/p);
            end
        end

        function [distSeq, locSeq, cardSeq] = computeSeq(Xseq, Yseq, c, p)
        % COMPUTESEQ  Compute OSPA distance for time sequences.
        %
        %   Inputs:
        %       Xseq - Truth sequence (cell array, each element is [dim x n_k])
        %       Yseq - Estimate sequence (cell array, each element is [dim x m_k])
        %       c    - Cutoff parameter (default: 100)
        %       p    - Order parameter (default: 1)
        %
        %   Outputs:
        %       distSeq - Total OSPA distance sequence [1 x K]
        %       locSeq  - Localization error sequence [1 x K]
        %       cardSeq - Cardinality error sequence [1 x K]

            if nargin < 3 || isempty(c)
                c = utils.OspaMetric.DEFAULT_C;
            end
            if nargin < 4 || isempty(p)
                p = utils.OspaMetric.DEFAULT_P;
            end

            K = length(Xseq);
            distSeq = zeros(1, K);
            locSeq = zeros(1, K);
            cardSeq = zeros(1, K);

            for k = 1:K
                [distSeq(k), locSeq(k), cardSeq(k)] = utils.OspaMetric.compute(Xseq{k}, Yseq{k}, c, p);
            end
        end

        function [meanDist, meanLoc, meanCard] = computeMean(Xseq, Yseq, c, p)
        % COMPUTEMEAN  Compute mean OSPA distance over time sequences.
        %
        %   Inputs:
        %       Xseq - Truth sequence (cell array)
        %       Yseq - Estimate sequence (cell array)
        %       c    - Cutoff parameter
        %       p    - Order parameter
        %
        %   Outputs:
        %       meanDist - Mean OSPA distance
        %       meanLoc  - Mean localization error
        %       meanCard - Mean cardinality error

            [distSeq, locSeq, cardSeq] = utils.OspaMetric.computeSeq(Xseq, Yseq, c, p);
            meanDist = mean(distSeq);
            meanLoc = mean(locSeq);
            meanCard = mean(cardSeq);
        end

    end
end
