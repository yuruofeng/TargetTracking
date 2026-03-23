classdef Hungarian
% UTILS.HUNGARIAN  Hungarian Algorithm Implementation
%   Solves optimal bipartite matching problems, used for state-measurement
%   association in multi-target tracking.
%
%   Methods:
%       solve - Solve minimum weight matching problem
%
%   Reference:
%       Kuhn, H. W. (1955). The Hungarian method for the
%       assignment problem. Naval Research Logistics.
%
%   See also: utils.OspaMetric, phd.MultiModelFilter

    methods (Static)

        function [Matching, Cost] = solve(Perf)
        % SOLVE  Solve minimum weight matching problem.
        %
        %   Inputs:
        %       Perf - MxN weight matrix (Inf indicates no edge)
        %
        %   Outputs:
        %       Matching - MxN matching matrix (1 for match, 0 otherwise)
        %       Cost     - Total cost of minimum matching

            Matching = zeros(size(Perf));

            num_y = sum(~isinf(Perf), 1);
            num_x = sum(~isinf(Perf), 2);

            x_con = find(num_x ~= 0);
            y_con = find(num_y ~= 0);

            P_size = max(length(x_con), length(y_con));
            P_cond = zeros(P_size);
            P_cond(1:length(x_con), 1:length(y_con)) = Perf(x_con, y_con);
            if isempty(P_cond)
                Cost = 0;
                return;
            end

            Edge = P_cond;
            Edge(P_cond ~= Inf) = 0;
            cnum = utils.Hungarian.min_line_cover(Edge);

            Pmax = max(max(P_cond(P_cond ~= Inf)));
            P_size = length(P_cond) + cnum;
            P_cond = ones(P_size) * Pmax;
            P_cond(1:length(x_con), 1:length(y_con)) = Perf(x_con, y_con);

            exit_flag = 1;
            stepnum = 1;
            M = [];
            r_cov = [];
            c_cov = [];
            while exit_flag
                switch stepnum
                    case 1
                        [P_cond, stepnum] = utils.Hungarian.step1(P_cond);
                    case 2
                        [r_cov, c_cov, M, stepnum] = utils.Hungarian.step2(P_cond);
                    case 3
                        [c_cov, stepnum] = utils.Hungarian.step3(M, P_size);
                    case 4
                        [M, r_cov, c_cov, Z_r, Z_c, stepnum] = utils.Hungarian.step4(P_cond, r_cov, c_cov, M);
                    case 5
                        [M, r_cov, c_cov, stepnum] = utils.Hungarian.step5(M, Z_r, Z_c, r_cov, c_cov);
                    case 6
                        [P_cond, stepnum] = utils.Hungarian.step6(P_cond, r_cov, c_cov);
                    case 7
                        exit_flag = 0;
                end
            end

            Matching(x_con, y_con) = M(1:length(x_con), 1:length(y_con));
            Cost = sum(sum(Perf(Matching == 1)));
        end

    end

    methods (Static, Access = private)

        function [P_cond, stepnum] = step1(P_cond)
            P_size = length(P_cond);
            for ii = 1:P_size
                rmin = min(P_cond(ii, :));
                P_cond(ii, :) = P_cond(ii, :) - rmin;
            end
            stepnum = 2;
        end

        function [r_cov, c_cov, M, stepnum] = step2(P_cond)
            P_size = length(P_cond);
            r_cov = zeros(P_size, 1);
            c_cov = zeros(P_size, 1);
            M = zeros(P_size);

            for ii = 1:P_size
                for jj = 1:P_size
                    if P_cond(ii, jj) == 0 && r_cov(ii) == 0 && c_cov(jj) == 0
                        M(ii, jj) = 1;
                        r_cov(ii) = 1;
                        c_cov(jj) = 1;
                    end
                end
            end

            r_cov = zeros(P_size, 1);
            c_cov = zeros(P_size, 1);
            stepnum = 3;
        end

        function [c_cov, stepnum] = step3(M, P_size)
            c_cov = sum(M, 1);
            if sum(c_cov) == P_size
                stepnum = 7;
            else
                stepnum = 4;
            end
        end

        function [M, r_cov, c_cov, Z_r, Z_c, stepnum] = step4(P_cond, r_cov, c_cov, M)
            P_size = length(P_cond);
            zflag = 1;
            Z_r = 0;
            Z_c = 0;

            while zflag
                row = 0; col = 0; exit_flag = 1;
                ii = 1; jj = 1;
                while exit_flag
                    if P_cond(ii, jj) == 0 && r_cov(ii) == 0 && c_cov(jj) == 0
                        row = ii;
                        col = jj;
                        exit_flag = 0;
                    end
                    jj = jj + 1;
                    if jj > P_size; jj = 1; ii = ii + 1; end
                    if ii > P_size; exit_flag = 0; end
                end

                if row == 0
                    stepnum = 6;
                    zflag = 0;
                    Z_r = 0;
                    Z_c = 0;
                else
                    M(row, col) = 2;
                    if sum(find(M(row, :) == 1)) ~= 0
                        r_cov(row) = 1;
                        zcol = find(M(row, :) == 1);
                        c_cov(zcol) = 0;
                    else
                        stepnum = 5;
                        zflag = 0;
                        Z_r = row;
                        Z_c = col;
                    end
                end
            end
        end

        function [M, r_cov, c_cov, stepnum] = step5(M, Z_r, Z_c, r_cov, c_cov)
            zflag = 1;
            ii = 1;
            while zflag
                rindex = find(M(:, Z_c(ii)) == 1);
                if ~isempty(rindex) && rindex > 0
                    ii = ii + 1;
                    Z_r(ii, 1) = rindex;
                    Z_c(ii, 1) = Z_c(ii - 1);
                else
                    zflag = 0;
                end

                if zflag == 1
                    cindex = find(M(Z_r(ii), :) == 2);
                    ii = ii + 1;
                    Z_r(ii, 1) = Z_r(ii - 1);
                    Z_c(ii, 1) = cindex;
                end
            end

            for ii = 1:length(Z_r)
                if M(Z_r(ii), Z_c(ii)) == 1
                    M(Z_r(ii), Z_c(ii)) = 0;
                else
                    M(Z_r(ii), Z_c(ii)) = 1;
                end
            end

            r_cov = r_cov .* 0;
            c_cov = c_cov .* 0;
            M(M == 2) = 0;

            stepnum = 3;
        end

        function [P_cond, stepnum] = step6(P_cond, r_cov, c_cov)
            a = find(r_cov == 0);
            b = find(c_cov == 0);
            minval = min(min(P_cond(a, b)));

            P_cond(find(r_cov == 1), :) = P_cond(find(r_cov == 1), :) + minval;
            P_cond(:, find(c_cov == 0)) = P_cond(:, find(c_cov == 0)) - minval;

            stepnum = 4;
        end

        function cnum = min_line_cover(Edge)
            [r_cov, c_cov, M, ~] = utils.Hungarian.step2(Edge);
            [c_cov, ~] = utils.Hungarian.step3(M, length(Edge));
            [~, r_cov, c_cov, ~, ~, ~] = utils.Hungarian.step4(Edge, r_cov, c_cov, M);
            cnum = length(Edge) - sum(r_cov) - sum(c_cov);
        end

    end
end
