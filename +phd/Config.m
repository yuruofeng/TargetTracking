classdef Config
% PHD.CONFIG  PHD Filter Model Configuration Class
%   Defines parameters for CV/CT/CA motion models, observation, clutter, and birth.
%
%   Properties:
%       Motion model: F_CV, F_CA, F_CT, Q_CV, Q_CA, Q_CT, etc.
%       Observation: H_*, R_*, D_*
%       Detection: P_D, Q_D, P_S, Q_S
%       Clutter: lambda_c, range_c, pdf_c
%       Birth: w_birth, m_birth*, P_birth*
%       Transition: Pmn (IMM), Sij (SIMM)
%
%   Usage:
%       model = phd.Config();           % Default configuration
%       model = phd.Config('T', 0.5);   % Custom sampling period
%
%   See also: phd.Scenario, phd.MultiModelFilter

    properties
        x_dimCV = 4       % CV state dimension [x vx y vy]
        x_dimCT = 5       % CT state dimension [x vx y vy omega]
        x_dimCA = 6       % CA state dimension [x vx ax y vy ay]
        z_dim   = 2       % Observation dimension [x y]
        M       = 3       % Number of models

        T = 1             % Sampling period

        Pmn               % IMM transition probability matrix
        Sij               % SIMM transition possibility matrix

        F_CV              % CV state transition matrix
        G_CV              % CV process noise gain
        Q_CV              % CV process noise covariance
        sigma_cv = 5      % CV process noise standard deviation
        H_CV              % CV observation matrix
        D_CV              % CV observation noise gain
        R_CV              % CV observation noise covariance

        F_CA              % CA state transition matrix
        G_CA              % CA process noise gain
        Q_CA              % CA process noise covariance
        sigma_ca = 8      % CA process noise standard deviation
        B_CA              % CA acceleration noise gain
        H_CA              % CA observation matrix
        D_CA              % CA observation noise gain
        R_CA              % CA observation noise covariance
        a                 % Acceleration bias vector
        ax = -0.05
        ay = 0.1

        F_CT              % CT state transition matrix
        G_CT              % CT process noise gain
        Q_CT              % CT process noise covariance
        omega = pi/180    % Turn rate (rad/s)
        sigma_vel = 5     % Velocity noise standard deviation
        sigma_turn = pi/720  % Turn rate noise standard deviation
        B_CT              % CT noise gain matrix
        H_CT              % CT observation matrix
        D_CT              % CT observation noise gain
        R_CT              % CT observation noise covariance

        G                 % Process noise gain cell array
        Q                 % Process noise covariance cell array

        maneuvers         % Maneuver interval matrix
        tbirth            % Birth time vector
        tdeath            % Death time vector

        P_S = 0.99        % Survival probability
        Q_S               % Death probability

        L_birth = 4       % Number of birth components
        w_birth           % Birth weight vector
        m_birthCV         % CV birth mean matrix
        B_birthCV         % CV birth covariance gain
        P_birthCV         % CV birth covariance
        m_birthCA         % CA birth mean matrix
        B_birthCA         % CA birth covariance gain
        P_birthCA         % CA birth covariance
        m_birthCT         % CT birth mean matrix
        B_birthCT         % CT birth covariance gain
        P_birthCT         % CT birth covariance

        P_D = 0.98        % Detection probability
        Q_D               % Miss detection probability

        lambda_c = 10     % Clutter rate (Poisson mean)
        range_c           % Clutter spatial range
        pdf_c             % Clutter spatial PDF
    end

    methods

        function obj = Config(varargin)
        % CONFIG  Create PHD model configuration object.
        %
        %   Inputs:
        %       'Name', Value - Optional name-value pairs
        %           'T'          - Sampling period (default: 1)
        %           'P_D'        - Detection probability (default: 0.98)
        %           'P_S'        - Survival probability (default: 0.99)
        %           'lambda_c'   - Clutter rate (default: 10)
        %           'omega'      - CT turn rate (default: pi/180)

            p = inputParser;
            addParameter(p, 'T', 1);
            addParameter(p, 'P_D', 0.98);
            addParameter(p, 'P_S', 0.99);
            addParameter(p, 'lambda_c', 10);
            addParameter(p, 'omega', pi/180);
            parse(p, varargin{:});

            obj.T = p.Results.T;
            obj.P_D = p.Results.P_D;
            obj.omega = p.Results.omega;
            obj.lambda_c = p.Results.lambda_c;
            obj.P_S = p.Results.P_S;

            obj = obj.initModelParams();
        end

        function obj = initModelParams(obj)
        % INITMODELPARAMS  Initialize all model parameters.

            obj.Q_S = 1 - obj.P_S;
            obj.Q_D = 1 - obj.P_D;

            obj.Pmn = [0.9, 0.05, 0.05;
                       0.05, 0.9, 0.05;
                       0.05, 0.05, 0.9];

            obj.Sij = [1,   0.2, 0.2;
                       0.2, 1,   0.2;
                       0.2, 0.2, 1];

            A0 = [1, obj.T; 0, 1];
            obj.F_CV = [A0, zeros(2); zeros(2), A0];

            G0 = [(obj.T^2)/2; obj.T];
            obj.G_CV = [G0, zeros(2,1); zeros(2,1), G0];
            obj.Q_CV = obj.sigma_cv^2 * (obj.G_CV * obj.G_CV');

            obj.H_CV = [1 0 0 0; 0 0 1 0];
            obj.D_CV = diag([5; 5]);
            obj.R_CV = obj.D_CV * obj.D_CV';

            obj.a  = [0; 0; 0; 0; -0.5; 1];

            A1 = [1, obj.T, (obj.T^2)/2; 0, 1, obj.T; 0, 0, 1];
            obj.F_CA = [A1, zeros(3); zeros(3), A1];

            obj.G_CA = [(obj.T^2)/2, 0;
                        obj.T,      0;
                        1,          0;
                        0, (obj.T^2)/2;
                        0,  obj.T;
                        0,  1];
            obj.B_CA = eye(2);
            obj.Q_CA = obj.B_CA * obj.B_CA';

            obj.H_CA = [1 0 0 0 0 0; 0 0 0 1 0 0];
            obj.D_CA = diag([5; 5]);
            obj.R_CA = obj.D_CA * obj.D_CA';

            a = sin(obj.T * obj.omega) / obj.omega;
            b = (1 - cos(obj.T * obj.omega)) / obj.omega;
            c = cos(obj.T * obj.omega);
            d = sin(obj.T * obj.omega);
            obj.F_CT = [1 a 0 -b 0; 0 c 0 -d 0; 0 b 1 a 0; 0 d 0 c 0; 0 0 0 0 1];

            bt = obj.sigma_vel * [(obj.T^2)/2; obj.T];
            obj.G_CT = [bt, zeros(2,2); zeros(2,1), bt, zeros(2,1); ...
                        zeros(1,2), obj.T * obj.sigma_turn];
            obj.B_CT = eye(3);
            obj.Q_CT = obj.B_CT * obj.B_CT';

            obj.H_CT = [1 0 0 0 0; 0 0 1 0 0; 0 0 0 0 1];
            obj.D_CT = diag([5; 5]);
            obj.R_CT = obj.D_CT * obj.D_CT';

            obj.G = {obj.G_CV; obj.G_CA; obj.G_CT};
            obj.Q = {obj.Q_CV; obj.Q_CA; obj.Q_CT};

            obj.maneuvers = [1, 31, 61; 1, 2, 3];
            obj.tbirth = [1, 31, 61];
            obj.tdeath = [30, 60, 101];

            obj.w_birth = zeros(obj.L_birth, 1);
            obj.m_birthCV = zeros(obj.x_dimCV, obj.L_birth);
            obj.B_birthCV = zeros(obj.x_dimCV, obj.x_dimCV, obj.L_birth);
            obj.P_birthCV = zeros(obj.x_dimCV, obj.x_dimCV, obj.L_birth);
            obj.m_birthCA = zeros(obj.x_dimCA, obj.L_birth);
            obj.B_birthCA = zeros(obj.x_dimCA, obj.x_dimCA, obj.L_birth);
            obj.P_birthCA = zeros(obj.x_dimCA, obj.x_dimCA, obj.L_birth);
            obj.m_birthCT = zeros(obj.x_dimCT, obj.L_birth);
            obj.B_birthCT = zeros(obj.x_dimCT, obj.x_dimCT, obj.L_birth);
            obj.P_birthCT = zeros(obj.x_dimCT, obj.x_dimCT, obj.L_birth);

            birthMeansCV = {[600;0;800;0], [400;0;-600;0], [-800;0;-200;0], [-200;0;800;0]};
            birthMeansCA = {[600;0;0;800;0;0], [400;0;0;-600;0;0], [-800;0;0;-200;0;0], [-200;0;0;800;0;0]};
            birthMeansCT = {[600;0;800;0;0], [400;0;-600;0;0], [-800;0;-200;0;0], [-200;0;800;0;0]};

            for ii = 1:obj.L_birth
                obj.w_birth(ii) = 3/100;

                obj.m_birthCV(:, ii)   = birthMeansCV{ii};
                obj.B_birthCV(:, :, ii) = diag(10 * ones(obj.x_dimCV, 1));
                obj.P_birthCV(:, :, ii) = obj.B_birthCV(:, :, ii)^2;

                obj.m_birthCA(:, ii)   = birthMeansCA{ii};
                obj.B_birthCA(:, :, ii) = diag(10 * ones(obj.x_dimCA, 1));
                obj.P_birthCA(:, :, ii) = obj.B_birthCA(:, :, ii)^2;

                obj.m_birthCT(:, ii)   = birthMeansCT{ii};
                obj.B_birthCT(:, :, ii) = diag([10; 10; 10; 10; 6*(pi/720)]);
                obj.P_birthCT(:, :, ii) = obj.B_birthCT(:, :, ii)^2;
            end

            obj.range_c  = [0, 2000; 0, 2000];
            obj.pdf_c    = 1 / prod(obj.range_c(:, 2) - obj.range_c(:, 1));
        end

    end
end
