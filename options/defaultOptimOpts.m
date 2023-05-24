function optimOpts = defaultOptimOpts()


%% Attitude optimization options
% Options for which optimization solver to use
% 'fmincon' using sqp algorithm for a local optimization solution
% 'surrogate' for a surrogate global algorithm
% 'MultiStart' for the MultiStart global solver
% 'GlobalSearch' for the GlobalSearch solver
optimOpts.optimizationMethod = 'surrogate';

optimOpts.forceReoptimize = false;

% Set to always use fmincon for 1D optimization problems regardless of what
% optimOpts.optimizationMethod is.
optimOpts.alwaysUseLocalOptimFor1D = true;

optimOpts.intermediateSave = true;

% optimOpts.OptimDisplay = 'iter-detailed';
optimOpts.OptimDisplay = 'none';
%%

optimOpts.AcrossSubjects = true;
optimOpts.AcrossConditions = true;
optimOpts.AcrossSensors = true;
optimOpts.AcrossSamplingFrequencies = false;

%% AHRS-specific optimization options
% For each AHRS selected, specify which parameters, their initial values,
% and lower and upper bounds for the optimization. See the class definition
% files for the options for parameters are and what they signify.

% optimOpts.FastKF.parameters.Names = {'Sigma_g_scalar' 'Sigma_a_scalar' 'Sigma_m11' 'Sigma_m22' 'Sigma_m33'};
% optimOpts.FastKF.parameters.Initial = [1 10 10000 10000 10000];
% optimOpts.FastKF.parameters.lb = 1e-9*ones(1,5);
% optimOpts.FastKF.parameters.ub = [10 100 20000 20000 20000];

% optimOpts.FastKF.parameters.Names = {'Sigma_g_scalar' 'Sigma_a_scalar' 'Sigma_m_scalar'};
% optimOpts.FastKF.parameters.Initial = [0.1 1 900];
% optimOpts.FastKF.parameters.lb = 1e-12*ones(1,3);
% optimOpts.FastKF.parameters.ub = [10 10 1000];
optimOpts.FastKF.parameters.Names = {'Sigma_g_scalar' 'Sigma_a_scalar' 'Sigma_m_scalar'};
% optimOpts.FastKF.parameters.Initial = [1e-12 1e-12 1500];
optimOpts.FastKF.parameters.Initial = [ 0.01    0.01      1500];
% optimOpts.FastKF.parameters.lb = 1e-12*ones(1,3);
optimOpts.FastKF.parameters.lb = 1e-9*ones(1,3);
optimOpts.FastKF.parameters.ub = [1 100 10000];

% Subject-averaged sensor specific condition specific MultiStart optimal
% optimOpts.FastKF.parameters.Initial = [0.50629      0.32825      0.37581];
% Subject-averaged, sensor specific condition specific global search optimal
optimOpts.FastKF.parameters.Initial = [0.54678      0.66592      0.33617];


optimOpts.HoornAHRS.parameters.Names = {'Beta'};
optimOpts.HoornAHRS.parameters.Initial = 0.1;
optimOpts.HoornAHRS.parameters.lb = 1e-9;
optimOpts.HoornAHRS.parameters.ub = 1;
optimOpts.HoornAHRS.optimizer = 'MultiStart';

optimOpts.WilsonAHRS = optimOpts.HoornAHRS;
optimOpts.MadgwickAHRS3 = optimOpts.HoornAHRS;
optimOpts.AdmiraalAHRS = optimOpts.HoornAHRS;

optimOpts.JustaAHRS.parameters.Names = {'wAcc' 'wMag'};
optimOpts.JustaAHRS.parameters.Initial = [0.00248 0.001];
% optimOpts.JustaAHRS.parameters.Initial = [0.00248 0.00001];
% optimOpts.JustaAHRS.parameters.lb = 1e-12*ones(1,2);
optimOpts.JustaAHRS.parameters.lb = 1e-9*ones(1,2);
% optimOpts.JustaAHRS.parameters.ub = 0.1*ones(1,2);
optimOpts.JustaAHRS.parameters.ub = 1*ones(1,2);
optimOpts.JustaAHRS.optimizer = 'MultiStart';



optimOpts.RiddickAHRS.parameters.Names = {'c_gyroscope' 'c_accelerometer' 'c_magnetometer'};
optimOpts.RiddickAHRS.parameters.Initial = [1 0.025 0.001];
optimOpts.RiddickAHRS.parameters.lb = [0.0001 0.0001 0.0001];
optimOpts.RiddickAHRS.parameters.ub = [1.1 1 1];
optimOpts.RiddickAHRS.optimizer = 'MultiStart';


optimOpts.KF.parameters.Names = {'Sigma_g_scalar' 'Sigma_a_scalar' 'Sigma_m_scalar'};
% optimOpts.FastKF.parameters.Initial = [1e-12 1e-12 1500];
optimOpts.KF.parameters.Initial = [ 0.01    0.01      1500];
% optimOpts.FastKF.parameters.lb = 1e-12*ones(1,3);
optimOpts.KF.parameters.lb = 1e-9*ones(1,3);
optimOpts.KF.parameters.ub = [1 100 10000];

optimOpts.KF2 = optimOpts.KF;

% optimOpts.ticcKF.parameters.Names = {'can2' 'can1' 'can0' 'cad1' 'cad0'...
%             'cmn2' 'cmn1' 'cmn0' 'cmd1' 'cmd0'};
% optimOpts.ticcKF.parameters.Initial = ones(1,10);
% % optimOpts.ticcKF.parameters.lb = 1e-9*ones(1,10);
% optimOpts.ticcKF.parameters.lb = -1000*ones(1,10);
% optimOpts.ticcKF.parameters.ub = 1000*ones(1,10);

optimOpts.ticcKF.parameters.Names = {'can2' 'cmn2' 'd2' 'd0'};
optimOpts.ticcKF.parameters.Initial = ones(1,4);
optimOpts.ticcKF.parameters.lb = 1e-9*ones(1,4);
% optimOpts.ticcKF.parameters.lb = -1000*ones(1,4);
optimOpts.ticcKF.parameters.ub = 1000*ones(1,4);

optimOpts.ticcKFexp.parameters.Names =  {'CA' 'LA' 'OA' 'CM' 'LM' 'OM'};
optimOpts.ticcKFexp.parameters.Initial = [9 0.5 0.001 9 0.5 0.001];
optimOpts.ticcKFexp.parameters.lb = 1e-5*ones(1,6);
optimOpts.ticcKFexp.parameters.ub = [100 1 1 100 1 1];

optimOpts.eKF.parameters.Names = {'Sigma_a_scalar' 'Sigma_m_scalar'};
optimOpts.eKF.parameters.Initial = [40 200];
optimOpts.eKF.parameters.lb = [1 1];
optimOpts.eKF.parameters.ub = [1000 10000];

optimOpts.esKF = optimOpts.eKF;
optimOpts.esKF2 = optimOpts.esKF;

%%
optimOpts.RiddickAHRS.normStepSize = false;

%% Contour options
optimOpts.useCostContourFor1D = true;
optimOpts.numPointsCostContour = 20;
%% Fmincon options
optimOpts.fmincon.MaxIterations = 6;
optimOpts.fmincon.MaxFunctionEvaluations = 100;
%% Surrogate options
% Number of iterations for surrogate optimizer
optimOpts.surrogate.Iterations = 100;
% True to do a local optimization with fmincon after the surrogate
% optimization.
optimOpts.surrogate.fminconAfter = false;
optimOpts.surrogate.MinSurrogatePoints = 40;
optimOpts.surrogate.UseParallel = true;
optimOpts.surrogate.MinSampleDistance = 1e-6;
%% Multistart options
% Number of start points for MultiStart solver
optimOpts.MultiStart.Number = 12;
optimOpts.MultiStart.UseParallel = true;
optimOpts.MultiStart.MaxIterations = 10; % Max number of local solver fmincon iteraionts
 optimOpts.MultiStart.MaxFunctionEvaluations = 200;
%% Global Search options
optimOpts.GlobalSearch.NumTrialPoints = 100;
optimOpts.GlobalSearch.NumStageOnePoints = 20;
optimOpts.GlobalSearch.FunctionTolerance = 1e-2;
optimOpts.GlobalSearch.XTolerance = 1e-6;
%% Justa optimization algorith motps
optimOpts.Justa.MaxIter = 10;


