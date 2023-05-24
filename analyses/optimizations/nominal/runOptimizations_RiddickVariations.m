% A series of optimizations where each subsequent optimization adds an
% additional correction or parameter for the Riddick AHRS filter.

%% Files
opts = defaultDataOpts;
opts.subjects = [1:4 6:12];
opts.conditions = {'standing_ref1' 'standing_bending' 'right_left_lift' ...
    'sitting_bend' 'sit_stand_normal' 'sit_stand_fast' 'walk_3kmh' ...
    'walk_5kmh' 'og_walk_sss' 'figure8_walking' 'og_jog_sss' };

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Make sure the data directory folder is set properly to the location on
% your computer within setFileNames.m
files = setFileNames(opts);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Where to save the optimized filter parameters
% nomFreq = 60;

% Basic 2 gain Riddick filter with no additional corrections or parameters
file_basicRiddick = [files.optimizedParametersDir 'basicRiddick.mat'];

% For the Riddick filters:
% Show difference when gradient step is un-normalized
file_Riddick_NormStep =  [files.optimizedParametersDir 'Riddick_UnnormedStep.mat'];

% Show the difference when using prediction correction
file_Riddick_PredictionCorrection = [files.optimizedParametersDir 'Riddick_PredictionCorrection.mat'];

% Optimize with a gain on gyro
file_Riddick_GyroGain = [files.optimizedParametersDir 'Riddick_GyroGain.mat'];

% Optimize with second order integration model
file_Riddick_2ndOrderIntegration = [files.optimizedParametersDir 'Riddick_2ndOrderIntegration.mat'];




% CHANGE THIS IF YOU WANT TO SAVE FIGURES SOMEWHERE
printFigs = false;


%% General options
% SET TO TRUE IF YOU'VE CHANGED OPTIONS FOR THE OPTIMIZATION AND WANT TO
% RERUN THE OPTIMIZATION WITHOUT HAVING TO DELETE THE OLD FILES.
forceReoptimize = true;

% Log space for sampling frequencies so we are not biased towards a
% frequency range when fitting an exponential model to error or parameters
% versus sampling frequcnies
% opts.SamplingFrequencies = nomFreq;
opts.SamplingFrequencies = fliplr(logspace(0,log10(190),10));


dataSetFile = [files.dataDir 'validation2021_dataset.mat'];


%% Load data
reExtractData = false;
if reExtractData || ~exist(dataSetFile,'file')
    D = extractData(opts,files);
    save(dataSetFile,'D');
else
    load(dataSetFile);
end

%% General Optimization options
optimOpts = defaultOptimOpts;

optimOpts.forceReoptimize = forceReoptimize;

% Option for Riddick filter to separate prediction & correction steps for
% the filter
opts.useCorrectionPrediction = false;

% Option for Riddick filter to specify higher order integration model
opts.integrationOrder = 1;

% Correct indices from old implementation to estimate state at next time
% step instead of current time step (doesn't change too much and also isn't as
% good as separating the correction/prediction steps as is done for Riddick
% filter)
opts.correctAHRSindices = true;

% For all GDFs (besides Riddick) normalize the gradient descent step (as is
% done in the references). Doesn't seem to actually affect overall optimal
% results so may be an unecessary computation
opts.GradDescent.normStepSize = true;
optimOpts.RiddickAHRS.normStepSize = true;


% Used for all filters besides FKF
optimOpts.MultiStart.Number = 4*2;
optimOpts.MultiStart.MaxIterations = 40;
optimOpts.MultiStart.UseParallel = true;


optimOpts.OptimDisplay = 'iter';
% optimOpts.OptimDisplay = 'none';

% Use magnetometer in the orientation estimates
opts.useMagnetometer = true;

% All sensors are used
opts.sensorsToUse = {'mdm650D' 'mdm64B1' 'mdm64C6' 'mdm64BF' 'mdm652E' 'mdm64A8' 'mdm64E1'};

% Which algorithms to use
opts.AHRS = [RiddickAHRS];


optimOpts.RiddickAHRS.parameters.Names = {'c_accelerometer' 'c_magnetometer'};
optimOpts.RiddickAHRS.parameters.Initial = [0.025 0.001];
optimOpts.RiddickAHRS.parameters.lb = [0.00001 0.00001];
optimOpts.RiddickAHRS.parameters.ub = [1 1];


opts.algorithms = [];
for i = 1 : length(opts.AHRS)
    opts.algorithms{i} = class(opts.AHRS(i));
end

sensorNames = opts.sensorsToUse;

optimOpts.AcrossSubjects = true;
optimOpts.AcrossConditions = true;
optimOpts.AcrossSensors = true;
optimOpts.AcrossSamplingFrequencies = false;
%% Basic Riddick filter

O = optimizeAHRSacrossDataset(D,file_basicRiddick,optimOpts,opts);
%% Un-normalized gradient step
optimOpts.RiddickAHRS.normStepSize = false;
O = optimizeAHRSacrossDataset(D,file_Riddick_NormStep,optimOpts,opts);

%% Optimize riddick filter without prediction correction
opts.useCorrectionPrediction = true;

O = optimizeAHRSacrossDataset(D,file_Riddick_PredictionCorrection ,optimOpts,opts);


%% Put gain on gyro

optimOpts.RiddickAHRS.parameters.Names = {'c_gyroscope' 'c_accelerometer' 'c_magnetometer'};
optimOpts.RiddickAHRS.parameters.Initial = [0.9 0.04 0.04];
optimOpts.RiddickAHRS.parameters.lb = [0.1 0.0001 0.0001];
optimOpts.RiddickAHRS.parameters.ub = [1.5 1 1];


O = optimizeAHRSacrossDataset(D,file_Riddick_GyroGain ,optimOpts,opts);

%% Test 2nd order integration
opts.integrationOrder = 2;


if opts.integrationOrder >= 2
    optimOpts.RiddickAHRS.parameters.Names = [optimOpts.RiddickAHRS.parameters.Names {'c_gyroscope2'}];
    optimOpts.RiddickAHRS.parameters.Initial = [optimOpts.RiddickAHRS.parameters.Initial 1];
    optimOpts.RiddickAHRS.parameters.lb = [optimOpts.RiddickAHRS.parameters.lb 0.0001];
    optimOpts.RiddickAHRS.parameters.ub = [optimOpts.RiddickAHRS.parameters.ub 1.1];
end

if opts.integrationOrder >= 3
    optimOpts.RiddickAHRS.parameters.Names = [optimOpts.RiddickAHRS.parameters.Names {'c_gyroscope3'}];
    optimOpts.RiddickAHRS.parameters.Initial = [optimOpts.RiddickAHRS.parameters.Initial 1];
    optimOpts.RiddickAHRS.parameters.lb = [optimOpts.RiddickAHRS.parameters.lb 0.0001];
    optimOpts.RiddickAHRS.parameters.ub = [optimOpts.RiddickAHRS.parameters.ub 1.1];
end

O = optimizeAHRSacrossDataset(D,file_Riddick_2ndOrderIntegration ,optimOpts,opts);