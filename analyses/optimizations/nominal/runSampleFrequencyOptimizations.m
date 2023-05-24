% Optimize filter parameters for various sets of data.
%
% 1) Across all participants & conditions at various sample frequencies
% 2) For each condition separately at each sample frequency

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

% Optimized across all participants & conditions for each sample frequency
file_SF = [files.optimizedParametersDir 'samplingFrequencyOptimized.mat'];
% Optimized across all participants for each specific condition at each
% sample frequency
file_SF_Condition = [files.optimizedParametersDir 'samplingFrequencyOptimized_perCondition.mat'];


% nomFreq = 60;
% Optimized across all participants and conditions at 60 Hz
% file_Nominal60 = [files.optimizedParametersDir 'nominalFrequency_60.mat'];
% % Optimized for each condition separately at 60 Hz
% file_Nominal60_perCondition = [files.optimizedParametersDir 'nominalFrequency_60_perCondition.mat'];


%% General options
% SET TO TRUE IF YOU'VE CHANGED OPTIONS FOR THE OPTIMIZATION AND WANT TO
% RERUN THE OPTIMIZATION WITHOUT HAVING TO DELETE THE OLD FILES.
forceReoptimize = true;

% Log space for sampling frequencies so we are not biased towards a
% frequency range when fitting an exponential model to error or parameters
% versus sampling frequcnies
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


optimOpts.alwaysUseLocalOptimFor1D = false;
optimOpts.useCostContourFor1D = false;

% Option for Riddick filter to separate prediction & correction steps for
% the filter
opts.useCorrectionPrediction = true;

% Option for Riddick filter to specify higher order integration model
opts.integrationOrder = 1;


% For all GDFs (besides Riddick) normalize the gradient descent step (as is
% done in the references). Doesn't seem to actually affect overall optimal
% results so may be an unecessary computation
opts.GradDescent.normStepSize = true;
optimOpts.RiddickAHRS.normStepSize = false;


% Used for all filters besides FKF
optimOpts.MultiStart.Number = 4*2;
optimOpts.MultiStart.MaxIterations = 40;
optimOpts.MultiStart.UseParallel = true;

% Used for fkf
optimOpts.FastKF.optimizer = 'surrogate';
optimOpts.surrogate.Iterations = 100;
optimOpts.surrogate.UseParallel = true;
optimOpts.surrogate.MinSurrogatePoints = 80;
optimOpts.surrogate.MinSampleDistance = 1e-6;

% Use magnetometer in the orientation estimates
opts.useMagnetometer = true;

% All sensors are used
opts.sensorsToUse = {'mdm650D' 'mdm64B1' 'mdm64C6' 'mdm64BF' 'mdm652E' 'mdm64A8' 'mdm64E1'};

% Which algorithms to use
opts.AHRS = [RiddickAHRS WilsonAHRS AdmiraalAHRS MadgwickAHRS3 FastKF JustaAHRS];
opts.algorithms = [];
for i = 1 : length(opts.AHRS)
    opts.algorithms{i} = class(opts.AHRS(i));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
optimOpts.startCount = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Optimize across all data for each sampling frequency
optimOpts.AcrossSubjects = true;
optimOpts.AcrossConditions = true;
optimOpts.AcrossSensors = true;
optimOpts.AcrossSamplingFrequencies = false;

optimOpts.OptimDisplay='iter';

O = optimizeAHRSacrossDataset(D,file_SF,optimOpts,opts);

%% Optimized for each condition and each sample frequency
optimOpts.AcrossConditions = false;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
optimOpts.startCount = 1;
% optimOpts.endCount = 12;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
optimOpts.OptimDisplay='none';
O = optimizeAHRSacrossDataset(D,file_SF_Condition,optimOpts,opts);

% %% Optimize at a nominal frequency across all conditions
% optimOpts.AcrossConditions = true;
% opts.SamplingFrequencies = nomFreq;
% 
% O = optimizeAHRSacrossDataset(D,file_Nominal60 ,optimOpts,opts);
% 
% %% Optimize at nominal frequency for each condition
% optimOpts.AcrossConditions = true;
% opts.SamplingFrequencies = nomFreq;
% 
% O = optimizeAHRSacrossDataset(D,file_Nominal60_perCondition,optimOpts,opts);
