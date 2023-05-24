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


figDir = [files.figDir 'parameterStudies' filesep];
saveFormats = {'fig' 'png' 'epsc'};

saveFigs = true;


% Graphics options
fontSize = 8;
fontName = 'Arial';
singleColumnWidth = 9;
doubleColumnWidth = 19;
onepointfiveColumnWidth = 14;

lightBlue=[0.3 0.8 0.95];
darkBlue = [0.03 0.07 0.6];
blueGrad = @(i,N) lightBlue + (darkBlue-lightBlue)*((i-1)/(N-1));


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
optimOpts.useCostContourFor1D = true;

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

optimOpts.AcrossSubjects = true;
optimOpts.AcrossConditions = true;
optimOpts.AcrossSensors = true;
optimOpts.AcrossSamplingFrequencies = false;

opts.SamplingFrequencies = opts.SamplingFrequencies(4);
origOpts = opts;
%% Parameter study - Riddick filter - across all tasks & participants at 33 Hz

magfile = [files.optimizedParametersDir 'Riddick_magnetometerGain_parameterStudy.mat'];
accfile = [files.optimizedParametersDir 'Riddick_accelerometerGain_parameterStudy.mat'];
gyrofile = [files.optimizedParametersDir 'Riddick_gyroscopeGain_parameterStudy.mat'];

optimOpts.dontOptimize = true;
optimOpts.intermediateSave = false;

optimOpts.contourMinValLog = -1;
optimOpts.contourMaxValLog = 0.3;
optimOpts.numPointsCostContour = 40;

% Which algorithms to use
opts.AHRS = [RiddickAHRS];
opts.algorithms = [];
for i = 1 : length(opts.AHRS)
    opts.algorithms{i} = class(opts.AHRS(i));
end

opts.visualize = 0;

% Magnetometer
optimOpts.RiddickAHRS.parameters.Names = {'c_magnetometer'};
optimOpts.RiddickAHRS.parameters.Initial = 1E-4;
optimOpts.RiddickAHRS.parameters.lb = 0;
optimOpts.RiddickAHRS.parameters.ub = 0.2;

if forceReoptimize || ~exist(magfile,'file')
    O = optimizeAHRSacrossDataset(D,file_SF,optimOpts,opts);
    save(magfile,'O');
else
    LD = load(magfile);
    O = LD.O;
end
Omag = O;

% Accelerometer
optimOpts.RiddickAHRS.parameters.Names = {'c_accelerometer'};
optimOpts.RiddickAHRS.parameters.Initial = 1E-4;
optimOpts.RiddickAHRS.parameters.lb = 0;
optimOpts.RiddickAHRS.parameters.ub = 0.2;


optimOpts.contourMinValLog = -2;
optimOpts.contourMaxValLog = 0;
optimOpts.numPointsCostContour = 40;

if forceReoptimize || ~exist(accfile,'file')
    O = optimizeAHRSacrossDataset(D,file_SF,optimOpts,opts);
    save(accfile,'O');
else
    LD = load(accfile);
    O = LD.O;
end
Oacc = O;

% Gyroscope
optimOpts.RiddickAHRS.parameters.Names = {'c_gyroscope'};
optimOpts.RiddickAHRS.parameters.Initial = 1;
optimOpts.RiddickAHRS.parameters.lb = 0;
optimOpts.RiddickAHRS.parameters.ub = 1.2;

optimOpts.contourMinValLog = -1;
optimOpts.contourMaxValLog = 0.5;
optimOpts.numPointsCostContour = 40;

if forceReoptimize || ~exist(gyrofile,'file')
    O = optimizeAHRSacrossDataset(D,file_SF,optimOpts,opts);
    save(gyrofile,'O');
else
    LD = load(gyrofile);
    O = LD.O;
end
Ogyro = O;

hf = figure;
hf.Color = [1 1 1];
hf.Units = 'centimeters';
hf.Position = [2 2 singleColumnWidth singleColumnWidth];

subplot(311)
semilogx(Ogyro.parameterContour{1},Ogyro.costContour{1},'k.-')
% plot(Ogyro.parameterContour{1},Ogyro.costContour{1},'k.-')
set(gca,'Box','Off')
title('Gyroscope gain')
ylabel ('Error (°)')

set(gca,'Box','Off')
set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');

subplot(312)
semilogx(Oacc.parameterContour{1},Oacc.costContour{1},'k.-')
% plot(Oacc.parameterContour{1},Oacc.costContour{1},'k.-')
set(gca,'Box','Off')
title('Accelerometer gain')
ylabel ('Error (°)')

set(gca,'Box','Off')
set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');

subplot(313)
semilogx(Omag.parameterContour{1},Ogyro.costContour{1},'k.-')
% plot(Omag.parameterContour{1},Omag.costContour{1},'k.-')
set(gca,'Box','Off')
title('Magnetometer gain')
ylabel ('Error (°)')
xlabel('Parameter Value (No Units)')

set(gca,'Box','Off')
set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');


if saveFigs
    for i = 1 : length(saveFormats)
        figName = [figDir 'Riddick_ParameterStudy'];% saveFormats{i}];
        saveas(gcf,figName,saveFormats{i});
    end
end


%% Parameter study - Kalman filter

magfile = [files.optimizedParametersDir 'FastKF_magnetometerGain_parameterStudy.mat'];
accfile = [files.optimizedParametersDir 'FastKF_accelerometerGain_parameterStudy.mat'];
gyrofile = [files.optimizedParametersDir 'FastKF_gyroscopeGain_parameterStudy.mat'];

optimOpts.dontOptimize = true;
optimOpts.intermediateSave = false;



% Which algorithms to use
opts.AHRS = [FastKF];
opts.algorithms = [];
for i = 1 : length(opts.AHRS)
    opts.algorithms{i} = class(opts.AHRS(i));
end

opts.visualize = 0;

% Magnetometer
optimOpts.FastKF.parameters.Names = {'Sigma_m_scalar'};
optimOpts.FastKF.parameters.Initial = 0.33;
optimOpts.FastKF.parameters.lb = 0;
optimOpts.FastKF.parameters.ub = 1;

optimOpts.contourMinValLog = -5;
optimOpts.contourMaxValLog = -2;
optimOpts.numPointsCostContour = 40;

if forceReoptimize || ~exist(magfile,'file')
    O = optimizeAHRSacrossDataset(D,file_SF,optimOpts,opts);
    save(magfile,'O');
else
    LD = load(magfile);
    O = LD.O;
end
Omag = O;

% Accelerometer
optimOpts.FastKF.parameters.Names = {'Sigma_a_scalar'};
optimOpts.FastKF.parameters.Initial = 0.66;
optimOpts.FastKF.parameters.lb = 0;
optimOpts.FastKF.parameters.ub = 0.2;

optimOpts.contourMinValLog = -4;
optimOpts.contourMaxValLog = -1;
optimOpts.numPointsCostContour = 40;

if forceReoptimize || ~exist(accfile,'file')
    O = optimizeAHRSacrossDataset(D,file_SF,optimOpts,opts);
    save(accfile,'O');
else
    LD = load(accfile);
    O = LD.O;
end
Oacc = O;

% Gyroscope
optimOpts.FastKF.parameters.Names = {'Sigma_g_scalar'};
optimOpts.FastKF.parameters.Initial = 0.5;
optimOpts.FastKF.parameters.lb = 0;
optimOpts.FastKF.parameters.ub = 1.2;

optimOpts.contourMinValLog = -4;
optimOpts.contourMaxValLog = 0;
optimOpts.numPointsCostContour = 40;

if forceReoptimize || ~exist(gyrofile,'file')
    O = optimizeAHRSacrossDataset(D,file_SF,optimOpts,opts);
    save(gyrofile,'O');
else
    LD = load(gyrofile);
    O = LD.O;
end
Ogyro = O;

figure;

subplot(311)
semilogx(Ogyro.parameterContour{1},Ogyro.costContour{1},'k.-')
% plot(Ogyro.parameterContour{1},Ogyro.costContour{1},'k.-')
set(gca,'Box','Off')
title('Gyroscope variance parameter')
ylabel ('Error (°)')

subplot(312)
semilogx(Oacc.parameterContour{1},Oacc.costContour{1},'k.-')
% plot(Oacc.parameterContour{1},Oacc.costContour{1},'k.-')
set(gca,'Box','Off')
title('Accelerometer variance parameter')
ylabel ('Error (°)')

subplot(313)
semilogx(Omag.parameterContour{1},Ogyro.costContour{1},'k.-')
% plot(Omag.parameterContour{1},Omag.costContour{1},'k.-')
set(gca,'Box','Off')
title('Magnetometer variance parameter')
ylabel ('Error (°)')
xlabel('Parameter Value (No Units)')

set(gca,'Box','Off')
set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');

%% Parameter study - Riddick filter -  speciific condi
opts = origOpts;
opts.conditions = {'og_jog_sss'};
opts.subjects = 1;

optimOpts.AcrossSensors = false;

magfile = [files.optimizedParametersDir 'Riddick_magnetometerGain_parameterStudy_' opts.conditions{1} '.mat'];
accfile = [files.optimizedParametersDir 'Riddick_accelerometerGain_parameterStudy_' opts.conditions{1} '.mat'];
gyrofile = [files.optimizedParametersDir 'Riddick_gyroscopeGain_parameterStudy_' opts.conditions{1} '.mat'];

optimOpts.dontOptimize = true;
optimOpts.intermediateSave = false;

optimOpts.contourMinValLog = -0.5;
optimOpts.contourMaxValLog = 0;
optimOpts.numPointsCostContour = 40;

% Which algorithms to use
opts.AHRS = [RiddickAHRS];
opts.algorithms = [];
for i = 1 : length(opts.AHRS)
    opts.algorithms{i} = class(opts.AHRS(i));
end

opts.visualize = 0;

% Magnetometer
optimOpts.RiddickAHRS.parameters.Names = {'c_magnetometer'};
optimOpts.RiddickAHRS.parameters.Initial = 1E-4;
optimOpts.RiddickAHRS.parameters.lb = 0;
optimOpts.RiddickAHRS.parameters.ub = 0.2;

if forceReoptimize || ~exist(magfile,'file')
    O = optimizeAHRSacrossDataset(D,file_SF,optimOpts,opts);
    save(magfile,'O');
else
    LD = load(magfile);
    O = LD.O;
end
Omag = O;

% Accelerometer
optimOpts.RiddickAHRS.parameters.Names = {'c_accelerometer'};
optimOpts.RiddickAHRS.parameters.Initial = 1E-4;
optimOpts.RiddickAHRS.parameters.lb = 0;
optimOpts.RiddickAHRS.parameters.ub = 0.2;


optimOpts.contourMinValLog = -2;
optimOpts.contourMaxValLog = 0;
optimOpts.numPointsCostContour = 40;

if forceReoptimize || ~exist(accfile,'file')
    O = optimizeAHRSacrossDataset(D,file_SF,optimOpts,opts);
    save(accfile,'O');
else
    LD = load(accfile);
    O = LD.O;
end
Oacc = O;

% Gyroscope
optimOpts.RiddickAHRS.parameters.Names = {'c_gyroscope'};
optimOpts.RiddickAHRS.parameters.Initial = 1;
optimOpts.RiddickAHRS.parameters.lb = 0;
optimOpts.RiddickAHRS.parameters.ub = 1.2;

optimOpts.contourMinValLog = -0.5;
optimOpts.contourMaxValLog = 0.1;
optimOpts.numPointsCostContour = 40;

if forceReoptimize || ~exist(gyrofile,'file')
    O = optimizeAHRSacrossDataset(D,file_SF,optimOpts,opts);
    save(gyrofile,'O');
else
    LD = load(gyrofile);
    O = LD.O;
end
Ogyro = O;

hf = figure;
hf.Color = [1 1 1];
hf.Units = 'centimeters';
hf.Position = [2 2 singleColumnWidth singleColumnWidth];

subplot(311)
semilogx(Ogyro.parameterContour{1},Ogyro.costContour{1},'k.-')
% plot(Ogyro.parameterContour{1},Ogyro.costContour{1},'k.-')
set(gca,'Box','Off')
title('Gyroscope gain')
ylabel ('Error (°)')

set(gca,'Box','Off')
set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');

subplot(312)
semilogx(Oacc.parameterContour{1},Oacc.costContour{1},'k.-')
% plot(Oacc.parameterContour{1},Oacc.costContour{1},'k.-')
set(gca,'Box','Off')
title('Accelerometer gain')
ylabel ('Error (°)')

set(gca,'Box','Off')
set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');

subplot(313)
semilogx(Omag.parameterContour{1},Ogyro.costContour{1},'k.-')
% plot(Omag.parameterContour{1},Omag.costContour{1},'k.-')
set(gca,'Box','Off')
title('Magnetometer gain')
ylabel ('Error (°)')
xlabel('Parameter Value (No Units)')

set(gca,'Box','Off')
set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');


if saveFigs
    for i = 1 : length(saveFormats)
        figName = [figDir 'PStudy_Riddick_subj1_sensor1_jogging' ];% saveFormats{i}];
        saveas(gcf,figName,saveFormats{i});
    end
end

%% Parameter study - Kalman filter - speciific condi
opts = origOpts;
opts.conditions = {'og_jog_sss'};
opts.subjects = 1;

optimOpts.AcrossSensors = false;

magfile = [files.optimizedParametersDir 'FastKF_magnetometerGain_parameterStudy_' opts.conditions{1} '.mat'];
accfile = [files.optimizedParametersDir 'FastKF_accelerometerGain_parameterStudy_' opts.conditions{1} '.mat'];
gyrofile = [files.optimizedParametersDir 'FastKF_gyroscopeGain_parameterStudy_' opts.conditions{1} '.mat'];

optimOpts.dontOptimize = true;
optimOpts.intermediateSave = false;



% Which algorithms to use
opts.AHRS = [FastKF];
opts.algorithms = [];
for i = 1 : length(opts.AHRS)
    opts.algorithms{i} = class(opts.AHRS(i));
end

opts.visualize = 0;

% Magnetometer
optimOpts.FastKF.parameters.Names = {'Sigma_m_scalar'};
optimOpts.FastKF.parameters.Initial = 0.33;
optimOpts.FastKF.parameters.lb = 0;
optimOpts.FastKF.parameters.ub = 1;

optimOpts.contourMinValLog = -4;
optimOpts.contourMaxValLog = 0.2;
optimOpts.numPointsCostContour = 40;

if forceReoptimize || ~exist(magfile,'file')
    O = optimizeAHRSacrossDataset(D,file_SF,optimOpts,opts);
    save(magfile,'O');
else
    LD = load(magfile);
    O = LD.O;
end
Omag = O;

% Accelerometer
optimOpts.FastKF.parameters.Names = {'Sigma_a_scalar'};
optimOpts.FastKF.parameters.Initial = 0.66;
optimOpts.FastKF.parameters.lb = 0;
optimOpts.FastKF.parameters.ub = 0.2;

optimOpts.contourMinValLog = -4;
optimOpts.contourMaxValLog = 2;
optimOpts.numPointsCostContour = 40;

if forceReoptimize || ~exist(accfile,'file')
    O = optimizeAHRSacrossDataset(D,file_SF,optimOpts,opts);
    save(accfile,'O');
else
    LD = load(accfile);
    O = LD.O;
end
Oacc = O;

% Gyroscope
optimOpts.FastKF.parameters.Names = {'Sigma_g_scalar'};
optimOpts.FastKF.parameters.Initial = 0.5;
optimOpts.FastKF.parameters.lb = 0;
optimOpts.FastKF.parameters.ub = 1.2;

optimOpts.contourMinValLog = -4;
optimOpts.contourMaxValLog = 0;
optimOpts.numPointsCostContour = 40;

if forceReoptimize || ~exist(gyrofile,'file')
    O = optimizeAHRSacrossDataset(D,file_SF,optimOpts,opts);
    save(gyrofile,'O');
else
    LD = load(gyrofile);
    O = LD.O;
end
Ogyro = O;

hf = figure;
hf.Color = [1 1 1];
hf.Units = 'centimeters';
hf.Position = [2 2 singleColumnWidth singleColumnWidth];

subplot(311)
semilogx(Ogyro.parameterContour{1},Ogyro.costContour{1},'k.-')
% plot(Ogyro.parameterContour{1},Ogyro.costContour{1},'k.-')
set(gca,'Box','Off')
title('Gyroscope variance parameter')
ylabel ('Error (°)')

set(gca,'Box','Off')
set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');


subplot(312)
semilogx(Oacc.parameterContour{1},Oacc.costContour{1},'k.-')
% plot(Oacc.parameterContour{1},Oacc.costContour{1},'k.-')
set(gca,'Box','Off')
title('Accelerometer variance parameter')
ylabel ('Error (°)')

set(gca,'Box','Off')
set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');


subplot(313)
semilogx(Omag.parameterContour{1},Ogyro.costContour{1},'k.-')
% plot(Omag.parameterContour{1},Omag.costContour{1},'k.-')
set(gca,'Box','Off')
title('Magnetometer variance parameter')
ylabel ('Error (°)')
xlabel('Parameter Value (No Units)')

set(gca,'Box','Off')
set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');


if saveFigs
    for i = 1 : length(saveFormats)
        figName = [figDir 'PStudy_FKF_subj1_sensor1_jogging' ];% saveFormats{i}];
        saveas(gcf,figName,saveFormats{i});
    end
end
