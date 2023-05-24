function opts = defaultDataOpts()
% Specify options for processing and loading of dataset

% Which subjects to process (from 1 to 12).
opts.subjects = [1:4 6:12];

% Which conditions to process
opts.conditions = {'standing_ref1' 'standing_bending' 'right_left_lift' ...
    'sitting_bend' 'sit_stand_normal' 'sit_stand_fast' 'walk_3kmh' ...
    'walk_5kmh' 'og_walk_sss' 'figure8_walking' 'og_jog_sss' };

opts.sensorsToUse = {'mdm650D' 'mdm64B1' 'mdm64C6' 'mdm64BF' 'mdm652E' 'mdm64A8' 'mdm64E1'};
% 0 = no plots, 1 = some plots, 2 = lots of plots
opts.visualize = 2;

opts.verbose = 2;



opts.interpError = false;

opts.useMex = true;

%% Sensor information
opts.sensorsToUse = {'mdm650D' 'mdm64B1' 'mdm64C6' 'mdm64BF' 'mdm652E' 'mdm64A8' 'mdm64E1'};
opts.sensorNames = opts.sensorsToUse;

% Sensor placement labels
opts.sensor_order = {'L5S1','T12L1'};

% In order: Top left, top right, bottom right, bottom left
opts.sensorMarkerNumbers = [1:4;5:8];

%% Motion capture processing
% LP filter options for mocap data
opts.lpFilterOrder = 4; 
opts.lpFilterFreq = 7.5;

opts.nanMethod = 'interp';
% opts.nanMethod = 'setToZero';

%% Sampling Rate options
% These options are for testing how sampling rate affects performance.
% Should be a vector of sampling rates in Hz.

opts.SamplingFrequencies = fliplr(logspace(0,log10(190),10));
% opts.SamplingFrequencies = 30;
%% Attitude estimation algorithms
% opts.AHRS = [FastKF HoornAHRS];
% opts.AHRS = [FastKF HoornAHRS WilsonAHRS MadgwickAHRS3];
% opts.AHRS = HoornAHRS;
opts.AHRS = RiddickAHRS;
for i = 1 : length(opts.AHRS)
    opts.algorithms{i} = class(opts.AHRS(i));
end

% Estimate how long the estimation takes
opts.calcEvalTime = true;



% Whether to use magnetometer data
opts.useMagnetometer = true;

opts.GradDescent.normStepSize = false;

%% Riddick filter options
opts.useCorrectionPrediction = true;
opts.integrationOrder = 1;



