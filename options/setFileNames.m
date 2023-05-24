function F = setFileNames(opts)
if nargin < 1
    opts = defaultDataOpts;
end
%%
% Should only need to change this folder to wherever you download the data
% folder on your computer

% dataDir = 'F:\research\painData\validation2021\';
% dataDir = 'D:\painData\validation2021\';
% dataDir = 'C:\Users\ryrid\Documents\Work\painData\validation2021\';
dataDir = 'I:\data\validation2021\';


F.dataDir = dataDir;
F.figDir = [dataDir 'figuresAndTables\'];
%%
subjNumStrs = string(opts.subjects);
subjectPaths = strcat(dataDir,'s',subjNumStrs,filesep);


% Alignment files
F.subjectDirs = subjectPaths;
F.alignmentFiles = strcat(subjectPaths,'s_',subjNumStrs,'_alignmentInfo.mat');
% Calibration files
F.calibrationFiles = strcat(subjectPaths,'s',subjNumStrs,'_magcal_gyroOffset.mat');
% Sensor placement/location files
F.sensorPlacementFiles = strcat(subjectPaths,'s_',subjNumStrs,'_SensorLocation.mat');
% Date files
F.dataFiles = string(subjectPaths).' + 's' + subjNumStrs.' + '_' + string(opts.conditions) + "_190.mat";

F.badMocapIndicesFileNames = strcat(subjectPaths,'s',subjNumStrs,'_badMocapIndices.mat');

% Where to store output of attitude estimation
% F.estDir = [dataDir 'estimated\'];

% Where to store optimized AHRS parameters
% F.optimizedParametersDir = [dataDir 'optimizedFilterParameters\'];
F.optimizedParametersDir = [dataDir 'filterParameters\'];
% F.optimizedMagDir = [F.optimizedParametersDir 'magDisturbed\'];