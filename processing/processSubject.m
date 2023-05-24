function O = processSubject(subject,conditions,files,opts)
opts.subject = subject;
ss = opts.subjects(subject);
%% Intermediate processing output files

% % Make sure output file directories exist
% if ~exist(files.estDir,'dir')
%     mkdir(files.estDir);
% end
% subjectOutputDir = [files.estDir 's' num2str(ss) filesep];
% if ~exist(subjectOutputDir,'dir')
%     mkdir(subjectOutputDir);
% end

%% Load calibration and alignment
load(files.alignmentFiles(subject));
load(files.calibrationFiles(subject));
load(files.sensorPlacementFiles(subject));

% Mean heading across both sensors and reference measures
for ii = length(opts.sensor_order): -1 : 1
    sensorNames{ii} = ['mdm' sensorlocation.(opts.sensor_order{ii})];
    headings(ii,:) = alignmentinfo.(sensorNames{ii}).heading;
end
magfield_heading = mean(headings(:));
alignmentinfo.magfield_heading = magfield_heading;

info.sensorNames = sensorNames;
info.alignment = alignmentinfo;
info.calibration = calibration_info;

% Store for output
O.sensorNames = sensorNames;
O.alignment = alignmentinfo;
O.calibration = calibration_info;
O.sensorlocation = sensorlocation;

%% Load bad mocap indices
d = load(files.badMocapIndicesFileNames(subject));

info.mocapNandex  = d.nandex;
%% Process each experimental condition
fprintf('\nLOADING...\n')
fprintf(['Subject' num2str(opts.subjects(subject)) '\n']);
for cc = 1 : length(conditions)
    fprintf([opts.conditions{cc} '\n']);

    cstr = opts.conditions{cc};

    OC = processCondition(subject,cc,info,files,opts);
    O.(cstr) = OC;

end

