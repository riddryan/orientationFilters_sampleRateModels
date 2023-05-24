function O = processCondition(subject,condition,info,files,opts)
%% Relabel
ss = subject;
cc = condition;
opts.condition = cc;
alignmentinfo = info.alignment;
%% Load Data
load(files.dataFiles(ss,cc));
info.index_save = index_save;
%% Sync Data
[sensordata, vicondata,IS,IE] = correctDelay(sensordata, vicondata, info.sensorNames, crosscorr_delay);

% Also sync nan indices for mocap
info.mocapNandex = info.mocapNandex.(opts.conditions{condition});
info.mocapNandex(IS,:) = [];
info.mocapNandex(IE,:) = [];
info.mocapNandex = info.mocapNandex(info.index_save:end,:);

%% Process motion capture data

% Align motion capture data to gravity and magnetic fields
mocap = alignMocapToWorld(vicondata,info.index_save,alignmentinfo);
badMarks = squeeze(all(isnan(mocap(:,1,:))));

% Lowpass Filter
Fs=vicondata.Fs;
[cu_corrected, order_corrected] = WinterCorrectCU(opts.lpFilterOrder , opts.lpFilterFreq , 'low', Fs);
[B,A] = butter(order_corrected,cu_corrected/(.5*Fs),'low');
mocap(:,:,~badMarks) = filtfilt(B,A,mocap(:,:,~badMarks));
info.Fs = Fs;
%% Perform processing for each sensor
for ii = 1 : length(info.sensorNames)
    O.(info.sensorNames{ii}) = processSensor(ii,sensordata,mocap,info,files,opts);
end