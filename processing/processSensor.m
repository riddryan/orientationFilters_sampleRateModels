function O = processSensor(sensor,sensordata,mocap,info,files,opts)
% Keep track of input options (optsIn) since the variable opts will be updated to
% keep track of individual options and results from different tracking
% algorithms.
optsIn = opts;
opts.sensor = sensor;
sensorname = info.sensorNames{sensor};
alignmentinfo = info.alignment;
calibration_info = info.calibration;
%% Align the marker cluster to the sensor
sensorMarkers = mocap(:,:,opts.sensorMarkerNumbers(sensor,:));
q_mocap = alignMocapToSensors(sensorMarkers,alignmentinfo,sensorname);

% Take note which mocap frames
IS = opts.sensorMarkerNumbers(sensor,:);
nanDex = info.mocapNandex(:,IS);
badFrames = any(nanDex,2);
O.badMocapFramesOriginal = badFrames;
%% Calibrate sensor data
sensorTable = sensordata.(sensorname).sensordata(info.index_save:end,:);
D = calibrateSensorData(sensorTable,calibration_info,sensorname);

% Use mocap quaternion as ground truth
D.ground_truth = compact(q_mocap);

O.Data = D;

%% Resampling for different sampling frequencies



for i = 1 : length(optsIn.SamplingFrequencies)


    % Resample data to desired freqency
    resampleFreq = optsIn.SamplingFrequencies(i);
    Di = resampleStruct(D,info.Fs,resampleFreq);
    opts.Fs = resampleFreq;

    % Resample which frames are bad as well
    dFrames = double(O.badMocapFramesOriginal);
    diFrames = interp1(D.time,dFrames,Di.time);
    O.badMocapFramesResampled{1,i} = logical(diFrames);

    Di.ground_truth(O.badMocapFramesResampled{1,i},:) = NaN;
    O.ResampleData(1,i) = Di;

    O.SampleRate(1,i) = optsIn.SamplingFrequencies(i);
    O.Condition{1,i} = opts.conditions{opts.condition};
    O.Subject(1,i) = opts.subjects(opts.subject);
    O.Sensor{1,i} = info.sensorNames{opts.sensor};

end