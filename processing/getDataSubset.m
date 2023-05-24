function T = getDataSubset(D,I,opts,includeOriginalSamplingFrequency)
%%
if isempty(I)
    I = struct();
end

if nargin < 4
    includeOriginalSamplingFrequency = false;
end
if isfield(I,'subjects') && ~isempty(I.subjects)
    subjects = I.subjects;
else
    subjects = opts.subjects;
end

if isfield(I,'conditions') && ~isempty(I.conditions)
    conditions = I.conditions;
else
    conditions = opts.conditions;
end

if isfield(I,'sensors') && ~isempty(I.sensors)
    sensors = I.sensors;
else
    sensors = opts.sensorsToUse;
end

if isfield(I,'SamplingFrequencies') && ~isempty(I.SamplingFrequencies)
    SamplingFrequencies = I.SamplingFrequencies;
else
    SamplingFrequencies = opts.SamplingFrequencies;
end

if isfield(I,'SensorLocations') && ~isempty(I.SensorLocations)
    locs = I.SensorLocations;
else
    locs = opts.sensor_order;
end


%%
NS = length(subjects);
NC = length(conditions);
NSE = length(sensors);
NFS = length(SamplingFrequencies);


O = struct('data',[],'samplingFrequency',[]);

count = 0;
for subject = 1 : NS
    ss = subjects(subject);
    sstr = ['Subject' num2str(ss)];
    for condition = 1:NC
        cstr = conditions{condition};
        for sensor = 1:NSE
            sestr = opts.sensorNames{sensor};
            if ~isfield(D.(sstr).(cstr),sestr)
                continue
            end

            % Determine where sensor is placed (one of two locations, L5s1
            % or T12L1)
            locnames = fieldnames(D.(sstr).sensorlocation);
            for locdex = 1 : length(locnames)
                II = regexp(sestr,D.(sstr).sensorlocation.(locnames{locdex}), 'once');
                if ~isempty(II)
                    loc = locnames{locdex};
                    break
                end
            end
            
            % Check to see if sensor matches any of the desired sensor
            % locations
            if ~any(strcmp(loc,locs))
                continue
            end

            for fs = 1:NFS
                count = count + 1;
                fsDex = SamplingFrequencies(fs) == D.(sstr).(cstr).(sestr).SampleRate;


                O(count).data = D.(sstr).(cstr).(sestr).ResampleData(fsDex);

                % Need for interpolating error at recorded sample
                % frequency. This turns out to be unecessary and slows code
                % down a lot
                if includeOriginalSamplingFrequency || opts.interpError
                    O(count).data.groundTruthOrig = D.(sstr).(cstr).(sestr).Data.ground_truth;
                    O(count).data.timeOrig = D.(sstr).(cstr).(sestr).Data.time;
                end

                O(count).samplingFrequency = D.(sstr).(cstr).(sestr).SampleRate(fsDex);
                O(count).subject = ss;
                O(count).condition = cstr;
                O(count).sensor = sestr;
                O(count).location = loc;
                O(count).data.bads = D.(sstr).(cstr).(sestr).badMocapFramesResampled{fsDex};
            end
        end
    end
end

T = struct2table(O);
end