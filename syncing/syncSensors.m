function [Ssynced,DELAYS,DELAYTIMES,deltaFS,figHands] = syncSensors(S,FS,stdThresh,Nsub,searchDex,showPlots,maxDelay,initResample)
if nargin < 3 || isempty(stdThresh)
    stdThresh = 5;
end
if nargin < 4
    Nsub = 200000; 
end
if nargin < 5
    searchDex = [];
end
if nargin < 6
    showPlots = true;
end
if nargin < 7
    maxDelay = [];
end
if nargin < 8
    initResample = true;
end
% number of sensors
NS = length(S);
% Signal names
dataNames = {'acc' 'gyro' 'mag'};
% Number of samples per segment to calculate cross-correlation lag between 2 sensors

figHands = [];

%% Resample to have same target sampling frequency

if initResample
Ssynced = struct('time',[],'acc',[],'gyro',[],'mag',[],'samplingFrequency',[],'datetime',[],'events',[]);
for i = 1 : NS
    t0(i) = S(i).time(1);

    Ssynced(i).time = linspace(0,S(i).time(end),S(i).time(end)*FS).';

    Ssynced(i).samplingFrequency = FS;

    for j = 1 : length(dataNames)
        if ~isfield(S(i),dataNames{j}) || isempty(S(i).(dataNames{j}))
            Ssynced(i).(dataNames{j}) = [];
            continue
        end
        Ssynced(i).(dataNames{j}) = interp1(S(i).time,S(i).(dataNames{j}),Ssynced(i).time);
        
        if isfield(S(i),'datetime') && ~isempty(S(i).datetime)
            Ssynced(i).datetime = S(i).datetime;
        end
    end
end
else
    Ssynced = S;
end


%% Bandpass filter
% We want to sync mostly based on hopping movements. Participants hopped 5
% times throughout the experiment at a roughly self-selected frequency.
% Filter the accelerations to be between 1 and 3 Hz.
[B,A] = butter(2,[1 3] ./ (FS/2),'bandpass');

syncDim = 1;

% ref = filtfilt(B,A,Ssynced(1).acc(:,syncDim));
% ref = filtfilt(B,A,Ssynced(5).acc(:,syncDim));
ref = filtfilt(B,A,vecnorm(Ssynced(1).acc,2,2));
refTime = Ssynced(1).time;

% figure


% dex = 105390:105500;
% plot(ref(dex,1))

% dex = 53774:53892;
% plot(ref(dex,1))

%%
% activPalstartGuess = 101659;
% dorsaviStartGuess = 53783;
% 
% d0 = dorsaviStartGuess - activPalstartGuess;

%% Sync sensors to 1st sensor based on accelerometer data
% ref = vecnorm(Ssynced(1).acc,2,2);

initDelay(1) = 0;
for i = 2 : NS
    time = Ssynced(i).time;

    goodDex = ~isnan(Ssynced(i).acc(:,syncDim));
%     accTest = filtfilt(B,A,Ssynced(i).acc(goodDex,syncDim));
    accTest = filtfilt(B,A,vecnorm(Ssynced(i).acc(goodDex,:),2,2));
    time = time(goodDex);
%     accTest = vecnorm(Ssynced(i).acc,2,2);

    initDelay(i) = time(1)*FS;

    nTime = length(time);

    % Pick out a certain range of data if supplied
    X1 = ref;
    X2 = accTest;

    if initDelay(i) < 0
        X2 = X2(-initDelay(i):end);
    elseif initDelay(i) > 0
        X1 = X1(initDelay(i):end);
    end

    if ~isempty(searchDex)
        X1 = X1(searchDex);
        X2 = X2(searchDex);
    end

    % Cross correlation over different chunks of data to see how much lag
    % there is between two sensor over time. Should be a linear function
    % over time.
    [delays,dtimes] = calcCrossCorrDrift(X1,X2,Nsub,FS,stdThresh,maxDelay);

    delays = delays + initDelay(i);

%     if 0
%         refN = (ref - min(ref)) ./ (max(ref) - min(ref));
%         accN = (accTest - min(accTest)) ./ (max(accTest) - min(accTest));
%         figure;
%         plot(refN);
%         hold on
%         plot(accN)
% 
%         searchDex = 53793:53883;
%         L = length(searchDex);
%         Y = fft(Ssynced(5).acc(searchDex,1));
%         f = FS * (0:(L/2))/L;
%         P2 = abs(Y/L);
% P1 = P2(1:L/2+1);
% P1(2:end-1) = 2*P1(2:end-1);
% figure;
% plot(f,P1);
% 
% 
%         [B,A] = butter(2,[1 3] ./ (FS/2),'bandpass');
%         accFilt = filtfilt(B,A,Ssynced(5).acc(:,1));
%     end
 


    if length(delays) > 1
    % Calculate the amount difference in Hz the sensor needs to resampled
    % by to match the reference signal. This is only a good estimate if the
    % delays are a linear function of delay times. This has been the case
    % in all tests so far.

        p = polyfit(dtimes,delays,1);
%     dd = polyval(p,dtimes(1));
      dd = p(2);

%     deltaFsr = mean(diff(delays) ./ diff(dtimes));
    deltaFsr = p(1);

    % Resample time and all signals of the sensor based to correct this
    % small error in sampling frequency
    timeResampled = linspace(time(1),time(end),(time(end)-time(1))*(FS+deltaFsr)).';
    for j = 1 : length(dataNames)
        oldSig = Ssynced(i).(dataNames{j});
        if isempty(oldSig)
            continue
        end
        newSig = interp1(time,oldSig,timeResampled);
        Ssynced(i).(dataNames{j}) = newSig;
    end


    % Find the overall delay between the two signals after resampling.
    % (This may not be necessary because you could calculate it based on
    % the delays and dtimes).
%     accMagResampled = interp1(time,accMag,timeResampled);
%     dd = finddelay(accMagResampled,ref,10000);
    % Shift second sensor to match time of 1st sensor
    timeSynced = linspace(timeResampled(1),timeResampled(1) + length(timeResampled)/FS,length(timeResampled)).';
%     timeSynced = timeSynced + dd/FS;

    else
        timeSynced = time + delays/FS;
        deltaFsr = 0;
    end

    if isfield(S(i),'events') && ~isempty(S(i).events)
%         newEventTimes = time(S(i).events) + dd/FS;
%         
%         for jj = 1 : length(newEventTimes)
%             [~,imin] = min(abs(newEventTimes(jj)-timeSynced));
%             newEventDex(jj,1) = imin;
%         end
%         Ssynced(i).events = newEventDex;

        Ssynced(i).events = S(i).events;
    end
    % Assign the synced time back to the sensor
    Ssynced(i).time = timeSynced;

                        refN = (ref - min(ref)) ./ (max(ref) - min(ref));
        accN = (accTest - min(accTest)) ./ (max(accTest) - min(accTest));

    if showPlots

        figHands(i-1) = figure;

        if ~isempty(searchDex)


        subplot(211)
        plot(S(1).time(searchDex),refN(searchDex))
        hold on
        plot(S(i).time(searchDex),accN(searchDex));
        title('Unsynced')
        legend('Sensor 1',['Sensor ' num2str(i)])

        subplot(212)
        plot(Ssynced(1).time(searchDex),refN(searchDex));
        hold on

        dd = searchDex;
        dd(dd>length(Ssynced(i).time)) = [];
        plot(Ssynced(i).time(dd),accN(dd))
        title('Synced')

        else

        

        T1 = Ssynced(1).time;
        A1 = refN;

        TI = Ssynced(i).time;
        AI = Ssynced(i).acc;
        AI = filtfilt(B,A,vecnorm(AI,2,2));
        AI = (AI - min(AI))./(max(AI)-min(AI));

        [NN] = min([length(S(i).time) length(accN)]);
        dex = 1:NN;
        
        subplot(211)
        plot(S(1).time,refN)
        hold on
        plot(S(i).time(dex),accN(dex));
        title('Unsynced')
        legend('Sensor 1',['Sensor ' num2str(i)])

        subplot(212)
        plot(T1,A1);
        hold on
        plot(TI,AI)
        title('Synced')
        end

    end

%     meanDelays(i-1) = mean(delays);

    DELAYS{i-1} = delays;
    DELAYTIMES{i-1} = dtimes;
    deltaFS(i-1) = deltaFsr;
end
