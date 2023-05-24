function O = optimizeAHRSacrossDataset(D,saveFile,optimOpts,opts)
% Optimize parameters for AHRS estimation algorithms across different sets
% of data. See defaultOptimOpts.m for different options for optimization.
%
% saveFile stores the optimization results for each round of optimization
% so that if execution gets interrupted it does not need to perform every
% optimization over again. You can see optimOpts.forceReoptimize to ensure
% you reoptimize with the new options. Or you can just delete saveFile so
% that there is no file to load.

%% Specify which parts of the data to group together
% For subjects, conditions, sensors, and sampling frequencies - we either
% perform an optimization for each one separately, or across all of them
% simulataneously. Set the corresponding options to optimize across all
% simultaneously:
%
% optimOpts.AcrossSubjects
% optimOpts.AcrossConditions
% optimOpts.AcrossSensors
% optimOpts.AcrossSamplingFrequncies
I.suStart = 1;
I.condiStart = 1;
I.sensorStart = 1;
I.fsStart = 1;

I.suEnd = 1;
I.condiEnd = 1;
I.sensorEnd = 1;
I.fsEnd = 1;
if optimOpts.AcrossSubjects
    I.suEnd = length(opts.subjects);
end

if optimOpts.AcrossConditions
    I.condiEnd = length(opts.conditions);
end


snames = fieldnames(D);
sensorNames = [];
for i = 1 : length(snames)
    sensorNames = [sensorNames D.(snames{i}).sensorNames];
end
sensorNames = unique(sensorNames,'stable');
opts.sensorNames = sensorNames;

if ~isfield(opts,'sensorsToUse')
    opts.sensorsToUse = {'mdm650D' 'mdm64B1' 'mdm64C6' 'mdm64BF' 'mdm652E' 'mdm64A8' 'mdm64E1'};
end

if optimOpts.AcrossSensors
    I.sensorEnd = length(sensorNames);
end
if optimOpts.AcrossSamplingFrequencies
    I.fsEnd = length(opts.SamplingFrequencies);
end
%% Count number of optimizations
numSubjOptim = (length(opts.subjects) - I.suEnd + 1);
numCondiOptim = (length(opts.conditions) - I.condiEnd + 1);
if optimOpts.AcrossSensors
    numSensorOptim = 1;
    NO = 1;
else
    numSensorOptim = (length(opts.sensorNames)  - I.sensorEnd + 1);
    NO = 2;
end
numSamplingOptim = (length(opts.SamplingFrequencies) - I.fsEnd + 1);
numAlgorithmOptim = length(opts.algorithms);

NumTotalOptim = numSubjOptim * numCondiOptim * NO * numSamplingOptim * numAlgorithmOptim;

% Load already completed data
loaded = [];
if ~isempty(saveFile) && exist(saveFile,'file')
    loaded = load(saveFile,'O');
end

startCount = 1;
if isfield(optimOpts,'startCount')
    startCount = optimOpts.startCount;
end
endCount = Inf;
if isfield(optimOpts,'endCount')
    endCount = optimOpts.endCount;
end
%% Output fields
ofields = {'parameters' 'cost' 'optimOutput' 'subject' 'condition' 'sensor' 'location' 'samplingFrequency' ...
    'algorithm' 'iParameters' 'iCosts' 'opts' 'optimOpts'};
ofields2 = {'parameterContour' 'costContour'};

ofields = [ofields ofields2];

% Initialize output structure
for i = 1 : length(ofields)
    O.(ofields{i}) = [];
end
%% Run optimizations
count = 0;
I0 = I;
opts0 = opts;
for subject = I.suStart : numSubjOptim
    if subject > 1 && ~optimOpts.AcrossSubjects
        I.suStart = I.suStart + 1;
        I.suEnd = I.suEnd + 1;
    end
    I.condiStart = I0.condiStart; I.condiEnd = I0.condiEnd;
    for condition = I.condiStart : numCondiOptim
        if condition > 1 && ~optimOpts.AcrossConditions
            I.condiStart = I.condiStart + 1;
            I.condiEnd = I.condiEnd + 1;
        else
            I.condiStart = I0.condiStart;
            I.condiEnd = I0.condiEnd;
        end
        I.sensorStart = I0.sensorStart; I.sensorEnd = I0.sensorEnd;
        for sensor = I.sensorStart : numSensorOptim
            if sensor > 1 && ~optimOpts.AcrossSensors
                I.sensorStart = I.sensorStart + 1;
                I.sensorEnd = I.sensorEnd + 1;
            else
                I.sensorStart = I0.sensorStart;
                I.sensorEnd = I0.sensorEnd;
            end
            if ~any(strcmp(sensorNames{sensor},opts.sensorsToUse))
                continue;
            end

            I.fsStart = I0.fsStart; I.fsEnd = I0.fsEnd;
            for fs = I.fsStart : numSamplingOptim
                if fs > 1 && ~optimOpts.AcrossSamplingFrequencies
                    I.fsStart = I.fsStart + 1;
                    I.fsEnd = I.fsEnd + 1;
                end
                for algo = 1 : numAlgorithmOptim
        
                    count = count + 1;

                    opts = opts0;
                    if isfield(optimOpts.(opts.algorithms{algo}),'normStepSize')
                        opts.GradDescent.normStepSize = optimOpts.(opts.algorithms{algo}).normStepSize;
                    end
                    
                    parmNames = optimOpts.(opts.algorithms{algo}).parameters.Names;

                    if exist(saveFile,'file') && ~isempty(loaded)

                        if ~optimOpts.forceReoptimize || count < startCount || count > endCount
                            % Load optimization parameters if already done
                            if length(loaded.O.cost) >= count
                                O = assignOutput(O,count,loaded.O);
                                continue
                            end
                        end

                    end
                    % otherwise optimize


                    % Check if sensor is present for this trial
                    sensorsPresent = checkSensorInTrial(D,I,opts);
                    if ~any(sensorsPresent)
                        % No data for the chosen sensors in this trial
%                         O = assignOutput(O,count);
                        count = count - 1;
                        continue
                    end
                    
                   
                    
                    fprintf('=============================================================\n')
                    fprintf('%d of %d optimizations...\n',count,NumTotalOptim);

                    if optimOpts.AcrossSubjects
                        suStr = 'Across subjects ';
                    else
                        suStr = sprintf('Subject %d',opts.subjects(subject));
                    end
                    if optimOpts.AcrossConditions
                        coStr = 'across conditions ';
                    else
                        coStr = opts.conditions{condition};
                    end
                    if optimOpts.AcrossSensors
                        seStr = 'across sensors ';
                    else
                        seStr = sensorNames{sensor};
                    end
                    if optimOpts.AcrossSamplingFrequencies
                        fsStr = 'across sampling frequenices ';
                    else
                        fsStr = sprintf('%.02f Hz',opts.SamplingFrequencies(fs));
                    end
                     fprintf('%s %s %s %s %s \n',suStr,coStr,seStr,fsStr,opts.algorithms{algo});

                    % Extract data for the optimization from the big data
                    % struct
                    DE = extractEnsembleData(D,I,opts);
                    

                    % Run a parameter study if desired. Only supported
                    % currently for AHRS filters with 1 parameter although
                    % this is easily extendable.
                    pContour = [];
                    costContour = [];
                    if length(parmNames) == 1 && optimOpts.useCostContourFor1D
%                         p = [0 logspace(-4,log10(0.1),optimOpts.numPointsCostContour-1)].';
%                         p = [0 logspace(-4,log10(1),optimOpts.numPointsCostContour-1)].';

                        p = [0 logspace(optimOpts.contourMinValLog,optimOpts.contourMaxValLog,optimOpts.numPointsCostContour-1)].';
                        [cContour] = ensembleCostContour(p,parmNames,algo,DE,opts);

                        if opts.visualize
                            if count == 1
                                cc = figure();
                                clf;
                            end
                            try figure(cc)
                            catch
                                cc = figure();
                            end
                            semilogx(p,cContour,'k.-');
                            hold on

                            drawnow
                        end
                        pContour = p;
                        costContour = cContour;
                    end


                    % Perform the optimization
                    cost = @(p) ensembleCost(p,parmNames,algo,DE,opts);
                    if ~isfield(optimOpts,'dontOptimize') || ~optimOpts.dontOptimize
                        [pStar,costStar,optimOutput] = optimizeAHRSparameters(cost,opts.algorithms{algo},optimOpts,DE(1).samplingFrequency);
                    else
                        % Allow for user to just use the initial guess as
                        % the final estimates for cost
                        pStar = optimOpts.(opts.algorithms{algo}).parameters.Initial;
                        costStar = cost(pStar);
                        optimOutput = [];
                    end
                    
%                     es = linspace(0.001,1000,100);
%                     clear cp
%                     for Z = 1 : length(es)
%                         pp = [0.36  1e-9 es(Z)];
%                         cp(Z) = cost(pp);
% 
%                     end
%                     figure;plot(es,cp)

                    % Get the individual costs for each trial in addition
                    % to the overall optimized cost averaged across all
                    % trials

                    tic;
                    [~,iCosts] = cost(pStar);
                    estTime = toc;


                    % plot
%                     ensembleCost(pStar,parmNames,algo,DE,opts,true);

                    % Update output structure
                    O = assignOutput(O,count,DE,opts,optimOpts,algo,optimOutput,pStar,costStar,iCosts,pContour,costContour,estTime);
                    
                    % Save results
                    if optimOpts.intermediateSave
                        save(saveFile,'O');
                    end
                end
            end
        end
    end
end

end

function O = assignOutput(O,count,DE,opts,optimOpts,algo,optimOutput,pStar,costStar,iCosts,pContour,costContour,estTime)
if nargin < 3
    % No result for this selection of data: assign NaN to everything
    onames = fieldnames(O);
    for i = 1 : length(onames)
        O.(onames{i}){count,1} = NaN;
    end
    return
elseif nargin == 3
    % Here we are assigning the loaded results directly from the structure
    % DE which should have the same fields as O
    onames = fieldnames(O);
    loadedResults = DE;
    for i = 1 : length(onames)
        O.(onames{i}){count,1} = loadedResults.(onames{i}){count,1};
    end
    return
end

% Otherwise DE is a structure output from extractEnsembleData, and the
% remaining inputs are the results of the optimization and parameter sweep
% study.

N = length(iCosts);

O.parameters{count,1} = pStar;
O.cost{count,1} = costStar;
O.iCosts{count,1} = iCosts;
O.iParameters{count,1} = repmat(pStar,N,1);
O.optimOutput{count,1} = optimOutput;

O.parameterContour{count,1} = pContour;
O.costContour{count,1} = costContour;

O.subject{count,1} = vertcat(DE.subject);
O.condition{count,1} = {DE.condition}.';
O.sensor{count,1} = {DE.sensor}.';
O.location{count,1} = {DE.location}.';
O.samplingFrequency{count,1} = vertcat(DE.samplingFrequency);
O.algorithm{count,1} = repmat(opts.algorithms(algo),N,1);

O.opts{count,1} = opts;
O.optimOpts{count,1} = optimOpts;

O.estTime{count,1} = estTime;
end


function [COST,costs] = ensembleCost(p,pNames,algorithm,D,opts,plotIt)
if nargin < 6
    plotIt = false;
end

N = length(D);
costs = NaN(N,1);
AHRS = opts.AHRS(algorithm);
for i = 1 : N
%     if ~opts.useCorrectionPrediction
%         [q,costs(i)] = estimateAttitude(AHRS,p,pNames,D(i).samplingFrequency,D(i).data,opts);
%     else
        [q,costs(i)] = estimateAttitude2(AHRS,p,pNames,D(i).samplingFrequency,D(i).data,opts);
%     end

    if plotIt
        figure(9);
        clf
        plotEstimate([],q,D(i).data,[],opts);
        drawnow;
    end
end
COST = mean(costs);
end

function [COST,costs] = ensembleCostContour(p,pNames,algorithm,D,opts)
% Averages the cost contour for algorithm for a given set of data
% trials D. p is a NS x NP matrix of parameters where NS is the number of
% different parameter values to sample over, and NP is the number of
% parameters for the algorithm.
N = length(D);
NP = size(p,1);

AHRS = opts.AHRS(algorithm);
costs = NaN(N,NP);
for i = 1 : N
    
    for pp = 1 : NP
        [~,costs(i,pp)] = estimateAttitude2(AHRS,p(pp,:),pNames,D(i).samplingFrequency,D(i).data,opts);
    end
end
COST = mean(costs,1);
end

function O = extractEnsembleData(D,I,opts)


NS = I.suEnd - I.suStart + 1;
NC = I.condiEnd - I.condiStart + 1;
if I .sensorEnd == I.sensorStart
    NSE = 1;
else
    NSE = 2;
end
NFS = I.fsEnd - I.fsStart + 1;

NTOTAL = NS*NC*NSE*NFS;
O = struct('data',[],'samplingFrequency',[]);

count = 0;
for subject = I.suStart : I.suEnd
    ss = opts.subjects(subject);
    sstr = ['Subject' num2str(ss)];
    for condition = I.condiStart : I.condiEnd
        cstr = opts.conditions{condition};
        for sensor = I.sensorStart : I.sensorEnd
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

            for fs = I.fsStart : I.fsEnd
                count = count + 1;
                
                fsDex = opts.SamplingFrequencies(fs) == D.(sstr).(cstr).(sestr).SampleRate;
                O(count).data = D.(sstr).(cstr).(sestr).ResampleData(fsDex);

                % Need for interpolating error at recorded sample
                % frequency. This turns out to be unecessary and slows code
                % down a lot
                if opts.interpError
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
end


function sensorsPresent = checkSensorInTrial(D,I,opts)
sensorsPresent = false;
for subject = I.suStart : I.suEnd
    ss = opts.subjects(subject);
    sstr = ['Subject' num2str(ss)];
    for condition = I.condiStart : I.condiEnd
        cstr = opts.conditions{condition};
        for sensor = I.sensorStart : I.sensorEnd
            sestr = opts.sensorNames{sensor};

            if isfield(D.(sstr).(cstr),sestr)
                sensorsPresent = true;
                return
            end
        end
    end
end
end