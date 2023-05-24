function [P,costStar,optimOutput,pContour,costContour,iCosts] = ensembleOptimizeAHRSparameters(D,saveFile,optimOpts,opts)
% Optimize parameters for AHRS estimation algorithms across different sets
% of data. See defaultOptimOpts.m for different options for optimization.
%
% saveFile stores the optimization results for each round of optimization
% so that if execution gets interrupted it does not need to perform every
% optimization over again. You can see optimOpts.forceReoptimize to ensure
% you reoptimize with the new options. Or you can just delete saveFile so
% that there is no file to load.

pContour = [];
costContour = [];
iCosts = [];
%% Specify which parts of the data to group together
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
%%
numSubjOptim = (length(opts.subjects) - I.suEnd + 1);
numCondiOptim = (length(opts.conditions) - I.condiEnd + 1);
if optimOpts.AcrossSensors
    numSensorOptim = 1;
else
    numSensorOptim = 2;
    % numSensorOptim = (length(opts.sensorNames)  - I.sensorEnd + 1);
end
numSamplingOptim = (length(opts.SamplingFrequencies) - I.fsEnd + 1);
numAlgorithmOptim = length(opts.algorithms);

NumTotalOptim = numSubjOptim * numCondiOptim * numSensorOptim * numSamplingOptim * numAlgorithmOptim;

count = 0;
I0 = I;

                    if isfield(optimOpts,'skipRedo')
                        skipRedo = optimOpts.skipRedo;
                    else
                        skipRedo = Inf;
                    end

if (~optimOpts.forceReoptimize || skipRedo < Inf) && exist(saveFile,'file')
    loaded = load(saveFile,'P','optimOutput','costStar','pContour','costContour','opts','optimOpts');
end
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
                    fprintf('%d of %d optimizations...\n',count,NumTotalOptim);

                    % Some options depend on the algorithm so have an opts
                    % that changes each loop
                    opts = opts0;

                    %                     switch opts.algorithms{algo}
                    %                         case 'WilsonAHRS'
                    %                             if isfield(optimOpts.WilsonAHRS,'normStepSize')
                    %                                 opts.GradDescent.normStepSize = false;
                    %                             end
                    %                     end

                    if isfield(optimOpts.(opts.algorithms{algo}),'normStepSize')
                        opts.GradDescent.normStepSize = optimOpts.(opts.algorithms{algo}).normStepSize;
                    end


                    parmNames = optimOpts.(opts.algorithms{algo}).parameters.Names;



                    if (~optimOpts.forceReoptimize || count <= skipRedo) && exist(saveFile,'file')
                        % Load optimization parameters if already done
                        %                         if isequal(opts,loaded.opts) && isequal(optimOpts,loaded.optimOpts)
                        try
                            P{subject,condition,sensor,fs,algo} = loaded.P{subject,condition,sensor,fs,algo};
                            optimOutput{subject,condition,sensor,fs,algo} = loaded.optimOutput{subject,condition,sensor,fs,algo};
                            costStar(subject,condition,sensor,fs,algo) = loaded.costStar(subject,condition,sensor,fs,algo);
                            if length(parmNames) == 1 && optimOpts.useCostContourFor1D
                                pContour{subject,condition,sensor,fs,algo} = loaded.pContour{subject,condition,sensor,fs,algo};
                                costContour{subject,condition,sensor,fs,algo} = loaded.costContour{subject,condition,sensor,fs,algo};
                            end
                            if costStar(subject,condition,sensor,fs,algo) ~=0
                                
                                if isfield(opts,'outputIndividualCosts') && opts.outputIndividualCosts
                                    parmNames = optimOpts.(opts.algorithms{algo}).parameters.Names;
                                    DE = extractEnsembleData(D,I,opts);
                                    [CC,ccc] = ensembleCost(P{subject,condition,sensor,fs,algo},parmNames,algo,DE,opts);
                                    iCosts.Cost{subject,condition,sensor,fs,algo} = ccc;
                                    iCosts.subject{subject,condition,sensor,fs,algo} = vertcat(DE.subject);
                                    iCosts.condition{subject,condition,sensor,fs,algo} = {DE.condition}.';
                                    iCosts.sensor{subject,condition,sensor,fs,algo} = {DE.sensor}.';
                                    iCosts.samplingFrequency{subject,condition,sensor,fs,algo} = vertcat(DE.samplingFrequency);
                                    iCosts.algorithm{subject,condition,sensor,fs,algo} = repmat(opts.algorithms(algo),length(ccc),1);
                                    iCosts.parameters{subject,condition,sensor,fs,algo} = repmat( P{subject,condition,sensor,fs,algo},length(ccc),1);
                                end

                                
                                continue
                            end
                            blah = 1;
                        catch
                            %                                 warning('Options for loaded parameters appear different than current options')
                        end
                        %                         end
                    end
                    % otherwise optimize
                    
                    sensorsPresent = checkSensorInTrial(D,I,opts);
                    if ~any(sensorsPresent)
                        % No data for the chosen sensors in this trial
                        P{subject,condition,sensor,fs,algo} = [];
                        optimOutput{subject,condition,sensor,fs,algo} = [];
                        costStar(subject,condition,sensor,fs,algo) = NaN;
                        continue

                    end

                    


                    % Extract data for the optimization from the big data
                    % struct
                    DE = extractEnsembleData(D,I,opts);

                    if length(parmNames) == 1 && optimOpts.useCostContourFor1D
                        %                           p = [0.001 0.004 linspace(0,0.15,optimOpts.numPointsCostContour-2)].';
                        %                           p = sort(p);
                        %                           p1 = linspace(0,0.05,round(optimOpts.numPointsCostContour/2));
                        %                           p2 = linspace(0.05,0.2,round(optimOpts.numPointsCostContour/2)+1);
                        %                           p = [p1 p2(2:end)].';
                        p = [0 logspace(-4,log10(0.1),optimOpts.numPointsCostContour-1)].';
                        %                           p = [0 logspace(-4,log10(1),optimOpts.numPointsCostContour-1)].';

                        %                           cContour = ensembleCostContour(p,parmNames,algo,D,I,opts);
                        cContour = ensembleCostContour(p,parmNames,algo,DE,opts);

                        if opts.visualize
                            if count == 1
                                cc = figure();
                                clf;
                            end
                            try figure(cc)
                            catch
                                cc = figure();
                            end
                            %                           plot(p,costContour);
                            %                           plot(log10(p),costContour);
                            semilogx(p,cContour);
                            hold on

                            drawnow
                        end
                        pContour{subject,condition,sensor,fs,algo} = p;
                        costContour{subject,condition,sensor,fs,algo} = cContour;
                    end



                    cost = @(p) ensembleCost(p,parmNames,algo,DE,opts);

                    %                     cost = @(p) ensembleCost(p,parmNames,algo,D,I,opts);
                  [pStar,COST,optimO] = optimizeAHRSparameters(cost,opts.algorithms{algo},optimOpts);
% catch
%     keyboard
% end

                    %                     ensembleCost(pStar,parmNames,algo,DE,opts,true);


%                     etime = timeit(@() cost(pStar));

                    tic;
                    cost(pStar);
                    etime = toc;


                    P{subject,condition,sensor,fs,algo} = pStar;
                    optimOutput{subject,condition,sensor,fs,algo} = optimO;
                    costStar(subject,condition,sensor,fs,algo) = COST;
                    if optimOpts.intermediateSave
                        save(saveFile,'P','optimOutput','costStar','pContour','costContour','opts','optimOpts')
                    end
                end
            end
        end
    end
end

if isfield(opts,'outputIndividualCosts') && opts.outputIndividualCosts && ~isempty(iCosts)
    fnames = fieldnames(iCosts);
    for i  = 1 : length(fnames)
        iCosts.(fnames{i}) = vertcat(iCosts.(fnames{i}){:});
    end

end
end

% function [COST] = ensembleCost(p,pNames,algorithm,D,I,opts,plotIt)
% % Averages the cost across different trials stored in the data structure D
% % for a given set of parameters p (1xNP) where NP is the number of
% % parameters for the algorithm to optimize over
% if nargin < 7
%     plotIt = false;
% end
% COST = 0;
% count = 0;
% for subject = I.suStart : I.suEnd
%     ss = opts.subjects(subject);
%     sstr = ['Subject' num2str(ss)];
%     for condition = I.condiStart : I.condiEnd
%         cstr = opts.conditions{condition};
%         for sensor = I.sensorStart : I.sensorEnd
%             sestr = opts.sensorNames{sensor};
%             if ~isfield(D.(sstr).(cstr),sestr)
%                 continue
%             end
%             for fs = I.fsStart : I.fsEnd
%                 count = count + 1;
%                 data = D.(sstr).(cstr).(sestr).ResampleData(fs);
% %                 AHRS = D.(sstr).(cstr).(sestr).Estimators{algorithm,fs};
%                 AHRS = opts.AHRS(algorithm);
%                 samplingFrequency = D.(sstr).(cstr).(sestr).SampleRate(fs);
%                 [q,cost] = estimateAttitude(AHRS,p,pNames,samplingFrequency,data,opts);
%                 COST = COST + cost;
%
%                 if plotIt
%                 figure(9);
%                 clf
%                 plotEstimate([],q,data,[],opts);
%                 drawnow;
%                 end
%             end
%         end
%     end
% end
% COST = COST / count;
% end

function [COST,costs] = ensembleCost(p,pNames,algorithm,D,opts,plotIt)
if nargin < 6
    plotIt = false;
end

N = length(D);
costs = NaN(N,1);
AHRS = opts.AHRS(algorithm);
for i = 1 : N
    [q,costs(i)] = estimateAttitude(AHRS,p,pNames,D(i).samplingFrequency,D(i).data,opts);

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
        [~,costs(i,pp)] = estimateAttitude(AHRS,p(pp,:),pNames,D(i).samplingFrequency,D(i).data,opts);
    end
end
COST = mean(costs);
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
O(NTOTAL) = struct('data',[],'samplingFrequency',[]);

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
            for fs = I.fsStart : I.fsEnd
                count = count + 1;

                O(count).data = D.(sstr).(cstr).(sestr).ResampleData(fs);
                O(count).data.bads = D.(sstr).(cstr).(sestr).badMocapFramesResampled{fs};
                O(count).samplingFrequency = D.(sstr).(cstr).(sestr).SampleRate(fs);
                O(count).subject = ss;
                O(count).condition = cstr;
                O(count).sensor = sestr;
            end
        end
    end
end
end

% function [COST] = ensembleCostContour(p,pNames,algorithm,D,I,opts)
% % Averages the cost contour for algorithm for a given set of data
% % trials D. p is a NS x NP matrix of parameters where NS is the number of
% % different parameter values to sample over, and NP is the number of
% % parameters for the algorithm.
% COST = zeros(size(p,1),1);
% count = 0;
% for subject = I.suStart : I.suEnd
%     ss = opts.subjects(subject);
%     sstr = ['Subject' num2str(ss)];
%     for condition = I.condiStart : I.condiEnd
%         cstr = opts.conditions{condition};
%         for sensor = I.sensorStart : I.sensorEnd
%             sestr = opts.sensorNames{sensor};
%             if ~isfield(D.(sstr).(cstr),sestr)
%                 continue
%             end
%             for fs = I.fsStart : I.fsEnd
%                 count = count + 1;
%                 data = D.(sstr).(cstr).(sestr).ResampleData(fs);
% %                 AHRS = D.(sstr).(cstr).(sestr).Estimators{algorithm,fs};
%                 AHRS = opts.AHRS(algorithm);
%                 samplingFrequency = D.(sstr).(cstr).(sestr).SampleRate(fs);
%                 cost = NaN(size(p,1),1);
%                 for pp = 1 : size(p,1)
%                 [~,cost(pp)] = estimateAttitude(AHRS,p(pp,:),pNames,samplingFrequency,data,opts);
%                 end
%                 COST = COST + cost;
%             end
%         end
%     end
% end
% COST = COST / count;
% end

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