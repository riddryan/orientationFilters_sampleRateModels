opts = defaultDataOpts;
opts.subjects = [1:4 6:12];
opts.conditions = {'standing_ref1' 'standing_bending' 'right_left_lift' ...
    'sitting_bend' 'sit_stand_normal' 'sit_stand_fast' 'walk_3kmh' ...
    'walk_5kmh' 'og_walk_sss' 'figure8_walking' 'og_jog_sss' };

files = setFileNames(opts);

saveFigs = false;
figDir = [files.figDir 'optimized' filesep];
saveFormats = {'fig' 'png' 'epsc'};
tableDir = [files.figDir 'tables' filesep];
%% Load results
% This file optimizes each filter's parameters across all participants and
% conditions for each sample frequency


% Optimized across all participants & conditions for each sample frequency
file_SF = [files.optimizedParametersDir 'samplingFrequencyOptimized.mat'];


loaded = load(file_SF);
T = convertDataStructToTable(loaded.O);
T = removeOutliers(T,outlierTable());

algos = unique(T.Algorithm,'stable');
FS = unique(T.SamplingFrequency);
FSorig = FS;

% Load dataset as well
dataSetFile = [files.dataDir 'validation2021_dataset.mat'];
load(dataSetFile);
%% Show an example of degradation of performance w.r.t. Sampling Frequency



AHRS = RiddickAHRS;
opts.GradDescent.normStepSize = false;

optimOpts = defaultOptimOpts;
pNames = optimOpts.RiddickAHRS.parameters.Names;




DD = D.Subject1.walk_3kmh.mdm650D;
testData = DD.ResampleData;
testFSdex = [5 7 10];
dataDex = 10 - testFSdex + 1;
testSamplingFrequencies = FS(testFSdex); % 10.2 Hz, 33.05 Hz, 190 Hz

clear q costs eaEst
for i = 1 : length(testSamplingFrequencies)

% Set optimal parameters
dex = T.SamplingFrequency == testSamplingFrequencies(i) & strcmp(T.Algorithm,'RiddickAHRS');
Tsub = T(dex,:);
PSTAR = mean(vertcat(Tsub.Parameters{:}));


[q{i},costs(i)] = estimateAttitude2(AHRS,PSTAR,pNames,testSamplingFrequencies(i),testData(dataDex(i)),opts);


eaEst{i} = eulerd(quaternion(q{i}),'ZYX','point');

end
eaTruth = eulerd(quaternion(DD.Data.ground_truth),'ZYX','point');


tRange = mean(DD.Data.time) + [0 4];
figure;

for dim = 1 : 3
    subplot(3,1,dim)

    [~,IS] = min(abs(DD.Data.time - tRange(1)));
    [~,IE] = min(abs(DD.Data.time - tRange(2)));
plot(DD.Data.time(IS:IE),eaTruth(IS:IE,dim),'k');
hold on

for i = 1 : length(testSamplingFrequencies)
    [~,IS] = min(abs(testData(dataDex(i)).time - tRange(1)));
    [~,IE] = min(abs(testData(dataDex(i)).time - tRange(2)));
    plot(testData(dataDex(i)).time(IS:IE),eaEst{i}(IS:IE,dim));
end
legend([{'Truth'};cellstr(num2str(FS(testFSdex)))])
end


%% Error vs sample frequency

FS = FSorig(4:end);

% fitFreqDex = 5 : length(FS);
% fitFreqs = FS(fitFreqDex);
fitFreqDex = 1 : length(FS);

modelType = 'exponential';
% modelType = 'hyperbolic';
% modelType = 'rat12';
% modelType = 'poly2';

% useLogLogScale = true;
useLogLogScale = false;
errorModel = 'constant';

hf = figure;
% hf.WindowState = 'maximized';
hf.Units = 'Inches';
hf.Position = [1 1 3.5 3.5];
hf.Color = [1 1 1];
cols = jet(length(algos));
for aa = 1 : length(algos)

    Tsub = T(strcmp(T.Algorithm,algos{aa}),:);

    switch modelType
        case 'exponential'
            model = @(phi,fs) phi(1).*exp(phi(2).*fs) + phi(3);
            beta0 = [7 -0.1 1];
        case 'hyperbolic'
            model = @(phi,fs) phi(1)./(fs) + phi(2);
            beta0 = [50 4];

        case 'rat22'
            model = @rat22model;
            beta0 = [0.06 -0.15 11.6 -11 74];
        case 'rat12'
            model = @rat12model;
            beta0 = [1e5 4e6 4e4 8e4];
        case 'rat11'
            model = @rat11model;
            beta0 = [2 86 2];
        case 'poly2'
            model = @(phi,fs) phi(1)*fs.^2 + phi(2)*fs + phi(3);
            beta0 = [0.1 -0.1 20];
    end


    meanCosts = [];
    for j = 1 : length(FS)
        meanCosts(j,1) = mean(Tsub.Cost(Tsub.SamplingFrequency==FS(j)));
    end

    meanCosts = meanCosts/3;

    mdl = fitnlm(FS(fitFreqDex),meanCosts(fitFreqDex),model,beta0,'ErrorModel',errorModel);

%     mdlAllMeas = fitnlm(Tsub.SamplingFrequency,Tsub.Cost,model,beta0,'ErrorModel',errorModel);

    Emods{aa} = mdl;

%             fsi = linspace(FS(1),FS(end),100).';
        fsi = linspace(1,200,100).';

    if useLogLogScale
%         semilogx(FS,meanCosts,'.','Color',cols(aa,:),'HandleVisibility','off','MarkerSize',10)
%         hold on
% 
% 
%         semilogx(fsi,predict(mdl,fsi),'Color',cols(aa,:))

        loglog(FS,meanCosts,'.','Color',cols(aa,:),'HandleVisibility','off','MarkerSize',10)
        hold on
        loglog(FS,predict(mdl,FS),'Color',cols(aa,:))

        set(gca,'XTick',[1 10 100 190]);
set(gca,'XTickLabel',get(gca,'XTick'))
set(gca,'Box','Off');


        ylabel('Error')
        xlabel('Log(Sample Frequency) (Hz)')
    else
        plot(FS,meanCosts,'.','Color',cols(aa,:),'HandleVisibility','off','MarkerSize',10)
        hold on
        plot(fsi,predict(mdl,fsi),'Color',cols(aa,:))
        ylabel('Error (°)')
        xlabel('Sampling Frequency (Hz)')
        set(gca,'Box','Off');
    end
end

set(gca,'YLim',[0 8]);

hL=legend(algos,'Interpreter','none');
hL.Box = 'off';

if saveFigs
    for i = 1 : length(saveFormats)
        figName = [figDir 'ErrorVsFrequency'];% saveFormats{i}];
        saveas(gcf,figName,saveFormats{i});
    end
end

%% Error table

    clear coeffsOverall rSquaredOverall coeffsCondis rSquaredCondis
    for aa = 1 : length(Emods)
        coeffs  = Emods{aa}.Coefficients.Estimate;
        % Transform coefficient variables to those in paper
        coeffs(1) = coeffs(1) + coeffs(3);
        coeffs(2) = -coeffs(2);
        coeffs = [coeffs(1) coeffs(3) coeffs(2)];
        coeffsOverall(aa,:) = coeffs;
        rSquaredOverall(aa,:) = Emods{aa}.Rsquared.Adjusted;


    end
        X = [coeffsOverall rSquaredOverall];
        X = [compose('%.02f',X(:,1:2)) compose('%.03f',X(:,3)) compose('%.02f',X(:,4))];
        errorTable = array2table(X,"VariableNames",{'E0' 'Einfinity'  'lambda'  'adj-rsquared'},"RowNames",algos);
        writetable(errorTable,[tableDir 'ErrorTable_CompareAlgos'  '.xlsx'],'WriteRowNames',true)

%% Parameters vs Sample Frequency
FS = FSorig;
TP = T;


% fitFreqDex = 1:length(FS);
fitFreqDex = 4 : length(FS);
fitFreqs = FS(fitFreqDex);

% Weights = [0.1*ones(1,4) 10*ones(1,6)];
Weights = ones(1,length(fitFreqs));
% Weights(end) = 100;

% modelType = 'exponential';
% modelType = 'gamma';
% modelType = 'hyperbolic';
% modelType = 'rat11';
% modelType = 'poly2';



% modelTypes = { {'exp' 'exp' 'exp'} {'exp'} {'exp'} {'exp'} {'rat22' 'rat11' 'rat12'} {'exp' 'exp'}};
% modelTypes = { {'rat22' 'rat22' 'rat22'} {'exp'} {'exp'} {'exp'} {'rat22' 'rat11' 'rat12'} {'exp' 'exp'}};
% modelTypes = { {'rat12' 'rat12' 'rat12'} {'exp'} {'exp'} {'exp'} {'rat22' 'rat11' 'rat12'} {'rat22' 'rat22'}};
% modelTypes = { {'rat12c' 'rat12c' 'rat12c'} {'rat12c'} {'rat12c'} {'rat12c'} {'rat22' 'rat11' 'rat12'} {'rat12c' 'rat12c'}};

% modelTypes = { {'exp' 'kalman1' 'kalman1'} {'kalman1'} {'kalman1'} {'kalman1'} {'rat22' 'rat11' 'rat12'} {'kalman2' 'kalman2'}};
% modelTypes = { {'exp' 'rat22c' 'rat22c'} {'rat12c'} {'rat12c'} {'rat12c'} {'rat22' 'rat11' 'rat12'} {'rat12c' 'rat12c'}};

modelTypes = { {'exp' 'exp' 'exp'} {'exp'} {'exp'} {'exp'} {'rat22' 'rat22' 'rat22'} {'exp' 'exp'}};

for algoDex = 1 : length(algos)
    hf = figure;
hf.WindowState = 'maximized';
hf.Color = [1 1 1];
    parmNames = optimOpts.(algos{algoDex}).parameters.Names;
    for pp = 1:length(parmNames)

        modelType = modelTypes{algoDex}{pp};
%         modelType = 'exponential';
        parmDex = pp;

        Tsub = TP(strcmp(TP.Algorithm,algos{algoDex}),:);

       


        PARMS = cell2mat(Tsub.Parameters);
        switch modelType
            case {'exponential' 'exp'}
                model = @(phi,fs) phi(1).*exp(phi(2).*fs) + phi(3);
                beta0 = [0.38 -0.28 0.02];
            case 'hyperbolic'
                model = @(phi,fs) phi(1)./(fs + phi(2)) + phi(3);
        
                beta0 = [1 1 0.05];
            case 'gamma'
                %             model = @(phi,fs) (fs.^(phi(1)-1).*exp(-phi(2)*fs).*phi(2)^(phi(1)))./gamma(phi(1));
                model = @gammaModel;
                beta0 = [3 2 0.06];
            case 'rat22'
                model = @rat22model;
                beta0 = [0.06 -0.15 11.6 -11 74];
            case 'rat22c'
                model = @rat22modelc;
%                 beta0 = [0.1 -0.004 0.0002 1 0.08 0.09];

beta0 = [0.1 0.0002 1 0.09];
            case 'rat11'
                model = @rat11model;
                beta0 = [2 86 2];
            case 'rat12'
                model = @rat12model;
%                 beta0 = [1e5 4e6 4e4 8e4];
                    beta0 = [0.5 2.7 -4 25];
            case 'rat12c'
                model = @rat12model_correctedOffset;
%                  beta0 = [0.5 2.7 -4 25 0.05];

%         beta0 = [0.15 -0.004 0.08 0.09 0.0002553];
%                 beta0 = [0.15 -0.004 0.0002 1 0.08 0.09];
                beta0 = [0.15 0.0002 1 0.09];
            case 'kalman1'
                model = @optimalKalman1model;
                beta0 = [1 1 1];
%                 beta0 = [1 1];
            case 'kalman2'
                model = @optimalKalman2model;
                                beta0 = [1 1];
            case 'kalman3'
                model = @optimalKalman3model;
                beta0 = [1 1 1];
            case 'rat01'
                model = @rat01model;
                beta0 = [2 2];
            case 'rat02'
                model = @rat02model;
                beta0 = [2 -11 2];
            case 'poly2'
                model = @(phi,fs) phi(1)*fs.^2 + phi(2)*fs + phi(3);
                beta0 = [0.1 -0.1 0.1];
        end

        fitDex = false(size(Tsub.SamplingFrequency));
        for i = 1 : length(fitFreqs)
            fitDex(Tsub.SamplingFrequency == fitFreqs(i)) = true;
        end


        meanParms= [];
        for j = 1 : length(FS)
            meanParms(j,parmDex) = mean(PARMS(Tsub.SamplingFrequency==FS(j),parmDex));
        end

        mdl = fitnlm(fitFreqs,meanParms(fitFreqDex,parmDex),model,beta0,'Weights',Weights);


        subplot(length(parmNames),1,pp)
        plot(FS,meanParms(:,pp),'.')
        hold on
        fsi = linspace(fitFreqs(1),fitFreqs(end),500).';
        plot(fsi,predict(mdl,fsi),'r')

        % Plot data points not used in the fit in green
        L = true(size(FS));
        L(fitFreqDex) = false;
        plot(FS(L),meanParms(L,pp),'g.')

        title([algos{algoDex} ' ' parmNames{pp}],'Interpreter','none')
    end

    if saveFigs
        for i = 1 : length(saveFormats)
            figName = [figDir 'ParametersVsFrequency_' algos{algoDex}];% saveFormats{i}];
            saveas(gcf,figName,saveFormats{i});
        end
    end
end
%% Compare filters at specific frequencies: Linear mixed effects model
mdl = 'Cost ~ Algorithm + (1|Condition) + (1|Subject) + (1|SensorLocation)';
clear Tfreq lme pVals predictedError predictedErrorCI peUCI peLCI

for i = 1 : length(FS)
    % Data for a certain freq
    Tfreq{i} = T(T.SamplingFrequency == FS(i),:);

    Tfreq{i}.Cost = Tfreq{i}.Cost/3;
    
    % Linear mixed model fitting
    lme{i} = fitlme(Tfreq{i},mdl);
    
    % Predict for each algorithm the estimated response due to the fixed
    % effect (Algorithm)
    Ttest = Tfreq{i}(1:length(algos),:);
    Ttest.Algorithm = algos;
    [predictedError(:,i),peCI] = predict(lme{i},Ttest,'Conditional',false,'Simultaneous',false);

    peUCI(:,i) = peCI(:,2);
    peLCI(:,i) = peCI(:,1);
    
    pVals(:,i) = lme{i}.Coefficients.pValue;

end

%% Bar plot: For each frequency (LME)
hf = figure;
hf.WindowState = 'maximized';
hf.Color = [1 1 1];
colororder(cols);
b = bar(predictedError.');

X = nan(length(FS), length(algos));
for i = 1:length(algos)
    X(:,i) = b(i).XEndPoints;
end

hold on
errorbar(X,predictedError.',peLCI.',peUCI.','k.')
set(gca,'XTickLabel',num2str(FS,'%.01f'));
xlabel('Sampling Frequency (Hz)')
legend(algos)
ylabel('Summed RMS error (°)')
title('Across all conditions')

if saveFigs
    for i = 1 : length(saveFormats)
        figName = [figDir 'ErrorVsFrequency_BarPlot'];% saveFormats{i}];
        saveas(gcf,figName,saveFormats{i});
    end
end

%% Bar plot: 190 & 10 Hz

hf = figure;
% hf.WindowState = 'maximized';
hf.Units = 'inches';
hf.Position = [1 1 3.5 1.75];
hf.Color = [1 1 1];

subplot(121)
fsdex = length(FS);
b = bar(predictedError(:,fsdex),'FaceColor','flat');
b.CData = cols;
hold on
errorbar(1:length(algos),predictedError(:,fsdex),peLCI(:,fsdex),peUCI(:,fsdex),'k.');
set(gca,'XTickLabel',algos);
ylabel('Error (°)')
title([num2str(FS(fsdex),'%.01f') ' Hz across all conditions']);

pp = pVals(2:end,fsdex);
pGroups = {[1 2] [1 3] [1 4] [1 5] [1 6]};
sigstar(pGroups,pp);
set(gca,'Box','Off')
    if saveFigs
        for i = 1 : length(saveFormats)
            figName = [figDir 'ErrorVsFrequency_BarPlot_190Hz'];% saveFormats{i}];
            saveas(gcf,figName,saveFormats{i});
        end
    end

subplot(122)
fsdex = 4;
b = bar(predictedError(:,fsdex),'FaceColor','flat');
b.CData = cols;
hold on
errorbar(1:length(algos),predictedError(:,fsdex),peLCI(:,fsdex),peUCI(:,fsdex),'k.');
set(gca,'XTickLabel',algos);
ylabel('Error (°)')
title([num2str(FS(fsdex),'%.01f') ' Hz across all conditions']);

set(gca,'Box','Off')

pp = pVals(2:end,fsdex);
pGroups = {[1 2] [1 3] [1 4] [1 5] [1 6]};
sigstar(pGroups,pp);

    if saveFigs
        for i = 1 : length(saveFormats)
            figName = [figDir 'ErrorVsFrequency_BarPlot_10_190_Hz'];% saveFormats{i}];
            saveas(gcf,figName,saveFormats{i});
        end
    end

%% Bar plot: 190 Hz (LME)

hf = figure;
% hf.WindowState = 'maximized';
hf.Units = 'inches';
hf.Position = [1 1 3.5 3.5];
hf.Color = [1 1 1];
fsdex = length(FS);
b = bar(predictedError(:,fsdex),'FaceColor','flat');
b.CData = cols;
hold on
errorbar(1:length(algos),predictedError(:,fsdex),peLCI(:,fsdex),peUCI(:,fsdex),'k.');
set(gca,'XTickLabel',algos);
ylabel('Error (°)')
title([num2str(FS(fsdex),'%.01f') ' Hz across all conditions']);

pp = pVals(2:end,fsdex);
pGroups = {[1 2] [1 3] [1 4] [1 5] [1 6]};
sigstar(pGroups,pp);
set(gca,'Box','Off')
    if saveFigs
        for i = 1 : length(saveFormats)
            figName = [figDir 'ErrorVsFrequency_BarPlot_190Hz'];% saveFormats{i}];
            saveas(gcf,figName,saveFormats{i});
        end
    end

%% Bar plot: 10 Hz (LME)
hf = figure;
hf.Units = 'inches';
hf.Position = [1 1 3.5 3.5];
hf.Color = [1 1 1];
fsdex = 5;
b = bar(predictedError(:,fsdex),'FaceColor','flat');
b.CData = cols;
hold on
errorbar(1:length(algos),predictedError(:,fsdex),peLCI(:,fsdex),peUCI(:,fsdex),'k.');
set(gca,'XTickLabel',algos);
ylabel('Error (°)')
title([num2str(FS(fsdex),'%.01f') ' Hz across all conditions']);

set(gca,'Box','Off')

pp = pVals(2:end,fsdex);
pGroups = {[1 2] [1 3] [1 4] [1 5] [1 6]};
sigstar(pGroups,pp);

    if saveFigs
        for i = 1 : length(saveFormats)
            figName = [figDir 'ErrorVsFrequency_BarPlot_10Hz'];% saveFormats{i}];
            saveas(gcf,figName,saveFormats{i});
        end
    end
%% Bar plot: 1 Hz (LME)
hf = figure;
hf.WindowState = 'maximized';
hf.Color = [1 1 1];
fsdex = 1;
b = bar(predictedError(:,fsdex),'FaceColor','flat');
b.CData = cols;
hold on
errorbar(1:length(algos),predictedError(:,fsdex),peLCI(:,fsdex),peUCI(:,fsdex),'k.');
set(gca,'XTickLabel',algos);
ylabel('Summed RMS error (°)')
title([num2str(FS(fsdex),'%.01f') ' Hz across all conditions']);

pp = pVals(2:end,fsdex);
pGroups = {[1 2] [1 3] [1 4] [1 5] [1 6]};
sigstar(pGroups,pp);

    if saveFigs
        for i = 1 : length(saveFormats)
            figName = [figDir 'ErrorVsFrequency_BarPlot_1Hz'];% saveFormats{i}];
            saveas(gcf,figName,saveFormats{i});
        end
    end