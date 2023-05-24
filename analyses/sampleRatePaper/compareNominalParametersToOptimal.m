%% Load results of optimal parameter values across all conditions
% This file optimizes each filter's parameters across all participants and
% conditions for each sample frequency

files = setFileNames();
% Optimized across all participants & conditions for each sample frequency
file_SF = [files.optimizedParametersDir 'samplingFrequencyOptimized.mat'];


loaded = load(file_SF);
T = convertDataStructToTable(loaded.O);
T = removeOutliers(T,outlierTable());

algos = unique(T.Algorithm,'stable');
FS = unique(T.SamplingFrequency);

opts = loaded.O.opts{1};
% % This needs to be set true to match the original optimization. Ideally
% should save the running options for the overall optimization rather than
% for each iteration separately so nothing needs to be set manually here.
opts.GradDescent.normStepSize = true; 
optimOpts = loaded.O.optimOpts{1};

saveFigs = false;

figDir = [files.figDir 'comparisonToNominal' filesep];
saveFormats = {'fig' 'png' 'epsc'};

% Graphics options
fontSize = 8;
fontName = 'Arial';
singleColumnWidth = 9;
doubleColumnWidth = 19;
onepointfiveColumnWidth = 14;

lightBlue=[0.3 0.8 0.95];
darkBlue = [0.03 0.07 0.6];
blueGrad = @(i,N) lightBlue + (darkBlue-lightBlue)*((i-1)/(N-1));

%% Pick out optimal parameter values for 190 Hz sampling rate
clear PSTAR
for aa = 1 : length(algos)
    dex = T.SamplingFrequency == FS(end) & strcmp(T.Algorithm,algos{aa});
    Tsub = T(dex,:);
    PSTAR{aa,1} = mean(vertcat(Tsub.Parameters{:}));
    optimOpts.(algos{aa}).parameters.Initial = PSTAR{aa,1};
end
%% Load dataset
dataSetFile = [files.dataDir 'validation2021_dataset.mat'];
load(dataSetFile);

%% Apply these nominal optimal 190 Hz parameter values at each sample frequency (from 1 to 190 Hz)
% Note we are just estimating error from using these 190 Hz parameters at
% each sampling frequency, not re-optimizing

nominalFile = [files.optimizedParametersDir 'nominal190results.mat'];
optimOpts.intermediateSave = false;
optimOpts.dontOptimize = true;



if exist(nominalFile,'file')
    load(nominalFile);
else
    O = optimizeAHRSacrossDataset(D,[],optimOpts,opts);
    save(nominalFile,'O')
end
TF = convertDataStructToTable(O);
TF = removeOutliers(TF,outlierTable());


%% Apply nominal value of Beta = 0.1 for GDF filters

nominalFileGDF = [files.optimizedParametersDir 'nominalGDFresults.mat'];

opts = loaded.O.opts{1};
% % This needs to be set true to match the original optimization. Ideally
% should save the running options for the overall optimization rather than
% for each iteration separately so nothing needs to be set manually here.
opts.GradDescent.normStepSize = true; 
optimOpts = loaded.O.optimOpts{1};
optimOpts.intermediateSave = false;
optimOpts.dontOptimize = true;

opts.AHRS = [RiddickAHRS WilsonAHRS AdmiraalAHRS MadgwickAHRS3];
opts.algorithms = [];
for i = 1 : length(opts.AHRS)
    opts.algorithms{i} = class(opts.AHRS(i));

    if strcmp(opts.algorithms{i},'RiddickAHRS')
        p0 = [1 0.1 0.1];
    else
        p0 = 0.1;
    end
    optimOpts.(opts.algorithms{i}).parameters.Initial = p0;
end

if exist(nominalFileGDF,'file')
    load(nominalFileGDF);
else
    O = optimizeAHRSacrossDataset(D,[],optimOpts,opts);
    save(nominalFileGDF,'O')
end
TG = convertDataStructToTable(O);
TG = removeOutliers(TG,outlierTable());

%% Apply optimal parameter values and recompute cost
% Testing bugs in code, should recover original optimization results
testFile = [files.optimizedParametersDir 'testNominalRun.mat'];


opts = loaded.O.opts{1};
% % This needs to be set true to match the original optimization. Ideally
% should save the running options for the overall optimization rather than
% for each iteration separately so nothing needs to be set manually here.
opts.GradDescent.normStepSize = true; 
optimOpts = loaded.O.optimOpts{1};
optimOpts.intermediateSave = false;
optimOpts.dontOptimize = true;

if exist(testFile,'file')
    load(testFile);

else

    clear tt
for i = 1 : length(FS)
    opts.SamplingFrequencies = FS(i);
    for aa = 1 : length(opts.AHRS)
        dex = T.SamplingFrequency == FS(i) & strcmp(T.Algorithm,algos{aa});
        Tsub = T(dex,:);
        PSTAR = mean(vertcat(Tsub.Parameters{:}));


        optimOpts.(opts.algorithms{aa}).parameters.Initial = PSTAR;

    end

    O = optimizeAHRSacrossDataset(D,[],optimOpts,opts);
    tt{i} = convertDataStructToTable(O);
    tt{i} = removeOutliers(tt{i},outlierTable());

end
TT = vertcat(tt{:});
save(testFile,'TT')
end


TW10 = T(T.SamplingFrequency==FS(5) & strcmp(T.Algorithm,'WilsonAHRS'),:);
TW10r = TT(TT.SamplingFrequency==FS(5) & strcmp(TT.Algorithm,'WilsonAHRS'),:);
figure;plot(TW10.Cost);hold on;plot(TW10r.Cost)

TRR10 = T(T.SamplingFrequency==FS(5) & strcmp(T.Algorithm,'RiddickAHRS'),:);
TRR10r = TT(TT.SamplingFrequency==FS(5) & strcmp(TT.Algorithm,'RiddickAHRS'),:);

TKF10 = T(T.SamplingFrequency==FS(5) & strcmp(T.Algorithm,'FastKF'),:);
TKF10r = TT(TT.SamplingFrequency==FS(5) & strcmp(TT.Algorithm,'FastKF'),:);

TJ10 = T(T.SamplingFrequency==FS(5) & strcmp(T.Algorithm,'JustaAHRS'),:);
TJ10r = TT(TT.SamplingFrequency==FS(5) & strcmp(TT.Algorithm,'JustaAHRS'),:);

TM10 = T(T.SamplingFrequency==FS(5) & strcmp(T.Algorithm,'MadgwickAHRS3'),:);
TM10r = TT(TT.SamplingFrequency==FS(5) & strcmp(TT.Algorithm,'MadgwickAHRS3'),:);

figure;plot(TKF10.Cost);hold on;plot(TKF10r.Cost)

%% Error vs sample frequency
fsdex = 4:10;


modelType = 'exponential';
% modelType = 'hyperbolic';
% modelType = 'rat11';

useLogLogScale = true;

GDFdex = 1:4;



hf = figure;
% hf.WindowState = 'maximized';
hf.Color = [1 1 1];
hf.Units = 'centimeters';
hf.Position = [2 2 doubleColumnWidth singleColumnWidth];

cols = jet(length(algos));
for aa = 1 : length(algos)
    subplot(2,3,aa)

    Tsub = T(strcmp(T.Algorithm,algos{aa}),:);
    Tsub2 = TF(strcmp(TF.Algorithm,algos{aa}),:);


    switch modelType
        case 'exponential'
                    model = @expmodel;
                    if algoDex == 1 && pp == 1
                        beta0 = [-0.2 -0.23 1];
                    else
                    beta0 = [0.3 -0.15 0.01];
                    end
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
    end

    meanCosts = [];
    meanCosts2 = [];
    for j = 1 : length(FS)
        meanCosts(j,1) = mean(Tsub.Cost(Tsub.SamplingFrequency==FS(j)));
        meanCosts2(j,1) = mean(Tsub2.Cost(Tsub2.SamplingFrequency==FS(j)));
    end
    
    % Divide by 3 to get average error (instead of summed across 3
    % dimensions)
    meanCosts = meanCosts/3;
    meanCosts2 = meanCosts2/3;

    mdl = fitnlm(FS(fsdex),meanCosts(fsdex),model,beta0);
    mdl2 = fitnlm(FS(fsdex),meanCosts2(fsdex),model,beta0);



    if any(aa == GDFdex)
        Tsub3 = TG(strcmp(TG.Algorithm,algos{aa}),:);

        meanCosts3 = [];
        for j = 1 : length(FS)
            meanCosts3(j,1) = mean(Tsub3.Cost(Tsub3.SamplingFrequency==FS(j)));
        end
        meanCosts3 = meanCosts3/3;

        mdl3 = fitnlm(FS,meanCosts3,model,beta0);
    end



    if useLogLogScale
        
        
        semilogx(FS(fsdex),predict(mdl,FS(fsdex)),'Color',[0 0 0])
        hold on
        semilogx(FS(fsdex),predict(mdl2,FS(fsdex)),'Color',[1 0 0])

        semilogx(FS(fsdex),meanCosts(fsdex),'.','Color',[0 0 0],'MarkerSize',10)
        semilogx(FS(fsdex),meanCosts2(fsdex),'o','Color',[1 0 0],'MarkerSize',6)
        

%         plot(FS,meanCosts,'.','Color',[0 0 0],'MarkerSize',10)
%         hold on
%         plot(FS,predict(mdl,FS),'Color',[0 0 0])
% 
%         plot(FS,meanCosts2,'o','Color',[1 0 0],'MarkerSize',6)
%         plot(FS,predict(mdl2,FS),'Color',[1 0 0])



        set(gca,'XTick',[1 10 100 190]);
        set(gca,'XTickLabel',get(gca,'XTick'))

        ylabel('Error')
        xlabel('Sample Frequency (Hz)')
    else
        plot(FS(fsdex),meanCosts(fsdex),'.','Color',[0 0 0],'MarkerSize',10)
%         plot(FS,meanCosts,'.','Color',[0 0 0],'HandleVisibility','off','MarkerSize',10)
        
        hold on
%         plot(FS,predict(mdl,FS),'Color',[0 0 0])
plot(FS(fsdex),meanCosts(fsdex),'o','Color',[0 0 0],'MarkerSize',6)
%         plot(FS,meanCosts2,'.','Color',[1 0 0],'HandleVisibility','off','MarkerSize',10)
%         plot(FS,predict(mdl2,FS),'Color',[1 0 0])

%         if any(aa == GDFdex)
% plot(FS,meanCosts3,'x-','Color',[0 0 1],'MarkerSize',6)
% %         plot(FS,meanCosts3,'.','Color',[0 0 1],'HandleVisibility','off','MarkerSize',10)
% %         plot(FS,predict(mdl3,FS),'Color',[0 0 1])
%         end



        ylabel('Error (°)')
        xlabel('Sample Frequency (Hz)')
    end

                set(gca,'Box','Off')
            set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');

%     set(gca,'Box','Off');
    title(algos{aa})

%     if any(aa == GDFdex)
%         legend('Optimized','Nominal-190','Nominal-0.1');
%     else
        
%     end

    optimModel{aa} = mdl;
    model190{aa} = mdl2;
    if any(aa == GDFdex)
        modelBeta{aa} = mdl3;
    else
        modelBeta{aa} = [];
    end

    if aa == length(algos)
        legend('Optimized','Nominal','Interpreter','none');
    end
    set(gca,'YLim',[0 15])
end



    if saveFigs
        for i = 1 : length(saveFormats)
            figName = [figDir 'OptimalAndNominalError_vsFrequency' ];% saveFormats{i}];
            saveas(gcf,figName,saveFormats{i});
        end
    end

    
%% Compute cut-off frequency or threshold on error
% This table shows the minimum frequency that there can be less than or
% equal to an average of 3 degrees or less in RMS error for each angle
% across the dataset for different filters (columns) and for 3 cases:
% 1) Row 1: Optimized for sample frequency
% 2) Row 2: The optimal parameters for 190 Hz applied at all sampling
% frequencies
% 3) Row 3: Beta = 0.1 for gradient descent filters as recommended by some
% of the original papers
%
% This cutoff is determined by the rational polynomial model fit to the
% error of each filter, across all conditions
%
% If we use the optimized parameters for the Riddick filter, the answer for
% "how low can we go?" is about 11 Hz, whereas if we use a typical GDF
% parameter of Beta = 0.1, or just use the sample parameter as is optimal
% for 190 Hz, the answer would be ~23 Hz. This means our filter adaptations
% + optimization results enable us to lower the sample rate by nearly half
% while stile acheiving our target performance level.

cutOff = 3; %Hz
for aa = 1 : length(algos)
cutOffFreqOptim(aa) = fsolve(@(fs) cutOff - predict(optimModel{aa},fs),10);
cutOffFreq190(aa) = fsolve(@(fs) cutOff - predict(model190{aa},fs),10);

if ~isempty(modelBeta{aa})
    cutOffFreqBeta(aa) = fsolve(@(fs) cutOff - predict(modelBeta{aa},fs),20);
else
    cutOffFreqBeta(aa) = NaN;
end
end
% cutOffs = [cutOffFreqOptim;cutOffFreq190;cutOffFreqBeta];
cutOffs = [cutOffFreqOptim;cutOffFreq190];

cutOffTable = array2table(cutOffs);
cutOffTable.Properties.VariableNames = algos;
% cutOffTable.Properties.RowNames = {'Optimized' 'Nominal190Parameters' 'Beta = 0.1'};
cutOffTable.Properties.RowNames = {'Optimized' 'Nominal190Parameters'};
cutOffTable.Properties.VariableUnits = {'Hz' 'Hz' 'Hz' 'Hz' 'Hz' 'Hz' };
cutOffTable

hf = figure;
% hf.WindowState = 'maximized';
hf.Color = [1 1 1];
hf.Units = 'centimeters';
hf.Position = [2 2 doubleColumnWidth 0.25*doubleColumnWidth];
b = bar(cutOffs.');

b(1).FaceColor = [0 0 0];
b(2).FaceColor = [1 0 0];
% b.CData = cols;
set(gca,'XTickLabel',algos)
ylabel('Sample Frequency (Hz)')
legend('Optimized','Nominal (190 Hz)','Beta = 0.1')
title('Min Sampling Frequency (for < 3° average RMS error)')

                set(gca,'Box','Off')
            set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');


if saveFigs
    for i = 1 : length(saveFormats)
        figName = [figDir 'ErrorThreshold_Barplots_3degees'];% saveFormats{i}];
        saveas(gcf,figName,saveFormats{i});
    end
end
