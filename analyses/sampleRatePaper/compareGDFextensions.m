opts = defaultDataOpts;
files = setFileNames(opts);

% Wilson Filter (1 param GDF). Also other filters in this file, need to
% pick wilson out
file_Wilson = [files.optimizedParametersDir 'samplingFrequencyOptimized.mat'];

% Basic 2 gain Riddick filter with no additional corrections or parameters
file_basicRiddick = [files.optimizedParametersDir 'basicRiddick.mat'];

% For the Riddick filters:
% Show difference when gradient step is un-normalized
file_Riddick_NormStep =  [files.optimizedParametersDir 'Riddick_UnnormedStep.mat'];

% Show the difference when using prediction correction
file_Riddick_PredictionCorrection = [files.optimizedParametersDir 'Riddick_PredictionCorrection.mat'];

% Optimize with a gain on gyro
file_Riddick_GyroGain = [files.optimizedParametersDir 'Riddick_GyroGain.mat'];

% Optimize with second order integration model
file_Riddick_2ndOrderIntegration = [files.optimizedParametersDir 'Riddick_2ndOrderIntegration.mat'];

% fnames = {file_Wilson file_basicRiddick file_Riddick_NormStep file_Riddick_PredictionCorrection ...
%     file_Riddick_GyroGain file_Riddick_2ndOrderIntegration};

fnames = {file_Wilson file_basicRiddick file_Riddick_NormStep file_Riddick_PredictionCorrection ...
    file_Riddick_GyroGain};

% titles = {'GDF (Wilson)' '2 gain' 'Unnormalized Step' 'Prediction Correction' 'Gyro gain' '2nd Order Integration'};
titles = {'Baseline (Wilson, 2019)' '2 gain' 'Unnormalized Gradients' 'Prediction Correction' 'Gyro gain'};

saveFigs = false;
figDir = [files.figDir 'gdfExtensions' filesep];
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

%% Load data

T = cell(1,length(titles));
for i = 1 : length(fnames)
    loaded = load(fnames{i});
    [T{i},estTime] = convertDataStructToTable(loaded.O);
    T{i} = removeOutliers(T{i},outlierTable());

    if i == 1
        wDex = strcmp(T{i}.Algorithm,'WilsonAHRS');
        T{i} = T{i}(wDex,:);

        FS = unique(T{i}.SamplingFrequency);
    end

    NT(i) = height(T{i});
    T{i}.Title = repmat(titles(i),NT(i),1);

    T{i}.SamplePeriod = 1./T{i}.SamplingFrequency;

    if i == 1
        EVALTIMES = estTime;
    end
end

% TT = vertcat(T{:});

%% Plot the error as a function of Sample Frequency

% useLogLogScale = false;
useLogLogScale = true;

% Choose a model type
modelType = 'rat11';
% modelType = 'exponential';
% modelType = 'hyperbolic';


errorModel = 'exponential';
%   errorModel = 'Constant';
approxType = 'LME';

% algos = opts.algorithms;

hf = figure;
hf.Color = [1 1 1];
hf.Units = 'centimeters';
hf.Position = [2.4 10 singleColumnWidth singleColumnWidth];
% hf.WindowState = 'maximized';

hf.Color = [1 1 1];
% cols = turbo(length(T));

for i = 1 : length(titles)
    Tsub = T{i};

    group = [categorical(Tsub.Subject),categorical(Tsub.Condition),categorical(Tsub.Sensor)];
    group = group(:,2);



    switch modelType
        case 'exponential'
            model = @(phi,fs) phi(1).*exp(phi(2).*fs) + phi(3);
            beta0 = [57.93 -0.3686 4.549];
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
    for j = 1 : length(FS)
        meanCosts(j) = mean(Tsub.Cost(Tsub.SamplingFrequency==FS(j)));
    end
    
    meanCosts = meanCosts / 3;
    mdl = fitnlm(FS,meanCosts,model,beta0);


    if i == 1
        col = [0 0 0];
        lineStyle = '--';
    elseif i < length(T)
    col = blueGrad(i-1,length(T)-2);
    lineStyle = '-';
    else
    col = [0 0 0];
    lineStyle = '-';
    end
    if useLogLogScale
%         semilogx(FS,meanCosts,'.','Color',col,'HandleVisibility','off','MarkerSize',10)
%         hold on
%        semilogx(FS,predict(mdl,FS),'Color',col,'LineStyle',lineStyle)

        loglog(FS,meanCosts,'.','Color',col,'HandleVisibility','off','MarkerSize',10)
        hold on
        loglog(FS,predict(mdl,FS),'Color',col,'LineStyle',lineStyle)
        set(gca,'YTickLabel',[1 10]);
    else
        plot(FS,meanCosts,'.','Color',col,'HandleVisibility','off','MarkerSize',10)
        hold on
        plot(FS,predict(mdl,FS),'Color',col,'LineStyle',lineStyle)
        
    end
    xlabel('Sample Frequency (Hz)')
    ylabel('Error (°)')
%     [beta,psi,stats,B] = nlmefit(Tsub.SamplingFrequency,Tsub.Cost,group,[],model,beta0,'ErrorModel',errorModel,'ApproximationType',approxType);


set(gca,'XTickLabel',[1 10 100])
set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');


% semilogy(FS,meanCosts,'.','Color',cols(i,:),'HandleVisibility','off')
% hold on
% semilogy(FS,model(beta,FS),'Color',cols(i,:))


    X = FS;
    Y = meanCosts;

end
hl=legend(titles{:});
hl.Box='Off';


set(gca,'XLim',[0 190])


    if saveFigs
        for i = 1 : length(saveFormats)
            figName = [figDir 'IterativeImprovements_GDF_VsSampleFrequency'];% saveFormats{i}];
            saveas(gcf,figName,saveFormats{i});
        end
    end

%% Bar Plots: High and low frequency

    meanCosts = [];
    stdCosts = [];
    for i = 1 : length(titles)
    for j = 1 : length(FS)
        cc = T{i}.Cost(T{i}.SamplingFrequency==FS(j));
        meanCosts(j,i) = mean(cc);
        stdCosts(j,i) = std(cc);
    end
    end

x = 1 : length(titles);
hf = figure;
hf.WindowState = 'maximized';
hf.Color = [1 1 1];
subplot(211)
bar(x,meanCosts(end,:))
hold on
errorbar(x,meanCosts(end,:),stdCosts(end,:),'.')
set(gca,'XTickLabel',titles)
title('190 Hz')

subplot(212)
fsdex = 1;
bar(x,meanCosts(fsdex,:))
hold on
errorbar(x,meanCosts(fsdex,:),stdCosts(fsdex,:),'.')
set(gca,'XTickLabel',titles)
title([num2str(FS(fsdex)) ' Hz'])

    if saveFigs
        for i = 1 : length(saveFormats)
            figName = [figDir 'IterativeImprovements_GDF_BarPlots_190Hz_1Hz'];% saveFormats{i}];
            saveas(gcf,figName,saveFormats{i});
        end
    end

%% Evaluation Time
% ealgos = repmat(algos,10,1);
% sf = repmat(flipud(FS),6,1);

figure;
bar(1:length(algos),EVALTIMES(1:6))
set(gca,'XTickLabel',algos)
ylabel('Time (sec)')
title('Evaluation Time for entire dataset')

    if saveFigs
        for i = 1 : length(saveFormats)
            % Save in the optimized folder
            fdir = [files.figDir 'optimized' filesep];
            figName = [fdir 'evaluationTime'];% saveFormats{i}];
            saveas(gcf,figName,saveFormats{i});
        end
    end

% %% 
% dataSetFile = [files.dataDir 'validation2021_dataset.mat'];
% load(dataSetFile);
% %% Orientation Estimation Evaluation time
% data = D.Subject2.walk_3kmh.mdm64B1.Data;
% q0 = data.ground_truth(1,:);
% 
% 
% clear Times
% for i = 1 : length(titles)
%     loaded = load(fnames{i});
%     opts = loaded.O.opts{1};
%     optimOpts = loaded.O.optimOpts{1};
% 
%     if i == 1
%         oclass = loaded.O.algorithm{2}{1};
%         gains = loaded.O.parameters{2};
%         dt = 1/loaded.O.samplingFrequency{2}(1);
%         normStepSize = true;
%         indexOffset = 0;
% 
% %         oclass = 'JustaAHRS';
% %         gains = [0.005 0.005];
%     else
%         oclass = loaded.O.algorithm{1}{1};
%         gains = loaded.O.parameters{1};
%         dt = 1/loaded.O.samplingFrequency{1}(1);
%         normStepSize = optimOpts.RiddickAHRS.normStepSize;
%         indexOffset = 0;
%         intOrder = opts.integrationOrder;
% 
%         if length(gains) < 3
%             gains = [1 gains];
%         end
%     end
% 
%             Times(i) = timeit(@() estimateOrientation_Fast2_mex(oclass,q0,data.gyro,data.acc,data.mag,gains,dt,normStepSize,[],intOrder));
% end

%% Plot the error as a function of sample period


% 
% % Choose a model type
% modelType = 'linear';
% % modelType = 'log';
% % modelType = 'exponential';
% % modelType = 'hyperbolic';
% errorModel = 'exponential';
% %   errorModel = 'Constant';
% approxType = 'LME';
% 
% SP = 1./FS;
% 
% modelTypes = {'linear' 'linear' 'linear' 'log' 'log' 'log'};
% 
% 
% figure
% cols = cool(length(titles));
% for i = 1 : length(titles)
%     Tsub = T{i};
%     group = [categorical(Tsub.Subject),categorical(Tsub.Condition),categorical(Tsub.Sensor)];
%     group = group(:,2);
% 
% 
% 
%     switch modelTypes{i}
%         case 'exponential'
%             model = @(phi,fs) phi(1).*exp(phi(2).*fs) + phi(3);
%             beta0 = [2000 0.03 -2000];
%         case 'hyperbolic'
%             model = @(phi,fs) phi(1)./(fs) + phi(2);
%             beta0 = [50 4];
%         case 'linear'
%             model = @(phi,fs) phi(1)*fs + phi(2);
%             beta0 = [50 0];
%         case 'log'
%             model = @(phi,fs) phi(1)*log(fs + phi(2)) + phi(3);
%             beta0 = [50 0.1 50];
%     end
%     
%     [beta,psi,stats,B] = nlmefit(Tsub.SamplePeriod,Tsub.Cost,group,[],model,beta0,'ErrorModel',errorModel,'ApproximationType',approxType);
% 
% 
% 
% % Check normality ore residual distribution(heteroscedasticity)
% % Exponential model for error terms make much more gaussian
% meanCosts = [];
% for j = 1 : length(SP)
%     meanCosts(j) = mean(Tsub.Cost(Tsub.SamplePeriod==SP(j)));
% end
% 
% 
% 
% 
% plot(SP,meanCosts,'.','Color',cols(i,:),'HandleVisibility','off')
% hold on
% plot(SP,model(beta,SP),'Color',cols(i,:))
% 
% % loglog(SP,meanCosts,'.','Color',cols(i,:),'HandleVisibility','off')
% % hold on
% % loglog(SP,model(beta,SP),'Color',cols(i,:))
% 
% % semilogy(SP,meanCosts,'.','Color',cols(i,:),'HandleVisibility','off')
% % hold on
% % semilogy(SP,model(beta,SP),'Color',cols(i,:))
% 
%     X = SP;
%     Y = meanCosts;
% 
% end
% legend(titles{:})
