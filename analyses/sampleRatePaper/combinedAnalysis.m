opts = defaultDataOpts;
opts.subjects = [1:4 6:12];
opts.conditions = {'standing_ref1' 'standing_bending' 'right_left_lift' ...
    'sitting_bend' 'sit_stand_normal' 'sit_stand_fast' 'walk_3kmh' ...
    'walk_5kmh' 'og_walk_sss' 'figure8_walking' 'og_jog_sss' };
optimOpts = defaultOptimOpts;

files = setFileNames(opts);
saveFigs = false;
figDir = [files.figDir 'optimizedByCondition' filesep];
tableDir = [files.figDir 'tables' filesep];
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
%% Load results: Overall
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
%% Load results: Optimized per condition
% This file optimizes each filter's parameters across all participants and
% conditions for each sample frequency


% Optimized across all participants & conditions for each sample frequency
file_SF_Condition = [files.optimizedParametersDir 'samplingFrequencyOptimized_perCondition.mat'];


loaded = load(file_SF_Condition);
T2 = convertDataStructToTable(loaded.O);
T2 = removeOutliers(T2,outlierTable());

%% Parameters vs sample freq: By condition
TP = T2;


fitFreqDex = 4:length(FS);
% fitFreqDex = 1 : length(FS);
fitFreqs = FS(fitFreqDex);

modelType = 'exponential';
% modelType = 'gamma';
% modelType = 'hyperbolic';
% modelType = 'rat12';

condis = opts.conditions.';
% modelTypes = { {'rat11' 'rat22' 'rat22'} {'rat12'} {'rat12'} {'rat12'} {'rat11' 'rat11' 'rat11'} {'rat22' 'rat22'}};

% modelTypes = { {'exp' 'exp' 'exp'} {'exp'} {'exp'} {'exp'} {'rat22' 'rat11' 'rat12'} {'exp' 'exp'}};
cols = jet(length(condis));

for algoDex = 1
    hf = figure;
hf.Color = [1 1 1];
hf.Units = 'centimeters';
hf.Position = [2 2 singleColumnWidth singleColumnWidth];
    parmNames = optimOpts.(algos{algoDex}).parameters.Names;
    for cc = 1 : length(condis)
        meanParms= [];
        for pp = 1:length(parmNames)

%             modelType = modelTypes{algoDex}{pp};
%             modelType = 'exponential';
            parmDex = pp;




            Tsub = TP(strcmp(TP.Algorithm,algos{algoDex}) & strcmp(TP.Condition,condis{cc}),:);

            % Fix bug saving parameter for gyro gain for Riddick filter
            % standing condition
            if algoDex == 1 && cc == 1
                for hh = 1 : height(Tsub)
                    if length(Tsub.Parameters{hh}) < 3
                        Tsub.Parameters{hh} = [0.9 Tsub.Parameters{hh}];
                    end
                end
            end

            PARMS = cell2mat(Tsub.Parameters);

            switch modelType
                case {'exponential' 'exp'}
%                     model = @(phi,fs) phi(1).*exp(phi(2).*fs) + phi(3);
                    model = @expmodel;
                    

                    if algoDex == 1 && pp == 1
                        beta0 = [-1 -0.2 1];
                    elseif algoDex ==1
                        beta0 = [0.4 -0.05 0.07];
                    else
                        beta0 = [0.3 -0.15 0.01];
                    end
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
                case 'rat11'
                    model = @rat11model;
                    beta0 = [2 86 2];
                case 'rat12'
                    model = @rat12model;
                    beta0 = [1e5 4e6 4e4 8e4];
            end

            fitDex = false(size(Tsub.SamplingFrequency));
            for i = 1 : length(fitFreqs)
                fitDex(Tsub.SamplingFrequency == fitFreqs(i)) = true;
            end


            
            for j = 1 : length(FS)
                meanParms(j,parmDex) = mean(PARMS(Tsub.SamplingFrequency==FS(j),parmDex));
            end

            if algoDex == 1

                            fopts = statset;
            fopts.MaxIter = 400;
            fopts.RobustWgtFun = 'cauchy';

%             mdl = fitnlm(fitFreqs,meanParms(fitFreqDex,parmDex),model,beta0);
            mdl = fitnlm(FS(fitFreqDex),meanParms(fitFreqDex,parmDex),model,beta0,'Options',fopts);


            riddickMods{cc,pp} = mdl;

    
            riddickParms{cc} = meanParms;

            R2C(cc,pp) = mdl.Rsquared.Adjusted;
            end


            subplot(length(parmNames),1,pp)
            plot(FS(fitFreqDex),meanParms(fitFreqDex,pp),'--','Color',cols(cc,:),'Marker','.')
%     semilogx(FS(fitFreqDex),meanParms(fitFreqDex,pp),':','Color',cols(cc,:),'Marker','.')
%             plot(FS(fitFreqDex),meanParms(fitFreqDex,pp),'.','Color',cols(cc,:))
%             plot(FS,meanParms(:,pp),'.')
            hold on

%             fsi = linspace(fitFreqs(1),fitFreqs(end),100).';
%             fsi = linspace(1,200,100).';
%             plot(fsi,predict(mdl,fsi),'Color',cols(cc,:))

            % Plot data points not used in the fit in black
%             L = true(size(FS));
%             L(fitFreqDex) = false;
%             plot(FS(L),meanParms(L,pp),'x','Color',[0.2 0.2 0.2])

            title([parmNames{pp}],'Interpreter','none')
            ylabel('No Units')

            if pp == length(parmNames)
                xlabel('Sampling Frequency (Hz)')
            end

            set(gca,'Box','Off')
            set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');
        end

        
    end

end

%% Parameters vs sample freq: Overall
TP = T;


fitFreqDex = 4:length(FS);
% fitFreqDex = 1 : length(FS);
fitFreqs = FS(fitFreqDex);

modelType = 'exponential';
% modelType = 'gamma';
% modelType = 'hyperbolic';
% modelType = 'rat12';


% modelTypes = { {'rat11' 'rat22' 'rat22'} {'rat12'} {'rat12'} {'rat12'} {'rat11' 'rat11' 'rat11'} {'rat22' 'rat22'}};

% modelTypes = { {'exp' 'exp' 'exp'} {'exp'} {'exp'} {'exp'} {'rat22' 'rat11' 'rat12'} {'exp' 'exp'}};
cols = jet(length(condis));

for algoDex = 1

    parmNames = optimOpts.(algos{algoDex}).parameters.Names;

        meanParms= [];
        for pp = 1:length(parmNames)
            modelType = 'exponential';
            parmDex = pp;
            Tsub = TP(strcmp(TP.Algorithm,algos{algoDex}),:);
            PARMS = cell2mat(Tsub.Parameters);

            switch modelType
                case {'exponential' 'exp'}
%                     model = @(phi,fs) phi(1).*exp(phi(2).*fs) + phi(3);
                    model = @expmodel;
                    if algoDex == 1 && pp == 1
                        beta0 = [-0.2 -0.23 1];
                    else
                    beta0 = [0.3 -0.15 0.01];
                    end
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
                case 'rat11'
                    model = @rat11model;
                    beta0 = [2 86 2];
                case 'rat12'
                    model = @rat12model;
                    beta0 = [1e5 4e6 4e4 8e4];
            end

            fitDex = false(size(Tsub.SamplingFrequency));
            for i = 1 : length(fitFreqs)
                fitDex(Tsub.SamplingFrequency == fitFreqs(i)) = true;
            end


            
            for j = 1 : length(FS)
                meanParms(j,parmDex) = mean(PARMS(Tsub.SamplingFrequency==FS(j),parmDex));
            end

            if algoDex == 1
%             mdl = fitnlm(fitFreqs,meanParms(fitFreqDex,parmDex),model,beta0);
            mdl = fitnlm(FS(fitFreqDex),meanParms(fitFreqDex,parmDex),model,beta0);


            riddickModsOverall{1,pp} = mdl;

    
            riddickParmsOverall = meanParms;
            end


            subplot(length(parmNames),1,pp)
%             plot(FS(fitFreqDex),meanParms(fitFreqDex,pp),'-','Color',[0 0 0],'LineWidth',1.5)
%             plot(FS(fitFreqDex),meanParms(fitFreqDex,pp),'.','Color',cols(cc,:))
%             plot(FS,meanParms(:,pp),'.')
%             hold on

%             fsi = linspace(fitFreqs(1),fitFreqs(end),100).';
            fsi = linspace(1,200,100).';
            predVal = predict(mdl,fsi);
            plot(fsi,predVal,'Color',[0 0 0],'LineWidth',1.5)
    
            if algoDex == 1
                MO(:,pp) = predVal;
            end


            set(gca,'YLim',[0 1.01])

            % Plot data points not used in the fit in black
%             L = true(size(FS));
%             L(fitFreqDex) = false;
%             plot(FS(L),meanParms(L,pp),'o','Color',[0.3 0.3 0.3])

            set(gca,'Box','Off')
            
            if pp < length(parmNames)
                set(gca,'XTickLabel',[]);
            end
        end

        

end

% subplot(311)
% % hL=legend([condis; {'Overall Model'}],'Interpreter','none');
% hL.Box='Off';

% figure;
% for pp = 1 : 3
%     plot(fsi,MO);
%     hold on
% end
% plot(fsi,sum(MO,2),'k');
% legend('gyro','acc','mag','sum')


    if saveFigs
        for i = 1 : length(saveFormats)
            figName = [figDir 'Riddick_ParametersVsCondition_VsSamplingFreq'];% saveFormats{i}];
            saveas(gcf,figName,saveFormats{i});
        end
    end

    %% Parameter model coefficients table

    clear coeffsOverall rSquaredOverall coeffsCondis rSquaredCondis
    for pp = 1 : 3
        coeffsOverall(pp,:) = riddickModsOverall{pp}.Coefficients.Estimate;
        rSquaredOverall(pp,:) = riddickModsOverall{pp}.Rsquared.Adjusted;
    end

    X = [coeffsOverall rSquaredOverall];
    X(:,2) = -X(:,2);
    X = [compose('%.03f',X(:,1:2)) compose('%.04f',X(:,3)) compose('%.02f',X(:,4))];
    X = [X(:,3) X(:,1:2) X(:,4)];
    parameterTable = array2table(X,"VariableNames",{'BetaInf' 'c' 'lambda'  'adj-rsquared'},"RowNames",{'gyro' 'acc' 'mag'});

    writetable(parameterTable,[tableDir 'OverallParameters_Table.xlsx'],'WriteRowNames',true)  

    for cc = 1 : length(condis)
        dex = 3*(cc-1) + (1:3);
        for pp = 1 : 3
             
        coeffsCondis(dex(pp),:) = riddickMods{cc,pp}.Coefficients.Estimate;
        rSquaredCondis(dex(pp),:) = riddickMods{cc,pp}.Rsquared.Adjusted;
        end

        X = [coeffsCondis(dex,:) rSquaredCondis(dex,:)];
        X(:,2) = -X(:,2); %consistent with paper
        X = [compose('%.03f',X(:,1:2)) compose('%.04f',X(:,3)) compose('%.02f',X(:,4))];
        X = [X(:,3) X(:,1:2) X(:,4)];
        PT = array2table(X,"VariableNames",{'BetaInf' 'c' 'lambda'  'adj-rsquared'},"RowNames",{'gyro' 'acc' 'mag'});
        writetable(PT,[tableDir 'Parameters_Table_AllConditions.xlsx'],'WriteRowNames',true,'Sheet',condis{cc})  
    end

    psnames = {'gyro' 'acc' 'mag'};
    for pp = 1:3

        clear COEFF R2
        for cc = 1 : length(condis)
            COEFF(cc,:) = riddickMods{cc,pp}.Coefficients.Estimate;
            R2(cc,:) = riddickMods{cc,pp}.Rsquared.Adjusted;
        end

        COEFF(:,2) = -COEFF(:,2); %consistent with paper
        X = [COEFF R2];
        X = [compose('%.03f',X(:,1:2)) compose('%.04f',X(:,3)) compose('%.02f',X(:,4))];
        X = [X(:,3) X(:,1:2) X(:,4)];
        PT = array2table(X,"VariableNames",{'BetaInf' 'c' 'lambda'  'adj-rsquared'},"RowNames",condis);
        writetable(PT,[tableDir 'Parameters_EachCondition_' psnames{pp} '.xlsx'],'WriteRowNames',true)
    end
   


        %% Error model coefficients table
