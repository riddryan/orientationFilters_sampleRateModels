opts = defaultDataOpts;
opts.subjects = [1:4 6:12];
opts.conditions = {'standing_ref1' 'standing_bending' 'right_left_lift' ...
    'sitting_bend' 'sit_stand_normal' 'sit_stand_fast' 'walk_3kmh' ...
    'walk_5kmh' 'og_walk_sss' 'figure8_walking' 'og_jog_sss' };

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Make sure the data directory folder is set properly to the location on
% your computer within setFileNames.m
files = setFileNames(opts);

saveFigs = false;
saveFormats = {'fig' 'png' 'epsc'};
figDir = [files.figDir 'individualAngles' filesep];

% Graphics options
fontSize = 8;
fontName = 'Arial';
singleColumnWidth = 9;
doubleColumnWidth = 19;
onepointfiveColumnWidth = 14;

lightBlue=[0.3 0.8 0.95];
darkBlue = [0.03 0.07 0.6];
blueGrad = @(i,N) lightBlue + (darkBlue-lightBlue)*((i-1)/(N-1));
%% General options
% SET TO TRUE IF YOU'VE CHANGED OPTIONS FOR THE OPTIMIZATION AND WANT TO
% RERUN THE OPTIMIZATION WITHOUT HAVING TO DELETE THE OLD FILES.
forceReoptimize = true;

% Log space for sampling frequencies so we are not biased towards a
% frequency range when fitting an exponential model to error or parameters
% versus sampling frequcnies
opts.SamplingFrequencies = fliplr(logspace(0,log10(190),10));


dataSetFile = [files.dataDir 'validation2021_dataset.mat'];

%% General Optimization options
optimOpts = defaultOptimOpts;

optimOpts.forceReoptimize = forceReoptimize;


optimOpts.alwaysUseLocalOptimFor1D = false;
optimOpts.useCostContourFor1D = false;

% Option for Riddick filter to separate prediction & correction steps for
% the filter
opts.useCorrectionPrediction = true;

% Option for Riddick filter to specify higher order integration model
opts.integrationOrder = 1;


% For all GDFs (besides Riddick) normalize the gradient descent step (as is
% done in the references). Doesn't seem to actually affect overall optimal
% results so may be an unecessary computation
opts.GradDescent.normStepSize = true;
optimOpts.RiddickAHRS.normStepSize = false;


% Used for all filters besides FKF
optimOpts.MultiStart.Number = 4*2;
optimOpts.MultiStart.MaxIterations = 40;
optimOpts.MultiStart.UseParallel = true;

% Used for fkf
optimOpts.FastKF.optimizer = 'surrogate';
optimOpts.surrogate.Iterations = 100;
optimOpts.surrogate.UseParallel = true;
optimOpts.surrogate.MinSurrogatePoints = 80;
optimOpts.surrogate.MinSampleDistance = 1e-6;

% Use magnetometer in the orientation estimates
opts.useMagnetometer = true;

% All sensors are used
opts.sensorsToUse = {'mdm650D' 'mdm64B1' 'mdm64C6' 'mdm64BF' 'mdm652E' 'mdm64A8' 'mdm64E1'};

% Which algorithms to use
opts.AHRS = [RiddickAHRS];
opts.algorithms = [];
for i = 1 : length(opts.AHRS)
    opts.algorithms{i} = class(opts.AHRS(i));
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
optimOpts.startCount = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Load data
reExtractData = false;
if reExtractData || ~exist(dataSetFile,'file')
    D = extractData(opts,files);
    save(dataSetFile,'D');
else
    load(dataSetFile);
end
%%

colorHeading = [55 126 184]/255;
colorFlexion = [228 26 28]/255;
colorLatFlexion = [77 175 74]/255;

COLS  = vertcat(colorHeading,colorFlexion,colorLatFlexion);

for SU = opts.subjects
    pNames = optimOpts.RiddickAHRS.parameters.Names;
    p = [1 0.15 0.05];

    FS = opts.SamplingFrequencies(1:7);
    % FS = opts.SamplingFrequencies;

    CONDIS = opts.conditions;
    suString = ['Subject' num2str(SU)];

    clear RMSE
    hf = figure;
    hf.Color = [1 1 1];
    hf.Units = 'centimeters';
    hf.Position = [2 2 doubleColumnWidth doubleColumnWidth];
    for cc = 1 : length(CONDIS)
        sensNames = fieldnames(D.(suString).(CONDIS{cc}));


        for i = 1 : length(FS)
            data = D.(suString).(CONDIS{cc}).(sensNames{1}).ResampleData(i);
            [qEst] = estimateAttitude2(opts.AHRS,p,pNames,FS(i),data,opts);
            [euler_error,rmse,sse] = calcFilterError_mex(qEst,data.ground_truth,isnan(data.ground_truth(:,1)));

            RMSE{cc}(i,:) = rmse;
        end
        subplot(4,3,cc)

        for dims = 1:3
        plot(FS,RMSE{cc}(:,dims),'.-','Color',COLS(dims,:));
            hold on
        end
        % semilogy(FS,RMSE{cc});
        % loglog(FS,RMSE{cc});
        xlabel('Freq (Hz)')
        ylabel('Error (°)')

                    set(gca,'Box','Off')
            set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');

        if cc == 1
            title([suString ' ' CONDIS{cc}],'interpreter','none')
%             legend('Heading','Flexion','Lateral Flexion')
        else
            title(CONDIS{cc},'interpreter','none')
        end
    end

    if saveFigs
        for i = 1 : length(saveFormats)
            figName = [figDir 'ErrorVsFreq_individualAngles ' suString];% saveFormats{i}];
            saveas(gcf,figName,saveFormats{i});
        end
    end


end