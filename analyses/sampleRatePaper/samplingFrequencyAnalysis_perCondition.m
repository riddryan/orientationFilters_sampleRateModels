opts = defaultDataOpts;
opts.subjects = [1:4 6:12];
opts.conditions = {'standing_ref1' 'standing_bending' 'right_left_lift' ...
    'sitting_bend' 'sit_stand_normal' 'sit_stand_fast' 'walk_3kmh' ...
    'walk_5kmh' 'og_walk_sss' 'figure8_walking' 'og_jog_sss' };
optimOpts = defaultOptimOpts;

files = setFileNames(opts);
saveFigs = true;
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


eulerRotType = 'frame';

%% Load results
% This file optimizes each filter's parameters across all participants and
% conditions for each sample frequency


% Optimized across all participants & conditions for each sample frequency
file_SF_Condition = [files.optimizedParametersDir 'samplingFrequencyOptimized_perCondition.mat'];


loaded = load(file_SF_Condition);
T = convertDataStructToTable(loaded.O);
T = removeOutliers(T,outlierTable());

algos = unique(T.Algorithm,'stable');
FS = unique(T.SamplingFrequency);
condis = unique(T.Condition,'stable');

% Load dataset as well
dataSetFile = [files.dataDir 'validation2021_dataset.mat'];
load(dataSetFile);

%% Show full examples for each condition



AHRS = RiddickAHRS;
opts.GradDescent.normStepSize = false;

optimOpts = defaultOptimOpts;
pNames = optimOpts.RiddickAHRS.parameters.Names;

% testFSdex = [3 4 5 6 7 10];

Subj = 3;

% CONDIS = {'right_left_lift' 'walk_3kmh' 'og_jog_sss'};

% CONDIS = {'sit_stand_normal' 'walk_3kmh' 'og_jog_sss'};
% CONDIS = {'sit_stand_normal' 'walk_3kmh' 'figure8_walking'};

CONDIS= opts.conditions;


hf = figure;
% hf.WindowState = 'maximized';
hf.Color = [1 1 1];
% cols = jet(length(algos));


% ANGLES
clear q costs ERR
for cc = 1 : length(CONDIS)
COND = CONDIS{cc};

testSubj = ['Subject' num2str(Subj)];
sensorNames = D.(testSubj).sensorNames;
testSensorName = sensorNames{1};
DD = D.(testSubj).(COND).(testSensorName);

testData = DD.ResampleData;

dataDex = 1;
testSamplingFrequencies = FS(end); % 10.2 Hz, 33.05 Hz, 190 Hz

dirLabs = {'Heading (°)' 'Flexion (°)' 'Lateral Flexion (°)'};



% Set optimal parameters
dex = T.SamplingFrequency == testSamplingFrequencies & strcmp(T.Algorithm,'RiddickAHRS') & strcmp(T.Condition,COND);
Tsub = T(dex,:);
PSTAR = mean(vertcat(Tsub.Parameters{:}));

[q{cc},costs(cc),~,EE{cc}] = estimateAttitude2(AHRS,PSTAR,pNames,testSamplingFrequencies,testData(dataDex),opts);


eaEst{cc} = eulerd(quaternion(q{cc}),'ZYX',eulerRotType);




badMocapDex = DD.badMocapFramesOriginal;
eaTruth = eulerd(quaternion(DD.Data.ground_truth),'ZYX',eulerRotType);

% ERR{cc} = eaTruth(1:length(eaEst{cc}),:) - eaEst{cc};


% tRange = mean(DD.Data.time) + [0 3] - 2;
tRange = [DD.Data.time(1) DD.Data.time(end)];

    subplot(3,4,cc);

    [~,IS] = min(abs(DD.Data.time - tRange(1)));
    [~,IE] = min(abs(DD.Data.time - tRange(2)));

    TIME = testData(dataDex).time;

    plotDex = false(length(TIME),1);
    plotDex(IS:IE) = true;
    plotDex(badMocapDex) = false;



    
        [~,IS] = min(abs(TIME - tRange(1)));
        [~,IE] = min(abs(TIME - tRange(2)));

     pp=plot(DD.Data.time(plotDex),eaTruth(plotDex,:),'k.','LineWidth',1,'MarkerSize',3);
     pp(2).Annotation.LegendInformation.IconDisplayStyle = 'off';
     pp(3).Annotation.LegendInformation.IconDisplayStyle = 'off';
     hold on

     plot(TIME(IS:IE),eaEst{cc}(IS:IE,:));
     title(CONDIS{cc},'Interpreter','none')
     if cc == 1
         legend('Truth','Z (up)','Y (ML)','X (fore-aft)')
     end
     xlabel('Time (sec)')
     ylabel('Angle (°)')

end

% ERROR
figure
for cc = 1 : length(CONDIS)
COND = CONDIS{cc};

testSubj = ['Subject' num2str(Subj)];
sensorNames = D.(testSubj).sensorNames;
testSensorName = sensorNames{1};
DD = D.(testSubj).(COND).(testSensorName);

testData = DD.ResampleData;

dataDex = 1;
testSamplingFrequencies = FS(end); % 10.2 Hz, 33.05 Hz, 190 Hz

dirLabs = {'Heading (°)' 'Flexion (°)' 'Lateral Flexion (°)'};



% Set optimal parameters
dex = T.SamplingFrequency == testSamplingFrequencies & strcmp(T.Algorithm,'RiddickAHRS') & strcmp(T.Condition,COND);
Tsub = T(dex,:);
PSTAR = mean(vertcat(Tsub.Parameters{:}));


% tRange = mean(DD.Data.time) + [0 3] - 2;
tRange = [DD.Data.time(1) DD.Data.time(end)];

    subplot(3,4,cc);

    [~,IS] = min(abs(DD.Data.time - tRange(1)));
    [~,IE] = min(abs(DD.Data.time - tRange(2)));

    TIME = testData(dataDex).time;



    
        [~,IS] = min(abs(TIME - tRange(1)));
        [~,IE] = min(abs(TIME - tRange(2)));


     plot(TIME(IS:IE),EE{cc}(IS:IE,:));
     title(CONDIS{cc},'Interpreter','none')
     if cc == 1
         legend('Z (up)','Y (ML)','X (fore-aft)')
     end
     xlabel('Time (sec)')
     ylabel('Error (°)')

end

%% Compare filters for each condition

optimOpts = defaultOptimOpts;
% filtNames = {'RiddickAHRS' 'WilsonAHRS' 'AdmiraalAHRS' 'MadgwickAHRS3' 'FastKF' 'JustaAHRS'};
filtNames = {'RiddickAHRS' 'WilsonAHRS' 'FastKF' 'JustaAHRS'};

AHRS1 = RiddickAHRS;
opts1=opts;
opts1.GradDescent.normStepSize = false;
pNames1 = optimOpts.RiddickAHRS.parameters.Names;

AHRS2 = WilsonAHRS;
opts2 = opts;
pNames2 = optimOpts.WilsonAHRS.parameters.Names;

% AHRS3 = AdmiraalAHRS;
% opts3 = opts;
% pNames3 = optimOpts.AdmiraalAHRS.parameters.Names;

AHRS3 = FastKF;
opts3 = opts;
pNames3 = optimOpts.FastKF.parameters.Names;

AHRS4 = JustaAHRS;
opts4 = opts;
pNames4 = optimOpts.JustaAHRS.parameters.Names;


allFilts = {AHRS1 AHRS2 AHRS3 AHRS4};
PNAMES = {pNames1 pNames2 pNames3 pNames4};
OPTS = {opts1 opts2 opts3 opts4};

% testFSdex = [3 4 5 6 7 10];

Subj = 2;

% CONDIS = {'right_left_lift' 'walk_3kmh' 'og_jog_sss'};

% CONDIS = {'sit_stand_normal' 'walk_3kmh' 'og_jog_sss'};
% CONDIS = {'sit_stand_normal' 'walk_3kmh' 'figure8_walking'};

CONDIS = opts1.conditions;



% cols = jet(length(algos));


% ANGLES

for cc = 1 : length(CONDIS)
    hf = figure;
    % hf.WindowState = 'maximized';
    hf.Color = [1 1 1];
    for ff = 1 : length(allFilts)
        clear q costs ERR
        COND = CONDIS{cc};

        testSubj = ['Subject' num2str(Subj)];
        sensorNames = D.(testSubj).sensorNames;
        testSensorName = sensorNames{1};
        DD = D.(testSubj).(COND).(testSensorName);

        testData = DD.ResampleData;

        dataDex = 1;
        testSamplingFrequencies = FS(end); % 10.2 Hz, 33.05 Hz, 190 Hz

        dirLabs = {'Heading (°)' 'Flexion (°)' 'Lateral Flexion (°)'};



        % Set optimal parameters
        dex = T.SamplingFrequency == testSamplingFrequencies & strcmp(T.Algorithm,filtNames{ff}) & strcmp(T.Condition,COND);
        Tsub = T(dex,:);
        PSTAR = mean(vertcat(Tsub.Parameters{:}));

        [q{cc},costs(cc),~,EE{cc}] = estimateAttitude2(allFilts{ff},PSTAR,PNAMES{ff},testSamplingFrequencies,testData(dataDex),OPTS{ff});


        eaEst{cc} = eulerd(quaternion(q{cc}),'ZYX',eulerRotType);




        badMocapDex = DD.badMocapFramesOriginal;
        eaTruth = eulerd(quaternion(DD.Data.ground_truth),'ZYX',eulerRotType);

        % ERR{cc} = eaTruth(1:length(eaEst{cc}),:) - eaEst{cc};


        % tRange = mean(DD.Data.time) + [0 3] - 2;
        tRange = [DD.Data.time(1) DD.Data.time(end)];

        for dim = 1 : 3
        subplot(3,1,dim);

        [~,IS] = min(abs(DD.Data.time - tRange(1)));
        [~,IE] = min(abs(DD.Data.time - tRange(2)));

        TIME = testData(dataDex).time;




        [~,IS] = min(abs(TIME - tRange(1)));
        [~,IE] = min(abs(TIME - tRange(2)));


        plot(TIME(IS:IE),EE{cc}(IS:IE,dim));
        hold on
        title(CONDIS{cc},'Interpreter','none')
        if dim == 1
            legend(filtNames{:})
            ylabel('Heading Error (°)')
        elseif dim == 2
            ylabel('Flexion Error (°)')
        elseif dim == 3
            ylabel('Lat Flexion Error (°)')
        end
        xlabel('Time (sec)')
        
        title(CONDIS{cc});


        end
    end
end


%% Show examples of degradation of performance w.r.t. Sampling Frequency



AHRS = RiddickAHRS;
opts.GradDescent.normStepSize = false;

optimOpts = defaultOptimOpts;
pNames = optimOpts.RiddickAHRS.parameters.Names;

condFS = {[3 6 10] [3 6 7 10] [6 7 8 10]};
% testFSdex = [3 4 5 6 7 10];

Subj = 2;

% CONDIS = {'right_left_lift' 'walk_3kmh' 'og_jog_sss'};

CONDIS = {'sit_stand_normal' 'walk_3kmh' 'og_jog_sss'};
% CONDIS = {'sit_stand_normal' 'walk_3kmh' 'figure8_walking'};


hf = figure;
% hf.WindowState = 'maximized';
hf.Color = [1 1 1];
hf.Units = 'centimeters';
hf.Position = [2.4 10 singleColumnWidth singleColumnWidth];
% cols = jet(length(algos));

for cc = 1 : length(CONDIS)
COND = CONDIS{cc};

testFSdex = condFS{cc};
% cols = jet(length(testFSdex));

testSubj = ['Subject' num2str(Subj)];

sensorNames = D.(testSubj).sensorNames;
testSensorName = sensorNames{1};
% DD = D.Subject1.walk_3kmh.mdm650D;
% DD = D.Subject2.(COND).mdm64B1;
% DD = D.Subject3.(COND).mdm64BF;

DD = D.(testSubj).(COND).(testSensorName);

% COND = 'og_jog_sss';
testData = DD.ResampleData;
% testFSdex = [5 7 10];

dataDex = 10 - testFSdex + 1;
testSamplingFrequencies = FS(testFSdex); % 10.2 Hz, 33.05 Hz, 190 Hz

dirLabs = {'Heading (°)' 'Flexion (°)' 'Lateral Flexion (°)'};

clear q costs
for i = 1 : length(testSamplingFrequencies)

% Set optimal parameters
dex = T.SamplingFrequency == testSamplingFrequencies(i) & strcmp(T.Algorithm,'RiddickAHRS') & strcmp(T.Condition,COND);
Tsub = T(dex,:);
PSTAR = mean(vertcat(Tsub.Parameters{:}));


[q{i},costs(i)] = estimateAttitude2(AHRS,PSTAR,pNames,testSamplingFrequencies(i),testData(dataDex(i)),opts);

eaEst{i,cc} = eulerd(quaternion(q{i}),'ZYX',eulerRotType);

% eaEst{i,cc} = eulerd(quaternion(q{i}),'ZYX',eulerRotType);
% eaEst_frame{i,cc} = eulerd(quaternion(q{i}),'ZYX',eulerRotType);

end

% figure;subplot(211);plot(eaEst{i,cc});subplot(212);plot(eaEst_frame{i,cc})

badMocapDex = DD.badMocapFramesOriginal;
% eaTruth = eulerd(quaternion(DD.Data.ground_truth),'ZYX',eulerRotType);
eaTruth = eulerd(quaternion(DD.Data.ground_truth),'ZYX',eulerRotType);


tRange = mean(DD.Data.time) + [0 3] - 2;

for dim = 1 : 3
    sbpDex = (dim-1)*3 + cc;
    subplot(3,3,sbpDex)

    [~,IS] = min(abs(DD.Data.time - tRange(1)));
    [~,IE] = min(abs(DD.Data.time - tRange(2)));

        plotDex = false(length(testData(dataDex(i)).time),1);
    plotDex(IS:IE) = true;
    plotDex(badMocapDex) = false;

plot(DD.Data.time(plotDex),eaTruth(plotDex,dim),'k','LineWidth',1.5);
hold on

    set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');


if dim == 1
    title(COND,'Interpreter','none');
end
if cc == 1
    ylabel(dirLabs{dim})
end
if dim == 3
    xlabel('Time (sec)')
end

for i = 1 : length(testSamplingFrequencies)
    [~,IS] = min(abs(testData(dataDex(i)).time - tRange(1)));
    [~,IE] = min(abs(testData(dataDex(i)).time - tRange(2)));

    col = blueGrad(i,length(testFSdex));
    plot(testData(dataDex(i)).time(IS:IE),eaEst{i,cc}(IS:IE,dim),'Color',col);
end
if dim == 2
hl=legend([{'Truth'};cellstr(num2str(FS(testFSdex),'%.01f'))]);
% hl.Box='Off';
end
end

end

if saveFigs
    for i = 1 : length(saveFormats)
        figName = [figDir 'Exemplar_SampFreqDegradation_SitStand-Walk-Jog'];% saveFormats{i}];
         saveas(gcf,figName,saveFormats{i});
    end
end



%% Estimate Position
% H1 = 1;
% H2 = 1.4;
% 
% N = length(data.acc);
% 
% hpFreq = 0.1;
% % hpFreq = 0.5;
% lpFreq = 5;
% hpOrder = 2;
% 
% [Ba,Aa] = butter(hpOrder,[hpFreq lpFreq] / (fs/2),'bandpass');
% [Bv,Av] = butter(hpOrder,[hpFreq lpFreq] / (fs/2),'bandpass');
% [Bp,Ap] = butter(hpOrder,[hpFreq lpFreq] / (fs/2),'bandpass');
% 
% [z,p,k] = butter(hpOrder,hpFreq / (fs/2),'high');
% sosAcc = zp2sos(z,p,k);
% [z,p,k] = butter(hpOrder,hpFreq / (fs/2),'high');
% sosVel = zp2sos(z,p,k);
% [z,p,k] = butter(hpOrder,hpFreq / (fs/2),'high');
% sosPos = zp2sos(z,p,k);
% 
% acc = data.acc;
% R1 = quatern2rotMat(qEstimated);
% R1 = permute(R1,[2 1 3]);
% 
% 
% accWorld = pagemtimes(R1,permute(acc,[2 3 1]));
% accWorld = permute(accWorld,[3 1 2]);
% accWorld = accWorld - [0 0 1];
% accWorld = accWorld * 9.81;
% accWorld = detrend(accWorld);
% % accWorld = sosfilt(sosAcc,accWorld);
% accWorld = filtfilt(Ba,Aa,accWorld);
% % accWorld = detrend(accWorld);
% 
% vel1 = cumtrapz(accWorld)/fs;
% vel1 = detrend(vel1,1);
% % vel1 = sosfilt(sosVel,vel1);
% vel1 = filtfilt(Bv,Av,vel1);
% vel1 = detrend(vel1);
% 
% p1 = cumtrapz(vel1)/fs;
% % p1 = detrend(p1);
% % p1 = sosfilt(sosPos,p1);
% p1 = filtfilt(Bp,Ap,p1);
% % p1 = detrend(p1);
% Pos1 = repmat([0 0 H1],N,1);
% Pos1 = Pos1 + p1;
% 
% 
% 
% acc2 = data2.acc;
% R2 = quatern2rotMat(qEstimated2);
% R2 = permute(R2,[2 1 3]);
% accWorld2 = pagemtimes(R2,permute(acc2,[2 3 1]));
% accWorld2 = permute(accWorld2,[3 1 2]);
% accWorld2 = accWorld2 - [0 0 1];
% accWorld2 = accWorld2 * 9.81;
% accWorld2 = detrend(accWorld2);
% 
% 
% vel2 = cumtrapz(accWorld2)/fs;
% vel2 = detrend(vel2,1);
% vel2 = filtfilt(Bv,Av,vel2);
% vel2 = detrend(vel2);
% 
% p2 = cumtrapz(vel2)/fs;
% % p1 = detrend(p1);
% % p1 = sosfilt(sosPos,p1);
% p2 = filtfilt(Bp,Ap,p2);
% % p1 = detrend(p1);
% Pos2 = repmat([0 0 H2],N,1);
% Pos2 = Pos2 + p2;
% 
% figure;
% tt = (1:length(vel1))/fs;
% subplot(311)
% plot(tt,accWorld)
% subplot(312)
% plot(tt,vel1)
% subplot(313)
% plot(tt,p1)
%% Animation

% freqDex = 2;
% condDex = 1;
% % dex = 1000:1500;
% % dex = 1 : length(p1);
% 
% H1 = 1;
% H2 = 1.4;
% 
% [X1,Y1,Z1] = ellipsoid(0,0,0,0.2,0.2,0.2);
% [X2,Y2,Z2] = ellipsoid(0,0,0,0.2,0.2,0.35);
% 
% O1 = H1;
% O2 = H2;
% figure;
% S1 = surf(X1,Y1,Z1+O1,'EdgeColor','none','FaceColor',[0.2 0.2 0.8],'FaceAlpha',0.6);
% % S1 = surf(X1,Y1,Z1+O1,'EdgeColor','none','FaceAlpha',0.8);
% hold on
% % S2 = surf(X2,Y2,Z2+O2,'EdgeColor','none','FaceColor',[0.8 0.2 0.2],'FaceAlpha',0.6);
% % S2 = surf(X2,Y2,Z2+O2,'EdgeColor','none','FaceAlpha',0.8);
% 
% camlight RIGHT
% 
% R1 = euler2rotMat(deg2rad(eaEst{freqDex,condDex}(:,1)),...
%     deg2rad(eaEst{freqDex,condDex}(:,2)),deg2rad(eaEst{freqDex,condDex}(:,3)));
% R1 = pagemtimes(R1(:,:,1).',R1);
% % R2 = euler2rotMat(deg2rad(eaEst2(:,1)),deg2rad(eaEst2(:,2)),deg2rad(eaEst2(:,3)));
% % R2 = pagemtimes(R2(:,:,1).',R2);
% % axang1 = rotm2axang(R1);
% % axang2 = rotm2axang(R2);
% 
% pp =cat(2,X1(:),Y1(:),Z1(:));
% rp1 = pagemtimes(R1,pp.');
% RSX1 = reshape(rp1(1,:,:),size(X1,1),size(X1,2),length(eaEst{freqDex,condDex}));
% RSY1 = reshape(rp1(2,:,:),size(X1,1),size(X1,2),length(eaEst{freqDex,condDex}));
% RSZ1 = reshape(rp1(3,:,:),size(X1,1),size(X1,2),length(eaEst{freqDex,condDex}));
% 
% % pp2 =cat(2,X2(:),Y2(:),Z2(:));
% % rp2 = pagemtimes(R2,pp2.');
% % RSX2 = reshape(rp2(1,:,:),size(X2,1),size(X2,2),length(eaEst2));
% % RSY2 = reshape(rp2(2,:,:),size(X2,1),size(X2,2),length(eaEst2));
% % RSZ2 = reshape(rp2(3,:,:),size(X2,1),size(X2,2),length(eaEst2));
% 
% axis equal
% set(gca,'XLimMode','manual');
% set(gca,'YLimMode','manual');
% set(gca,'ZLimMode','manual');
% 
% rr = 1*[-1 1];
% set(gca,'XLim',0 + rr)
% set(gca,'YLim',0 + rr)
% set(gca,'ZLim',1 + rr)
% 
% axis off
% fs = FS(testFSdex(4));
% dex = 1 : 5 : length(eaEst{4});
% for i = dex
% %     S1.XData = RSX1(:,:,i);
% %     S1.YData = RSY1(:,:,i);
% %     S1.ZData = RSZ1(:,:,i) + O1;
% % 
% %         S2.XData = RSX2(:,:,i);
% %     S2.YData = RSY2(:,:,i);
% %     S2.ZData = RSZ2(:,:,i) + O2;
% 
% %     S1.XData = RSX1(:,:,i) + Pos1(i,1);
% %     S1.YData = RSY1(:,:,i)+ Pos1(i,2);
% %     S1.ZData = RSZ1(:,:,i) + Pos1(i,3);
% 
%     S1.XData = RSX1(:,:,i);
%     S1.YData = RSY1(:,:,i);
%     S1.ZData = RSZ1(:,:,i) + H1;
% 
% %         S2.XData = RSX2(:,:,i) + Pos2(i,1);
% %     S2.YData = RSY2(:,:,i) + Pos2(i,2);
% %     S2.ZData = RSZ2(:,:,i) + Pos2(i,3);
% 
% pause(1/30 * fs/30);
% drawnow
% end
%% Error vs Sample Freq: Comparison of different filters

% modelType = 'exponential';
% modelType = 'hyperbolic';
modelType = 'rat11';

useLogLogScale = true;

hf = figure;
hf.Color = [1 1 1];
hf.Units = 'centimeters';
hf.Position = [2 2 doubleColumnWidth doubleColumnWidth];


for cc = 1 : length(condis)
% figure
subplot(4,3,cc)
for aa = 1 : length(algos)

    Tsub = T(strcmp(T.Algorithm,algos{aa}) & strcmp(T.Condition,condis{cc}),:);


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
        meanCosts(j,1) = mean(Tsub.Cost(Tsub.SamplingFrequency==FS(j)));
    end

    meanCosts = meanCosts/3;

    mdl = fitnlm(FS,meanCosts,model,beta0);



    col = blueGrad(aa,length(algos));

    if useLogLogScale
        semilogx(FS,meanCosts,'.','Color',col,'HandleVisibility','off','MarkerSize',10)
        hold on
        semilogx(FS,predict(mdl,FS),'Color',col)
        ylabel('Error')
        xlabel('Log(Sample Frequency) (Hz)')

%         loglog(FS,meanCosts,'.','Color',col,'HandleVisibility','off','MarkerSize',10)
%         hold on
%         loglog(FS,predict(mdl,FS),'Color',col)
%         ylabel('Error')
%         xlabel('Log(Sample Frequency) (Hz)')
          
set(gca,'XTick',[1 10 100 190]);
set(gca,'XTickLabel',get(gca,'XTick'))
set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');
    else
        plot(FS,meanCosts,'.','Color',col,'HandleVisibility','off','MarkerSize',10)
        hold on
        plot(FS,predict(mdl,FS),'Color',col)
        ylabel('Error')
        xlabel('Sample Frequency (Hz)')
    end
end


title(condis{cc},'Interpreter','none')
end
legend(algos,'Interpreter','none');

if saveFigs
    for i = 1 : length(saveFormats)
        figName = [figDir 'ErrorVsFrequency_byCondition'];% saveFormats{i}];
         saveas(gcf,figName,saveFormats{i});
    end
end

%% Compare error vs freq for each condition (Riddick filter)
fitFreqDex = 4 : length(FS);
modelType = 'exponential';
% modelType = 'hyperbolic';
% modelType = 'rat11';

useLogLogScale = true;

hf = figure;
% hf.WindowState = 'maximized';
hf.Color = [1 1 1];
hf.Units = 'centimeters';
hf.Position = [2 2 singleColumnWidth singleColumnWidth];
cols = jet(length(condis));

aa = 1;

for cc = 1 : length(condis)
% figure

    Tsub = T(strcmp(T.Algorithm,algos{aa}) & strcmp(T.Condition,condis{cc}),:);


    switch modelType
        case 'exponential'
            model = @(phi,fs) phi(1).*exp(phi(2).*fs) + phi(3);
            beta0 = [2.719 -0.085 0.64];
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
        meanCosts(j,1) = mean(Tsub.Cost(Tsub.SamplingFrequency==FS(j)));
    end

    meanCosts = meanCosts/3;

    mdl = fitnlm(FS(fitFreqDex),meanCosts(fitFreqDex,1),model,beta0);

    EMOD{cc} = mdl;



fsi = linspace(1,200,100).';
    if useLogLogScale
%         semilogx(FS,meanCosts,'.','Color',cols(cc,:),'HandleVisibility','off','MarkerSize',10)
%         hold on
%         semilogx(fsi,predict(mdl,fsi),'Color',cols(cc,:))


        semilogy(FS(fitFreqDex),meanCosts(fitFreqDex),'.','Color',cols(cc,:),'HandleVisibility','off','MarkerSize',10)
        hold on
        semilogy(fsi,predict(mdl,fsi),'Color',cols(cc,:))

        ylabel('Error (°)')
        xlabel('Sample Frequency (Hz)')

%         loglog(FS,meanCosts,'.','Color',cols(cc,:),'HandleVisibility','off','MarkerSize',10)
%         hold on
%         loglog(fsi,predict(mdl,fsi),'Color',cols(cc,:))
%         ylabel('Error')
%         xlabel('Log(Sample Frequency) (Hz)')
          
set(gca,'YTick',[0.5 1 2 3 4 5 6 7 8 9 10])
% set(gca,'XTick',[1 10 100 190]);
% set(gca,'XTickLabel',get(gca,'XTick'))
    else
%         plot(FS,meanCosts,'.','Color',cols(cc,:),'HandleVisibility','off','MarkerSize',10)
%         hold on
%         plot(fsi,predict(mdl,fsi),'Color',cols(cc,:))

                plot(FS(fitFreqDex),meanCosts(fitFreqDex),'.','Color',cols(cc,:),'HandleVisibility','off','MarkerSize',10)
        hold on
        plot(fsi,predict(mdl,fsi),'Color',cols(cc,:))

        ylabel('Error')
        xlabel('Sample Frequency (Hz)')
    end

    set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');

end
hL=legend(condis,'Interpreter','none');
hL.Box = 'Off';
set(gca,'YLim',[0 6]);
set(gca,'Box','Off')

if saveFigs
    for i = 1 : length(saveFormats)
        figName = [figDir 'ErrorVsFrequency_compareConditions_Riddick'];% saveFormats{i}];
         saveas(gcf,figName,saveFormats{i});
    end
end

%% Error table

    clear coeffsOverall rSquaredOverall coeffsCondis rSquaredCondis
    for cc = 1 : length(condis)
        coeffs = EMOD{cc}.Coefficients.Estimate;
        % Transform coefficient variables to those in paper
        coeffs(1) = coeffs(1) + coeffs(3);
        coeffs(2) = -coeffs(2);
        coeffs = [coeffs(1) coeffs(3) coeffs(2)];
        coeffsOverall(cc,:) = coeffs;
        rSquaredOverall(cc,:) = EMOD{cc}.Rsquared.Adjusted;


    end
        X = [coeffsOverall rSquaredOverall];
        X = [compose('%.02f',X(:,1:2)) compose('%.03f',X(:,3)) compose('%.02f',X(:,4))];
        errorTable = array2table(X,"VariableNames",{'E0' 'Einfinity' 'lambda'  'adj-rsquared'},"RowNames",condis);
        writetable(errorTable,[tableDir 'ErrorTable_CompareCondis_RiddickFilter'  '.xlsx'],'WriteRowNames',true)
%% Parameters vs sample freq
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
for algoDex = 1 : length(algos)
hf = figure;
% hf.WindowState = 'maximized';
hf.Color = [1 1 1];
    parmNames = optimOpts.(algos{algoDex}).parameters.Names;
    for cc = 1 : length(condis)
        meanParms= [];
        for pp = 1:length(parmNames)

%             modelType = modelTypes{algoDex}{pp};
            modelType = 'exponential';
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
            mdl = fitnlm(FS,meanParms(:,parmDex),model,beta0);


            riddickMods{cc,pp} = mdl;

    
            riddickParms{cc} = meanParms;
            end


            subplot(length(parmNames),1,pp)

            plot(FS(fitFreqDex),meanParms(fitFreqDex,pp),'.-','Color',cols(cc,:))
%             semilogy(FS(fitFreqDex),meanParms(fitFreqDex,pp),'.-','Color',cols(cc,:))
%             plot(FS(fitFreqDex),meanParms(fitFreqDex,pp),'.-','Color',cols(cc,:))
%             plot(FS(fitFreqDex),meanParms(fitFreqDex,pp),'.','Color',cols(cc,:))
%             plot(FS,meanParms(:,pp),'.')
            hold on

%             fsi = linspace(fitFreqs(1),fitFreqs(end),100).';
%             fsi = linspace(1,200,100).';
%             plot(fsi,predict(mdl,fsi),'Color',cols(cc,:))

            % Plot data points not used in the fit in black
%             L = true(size(FS));
%             L(fitFreqDex) = false;
%             plot(FS(L),meanParms(L,pp),'k.')

            title([algos{algoDex} ' ' parmNames{pp}],'Interpreter','none')
            ylabel('No Units')

            if pp == length(parmNames)
                xlabel('Sampling Frequency (Hz)')
            end

            set(gca,'Box','Off')
        end

        
    end
    legend(condis{:},'Interpreter','none')

    if saveFigs
        for i = 1 : length(saveFormats)
            figName = [figDir 'ParametersVsFrequency_byCondition_' algos{algoDex}];% saveFormats{i}];
             saveas(gcf,figName,saveFormats{i});
        end
    end
end


%% Riddick: Optimal parameters vs task & freq
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
hf = figure;
% hf.WindowState = 'maximized';
hf.Color = [1 1 1];
parmNames = optimOpts.(algos{algoDex}).parameters.Names;

meanParms= [];
for pp = 1:length(parmNames)
    figure;
    for cc = 1 : length(condis)


        %             modelType = modelTypes{algoDex}{pp};
        modelType = 'exponential';
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
            %             mdl = fitnlm(fitFreqs,meanParms(fitFreqDex,parmDex),model,beta0);

            fopts = statset;
            fopts.MaxIter = 400;
            fopts.RobustWgtFun = 'cauchy';
            mdl = fitnlm(FS(fitFreqDex),meanParms(fitFreqDex,parmDex),model,beta0,'Options',fopts);


            riddickMods{cc,pp} = mdl;


            riddickParms{cc} = meanParms;

            R2(cc,pp) = mdl.Rsquared.Adjusted;
        end


        subplot(4,3,cc)

%         plot(FS(fitFreqDex),meanParms(fitFreqDex,pp),'.-','Color',cols(cc,:))
        %             semilogy(FS(fitFreqDex),meanParms(fitFreqDex,pp),'.-','Color',cols(cc,:))
        %             plot(FS(fitFreqDex),meanParms(fitFreqDex,pp),'.-','Color',cols(cc,:))
                    plot(FS(fitFreqDex),meanParms(fitFreqDex,pp),'.','Color',[0 0 0])
        %             plot(FS,meanParms(:,pp),'.')
        hold on

%                     fsi = linspace(fitFreqs(1),fitFreqs(end),100).';
                    fsi = linspace(1,200,100).';
                    plot(fsi,predict(mdl,fsi),'Color',[0 0 0])

        % Plot data points not used in the fit in black
        %             L = true(size(FS));
        %             L(fitFreqDex) = false;
        %             plot(FS(L),meanParms(L,pp),'k.')

        title([parmNames{pp} ' ' condis{cc}],'Interpreter','none')
        ylabel('No Units')

        if pp == length(parmNames)
            xlabel('Sampling Frequency (Hz)')
        end
        if cc == 1
            legend('Data','Model')
        end

        set(gca,'Box','Off')
    end


end
% legend(condis{:},'Interpreter','none')


end


%% Find when gyroscope gain crosses threshold
% % The proposed filter has a gain on the gyroscope. As the sampling frequency
% % is lower, the optimal gain for the gyroscope decreases, since the linear
% % integration model gets less accurate as sampling frequency decreases. At
% % low enough sampling frequencies, this gain tends to approach zero
% % exponentially. One way of estimating how well the filter reproduces the
% % dynamics of the task is the level of this gain. At a level of 1, the
% % filter attempts to integrate all of the dynamcis in the task. At a level
% % of zero, it integrates none. Choosing an arbitrary value of 0.8 gives and
% % indication at one sampling frequency the filter is only attempting to
% % integrate 80% of the higher frequency dynamics.
% 
% gainThresh = 0.8;
% 
% figure
% for cc = 1 : length(condis)
%     subplot(4,3,cc)
% mdl = riddickMods{cc,1};
% gyroParms = riddickParms{cc}(:,1);
% 
% plot(FS,gyroParms,'k.');
% hold on
% predictedGyroGains = predict(mdl,FS);
% plot(FS,predictedGyroGains,'r');
% 
% niDex = 1 : fitFreqDex(1)-1;
% plot(FS(niDex),gyroParms(niDex),'g.');
% title(condis{cc},'Interpreter','none')
% 
% G0 = predictedGyroGains(end);
% cutOff = gainThresh*G0;
% cutOffFreq(cc) = fsolve(@(fs) cutOff - predict(mdl,fs),10);
% end
% 
% cutOffFreq(cutOffFreq<0) = 0;
% cutOffFreq_nonLocomotion = mean(cutOffFreq(1:6))
% cutOffFreq_locomotion = mean(cutOffFreq(7:11))
% 
% hf = figure;
% [~,II] = sort(cutOffFreq);
% subplot(312)
% bb = bar(cutOffFreq(II));
% bb.FaceColor = [0 0 0];
% hold on
% set(gca,'XTickLabel',condis(II))
% set(gca,'TickLabelInterpreter','none')
% % set(gca,'XTickLabel',[])
% title('Min Sampling Frequency for >80% belief in integration model')
% ylabel('FS (Hz)')

%% Compute cut-off frequency or threshold on error
% % cutOffmethod = 'Absolute';
% % cutOffmethod = 'RelativeToHigh';
% cutOffmethod = 'RelativeToRange';
% 
% 
% 
% for cc = 1 : length(condis)
% 
%     switch cutOffmethod
%         case 'Absolute'
%             cutOff = 5; %Hz (absolute cutoff)
%             cutOffFreq1(cc) = fsolve(@(fs) cutOff - predict(EMOD{cc},fs),10);
%         case 'RelativeToHigh'
%             cutOffFactor = 0.5; % Relative to error at 190 Hz (relative cutoff)
%             f0 = table2array(EMOD{cc}.Coefficients(3,1));
%             cutOffFreq1(cc) = fsolve(@(fs) (1+cutOffFactor)*f0 - predict(EMOD{cc},fs),10);
%         case 'RelativeToRange'
%             cutOffFactor = 0.2;
%             coeffs = table2array(EMOD{cc}.Coefficients(:,1));
%             Ehigh = coeffs(3); % Error at high freq
%             Erange = coeffs(1); % amount of error increases from high frequency to 0 hz (y intercept)
%             cutOff = Ehigh + cutOffFactor*Erange;
%             cutOffFreq1(cc) = fsolve(@(fs) cutOff - predict(EMOD{cc},fs),10);
%     end
% 
% end
% 
% 
% [CO,II1] = sort(cutOffFreq1);
% 
% figure(hf)
% subplot(311)
% % hf.WindowState = 'maximized';
% hf.Color = [1 1 1];
% 
% 
% b = bar(CO.');
% set(gca,'XTickLabel',condis(II1))
% 
% % b = bar(cutOffFreq1(II).');
% % set(gca,'XTickLabel',condis(II))
% 
% 
% b.FaceColor = [0 0 0];
% % b.CData = cols;
% 
% % set(gca,'XTickLabel',[])
% set(gca,'TickLabelInterpreter','none')
% ylabel('FS (Hz)')
% switch cutOffmethod
%     case 'Absolute'
% title('Min Sampling Frequency for < 5° average RMS error')
%     case 'RelativeToHigh'
%     title('Min Sampling Freq for < 50% relative increase in error (baseline 190 Hz)')
%     case 'RelativeToRange'
%          title('Min Sampling Freq for < 20% relative increase in error (baseline range 0-190 Hz)')
% end
% ylabel('Sampling Frequency (Hz)')
% % title('Min Sampling Frequency (for < 50% increase average RMS error)')

%% Max of both cut-off freqs
% OCUT = max([cutOffFreq1;cutOffFreq]);
% figure(hf)
% subplot(313)
% b3 = bar(OCUT(II));
% b3.FaceColor = [0 0 0];
% set(gca,'XTickLabel',condis(II))
% set(gca,'TickLabelInterpreter','none')
% ylabel('FS (Hz)')
% title('Min sampling Frequency for overall acceptable performance')
% set(gca,'YTick',[0 10 20 30 40 50])

%% Compute cut-off frequency or threshold on error
% cutOffmethod = 'Absolute';
% cutOffmethod = 'RelativeToHigh';
cutOffmethod = 'RelativeToRange';



for cc = 1 : length(condis)

    switch cutOffmethod
        case 'Absolute'
            cutOff = 3; %Hz (absolute cutoff)
            cutOffFreq1(cc) = fsolve(@(fs) cutOff - predict(EMOD{cc},fs),10);
        case 'RelativeToHigh'
            cutOffFactor = 0.5; % Relative to error at 190 Hz (relative cutoff)
            f0 = table2array(EMOD{cc}.Coefficients(3,1));
            cutOffFreq1(cc) = fsolve(@(fs) (1+cutOffFactor)*f0 - predict(EMOD{cc},fs),10);
        case 'RelativeToRange'
            cutOffFactor = 0.2;
            coeffs = table2array(EMOD{cc}.Coefficients(:,1));
            Ehigh = coeffs(3); % Error at high freq
            Erange = coeffs(1); % amount of error increases from high frequency to 0 hz (y intercept)
            cutOff = Ehigh + cutOffFactor*Erange;
            cutOffFreq1(cc) = fsolve(@(fs) cutOff - predict(EMOD{cc},fs),10);
    end

end


[CO,II1] = sort(cutOffFreq1);

hf = figure;
hf.Color = [1 1 1];
hf.Units = 'centimeters';
hf.Position = [2 2 doubleColumnWidth singleColumnWidth*0.5];


b = bar(CO.');
set(gca,'XTickLabel',condis(II1))

% b = bar(cutOffFreq1(II).');
% set(gca,'XTickLabel',condis(II))


b.FaceColor = [0 0 0];
% b.CData = cols;

% set(gca,'XTickLabel',[])
set(gca,'TickLabelInterpreter','none')
set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');
ylabel('FS (Hz)')
% switch cutOffmethod
%     case 'Absolute'
% title('Min Sampling Frequency for < 5° average RMS error')
%     case 'RelativeToHigh'
%     title('Min Sampling Freq for < 50% relative increase in error (baseline 190 Hz)')
%     case 'RelativeToRange'
%          title('Min Sampling Freq for < 20% relative increase in error (baseline range 0-190 Hz)')
% end
ylabel('Sampling Frequency (Hz)')
% title('Min Sampling Frequency (for < 50% increase average RMS error)')

title('Recommended minimum sampling frequency')

if saveFigs
    for i = 1 : length(saveFormats)
        figName = [figDir 'Rec_SampFreq_AllCondis_barChart'];% saveFormats{i}];

        saveas(gcf,figName,saveFormats{i});

%         saveas(gcf,[figDir 'Rec_SampFreq_AllCondis_barChart'],saveFormats{i});
    end
end

%% Recommended frequencies table


labels = condis(II1);

        X = cutOffFreq1(II1);
        X = compose('%.02f',X);
        recFreqTable = array2table(X,"VariableNames",labels);
        writetable(recFreqTable,[tableDir 'RecommendedSamplingFrequencies_ByTask'  '.xlsx']);


%% Show examples of degradation of performance w.r.t. Sampling Frequency - PART 2
% Here we show the difference between sampling at 190 Hz, versus our
% recommended minimum frequency, versus a lower sampling frequency
%
% Let's focus on just one joint angle?



AHRS = RiddickAHRS;
opts.GradDescent.normStepSize = false;

optimOpts = defaultOptimOpts;
pNames = optimOpts.RiddickAHRS.parameters.Names;

% condFS = {[3 5 6 10] [3 5 6 7 10] [6 7 8 10]};
% testFSdex = [3 4 5 6 7 10];

Subj = 10;

% CONDIS = {'right_left_lift' 'walk_3kmh' 'og_jog_sss'};
CONDIS = condis.';

clear CDEX testSamplingFrequencies
for i = 1 : length(CONDIS)
    CDEX(i) = find(strcmp(CONDIS{i},condis));

    testSamplingFrequencies{i,1} = [190 cutOffFreq1(CDEX(i)) 8];
end
% testSamplingFrequencies = {[190 cutOffFreq1(CDEX(1)) 5] [190 cutOffFreq1(CDEX(2)) 5] [190 cutOffFreq1(CDEX(3)) 5]};

% CONDIS = {'sit_stand_normal' 'walk_3kmh' 'og_jog_sss'};
% CONDIS = {'sit_stand_normal' 'walk_3kmh' 'figure8_walking'};


hf = figure;
hf.Color = [1 1 1];
hf.Units = 'centimeters';
hf.Position = [2 2 doubleColumnWidth doubleColumnWidth*0.75];

cols = winter(2);

cols = [cols(1,:);[1 0 0];cols(2:end,:)];

for cc = 1 : length(CONDIS)
COND = CONDIS{cc};

% testFSdex = condFS{cc};

testSubj = ['Subject' num2str(Subj)];

sensorNames = D.(testSubj).sensorNames;
testSensorName = sensorNames{1};
% DD = D.Subject1.walk_3kmh.mdm650D;
% DD = D.Subject2.(COND).mdm64B1;
% DD = D.Subject3.(COND).mdm64BF;

DD = D.(testSubj).(COND).(testSensorName);

% COND = 'og_jog_sss';
testData = DD.ResampleData;
% testFSdex = [5 7 10];

% dataDex = 10 - testFSdex + 1;
% testSamplingFrequencies = FS(testFSdex); % 10.2 Hz, 33.05 Hz, 190 Hz

dirLabs = {'Heading (°)' 'Flexion (°)' 'Lateral Flexion (°)'};

clear q costs TD
for i = 1 : length(testSamplingFrequencies{cc})
% For each sampling frequency, down sample from 190 Hz
testFS = testSamplingFrequencies{cc}(i);

TD(i) = testData(1);

if testFS ~= 190
tOrig = TD(i).time;
tNew = linspace(tOrig(1),tOrig(end),round(range(tOrig)*testFS)).';

TD(i).time = tNew;
snames = fieldnames(TD(i));
snames(strcmp(snames,'time')) = [];
for jj = 1 : length(snames)
    TD(i).(snames{jj}) = interp1(tOrig,TD(i).(snames{jj}),tNew);
end
end

clear PSTAR
for pp = 1 : 3
mod = riddickMods{cc,pp};
  PSTAR(1,pp) = predict(mod,testFS);
end
% dex = T.SamplingFrequency == testSamplingFrequencies(i) & strcmp(T.Algorithm,'RiddickAHRS') & strcmp(T.Condition,COND);
% Tsub = T(dex,:);
% PSTAR = mean(vertcat(Tsub.Parameters{:}));


[q{i},costs(i)] = estimateAttitude2(AHRS,PSTAR,pNames,testFS,TD(i),opts);


eaEst{i,cc} = eulerd(quaternion(q{i}),'ZYX',eulerRotType);

end
badMocapDex = DD.badMocapFramesOriginal;
eaTruth = eulerd(quaternion(DD.Data.ground_truth),'ZYX',eulerRotType);


tRange = mean(DD.Data.time) + [0 3] - 2;

if Subj == 6 && strcmp(CONDIS{cc},'og_walk_sss')
    tRange = tRange - 1;
elseif Subj == 6 && strcmp(CONDIS{cc},'og_jog_sss')
    tRange = tRange;
    elseif Subj == 3 && strcmp(CONDIS{cc},'og_jog_sss')
        tRange = tRange + 2.5;
            elseif Subj == 4 && strcmp(CONDIS{cc},'og_jog_sss')
        tRange = tRange + 1;
end

for dim = 2
    subplot(3,4,cc)

    [~,IS] = min(abs(DD.Data.time - tRange(1)));
    [~,IE] = min(abs(DD.Data.time - tRange(2)));

        plotDex = false(length(testData(dataDex(i)).time),1);
    plotDex(IS:IE) = true;
    plotDex(badMocapDex) = false;

plot(DD.Data.time(plotDex),eaTruth(plotDex,dim),'k','LineWidth',1.5);
hold on
ylabel(dirLabs{dim})

set(gca,'fontname',fontName,'fontsize',fontSize,'Box','off');
% if dim == 1
    title(COND,'Interpreter','none');
% end
% if cc == 1
%     ylabel(dirLabs{dim})
% end
% if dim == 3
%     xlabel('Time (sec)')
% end

for i = 1 : length(testSamplingFrequencies{cc})
    [~,IS] = min(abs(TD(i).time - tRange(1)));
    [~,IE] = min(abs(TD(i).time - tRange(2)));

    plot(TD(i).time(IS:IE),eaEst{i,cc}(IS:IE,dim),'Color',cols(i,:));
end
if cc == 1
    lStr = [{'Truth'};cellstr(num2str(testSamplingFrequencies{cc}.','%.01f'))];
    lStr{3} = [lStr{3} ' (rec)'];
    legend(lStr)
else
    lStr = [{'Truth'};cellstr(num2str(testSamplingFrequencies{cc}.','%.01f'))];
    text(mean(TD(i).time(IS:IE)),mean(get(gca,'YLIM')),lStr{3},'Color','r');
end
end



end

if saveFigs
    for i = 1 : length(saveFormats)
        figName = [figDir 'Exemplar_RecSampling_Performance_allCondis' saveFormats{i}];
         saveas(gcf,figName,strrep(saveFormats{i},'.',''));
%         if ~any(regexpi('eps',saveFormats{i}))
%             saveas(gcf,figName);
%         else
%             figName = [figDir 'Exemplar_RecSampling_Performance_allCondis' saveFormats{i}];
%             saveas(gcf,figName,'epsc');
%         end
    end
end
%% Optimal Parameters by Condition - 190 HZ
TP = T;

for algoDex = 1 : length(algos)
    parmNames = optimOpts.(algos{algoDex}).parameters.Names;
     meanParms= [];

    for cc = 1 : length(condis)
        Tsub = TP(strcmp(TP.Algorithm,algos{algoDex}) & strcmp(TP.Condition,condis{cc}) & TP.SamplingFrequency == FS(end),:);
        PARMS = cell2mat(Tsub.Parameters);
                    % Fix bug saving parameter for gyro gain for Riddick filter
            % standing condition
            if algoDex == 1 && cc == 1
                for hh = 1 : height(Tsub)
                    if length(Tsub.Parameters{hh}) < 3
                        Tsub.Parameters{hh} = [0.9 Tsub.Parameters{hh}];
                    end
                end
            end


        for pp = 1:length(parmNames)

            parmDex = pp;


            
            meanParms(cc,parmDex) = mean(PARMS(:,parmDex));

        end
    end

    x = 1 : length(condis);

    hf = figure;
    hf.WindowState = 'maximized';
    hf.Color = [1 1 1];

    for pp = 1 : length(parmNames)
        subplot(length(parmNames),1,pp);
        [~,I] = sort(meanParms(:,pp));
        bar(x,meanParms(I,pp));
        set(gca,'XTickLabel',condis(I))
        set(gca,'TickLabelInterpreter','none');
        title([algos{algoDex} ' ' parmNames{pp} ' 190 Hz'],'Interpreter','none')
    end
%     legend(condis{:},'Interpreter','none')

    if saveFigs
        for i = 1 : length(saveFormats)
            figName = [figDir 'OptimalParametersAt190Hz_byCondition_' algos{algoDex} saveFormats{i}];
             saveas(gcf,figName,strrep(saveFormats{i},'.',''));
        end
    end
end
%% Compare filters at specific frequencies: Linear mixed effects model
mdl = 'Cost ~ Algorithm + (1|Subject) + (1|SensorLocation)';

clear Tfreq lme pVals predictedError predictedErrorCI peUCI peLCI

for cc = 1 : length(condis)
for i = 1 : length(FS)
    % Data for a certain freq & condition
    Tfreq{i,cc} = T(T.SamplingFrequency == FS(i) & strcmp(T.Condition,condis{cc}),:);

    Tfreq{i,cc}.Cost = Tfreq{i,cc}.Cost/3;


    % Linear mixed model fitting
    lme{i,cc} = fitlme(Tfreq{i,cc},mdl);
    
    % Predict for each algorithm the estimated response due to the fixed
    % effect (Algorithm)
    Ttest = Tfreq{i,cc}(1:length(algos),:);
    Ttest.Algorithm = algos;
    [predictedError{cc}(:,i),peCI] = predict(lme{i,cc},Ttest,'Conditional',false,'Simultaneous',false);

    peUCI{cc}(:,i) = peCI(:,2);
    peLCI{cc}(:,i) = peCI(:,1);
    
    pVals{cc}(:,i) = lme{i,cc}.Coefficients.pValue;

end
end
%% Bar plot: For each frequency & Condition (LME)
cols = jet(length(algos));
for cc = 1 : length(condis)
    hf = figure;
    hf.WindowState = 'maximized';
    hf.Color = [1 1 1];

    colororder(cols);
    b = bar(predictedError{cc}.');

    X = nan(length(FS), length(algos));
    for i = 1:length(algos)
        X(:,i) = b(i).XEndPoints;
    end

    hold on
    errorbar(X,predictedError{cc}.',peLCI{cc}.',peUCI{cc}.','k.')
    set(gca,'XTickLabel',num2str(FS,'%.01f'));
    xlabel('Sampling Frequency (Hz)')
    legend(algos)
    ylabel('Summed RMS error (°)')
    title(condis{cc},'Interpreter','none');

    if saveFigs
        for i = 1 : length(saveFormats)
            figName = [figDir 'ErrorVsFrequency_BarPlot_' condis{cc} saveFormats{i}];
             saveas(gcf,figName,strrep(saveFormats{i},'.',''));
        end
    end
end

%% Bar plot: Error vs Condition (Riddick filter, 190 Hz)
algoDex = 1;
fsDex = 10;
E = NaN(1,length(condis));
ELCI = NaN(1,length(condis));
EUCI = NaN(1,length(condis));
for cc = 1 : length(condis)
    E(cc) = predictedError{cc}(algoDex,fsDex);
    ELCI(cc) = peLCI{cc}(algoDex,fsDex);
    EUCI(cc) = peUCI{cc}(algoDex,fsDex);
end
[E,I] = sort(E);
ELCI = ELCI(I);
EUCI = EUCI(I);

cols = jet(length(algos));
    hf = figure;
    hf.Units = 'Inches';
    hf.Position = [1 1 7.2 7.2];
    hf.Color = [1 1 1];
    colororder(cols);
    X = 1 : length(condis);
    b = bar(X,E);
%     b.Color = 
    hold on
    errorbar(X,E,ELCI,EUCI,'k.');
    set(gca,'XTickLabel',condis);
    ylabel('Summed RMS error (°)')
    title([algos{algoDex} ' ' num2str(FS(fsDex),'%.01f') ' Hz']);





%% Bar plot: Error vs Condition (compare all algos, 190 Hz)

fsDex = 10;
E = NaN(length(algos),length(condis));
ELCI = NaN(length(algos),length(condis));
EUCI = NaN(length(algos),length(condis));
for algoDex = 1 : length(algos)
    for cc = 1 : length(condis)
        E(algoDex,cc) = predictedError{cc}(algoDex,fsDex);
        ELCI(algoDex,cc) = peLCI{cc}(algoDex,fsDex);
        EUCI(algoDex,cc) = peUCI{cc}(algoDex,fsDex);
    end
end
[~,I] = sort(mean(E));
E = E(:,I);
ELCI = ELCI(:,I);
EUCI = EUCI(:,I);

cols = jet(length(algos));
    hf = figure;
    hf.WindowState = 'maximized';
    hf.Color = [1 1 1];
    colororder(cols);
    b = bar(E.');

    X = nan(length(algos),length(condis));
    for i = 1:length(algos)
        X(i,:) = b(i).XEndPoints;
    end

    hold on
    errorbar(X.',E.',ELCI.',EUCI.','k.');
    set(gca,'XTickLabel',condis);
    ylabel('Summed RMS error (°)')
    legend(algos)
    title([num2str(FS(fsDex),'%.01f') ' Hz']);


    if saveFigs
        for i = 1 : length(saveFormats)
            figName = [figDir 'ErrorVsConditions_Bar_AllAlgos' saveFormats{i}];
             saveas(gcf,figName,strrep(saveFormats{i},'.',''));
        end
    end

%% Bar plot: 190 Hz (LME) for each condition: Error
fsdex = length(FS);

hf = figure;
hf.WindowState = 'maximized';
hf.Color = [1 1 1];
for cc = 1 : length(condis)
    subplot(3,4,cc)

b = bar(predictedError{cc}(:,fsdex),'FaceColor','flat');
b.CData = cols;
hold on
errorbar(1:length(algos),predictedError{cc}(:,fsdex),peLCI{cc}(:,fsdex),peUCI{cc}(:,fsdex),'k.');
set(gca,'XTickLabel',algos);
ylabel('Summed RMS error (°)')
title([num2str(FS(fsdex),'%.01f') ' Hz ' condis{cc}],'Interpreter','none');

pp = pVals{cc}(2:end,fsdex);
pGroups = {[1 2] [1 3] [1 4] [1 5] [1 6]};
sigstar(pGroups,pp);
end

    if saveFigs
        for i = 1 : length(saveFormats)
            figName = [figDir 'ErrorVsFrequency_BarPlot_190Hz' saveFormats{i}];
             saveas(gcf,figName,strrep(saveFormats{i},'.',''));
        end
    end

%% Bar plot: 10 Hz (LME) for each condition: Error
fsdex = 5;

hf = figure;
hf.WindowState = 'maximized';
hf.Color = [1 1 1];
for cc = 1 : length(condis)
    subplot(3,4,cc)

b = bar(predictedError{cc}(:,fsdex),'FaceColor','flat');
b.CData = cols;
hold on
errorbar(1:length(algos),predictedError{cc}(:,fsdex),peLCI{cc}(:,fsdex),peUCI{cc}(:,fsdex),'k.');
set(gca,'XTickLabel',algos);
ylabel('Summed RMS error (°)')
title([num2str(FS(fsdex),'%.01f') ' Hz ' condis{cc}],'Interpreter','none');

pp = pVals{cc}(2:end,fsdex);
pGroups = {[1 2] [1 3] [1 4] [1 5] [1 6]};
sigstar(pGroups,pp);
end

    if saveFigs
        for i = 1 : length(saveFormats)
            figName = [figDir 'ErrorVsFrequency_BarPlot_10Hz' saveFormats{i}];
             saveas(gcf,figName,strrep(saveFormats{i},'.',''));
        end
    end