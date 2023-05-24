% There is an issue where the NaNs in the mocap data were set to zero
% during pre-processing and then resampled. This makes it difficult to
% later figure out which frames of motion capture data are missing. Here we
% simply reload the original mocap files (.c3d) and output which frames the
% markers were missing for each condition for each participant. This data
% is then stored in nandexFileName.
%
% When comparing the AHRS filters to the ground truth mocap data, you
% should not include these indices in which mocap is missing data for that
% sensor when computing the error between the two estimates. This is mostly
% only an issue for the overground walking and running processing and to a
% lesser extent treadmill running.
%
% Additionally, when downsampling from 200 Hz to 190 Hz, setting the
% missing data to zeros introduces high frequency oscillations near each
% missing data segment. This script also fixes this issue by interpolating
% each good data segment separately by ignoring all NaN data. It then
% rewrites the datafiles with the re-interpolated data that no longer has
% this high frequency oscillation issue. This should only be causing issues
% really in the overground trials and occassionally in treadmill jogging.



subjects = 1:12;

% dataDir = 'F:\research\painData\validation2021\'; % Parent folder
% dataDir = 'D:\painData\validation2021\'; % Parent folder
dataDir = 'C:\Users\ryrid\Documents\Work\painData\validation2021\';

% Subject 1 had an error in the file numbers - I have shifted them to start
% at 19 instead of 1. Need to check with Wolly on whether this is correct,
% but the files look like they are the correct labels.
conditionsFile = [dataDir 'condition_filenumbers_s1corrected.xlsx']; %file listing condition info
[num,txt,raw] = xlsread(conditionsFile);
import org.opensim.modeling.*
for s = subjects
%%

pathn=[dataDir 's',num2str(s), filesep]; %Subject-specific folder

sensorDataName=[pathn 'dorsaVi_validation_s',num2str(s),'.mat']; % Data from dorsaVi IMUs
saveDataName = [pathn,'s',num2str(s),'_data.mat']; % Output data file
sensorLocationSaveName = [pathn,'s_',num2str(s),'_SensorLocation.mat']; %output sensor location file

nandexFileName = [pathn 's',num2str(s) '_badMocapIndices.mat'];

%%


% the collected conditions
conditions={
    'cluster_alignment'
    'standing_ref1'
    'standing_bending'
%     'front_lift'
    'right_left_lift'
    'left_right_lift'
    'sitting_bend'
    'sit_stand_normal'
    'sit_stand_fast'
    'walk_3kmh'
    'walk_5kmh'
    'jog_10kmh'
    'step_up_down'
    'figure8_walking'   % use for mag correction paper later
    'og_walk_sss'       % use for mag correction paper later
    'og_jog_sss'        % use for mag correction paper later
    'standing_ref2'};

DO = load(saveDataName);
data = DO.data;
for ii = 2:17%16%2:17 % aligning with the rows in 'raw'
    condition = raw{ii,1};
    filenumber = raw{ii,s+1};
    c3dname=['Trial',pad(num2str(filenumber),2,'left','0'),'.c3d'];
%     c3dname=['Trial',z_pad(num2str(filenumber),2),'.c3d'];
    matname=[c3dname(1:end-3),'mat'];

    condiFname = [pathn,'s',num2str(s),'_',condition,'_190.mat'];
    % Import the c3d:
    
            
        [x,y,z,Fs,pointsLabel,unlabelled,time]=Import_c3d_data_osim_dorsaVi([pathn,c3dname]);


        % Nan dex at 200 Hz
        isnan_index = isnan(x);
        % downsample these indices. These are logical vectors so this
        % operation isn't defined. instead, transform to double and then
        % define any nandex which has a value greater than zero (I.E. it
        % it has a bad sample on at least one side of it at original
        % frequency)

        t=time;
        ti=0:1/190:t(end);

        nd = double(isnan_index);
        ndi = interp1(time,nd,ti);
        isnan_index = logical(ndi);

        nandex.(condition) = isnan_index;

        
        
        % Down sample from 200 to 190 Hz
        % In the previous implementation of this, setting the NaNs to zero
        % and then interpolating resulted in high frequency oscillations
        % near NaN segments. This leads to unecessary errors. The method
        % here interpolates across each good segment of data separately to
        % avoid this.
        %
        % Ignore missing (NaN) values and interpolate over them using
        % interpMethod.
        % If NaNs are located at beginning or end, we need to extrapolate
        % using extrapMethod.
        numMarkers = size(z,2);
        xi = NaN(length(ti),numMarkers);
        yi = NaN(length(ti),numMarkers);
        zi = NaN(length(ti),numMarkers);
        interpMethod = 'pchip';
        extrapMethod = 'nearest';
        for mm = 1 : numMarkers
           dex = ~isnan(z(:,mm));
           if ~any(dex)
               continue
           end
           xi(:,mm) = interp1(t(dex),x(dex,mm),ti,interpMethod,NaN);
           yi(:,mm) = interp1(t(dex),y(dex,mm),ti,interpMethod,NaN);
           zi(:,mm) = interp1(t(dex),z(dex,mm),ti,interpMethod,NaN);
        end
        xi = fillmissing(xi,extrapMethod);
        yi = fillmissing(yi,extrapMethod);
        zi = fillmissing(zi,extrapMethod);


        % Convert to meters  
        xi=xi/1000;
        yi=yi/1000;
        zi=zi/1000;

%         
%         figure
%         plot(data.vicon.(condition).time,data.vicon.(condition).z(:,1))
%         hold on
%         plot(ti,zi(:,1))
%         plot(ti(isnan_index(:,1)),zi(isnan_index(:,1)),'g.')
%         legend('Old','New','Ignore')
%         xlabel('Time (seconds)')
%         ylabel('Vertical Marker Position (m)')
        
        
        % Store data and rewrite condition data file
        DC = load(condiFname);
        DC.vicondata.x = xi;
        DC.vicondata.y = yi;
        DC.vicondata.z = zi;
        DC.vicondata.nandex = isnan_index;
        save(condiFname,'-struct','DC')

        % Store data for rewriting big data file (all conditions)
        data.vicon.(condition).x = xi;
        data.vicon.(condition).y = yi;
        data.vicon.(condition).z = zi;
end
save(saveDataName,'data');
save(nandexFileName,'nandex');

end