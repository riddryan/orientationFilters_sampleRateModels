function [offset,I,regionDex] = autoDetectGyroBiasRegions(gyro,threshFactor,numRegions,regionLengthMin,plotRegions,iter)
% Automatically detect regions in which there is low activity for the
% purposes of estimating the bias in a gyroscope measurement.

%% Parse inputs
if nargin < 2 || isempty(threshFactor)
    threshFactor = 0.05;
end
if nargin < 3
    numRegions = 10;
end
if nargin < 4
    regionLengthMin = 100;
end
if nargin < 5
    plotRegions = false;
end
if nargin < 6
    iter = 1;
end

maxIterations = 10;
iterIncreaseFactor = 1.05;

if iter > maxIterations
    error('Max iterations searching for gyro bias regions reached. Check data or increase baseline threshold.')
end

%% Pre-processing

gyroMag = vecnorm(gyro,2,2);
N = length(gyroMag);
sg = std(gyroMag);
% mg = min(abs(gyroMag));
mg = median(abs(gyroMag));
nSamp = floor(N/numRegions);

%% Loop through each region
dex = 0;
count = 0;
regionDex = {};
I = [];
offset = [];
for i = 1 : numRegions

    % Pick out the indices for the sub-region of data
    if i == numRegions
        dex = dex(end)+1:N;
    else
        dex = dex(end)+1:dex(end)+nSamp;
    end

    % Low values for gyro
        lows = (gyroMag(dex) - mg)/sg < threshFactor;

    % Find all runs (where gyro is all high, or all low values)
    [~,n,bi] = RunLength(lows);

    % Remove runs where the gyro is a high value
    rmDex = lows(bi) == 0;
    n(rmDex) = [];
    bi(rmDex) = [];

    % If there are no runs of low values, don't continue
    if isempty(bi)
        continue
    end

    % Find the longest run of low values
    [~, index] = max(n);
    
    % If the run isn't long enough, don't continue
    if n(index) < regionLengthMin
        continue
    end

    % Store the indices for the longest run of low values, the average
    % index, and the mean offset of the gyro
    count = count + 1;
    regionDex{count} = dex(bi(index):bi(index) + n(index) - 1);
    I(count,1) = mean(regionDex{count});
    offset(count,:) = mean(gyro(regionDex{count},:));
end

% Recursively make threshold bigger if we don't get any regions to calc
% offset
if isempty(offset)
    iter = iter + 1;
    newThresh = threshFactor * iterIncreaseFactor;
    [offset,I,regionDex] = autoDetectGyroBiasRegions(gyro,newThresh,numRegions,regionLengthMin,false,iter);
end



%%  Plot results
if ~plotRegions
    return
end

figure;
for dim = 1 : 3
    subplot(3,1,dim)
plot(gyro(:,dim));
hold on
plot(I(:),offset(:,dim),'k.-','MarkerSize',12);
YV = get(gca,'YLim');
for j = 1 : length(regionDex)
    XV = [regionDex{j}(1) regionDex{j}(end)];
    hp = patch([XV(1) XV(1) XV(2) XV(2)],[YV(1) YV(2) YV(2) YV(1)],'k','LineStyle','none');
    hp.FaceAlpha = 0.8;
end
end