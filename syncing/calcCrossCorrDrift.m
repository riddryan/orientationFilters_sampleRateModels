function [delays,dtimes] = calcCrossCorrDrift(A1,A2,Nsub,FS,minThresh,maxDelay)
% Return estimate delays between the two signals A1 and A2 across time
% (times given by output variable dtimes). The signals are divided into
% chunks of Nsub length long for the cross correlation. The delay for each
% chunk is only output if the max value of each signal exceeds
% a threshold specified by minThresh for each signal during that chunk. The
% threshold gives how many standard deviations above the overall standard
% deviation the max for that chunk has to be to be counted for the delay
% calculation. This is done to avoid calculating a delay during periods of
% rest which would throw off the calculation. 
% 
%  If the data is busier than sporadic max peaks of movement then this
%  criterion could be changed to be based on only choosing segments where
%  there is a lot of movement (high std) instead of sporadic peaks (high
%  max value).
%
% Also this code assumes the signals are close enough together in time that
% we can look at the same indices for each signal for the comparison. If
% they are not close, try resampling first to something close, or choose
% fewer segments to look at (bigger Nsub).


if nargin < 5
    minThresh = 8;
end
if nargin < 6
    maxDelay = [];
end

nsampmin = min([length(A1) length(A2)]);

Nseg = ceil(nsampmin/Nsub);

% Nseg = 5;
% Nsub = round(nsampmin/Nseg);

s1 = std(A1);
s2 = std(A2);

delAP = NaN(Nseg,1);
gdex = false(Nseg,1);
for i = 1 : Nseg
    dex = (i-1)*Nsub+1 : min(nsampmin,i*Nsub);
    x2 = A2(dex);
    x1 = A1(dex);
    x2 = x2 - mean(x2);
    x1 = x1 - mean(x1);



    w1 = max(x1)/s1;
    w2 = max(x2)/s2;
    gdex(i) = w1 > minThresh & w2 > minThresh;

%     gdex(i) = std(x1) > s1 & std(x2) > s2;
% gdex(i) = true;
    if gdex(i)
        delAP(i) = finddelay(x2,x1,maxDelay);
    end
end

dtimes = (find(gdex)*Nsub - Nsub/2)/FS;
delays = delAP(gdex);

if length(delays) > 1
    p = polyfit(dtimes,delays,1);
    pDelays = polyval(p,dtimes);

    [~,rmDex] = rmoutliers(delays-pDelays,'median');
    delays = delays(~rmDex);
    dtimes = dtimes(~rmDex);

    if length(delays) > 1
    p = polyfit(dtimes,delays,1);
    pDelays = polyval(p,dtimes);

    [~,rmDex] = rmoutliers(delays-pDelays,'median');
    delays = delays(~rmDex);
    dtimes = dtimes(~rmDex);
    end
end