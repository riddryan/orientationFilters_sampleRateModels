function mDisturbed = simulateMagneticDisturbance(time,m,seed)
% m is the input signal undisturbed magnetometer data

if nargin < 3
    seed = 0;
end
N = length(time);
sampleRate = N/(time(end) - time(1));

dtPerturb = 3;
pertTime = 1;

tCycle = dtPerturb * 5;

Ttotal = time(end);
nPertCycles = floor(Ttotal/tCycle);

% pertMax = 0.3 * nanmean(m);
pertMax = [1 1 1];

mPert = zeros(size(m));
dex = 1 : sampleRate*pertTime;
for i = 1 : nPertCycles
    X = genSigs(seed,sampleRate,pertTime,pertMax);
    
    fnames = fieldnames(X);
    for j = 1 : length(fnames)
    if dex(end) < N
        mPert(dex,:) = X.(fnames{j});
    end
    dex = dex + round(dtPerturb*sampleRate);
    end
end

mDisturbed = mPert + m;

end


function X = genSigs(seed,sampleRate,pertTime,pertMax)
if nargin < 4
    pertMax = 5;
end
% minPeriod = 0.1;
% possiblePeriods = pertTime*[1 1/2 1/3];
possiblePeriods = pertTime*[1 1/2];
%% Saw-tooth
rng(10*seed + 1);
sawToothAmplitude = pertMax*rand(1);
rng(20*seed + 1);

sawToothPeriod = possiblePeriods(randi(length(possiblePeriods),1));
% sawToothPeriod = (minPeriod - 0.1)*rand(1) + minPeriod;
sawToothTime = pertTime;


nPeriod = sawToothPeriod * sampleRate;
numPeriods = sawToothTime/sawToothPeriod;

t = linspace(0,2*pi * numPeriods,round(nPeriod * numPeriods));
xSaw = sawToothAmplitude.*sawtooth(t).';

%% Triangle
rng(10*seed + 2);
triangleAmplitude = pertMax*rand(1);
rng(20*seed + 2);
trianglePeriod = possiblePeriods(randi(length(possiblePeriods),1));
% trianglePeriod = (minPeriod - 0.1)*rand(1) + minPeriod;
triangleTime = pertTime;

nPeriod = trianglePeriod * sampleRate;
numPeriods = triangleTime/trianglePeriod;

t = linspace(0,2*pi * numPeriods,round(nPeriod * numPeriods));
xTriangle = triangleAmplitude.*sawtooth(t,0.5).';
%% Square wave
rng(10*seed + 3);
squareAmplitude = pertMax*rand(1);
rng(20*seed + 3);
squarePeriod = possiblePeriods(randi(length(possiblePeriods),1));
% squarePeriod = (minPeriod - 0.1)*rand(1) + minPeriod;
squareTime = pertTime;

nPeriod = squarePeriod * sampleRate;
numPeriods = squareTime/squarePeriod;

t = linspace(0,2*pi * numPeriods,round(nPeriod * numPeriods));
xSquare = squareAmplitude.*square(t,0).';
%% Sin-wave 1: slower
rng(10*seed + 4);
sin1Amplitude = pertMax*rand(1);
rng(10*seed + 5);
sin1Frequency = 1/(pertTime);
possibleFreqs = 1 ./ possiblePeriods;
sin1Frequency = possibleFreqs(1);
% sin1Frequency = 2 + 8*rand(1);
sin1Time = pertTime;

t = linspace(0,sin1Time,round(sin1Time*sampleRate));

xSin1 = sin1Amplitude.*sin(2*pi*sin1Frequency*t).';

%% Sin-wave 2: faster
rng(10*seed + 6);
sin2Amplitude = pertMax*rand(1);
rng(10*seed + 7);
% possibleFreqs = 1/(pertTime/2);
% possibleFreqs = possibleFreqs*[2 3 4];
possibleFreqs = 1 ./ possiblePeriods;
sin2Frequency = possibleFreqs(randi(length(possibleFreqs),1));
sin2Frequency = possibleFreqs(2);
% sin2Frequency = 0.1 + 0.9*rand(1);
sin2Time = pertTime;

t = linspace(0,sin2Time,round(sin2Time*sampleRate));

xSin2 = sin2Amplitude.*sin(2*pi*sin2Frequency*t).';

%%
X.saw = xSaw;
X.triangle = xTriangle;
X.square = xSquare;
X.sin1 = xSin1;
X.sin2 = xSin2;
end


