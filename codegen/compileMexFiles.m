% %%
%
% q = ones(1,4);
% gyro = ones(1,3);
% acc = ones(1,3);
% mag = ones(1,3);
% gains = ones(1,3);
% dt = 1;
% normStepSize = false;
% ex = {q,gyro,acc,mag,gains,dt,normStepSize};
%
% codegen -d codegen\mex RiddickAHRS_update -args ex
%
% %%
% maxSamples = 1000000;
%
% filter = coder.typeof('FastKF',[1 30],[0 1]);
% q0 = ones(1,4);
% gyro = coder.typeof(0,[maxSamples 3],[1 0]);
% acc = coder.typeof(0,[maxSamples 3],[1 0]);
% mag = coder.typeof(0,[maxSamples 3],[1 0]);
% gains = coder.typeof(1,[1 40],[0 1]);
% dt = 1;
% normStepSize = false;
% indexOffset = 0;
% P = coder.typeof(1,[4 4],[1 1]);
% ex = {filter,q0,gyro,acc,mag,gains,dt,normStepSize,indexOffset,P};
%
% codegen -d codegen\mex estimateOrientation_Fast -args ex

%% Orientation Estimation loop
maxSamples = 20000000;

filter = coder.typeof('FastKF',[1 30],[0 1]);
q0 = ones(1,4);
gyro = coder.typeof(0,[maxSamples 3],[1 0]);
acc = coder.typeof(0,[maxSamples 3],[1 0]);
mag = coder.typeof(0,[maxSamples 3],[1 0]);
gains = coder.typeof(1,[1 100],[0 1]);
dt = 1;
normStepSize = false;
P = coder.typeof(1,[4 4],[1 1]);
intOrder = 1;
ex = {filter,q0,gyro,acc,mag,gains,dt,normStepSize,P,intOrder};

codegen -d codegen\mex estimateOrientation_Fast2 -args ex

%% Filter error calculation
if 1
    maxSamples = 1000000;

    qEst = coder.typeof(0,[maxSamples 4],[1 0]);
    qTruth = qEst;
    bads = coder.typeof(false,[maxSamples 1],[1 0]);
    ex = {qEst,qTruth,bads};
    codegen -d codegen\mex calcFilterError -args ex
end