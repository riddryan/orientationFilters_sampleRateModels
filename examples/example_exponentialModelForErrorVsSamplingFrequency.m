
fs = linspace(0,200,1000);
dt = 1./fs;

% Error Model
% Model parameters for the proposed GDF, table 4
E0 = 6.44;
EINF = 1.05;
lambda = 0.08;

modelledError = (E0-EINF)*exp(-lambda./dt)+EINF;

figure;
subplot(211)
plot(fs,modelledError)
xlabel('Sampling Freq (Hz)')
ylabel('Error (degrees)')
title('Modelled Error for proposed GDF')

% Optimal gain model
BINF_GYRO = 0.9937;
C_GYRO = -1.064;
LAMBDA_GYRO = 0.379;
BETA_GYRO = C_GYRO*exp(-LAMBDA_GYRO./dt) + BINF_GYRO;

BINF_ACC = 0.0507;
C_ACC = 0.160;
LAMBDA_ACC = 0.025;
BETA_ACC = C_ACC*exp(-LAMBDA_ACC./dt) + BINF_ACC;

BINF_MAG = -0.0013;
C_MAG = 0.201;
LAMBDA_MAG = 0.064;
BETA_MAG = C_MAG*exp(-LAMBDA_MAG./dt) + BINF_MAG;

subplot(212)
plot(fs,BETA_GYRO);
hold on
plot(fs,BETA_ACC);
plot(fs,BETA_MAG);
xlabel('Sampling Freq (Hz)')
ylabel('Gain Value')

legend('Gyro','Acc','Mag')
title('Optimal Gain Model')
