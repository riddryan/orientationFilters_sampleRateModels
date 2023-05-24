clear, clc, close all

% test signals' parameters
fs = 44100;
f0 = 50;
T = 0.1;
N = round(T*fs);
t = (0:N-1)/fs;

% generation of test signals with -pi/6 rad (-30 deg) phase difference
x = sin(2*pi*f0*t) + 0.05*randn(1, N);
y = 0.5*sign(sin(2*pi*f0*t - pi/6)) + 0.05*randn(1, N);

% phase difference measurement
PhDiff = phdiffmeasure(x, y);
PhDiff = rad2deg(PhDiff);

% display the phase difference
disp(['Phase difference Y->X = ' num2str(PhDiff) ' deg'])
commandwindow

% plot the signals
figure(1)
plot(t, x, 'b', 'LineWidth', 1.5)
grid on
hold on
plot(t, y, 'r', 'LineWidth', 1.5)
xlim([0 T])
ylim([-1.5 1.5])
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14)
xlabel('Time, s')
ylabel('Amplitude, V')
title('Two signals with phase difference')
legend('First signal X', 'Second signal Y')