% A basic example of why including higher order terms for integrating the
% gyroscape can be useful (mostly at low frequencies).

time = linspace(-1,1,100);
y = time.^2;
DT = 0.6;

figure;
plot(time,y)
%% Measurement
tMeas = [-DT 0];
yDotMeas = 2*tMeas;
yMeas = tMeas.^2;
hold on
plot(tMeas,yMeas,'k.','MarkerSize',20)
%% First Order Model
gradient = yDotMeas(2);
tPred = DT;
yPred = yMeas(2) + gradient*tPred;

plot(tPred,yPred,'r.','MarkerSize',20)
%% Second Order model



hess = (yDotMeas(2) - yDotMeas(1))/DT;

tPred = DT;
yPred = yMeas(2) + gradient*tPred + 1/2*hess*tPred.^2;

yEst = [yMeas yPred];
hold on
plot(tPred,yPred,'g.','MarkerSize',20)

legend('true','Measured','First Order Predicted','Second Order Predicted')