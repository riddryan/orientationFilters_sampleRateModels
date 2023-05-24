function plotEstimate(ahrs,q,data,dataOrig,opts)
eaStar = eulerd(quaternion(q),'ZYX','point');

% eaGroundOrig = eulerd(quaternion(dataOrig.ground_truth),'ZYX','point');
eaGround = eulerd(quaternion(data.ground_truth),'ZYX','point');

clf;
plot(data.time,eaGround,'LineWidth',1.2)
hold on
plot(data.time,eaStar,'k--')

% plot(dataOrig.time,eaGroundOrig,'b');
% plot(ea0,'b');
% title([class(ahrs) ' ' opts.conditions{opts.condition} ' ' num2str(round(opts.Fs)) ' Hz'],'Interpreter','none');
% legend('ground truth','estimate')
ylabel('Euler Angles');
drawnow;