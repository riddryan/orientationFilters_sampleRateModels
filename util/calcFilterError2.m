function [euler_error,rmse,sse] = calcFilterError2(qEst,qTruth,bads)
if nargin < 3
    bads = [];
end

% q_error = quaternProd(qTruth,quaternConj(qEst));
% euler_error = eulerd(quaternion(q_error),'ZYX','point');
% 
eaTruth = quatern2euler(quaternConj(qTruth))*180/pi;


eaEst = quatern2euler(quaternConj(qEst))*180/pi;
eaEst(:,2) = -eaEst(:,2);

euler_error = eaEst - eaTruth;

euler_error(bads,:) = [];
rmse = rms(euler_error);
sse = sum(rmse);