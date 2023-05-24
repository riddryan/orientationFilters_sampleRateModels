function [euler_error,rmse,sse] = calcFilterError(qEst,qTruth,bads)
if nargin < 3
    bads = [];
end

q_error = quaternProd(qTruth,quaternConj(qEst));
euler_error = eulerd(quaternion(q_error),'ZYX','frame');
% 
% eaTruth = quatern2euler(quaternConj(qTruth))*180/pi;
% eaEst = quatern2euler(quaternConj(qEst)*180/pi;
% eulerError = eaEst - eaTruth;

euler_error(bads,:) = [];
rmse = rms(euler_error);
sse = sum(rmse);