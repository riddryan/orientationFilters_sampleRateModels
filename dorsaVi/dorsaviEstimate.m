function [q,cost] = dorsaviEstimate(dorsaviAttitude,qmocap)
eaTruth = eulerd(quaternion(qmocap),'ZYX','point');

% Get dorsaVi sensor estimate of attitude
gimcRimu=quat2dcm(quaternConj(dorsaviAttitude));
% in VU notation
gimcRimu=reshape(gimcRimu,9,size(dorsaviAttitude,1))';
% swap the axes x = -z; y = -y; z = -x
gimcRimu=[-gimcRimu(:,7:9) -gimcRimu(:,4:6) -gimcRimu(:,1:3)];

eaDorsavi = Euler_all('yxz', gimcRimu);
% Set first frame to be equal b/w mocap and dorsaVi estimate
eaDorsavi = eaDorsavi - eaDorsavi(1,:) + eaTruth(1,:);
cost = sum(rms(eaTruth - eaDorsavi));
q = compact(quaternion(eaDorsavi,'eulerd','ZYX','point'));