function [qPredicted,pPredicted] = FastKF_update(q,gyro,acc,mag,gains,dt,P)
Sigma_g = reshape(gains(1:9),3,3);
Sigma_a = reshape(gains(10:18),3,3);
Sigma_m = reshape(gains(19:27),3,3);

%% Normalize
acc = acc / norm(acc);
mag = mag / norm(mag);

%%

[qMeasured,Jacobian] = processMeasurement(q,acc,mag);
R = measurementCovariance(Sigma_a,Sigma_m,Jacobian);



[qPredicted,A] = stateFcn(q.',gyro,dt);
Q = stateCovariance(q,dt,Sigma_g);


[qEstimated,PEstimated] = kalmanUpdate(qPredicted,qMeasured,A,Q,R,P);

% Store current estimate of orientation for next iteration
qPredicted = qEstimated.';
% Store current estimate of state covariance for next iteration
pPredicted = PEstimated;
end

function [qy,Jacob] = processMeasurement(q,acc,mag)
% Measurement (qy). This code works on the premise that gravity
% should be in the z-direction should be [0 0 1]', and the
% earth's magnetic field should be in the x-direction (heading)
% and z-direction (inclination), ie m = [mx 0 mz]'. This code
% transforms the sensor readings using the current orientation
% (obj.Quaternion) and enforces these constraints on the
% accelerometer and magnetometr readings. This produces a new
% measurement quaternion qy. It also returns the jacobian
% (Jacob) so that the measurement noise can be transformed by
% the identical transformation.
[qy,Jacob] = measurement_quaternion_acc_mag(acc,mag, q.');
% qy=qy./norm(qy);
end
function [qPredicted,A] = stateFcn(q,gyro,dt)

% Linear state dynamics (A). The state is the integrated
% (across dt) gyroscope measurements. Returns predicted
% state at next time step (qPredicted) based on the state
% dynamics (A), which are linear dynamics in which the
% gyroscope measurement is integrated across the time step.
wx = gyro(1);
wy = gyro(2);
wz = gyro(3);

omega4=[0,-wx,-wy,-wz;
    wx,0,wz,-wy;
    wy,-wz,0,wx;
    wz,wy,-wx,0];
A = eye(4)+dt/2*omega4;

qPredicted = A*q;
end
function Q = stateCovariance(q,dt,Sigma_g)
qk_1=q.';
q0=qk_1(1);
q1=qk_1(2);
q2=qk_1(3);
q3=qk_1(4);
Dk=[q1 q2 q3;
    -q0 -q3 -q2;
    q2 -q0 -q1;
    -q2 q1 -q0];
Q = dt*dt/4*(Dk*Sigma_g*Dk.');
end

function R = measurementCovariance(Sigma_a,Sigma_m,J)
R_sensor = blkdiag(Sigma_a,Sigma_m);
R = J * R_sensor * J.';
end

%%

function [q,Jacob] = measurement_quaternion_acc_mag(acc,mag,q_)
% measurement_quaternion_acc_mag estimates the orientation of
% the sensor based on the accelerometer and magnetometer
% measurements.
%
% [Q,JACOB]=measurement_quaternion_acc_mag(ACC,MAG,Q_)
%
% Inputs
% ACC is a 1x3 accelerometer reading
% MAG is a 1x3 magnetomer reading
% Q_ is a 1x4 current estimate of orientation in quaternion
% form
%
% Outputs
% Q is the quaternion estimate of the current orientation based
% on the detected direction of gravity and earth's magnetic
% field
% JACOB is the jacobian used to transform the magnetometer and
% accelorometer readings from their nominal coordinate frame
% into the quaternion orientation estimate.

ax=acc(1);  ay=acc(2);  az=acc(3);
mx=mag(1);  my=mag(2);  mz=mag(3);

mD=dot(acc,mag);
mN=sqrt(1-mD^2);

q0=q_(1);   q1=q_(2);   q2=q_(3);   q3=q_(4);

q=zeros(4,1);
Jacob=zeros(4,6);

q(1)= (ay*mD*my + (1 + az)*(1 + mN*mx + mD*mz) + ax*(mD*mx - mN*mz))*q0 + ((mD + az*mD - ax*mN)*my + ay*(1 + mN*mx - mD*mz))*q1 + ...
    (ay*mN*my + ax*(-1 + mN*mx + mD*mz) + (1 + az)*(-(mD*mx) + mN*mz))*q2 + (-((ax*mD + mN + az*mN)*my) + ay*(mD*mx + mN*mz))*q3;

q(2)= ((mD - az*mD - ax*mN)*my + ay*(1 + mN*mx + mD*mz))*q0 + (ay*mD*my - (-1 + az)*(1 + mN*mx - mD*mz) + ax*(mD*mx + mN*mz))*q1 + ...
    ((ax*mD + mN - az*mN)*my + ay*(-(mD*mx) + mN*mz))*q2 + (-(ay*mN*my) + ax*(1 - mN*mx + mD*mz) - (-1 + az)*(mD*mx + mN*mz))*q3;

q(3)= (-(ay*mN*my) - ax*(1 + mN*mx + mD*mz) + (-1 + az)*(mD*mx - mN*mz))*q0 + ((-(ax*mD) + mN - az*mN)*my + ay*(mD*mx + mN*mz))*q1 + ...
    (ay*mD*my + (-1 + az)*(-1 + mN*mx + mD*mz) + ax*(mD*mx - mN*mz))*q2 + ((mD - az*mD + ax*mN)*my + ay*(1 - mN*mx + mD*mz))*q3;

q(4)= ax*(q1 + mN*mx*q1 + mN*my*q2 + mN*mz*q3 + mD*(my*q0 - mz*q1 + mx*q3)) + (1 + az)*(mD*mx*q1 + mD*my*q2 + q3 + mD*mz*q3 - mN*(my*q0 - mz*q1 + mx*q3)) + ...
    ay*(mN*mz*q0 + mN*my*q1 + q2 - mN*mx*q2 - mD*(mx*q0 + mz*q2 - my*q3));



Jacob(1,1)= -q2 - mN*(mz*q0 + my*q1 - mx*q2) + mD*(mx*q0 + mz*q2 - my*q3);
Jacob(1,2)= q1 + mN*mx*q1 + mN*my*q2 + mN*mz*q3 + mD*(my*q0 - mz*q1 + mx*q3);
Jacob(1,3)= q0 + mN*mx*q0 + mD*mz*q0 + mD*my*q1 - mD*mx*q2 + mN*mz*q2 - mN*my*q3;
Jacob(1,4)= (ax*mD + mN + az*mN)*q0 + ay*mN*q1 + (-((1 + az)*mD) + ax*mN)*q2 + ay*mD*q3;
Jacob(1,5)= ay*mD*q0 + (mD + az*mD - ax*mN)*q1 + ay*mN*q2 - (ax*mD + mN + az*mN)*q3;
Jacob(1,6)= mD*(q0 + az*q0 - ay*q1 + ax*q2) + mN*(-(ax*q0) + q2 + az*q2 + ay*q3);

Jacob(2,1)= q3 - mN*(my*q0 - mz*q1 + mx*q3) + mD*(mx*q1 + my*q2 + mz*q3);
Jacob(2,2)= q0 + mN*mx*q0 + mD*mz*q0 + mD*my*q1 - mD*mx*q2 + mN*mz*q2 - mN*my*q3;
Jacob(2,3)= -((1 + mN*mx)*q1) - mD*(my*q0 - mz*q1 + mx*q3) - mN*(my*q2 + mz*q3);
Jacob(2,4)= ay*(mN*q0 - mD*q2) - (-1 + az)*(mN*q1 + mD*q3) + ax*(mD*q1 - mN*q3);
Jacob(2,5)= mD*(q0 - az*q0 + ay*q1 + ax*q2) - mN*(ax*q0 + (-1 + az)*q2 + ay*q3);
Jacob(2,6)= ay*(mD*q0 + mN*q2) + mD*((-1 + az)*q1 + ax*q3) + mN*(ax*q1 + q3 - az*q3);

Jacob(3,1)= -((1 + mN*mx + mD*mz)*q0) - mD*my*q1 + mD*mx*q2 - mN*mz*q2 + mN*my*q3;
Jacob(3,2)= q3 - mN*(my*q0 - mz*q1 + mx*q3) + mD*(mx*q1 + my*q2 + mz*q3);
Jacob(3,3)= -q2 - mN*(mz*q0 + my*q1 - mx*q2) + mD*(mx*q0 + mz*q2 - my*q3);
Jacob(3,4)= mD*((-1 + az)*q0 + ay*q1 + ax*q2) - mN*(ax*q0 + q2 - az*q2 + ay*q3);
Jacob(3,5)= ay*(-(mN*q0) + mD*q2) - (-1 + az)*(mN*q1 + mD*q3) + ax*(-(mD*q1) + mN*q3);
Jacob(3,6)= mN*(q0 - az*q0 + ay*q1) - ax*(mD*q0 + mN*q2) + mD*((-1 + az)*q2 + ay*q3);

Jacob(4,1)= q1 + mN*mx*q1 + mN*my*q2 + mN*mz*q3 + mD*(my*q0 - mz*q1 + mx*q3);
Jacob(4,2)= q2 + mN*(mz*q0 + my*q1 - mx*q2) - mD*(mx*q0 + mz*q2 - my*q3);
Jacob(4,3)= q3 - mN*(my*q0 - mz*q1 + mx*q3) + mD*(mx*q1 + my*q2 + mz*q3);
Jacob(4,4)= -(ay*(mD*q0 + mN*q2)) + ax*(mN*q1 + mD*q3) + (1 + az)*(mD*q1 - mN*q3);
Jacob(4,5)= (1 + az)*(-(mN*q0) + mD*q2) + ax*(mD*q0 + mN*q2) + ay*(mN*q1 + mD*q3);
Jacob(4,6)= ay*(mN*q0 - mD*q2) + (1 + az)*(mN*q1 + mD*q3) + ax*(-(mD*q1) + mN*q3);

Jacob=Jacob.*0.25;

end
function [qEstimated,PEstimated] = kalmanUpdate(qPredicted,qMeasured,A,Q,R,P)
% Propagate process noise
P_ = A*P*A.' + Q;
% Kalman Gain
L = P_ / (P_ + R);
% Correct current estimate of state
qEstimated = qPredicted + L*(qMeasured - qPredicted);
% qEstimated = qPredicted + L*(qMeasured);
% Normalize quaternion
qEstimated = qEstimated./norm(qEstimated);
% Correct state covariance
PEstimated = (eye(4) - L)*P_;
end