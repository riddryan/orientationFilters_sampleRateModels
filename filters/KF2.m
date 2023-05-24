classdef KF2 < AHRSfilter
    properties
        % State estimation covariance
        P = 0.00*eye(4);

        use3x3covs = true;
    end
    properties (Dependent)
        % 3x3 Gyroscope covariance matrix.
        %
        % Can be set:
        % 1) by the scalar to assume a diagonal matrix with
        % each diagonal element equal to KF.Sigma_g_scalar
        % or
        % 2) by setting individual elements of the matrix such as
        % KF.Sigma_g11 through .KF.Sigma_g44 where the 1st number
        % represents the row and the second number represents the column
        Sigma_g

        % 3x3 Accelerometer covariance matrix
        Sigma_a

        % 3x3 Magnetometer covariance matrix
        Sigma_m



    end
    methods
        function obj = KF(varargin)
        end
        function P = allParams(obj)
            P = [obj.Sigma_g(:);obj.Sigma_a(:);obj.Sigma_m(:)].';
        end
        function [qEstimated,PEstimated] = update(obj,Gyroscope,Accelerometer,Magnetometer)
            if nargin > 1
                obj.Gyroscope = Gyroscope;
            end
            if nargin > 2
                obj.Accelerometer = Accelerometer;
            end
            if nargin > 3
                obj.Magnetometer = Magnetometer;
            end
            q = obj.Quaternion;
            acc = obj.Accelerometer ./ norm(obj.Accelerometer);
            mag = obj.Magnetometer ./ norm(obj.Magnetometer);


            q1 = q(1);q2 = q(2);q3 = q(3);q4 =q(4);
            ax = acc(1); ay=acc(2); az= acc(3);

%             qa = [q1
%                 q2 - ax - 2*q1*q3 + 2*q2*q4
%                 q3 - ay + 2*q1*q2 + 2*q3*q4
%                 q1^2 - q2^2 - q3^2 + q4^2 + q4 - az];
% 
%             Ja = [    1,        0,        0,        0
%                 -2*q3, 2*q4 + 1,    -2*q1,     2*q2
%                 2*q2,     2*q1, 2*q4 + 1,     2*q3
%                 2*q1,    -2*q2,    -2*q3, 2*q4 + 1];
% 
% 
% 
%             Hacc = [ az,  ay, -ax,  0
%                 ay, -az,   0, ax
%                 -ax,   0, -az, ay
%                 0,  ax,  ay, az];



            dqAcc = [2*q1^3 + 2*q1*q2^2 + 2*q1*q3^2 + 2*q1*q4^2 - 2*az*q1 - 2*ay*q2 + 2*ax*q3
                2*q1^2*q2 - 2*ay*q1 + 2*q2^3 + 2*q2*q3^2 + 2*q2*q4^2 + 2*az*q2 - 2*ax*q4
                2*q1^2*q3 + 2*ax*q1 + 2*q2^2*q3 + 2*q3^3 + 2*q3*q4^2 + 2*az*q3 - 2*ay*q4
                2*q1^2*q4 + 2*q2^2*q4 - 2*ax*q2 + 2*q3^2*q4 - 2*ay*q3 + 2*q4^3 - 2*az*q4];


%             Ja = [-2*q3,  2*q4, -2*q1, 2*q2
%                 2*q2,  2*q1,  2*q4, 2*q3
%                 2*q1, -2*q2, -2*q3, 2*q4];

            % Fa = [2*q2*q4 - 2*q1*q3 - ax
            %         2*q1*q2 - ay + 2*q3*q4
            % q1^2 - q2^2 - q3^2 + q4^2 - az];
            %
            % Ja = [-2*q3, 2*q2,  2*q1
            %  2*q4, 2*q1, -2*q2
            % -2*q1, 2*q4, -2*q3
            %  2*q2, 2*q3,  2*q4];


            % qa = [q1
            %     - ax*q1^2 - 2*az*q1*q3 + 2*ay*q1*q4 - ax*q2^2 - 2*ay*q2*q3 - 2*az*q2*q4 + q2 + ax*q3^2 + ax*q4^2
            %     - ay*q1^2 + 2*az*q1*q2 - 2*ax*q1*q4 + ay*q2^2 - 2*ax*q2*q3 - ay*q3^2 - 2*az*q3*q4 + q3 + ay*q4^2
            % - az*q1^2 - 2*ay*q1*q2 + 2*ax*q1*q3 + az*q2^2 - 2*ax*q2*q4 + az*q3^2 - 2*ay*q3*q4 - az*q4^2 + q4 + 1];
            %
            % Ja = [                          1,                               0,                               0,                               0
            % 2*ay*q4 - 2*ax*q1 - 2*az*q3, 1 - 2*ay*q3 - 2*az*q4 - 2*ax*q2,     2*ax*q3 - 2*ay*q2 - 2*az*q1,     2*ax*q4 + 2*ay*q1 - 2*az*q2
            % 2*az*q2 - 2*ay*q1 - 2*ax*q4,     2*ay*q2 - 2*ax*q3 + 2*az*q1, 1 - 2*ay*q3 - 2*az*q4 - 2*ax*q2,     2*ay*q4 - 2*ax*q1 - 2*az*q3
            % 2*ax*q3 - 2*ay*q2 - 2*az*q1,     2*az*q2 - 2*ay*q1 - 2*ax*q4,     2*ax*q1 - 2*ay*q4 + 2*az*q3, 1 - 2*ay*q3 - 2*az*q4 - 2*ax*q2];


            % Estimate of orientation from magnetometer. Orthogonal to
            % accelerometer estimate
            Vm = cross(acc, mag);
            Vm = Vm / norm(Vm);

            mx = Vm(1);my = Vm(2); mz = Vm(3);

%             mx = -mx;
%             mz = -mz;

%             qm = [q1
%                 q2 - mx + 2*q1*q4 + 2*q2*q3
%                 q1^2 - q2^2 + q3^2 + q3 - q4^2 - my
%                 q4 - mz - 2*q1*q2 + 2*q3*q4];
% 
%             Jm = [    1,        0,        0,        0
%                 2*q4, 2*q3 + 1,     2*q2,     2*q1
%                 2*q1,    -2*q2, 2*q3 + 1,    -2*q4
%                 -2*q2,    -2*q1,     2*q4, 2*q3 + 1];
% 
% 
%             Hmag = [my, -mz,  0,  mx
%                 mz, -my, mx,   0
%                 0,  mx, my,  mz
%                 mx,   0, mz, -my];



            dqMag =  [2*q1^3 + 2*q1*q2^2 + 2*q1*q3^2 + 2*q1*q4^2 - 2*my*q1 + 2*mz*q2 - 2*mx*q4
                2*q1^2*q2 + 2*mz*q1 + 2*q2^3 + 2*q2*q3^2 + 2*q2*q4^2 + 2*my*q2 - 2*mx*q3
                2*q1^2*q3 + 2*q2^2*q3 - 2*mx*q2 + 2*q3^3 + 2*q3*q4^2 - 2*my*q3 - 2*mz*q4
                2*q1^2*q4 - 2*mx*q1 + 2*q2^2*q4 + 2*q3^2*q4 - 2*mz*q3 + 2*q4^3 + 2*my*q4];

%             Jm =        [ 2*q4,  2*q3, 2*q2,  2*q1
%                 2*q1, -2*q2, 2*q3, -2*q4
%                 -2*q2, -2*q1, 2*q4,  2*q3];





            %             Hmag = -Hmag;


            % qm = [q1
            %     - mx*q1^2 - 2*mz*q1*q3 + 2*my*q1*q4 - mx*q2^2 - 2*my*q2*q3 - 2*mz*q2*q4 + q2 + mx*q3^2 + mx*q4^2
            % - my*q1^2 + 2*mz*q1*q2 - 2*mx*q1*q4 + my*q2^2 - 2*mx*q2*q3 - my*q3^2 - 2*mz*q3*q4 + q3 + my*q4^2 + 1
            %     - mz*q1^2 - 2*my*q1*q2 + 2*mx*q1*q3 + mz*q2^2 - 2*mx*q2*q4 + mz*q3^2 - 2*my*q3*q4 - mz*q4^2 + q4];

            % Jm = [                          1,                               0,                               0,                               0
            % 2*my*q4 - 2*mx*q1 - 2*mz*q3, 1 - 2*my*q3 - 2*mz*q4 - 2*mx*q2,     2*mx*q3 - 2*my*q2 - 2*mz*q1,     2*mx*q4 + 2*my*q1 - 2*mz*q2
            % 2*mz*q2 - 2*my*q1 - 2*mx*q4,     2*my*q2 - 2*mx*q3 + 2*mz*q1, 1 - 2*my*q3 - 2*mz*q4 - 2*mx*q2,     2*my*q4 - 2*mx*q1 - 2*mz*q3
            % 2*mx*q3 - 2*my*q2 - 2*mz*q1,     2*mz*q2 - 2*my*q1 - 2*mx*q4,     2*mx*q1 - 2*my*q4 + 2*mz*q3, 1 - 2*my*q3 - 2*mz*q4 - 2*mx*q2];


% JJ= [                                                                                    8*q4*(q1^2*q2 + mz*q1 + q2^3 + q2*q3^2 + q2*q4^2 + my*q2 - mx*q3),                           8*q1^2*q3*q4 + 4*mz*q1^2 + 8*my*q1*q2 - 8*mx*q1*q3 + 8*q2^2*q3*q4 - 4*mz*q2^2 + 8*q3^3*q4 - 4*mz*q3^2 + 8*q3*q4^3 - 4*mz*q4^2, - 4*q1^4 - 8*q1^2*q2^2 - 8*q1^2*q3^2 + 4*my*q1^2 - 8*mz*q1*q2 - 4*q2^4 - 8*q2^2*q3^2 - 4*my*q2^2 + 8*mx*q2*q3 - 4*q3^4 + 4*my*q3^2 + 4*q4^4 + 4*my*q4^2,                                                                                       8*q3*(q1^2*q2 - ay*q1 + q2^3 + q2*q3^2 + q2*q4^2 + az*q2 - ax*q4), - 4*q1^4 - 8*q1^2*q2^2 - 8*q1^2*q4^2 + 4*az*q1^2 + 8*ay*q1*q2 - 4*q2^4 - 8*q2^2*q4^2 - 4*az*q2^2 + 8*ax*q2*q4 + 4*q3^4 + 4*az*q3^2 - 4*q4^4 + 4*az*q4^2,                         8*q1^2*q3*q4 + 4*ay*q1^2 - 8*az*q1*q2 + 8*ax*q1*q4 + 8*q2^2*q3*q4 - 4*ay*q2^2 + 8*q3^3*q4 - 4*ay*q3^2 + 8*q3*q4^3 - 4*ay*q4^2
%                         8*q1^2*q2*q3 - 4*mx*q1^2 + 8*my*q1*q4 + 8*q2^3*q3 - 4*mx*q2^2 + 8*q2*q3^3 + 8*q2*q3*q4^2 - 8*mz*q2*q4 - 4*mx*q3^2 + 4*mx*q4^2, - 4*q1^4 - 8*q1^2*q2^2 - 8*q1^2*q4^2 + 4*my*q1^2 - 8*mz*q1*q2 + 8*mx*q1*q4 - 4*q2^4 - 8*q2^2*q4^2 - 4*my*q2^2 + 4*q3^4 - 4*my*q3^2 - 4*q4^4 - 4*my*q4^2,                           8*q1^2*q3*q4 - 4*mz*q1^2 - 8*my*q1*q2 + 8*q2^2*q3*q4 + 4*mz*q2^2 - 8*mx*q2*q4 + 8*q3^3*q4 - 4*mz*q3^2 + 8*q3*q4^3 - 4*mz*q4^2,                         - 8*q1^3*q3 - 4*ax*q1^2 - 8*q1*q2^2*q3 - 8*q1*q3^3 - 8*q1*q3*q4^2 + 8*ay*q1*q4 - 4*ax*q2^2 - 8*az*q2*q4 - 4*ax*q3^2 + 4*ax*q4^2,                           8*q1^2*q3*q4 + 4*ay*q1^2 - 8*az*q1*q2 + 8*ax*q1*q4 + 8*q2^2*q3*q4 - 4*ay*q2^2 + 8*q3^3*q4 - 4*ay*q3^2 + 8*q3*q4^3 - 4*ay*q4^2, 4*q1^4 + 8*q1^2*q2^2 + 8*q1^2*q4^2 - 4*az*q1^2 - 8*ay*q1*q2 + 4*q2^4 + 8*q2^2*q4^2 + 4*az*q2^2 - 8*ax*q2*q4 - 4*q3^4 - 4*az*q3^2 + 4*q4^4 - 4*az*q4^2
% 4*q1^4 + 8*q1^2*q3^2 + 8*q1^2*q4^2 - 4*my*q1^2 - 8*mx*q1*q4 - 4*q2^4 - 4*my*q2^2 + 4*q3^4 + 8*q3^2*q4^2 - 4*my*q3^2 - 8*mz*q3*q4 + 4*q4^4 + 4*my*q4^2,                         - 8*q1^2*q2*q3 - 4*mx*q1^2 - 8*mz*q1*q3 + 8*my*q1*q4 - 8*q2^3*q3 + 4*mx*q2^2 - 8*q2*q3^3 - 8*q2*q3*q4^2 + 4*mx*q3^2 + 4*mx*q4^2,                                                                                      -8*q2*(q1^2*q4 - mx*q1 + q2^2*q4 + q3^2*q4 - mz*q3 + q4^3 + my*q4),                         - 8*q1^2*q3*q4 - 4*ay*q1^2 + 8*az*q1*q2 - 8*ax*q1*q4 - 8*q2^2*q3*q4 + 4*ay*q2^2 - 8*q3^3*q4 + 4*ay*q3^2 - 8*q3*q4^3 + 4*ay*q4^2,                         - 8*q1^3*q3 - 4*ax*q1^2 - 8*q1*q2^2*q3 - 8*q1*q3^3 - 8*q1*q3*q4^2 + 8*ay*q1*q4 - 4*ax*q2^2 - 8*az*q2*q4 - 4*ax*q3^2 + 4*ax*q4^2,                                                                                     8*q3*(q1^2*q2 - ay*q1 + q2^3 + q2*q3^2 + q2*q4^2 + az*q2 - ax*q4)
%                       - 8*q1^3*q2 - 4*mz*q1^2 - 8*q1*q2^3 - 8*q1*q2*q3^2 - 8*q1*q2*q4^2 - 4*mz*q2^2 + 8*mx*q2*q4 - 4*mz*q3^2 + 8*my*q3*q4 + 4*mz*q4^2,                                                                                      -8*q1*(q1^2*q3 + q2^2*q3 - mx*q2 + q3^3 + q3*q4^2 - my*q3 - mz*q4),                         - 8*q1^3*q4 + 4*mx*q1^2 - 8*q1*q2^2*q4 - 8*q1*q3^2*q4 - 8*q1*q4^3 - 4*mx*q2^2 - 8*my*q2*q3 - 8*mz*q2*q4 + 4*mx*q3^2 + 4*mx*q4^2, - 4*q1^4 - 8*q1^2*q2^2 - 8*q1^2*q4^2 + 4*az*q1^2 + 8*ay*q1*q2 - 4*q2^4 - 8*q2^2*q4^2 - 4*az*q2^2 + 8*ax*q2*q4 + 4*q3^4 + 4*az*q3^2 - 4*q4^4 + 4*az*q4^2,                                                                                      -8*q3*(q1^2*q2 - ay*q1 + q2^3 + q2*q3^2 + q2*q4^2 + az*q2 - ax*q4),                       - 8*q1^3*q3 - 4*ax*q1^2 - 8*q1*q2^2*q3 - 8*q1*q3^3 - 8*q1*q3*q4^2 + 8*ay*q1*q4 - 4*ax*q2^2 - 8*az*q2*q4 - 4*ax*q3^2 + 4*ax*q4^2];
% 


            % Error vector
            E = -[dqAcc;dqMag];
             H = [eye(4);eye(4)];
             R = blkdiag(obj.Sigma_a,obj.Sigma_m);



% %             R = blkdiag(Ja*obj.Sigma_a(1:3,1:3)*Ja.',Jm*obj.Sigma_m(1:3,1:3)*Jm.');

% E = quaternProd(-dqAcc.',-dqMag.').';
% H = eye(4);
% Rsub = blkdiag(obj.Sigma_a(1:3,1:3),obj.Sigma_m(1:3,1:3));
% R = JJ * Rsub * JJ.';



            %             R(1,1) = 0;
            %             R(5,5) = 0;

            %             H = [eye(4);eye(4)];
            %             H = [Ja;Jm];

            %               H = [Hacc;Hmag];
%             H = Hacc + Hmag;
           


            % R = Ja*obj.Sigma_a*Ja.' + Jm*obj.Sigma_m*Jm.';
            % R = Ja*obj.Sigma_a(1:3,1:3)*Ja.' + Jm*obj.Sigma_m(1:3,1:3)*Jm.';

            % EA = obj.Sigma_a;EA(1,1) = 0;
            % EM = obj.Sigma_m;EM(1,1) = 0;
            % R = Ja*EA*Ja.' + Jm*EM*Jm.';

            [qPredicted,A] = stateFcn(obj,obj.Quaternion.');
            Q = stateCovariance(obj);

            % H = [eye(4);eye(4)];
            % Propagate process noise
%               P = zeros(4,4);
%             P = obj.P;
%             P_ = A*P*A.' + Q;
%             S = H*P_*H.' + R;
            
            P_ = Q;
            S = H*P_*H.' + R;
            % Kalman Gain
            L = P_ * H.' / (S);
            %             mL = max(L(:));
            %             L = diag([mL mL mL mL]);
            %             L = eye(4);
            %             L = [eye(4) eye(4)];
            % Correct current estimate of state

            %             CC = [eye(4);eye(4)];
            %             qEstimated = qPredicted + L*(qMeasured - CC*qPredicted);
            %             qEstimated = qPredicted + L*(qMeasured - H*qPredicted);
            
            qEstimated = qPredicted + L*E;

% qEstimated = quaternProd((L*E).',qPredicted.').';
            
%               qEstimated = qPredicted + L(1:4,1:4)*E(1:4);

%             qEstimated = qPredicted + L(1:4,5:8)*E(5:8);
%               qEstimated = qPredicted + 0.1*E(5:8);

%               qEstimated = gdfPredict(Jm,Fm,q,obj.Gyroscope,0.1,obj.SamplePeriod,false);
%               qEstimated = qEstimated.';

%             qEstimated = qPredicted + 0.001 * (qa - qPredicted) + 0.1 * (qm - qPredicted);

            %             qEstimated = qMeasured(5:8);
            %              qEstimated = qMeasured(1:4);
            %             qEstimated = qMeasured(1:4) + qMeasured(5:8);

            %                         qEstimated = qPredicted + L*(qErr);
            % Normalize quaternion
            qEstimated = qEstimated./norm(qEstimated);
            % Correct state covariance
%             PEstimated = (eye(4) - L*H)*P_;



            % [qEstimated,PEstimated] = KF.kalmanUpdate(qPredicted,qMeasured,A,Q,R,obj.P);

            qEstimated = qEstimated.';
            % Store current estimate of orientation for next iteration
            obj.Quaternion = qEstimated;
            % Store current estimate of state covariance for next iteration
%             obj.P = PEstimated;
        end

        function [qy,Jacob] = processMeasurement(obj)
            Accelerometer = obj.Accelerometer;
            Magnetometer = obj.Magnetometer;

            % Normalize accelerometer and magnetometer measurements.
            Accelerometer=Accelerometer./norm(Accelerometer);
            Magnetometer=Magnetometer./norm(Magnetometer);



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
            [qy,Jacob] = KF.measurement_quaternion_acc_mag(Accelerometer,Magnetometer, obj.Quaternion');
            qy=qy./norm(qy);
        end
        function [qPredicted,A] = stateFcn(obj,q)

            % Linear state dynamics (A). The state is the integrated
            % (across dt) gyroscope measurements. Returns predicted
            % state at next time step (qPredicted) based on the state
            % dynamics (A), which are linear dynamics in which the
            % gyroscope measurement is integrated across the time step.
            dt=obj.SamplePeriod;
            wx = obj.Gyroscope(1);
            wy = obj.Gyroscope(2);
            wz = obj.Gyroscope(3);

%             omega4=[0,-wx,-wy,-wz;
%                 wx,0,wz,-wy;
%                 wy,-wz,0,wx;
%                 wz,wy,-wx,0];

            omega4 = [0,-wx,-wy,-wz;
                wx,0,wz,-wy;
                wy,-wz,0,wx;
                wz,wy,-wx,0];
            A = eye(4)+dt/2*omega4;

            qPredicted = A*q;
        end
        function Q = stateCovariance(obj)
            dt=obj.SamplePeriod;
            if obj.use3x3covs
                qk_1=obj.Quaternion.';
                q0=qk_1(1);
                q1=qk_1(2);
                q2=qk_1(3);
                q3=qk_1(4);
                Dk=[q1 q2 q3;
                    -q0 -q3 -q2;
                    q2 -q0 -q1;
                    -q2 q1 -q0];
                Q = dt*dt/4*(Dk*obj.Sigma_g(1:3,1:3)*Dk.');
            else
                Q = dt*dt/4*(obj.Sigma_g);
            end
        end

        function R = measurementCovariance(obj,J)
            if obj.use3x3covs
                R_sensor = blkdiag(obj.Sigma_a(1:3,1:3),obj.Sigma_m(1:3,1:3));
                R = J * R_sensor * J.';
            else
                R = blkdiag(obj.Sigma_a,obj.Sigma_m);
            end

        end
    end
    %%
    methods (Static=true)
        function [q,Jacob]=measurement_quaternion_acc_mag(acc,mag,q_)
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
            % Normalize quaternion
            qEstimated = qEstimated./norm(qEstimated);
            % Correct state covariance
            PEstimated = (eye(4) - L)*P_;
        end
    end


    %%
    % Allow access of individual elements of the covariance matrices to
    % allow for use as optimization parameters. Note these covariance
    % matrices are defined to be symmetric and so are defined by only 6
    % parameters even though there are 9 elements (4x4 matrices).
    properties
        parameterNames = {'Sigma_g_scalar' 'Sigma_a_scalar' 'Sigma_m_scalar' ...
            'Sigma_g11' 'Sigma_g22' 'Sigma_g33' 'Sigma_g12' 'Sigma_g13' 'Sigma_g23' ...
            'Sigma_a11' 'Sigma_a22' 'Sigma_a33' 'Sigma_a12' 'Sigma_a13' 'Sigma_a23' ...
            'Sigma_m11' 'Sigma_m22' 'Sigma_m33' 'Sigma_m12' 'Sigma_m13' 'Sigma_m23'};
    end
    properties (Hidden)
        % 1st row and 1st column of gyroscope covariance matrix
        Sigma_g11(1,1) double = 1;
        Sigma_g22(1,1) double = 1;
        Sigma_g33(1,1) double = 1;
        Sigma_g12(1,1) double = 0;
        Sigma_g13(1,1) double = 0;
        Sigma_g23(1,1) double = 0;


        Sigma_a11(1,1) double = 0.01;
        Sigma_a22(1,1) double = 0.01;
        Sigma_a33(1,1) double = 0.01;
        Sigma_a12(1,1) double = 0;
        Sigma_a13(1,1) double = 0;
        Sigma_a23(1,1) double = 0;


        Sigma_m11(1,1) double = 0.01;
        Sigma_m22(1,1) double = 0.01;
        Sigma_m33(1,1) double = 0.01;
        Sigma_m12(1,1) double = 0;
        Sigma_m13(1,1) double = 0;
        Sigma_m23(1,1) double = 0;
    end

    % Use these parameters to set the covariance matrix to a diagonal
    % matrix where each diagonal element has the same value
    properties (Hidden)
        Sigma_g_scalar double
        Sigma_a_scalar double
        Sigma_m_scalar double
    end

    methods
        % Set and get covariance matrix from individual parameters defining
        % elements of the matrix
        function Sigma_g = get.Sigma_g(obj)


            % If user has specified the scalar parameters for the
            % covariance matrix, use them.
            if ~isempty(obj.Sigma_g_scalar)
                Sigma_g = eye(3)*obj.Sigma_g_scalar;
                return
            end
            Sigma_g = zeros(3);
            Sigma_g(1,1) = obj.Sigma_g11;
            Sigma_g(2,2) = obj.Sigma_g22;
            Sigma_g(3,3) = obj.Sigma_g33;
            Sigma_g(1,2) = obj.Sigma_g12;
            Sigma_g(2,1) = Sigma_g(1,2);
            Sigma_g(1,3) = obj.Sigma_g13;
            Sigma_g(3,1) = Sigma_g(1,3);
            Sigma_g(2,3) = obj.Sigma_g23;
            Sigma_g(3,2) = Sigma_g(2,3);
        end
        function Sigma_a = get.Sigma_a(obj)

            % If user has specified the scalar parameters for the
            % covariance matrix, use them.
            if ~isempty(obj.Sigma_a_scalar)
                Sigma_a = eye(3)*obj.Sigma_a_scalar;
                return
            end
            Sigma_a = zeros(3);
            Sigma_a(1,1) = obj.Sigma_a11;
            Sigma_a(2,2) = obj.Sigma_a22;
            Sigma_a(3,3) = obj.Sigma_a33;
            Sigma_a(1,2) = obj.Sigma_a12;
            Sigma_a(2,1) = Sigma_a(1,2);
            Sigma_a(1,3) = obj.Sigma_a13;
            Sigma_a(3,1) = Sigma_a(1,3);
            Sigma_a(2,3) = obj.Sigma_a23;
            Sigma_a(3,2) = Sigma_a(2,3);
        end
        function Sigma_m = get.Sigma_m(obj)

            % If user has specified the scalar parameters for the
            % covariance matrix, use them.
            if ~isempty(obj.Sigma_m_scalar)
                Sigma_m = eye(3)*obj.Sigma_m_scalar;
                return
            end
            Sigma_m = zeros(3);
            Sigma_m(1,1) = obj.Sigma_m11;
            Sigma_m(2,2) = obj.Sigma_m22;
            Sigma_m(3,3) = obj.Sigma_m33;
            Sigma_m(1,2) = obj.Sigma_m12;
            Sigma_m(2,1) = Sigma_m(1,2);
            Sigma_m(1,3) = obj.Sigma_m13;
            Sigma_m(3,1) = Sigma_m(1,3);
            Sigma_m(2,3) = obj.Sigma_m23;
            Sigma_m(3,2) = Sigma_m(2,3);

        end
        function set.Sigma_g(obj,I)
            if isdiag(I) && I(1,1)==I(2,2) && I(1,1)==I(3,3)
                obj.Sigma_g_scalar = I(1,1);
                return
            end
            obj.Sigma_g_scalar = [];
            obj.Sigma_g11 = I(1,1);
            obj.Sigma_g22 = I(2,2);
            obj.Sigma_g33 = I(3,3);
            obj.Sigma_g12 = I(1,2);
            obj.Sigma_g13 = I(1,3);
            obj.Sigma_g23 = I(2,3);
        end
        function set.Sigma_a(obj,I)
            if isdiag(I) && I(1,1)==I(2,2) && I(1,1)==I(3,3)
                obj.Sigma_a_scalar = I(1,1);
                return
            end
            obj.Sigma_a_scalar = [];

            obj.Sigma_a11 = I(1,1);
            obj.Sigma_a22 = I(2,2);
            obj.Sigma_a33 = I(3,3);
            obj.Sigma_a12 = I(1,2);
            obj.Sigma_a13 = I(1,3);
            obj.Sigma_a23 = I(2,3);
        end
        function set.Sigma_m(obj,I)
            if isdiag(I) && I(1,1)==I(2,2) && I(1,1)==I(3,3)
                obj.Sigma_m_scalar = I(1,1);
                return
            end
            obj.Sigma_m_scalar = [];

            obj.Sigma_m11 = I(1,1);
            obj.Sigma_m22 = I(2,2);
            obj.Sigma_m33 = I(3,3);
            obj.Sigma_m12 = I(1,2);
            obj.Sigma_m13 = I(1,3);
            obj.Sigma_m23 = I(2,3);
        end
    end


end