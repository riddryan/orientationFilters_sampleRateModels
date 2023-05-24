classdef FastKF < AHRSfilter
    % FastKF is a computationally quick Kalman filter for attitude
    % estimation from Guo, Wu, and Qian 2017.
    %
    % Tracks orientation of an IMU as states in quaternion form. Gyroscope
    % measurements are used as the process model. Accelerometer and
    % magnetometer are treated as measurements and are transformed to align
    % with expected behaviours of gravity and the world's magnetic field.
    % The kalman filter itself is a standard time-varying kalman filter.
    %
    % See:
    % Guo, S.; Wu, J.; Wang, Z.; Qian, J. Novel MARG-Sensor Orientation Estimation Algorithm Using
    % Fast Kalman Filter. Journal of Sensors 2017, Article ID 8542153
    %
    % Implementation Justa, based on <https://github.com/zarathustr/FKF>
    properties
        % State estimation covariance
        P = 0.001*eye(4);
    end
    properties (Dependent)
        % 3x3 Gyroscope covariance matrix. 
        % 
        % Can be set:
        % 1) by the scalar to assume a diagonal matrix with
        % each diagonal element equal to FastKF.Sigma_g_scalar
        % or 
        % 2) by setting individual elements of the matrix such as
        % FastKF.Sigma_g11 through Fast.KF.Sigma_g33 where the 1st number
        % represents the row and the second number represents the column
        Sigma_g

        % 3x3 Accelerometer covariance matrix
        Sigma_a

        % 3x3 Magnetometer covariance matrix
        Sigma_m

%         Sigma_g(3,3) double
%         
%         % 3x3 Accelerometer covariance matrix
%         Sigma_a(3,3) double
% 
%         % 3x3 Magnetometer covariance matrix
%         Sigma_m(3,3) double
    end
    methods
        function obj = FastKF(varargin)
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

            [qMeasured,Jacobian] = processMeasurement(obj);
             R = measurementCovariance(obj,Jacobian);
            
           

            [qPredicted,A] = stateFcn(obj,obj.Quaternion.');
            Q = stateCovariance(obj);


            [qEstimated,PEstimated] = FastKF.kalmanUpdate(qPredicted,qMeasured,A,Q,R,obj.P);

            % Store current estimate of orientation for next iteration
            obj.Quaternion = qEstimated.';
            % Store current estimate of state covariance for next iteration
            obj.P = PEstimated;
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
            [qy,Jacob] = FastKF.measurement_quaternion_acc_mag(Accelerometer,Magnetometer, obj.Quaternion');
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

            omega4=[0,-wx,-wy,-wz;
                wx,0,wz,-wy;
                wy,-wz,0,wx;
                wz,wy,-wx,0];
            A = eye(4)+dt/2*omega4;

            qPredicted = A*q;
        end
        function Q = stateCovariance(obj)
            dt=obj.SamplePeriod;
            qk_1=obj.Quaternion.';
            q0=qk_1(1);
            q1=qk_1(2);
            q2=qk_1(3);
            q3=qk_1(4);
            Dk=[q1 q2 q3;
                -q0 -q3 -q2;
                q2 -q0 -q1;
                -q2 q1 -q0];
            Q = dt*dt/4*(Dk*obj.Sigma_g*Dk.');
        end

        function R = measurementCovariance(obj,J)
            R_sensor = blkdiag(obj.Sigma_a,obj.Sigma_m);
            R = J * R_sensor * J.';
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
    % parameters even though there are 9 elements (3x3 matrices).
    properties
        parameterNames = {'Sigma_g_scalar' 'Sigma_a_scalar' 'Sigma_m_scalar' ...
            'Sigma_g11' 'Sigma_g22' 'Sigma_g33' 'Sigma_g12' 'Sigma_g13' 'Sigma_g23' ...
            'Sigma_a11' 'Sigma_a22' 'Sigma_a33' 'Sigma_a12' 'Sigma_a13' 'Sigma_a23' ...
            'Sigma_m11' 'Sigma_m22' 'Sigma_m33' 'Sigma_m12' 'Sigma_m13' 'Sigma_m23'};
    end
    properties (Hidden)
        % 1st row and 1st column of gyroscope covariance matrix
        Sigma_g11(1,1) double = 0.01;
        Sigma_g22(1,1) double = 0.01;
        Sigma_g33(1,1) double = 0.01;
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