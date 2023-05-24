classdef RiddickAHRS < AHRSfilter
    properties
        c_gyroscope = 1;
        c_accelerometer = 0.033;
        c_magnetometer = 0.030;

        normStepSize = true;
        normAccelerometer = true;
        normMagnetometer = true;

        % Higher order integration gyro gains
        c_gyroscope2 = 1;
        c_gyroscope3 = 1;
    end
    methods
        function obj = RiddickAHRS()
        end
        
        function P = allParams(obj)
            P = [obj.c_gyroscope obj.c_accelerometer obj.c_magnetometer obj.c_gyroscope2 obj.c_gyroscope3];
        end
        function [qPredicted] = update(obj,Gyroscope,Accelerometer,Magnetometer)
            gains = [obj.c_gyroscope obj.c_accelerometer obj.c_magnetometer];

            qPredicted = RiddickAHRS_update(obj.Quaternion,Gyroscope,Accelerometer,Magnetometer,gains,obj.SamplePeriod,obj.normStepSize);
            obj.Quaternion = qPredicted;
        end

        function [qPredicted,qCorrected] = updateSlow(obj,Gyroscope,Accelerometer,Magnetometer)
            if nargin > 1
                obj.Gyroscope = Gyroscope;
            end
            if nargin > 2
                obj.Accelerometer = Accelerometer;
            end
            if nargin > 3
                obj.Magnetometer = Magnetometer;
            end

            if obj.normAccelerometer
                obj.Accelerometer = obj.Accelerometer ./ norm(obj.Accelerometer);;
            end
            if obj.normMagnetometer
                obj.Magnetometer = obj.Magnetometer ./ norm(obj.Magnetometer);
            end

            

            q = obj.Quaternion;

            %% Correct
            % Estimate of orientation from accelerometer
            Fa = [-2*q(1)*q(3) + 2*q(2)*q(4)                             - obj.Accelerometer(1)
                2*q(1)*q(2)   + 2*q(3)*q(4)                             - obj.Accelerometer(2)
                q(1)*q(1)     - q(2)*q(2) - q(3)*q(3) + q(4)*q(4)       - obj.Accelerometer(3)];
            % Fa = quaternProd(quaternConj(q),quaternProd([0 0 0 1],q)) - [0 obj.Accelerometer]
            %             Fm = quaternProd(quaternConj(q),quaternProd(b,q)) - [0 obj.Magnetometer]

            Ja = 2* [ -q(3)  q(2)  q(1)
                q(4)  q(1) -q(2)
                -q(1)  q(4) -q(3)
                q(2)  q(3)  q(4)];

            qa = (Ja*Fa).';

            % Estimate of orientation from magnetometer. Orthogonal to
            % accelerometer estimate
            Vm = cross(acc, mag);
            Vm = Vm / norm(Vm);

            Fm =                 [2*q(1)*q(4) + 2*q(2)*q(3)                               - Vm(1)
                q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2                       - Vm(2)
                2*q(3)*q(4) - 2*q(1)*q(2)                               - Vm(3)];

            Jm = 2 * [q(4)  q(1) -q(2)
                q(3) -q(2) -q(1)
                q(2)  q(3)  q(4)
                q(1) -q(4)  q(3)];
            qm = (Jm*Fm).';
            
            dt = obj.SamplePeriod;

%             if obj.normStepSize
%                 qa = qa ./ norm(qa);
%                 qm = qm ./ norm(qm);
%             end

%             qCorrected = (q - obj.c_accelerometer * qa - obj.c_magnetometer * qm);
            qCorrected = (q - obj.c_accelerometer * qa * dt - obj.c_magnetometer * qm * dt);


%             step = qa + qm;
%             step = step ./ norm(step);
%             qCorrected = q - obj.c_accelerometer*step*dt;
            %% Predict
            qDotMeasured = 0.5 * quaternProd(q,[0 obj.Gyroscope]);
            qPredicted = qCorrected + obj.c_gyroscope * dt* qDotMeasured;

            qCorrected = qCorrected ./ norm(qCorrected);
            qPredicted = qPredicted ./ norm(qPredicted);

            obj.Quaternion = qPredicted;
        end
    end
end