classdef ticcKF < AHRSfilter
    properties
        c_gyroscope = 1;

%         can2 = 1;
%         can1 = 1;
%         can0 = 1;
%         cad1 = 1;
%         cad0 = 1;
% 
%         cmn2 = 1;
%         cmn1 = 1;
%         cmn0 = 1;
%         cmd1 = 1;
%         cmd0 = 1;
        
        can2 = 1;
        can0 = 1;
        cmn2 = 1;
        cmn0 = 1;
%         d2 = 1;
        d0 = 1;

        normStepSize = true;
        normAccelerometer = true;
        normMagnetometer = true;

        % Higher order integration gyro gains
        c_gyroscope2 = 1;
        c_gyroscope3 = 1;
    end
    properties
%         pnames = {'can2' 'can1' 'can0' 'cad1' 'cad0'...
%             'cmn2' 'cmn1' 'cmn0' 'cmd1' 'cmd0'};
%         pnames = {'can2' 'cmn2' 'd2' 'd0'};
            pnames = {'can2' 'can0' 'cmn2' 'cmn0' 'd0'};
    end
    methods
        function obj = ticcKF()
        end
        
        function P = allParams(obj)
            P = NaN(1,length(obj.pnames));
            for i = 1 : length(obj.pnames)
                P(i) = obj.(obj.pnames{i});
            end
        end

        function [qPredicted,qCorrected] = update(obj,Gyroscope,Accelerometer,Magnetometer)
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

%             CA = (obj.can2*dt^2 + obj.can1*dt + obj.can0) / (dt^2 + obj.cad1*dt + obj.cad0);
%             CM = (obj.cmn2*dt^2 + obj.cmn1*dt + obj.cmn0) / (dt^2 + obj.cmd1*dt + obj.cmd0);
            CA = (obj.can2*dt^2 + obj.can0) / (dt^2 + obj.d0);
            CM = (obj.cmn2*dt^2 + obj.cmn0) / (dt^2 + obj.d0);

%             qCorrected = (q - obj.c_accelerometer * qa - obj.c_magnetometer * qm);
            qCorrected = (q - CA * qa - CM * qm);


%             step = qa + qm;
%             step = step ./ norm(step);
%             qCorrected = q - obj.c_accelerometer*step*dt;
            %% Predict
            qDotMeasured = 0.5 * quaternProd(q,[0 obj.Gyroscope]);
            qPredicted = qCorrected + dt* qDotMeasured;

            qCorrected = qCorrected ./ norm(qCorrected);
            qPredicted = qPredicted ./ norm(qPredicted);

            obj.Quaternion = qPredicted;
        end
    end
end