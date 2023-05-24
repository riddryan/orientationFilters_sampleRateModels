classdef Madgwick2step < AHRSfilter
    properties (Access = public)
        Beta= 0.1;
        normStepSize = true;
        normAccelerometer = true;
        normMagnetometer = true;

        %         Beta(1,1) double = 0.1;
        %         normStepSize logical = true;
        %         normAccelerometer logical = true;
        %         normMagnetometer logical = true;
    end

    methods
        function obj = Madgwick2step()
        end
    end


    methods
        function qPredicted = update(obj,Gyroscope,Accelerometer,Magnetometer)
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
                obj.Accelerometer = obj.Accelerometer ./ norm(obj.Accelerometer);
            end
            if obj.normMagnetometer
                obj.Magnetometer = obj.Magnetometer ./ norm(obj.Magnetometer);
            end

            % Optimization problem: orientation of sensor is found which
            % aligns a pre-defined reference direction of the field in the
            % earth frame with the measured field in the sensor frame.
            %
            % The algorithms generally use one reference direction for
            % gravity, and one reference direction for the magnetometer
            % heading.


            % Step 1: Accelerometer
            [Fa,Ja] = objectiveA(obj); % cost and cost jacobian
            [Fm,Jm] = objectiveM(obj);

            step = Ja.*Fa + Jm.*Fm;
            qDotCorrection = -obj.Beta * step.';

            % Make correction
            qDotMeasured = quaternProd(obj.Quaternion,[0 obj.Gyroscope]);
            qDot = 0.5 * qDotMeasured + qDotCorrection;
            
            % Predict quaternion at next time step
            qPredicted = predict(obj,qDot);

            obj.Quaternion = qPredicted;

        end
        function [Fa,Ja] = objectiveA(obj)
            q = obj.Quaternion;

            % Reference direction of Earth's magnetic feild
            h = quaternProd(q, quaternProd([0 obj.Magnetometer], quaternConj(q)));
            b = [0 norm([h(2) h(3)]) 0 h(4)];
            %             Fa = quaternProd(quaternConj(q),quaternProd([0 0 0 1],q)) - [0 Accelerometer]
            %             Fm = quaternProd(quaternConj(q),quaternProd(b,q)) - [0 Magnetometer]
            % Gradient decent algorithm corrective step
            % (this is q-1 * vr * q -vm) --> F that needs to be minimised
            Fa = [2*(q(2)*q(4) - q(1)*q(3)) - obj.Accelerometer(1)
                2*(q(1)*q(2) + q(3)*q(4)) - obj.Accelerometer(2)
                2*(0.5 - q(2)^2 - q(3)^2) - obj.Accelerometer(3)];

            % Jacobian are the partial derivatives of F (below is not the
            % actual partial derivatives)
            Ja = [-2*q(3),                 	2*q(4),                    -2*q(1),                         2*q(2)
                2*q(2),                 	2*q(1),                    	2*q(4),                         2*q(3)
                0,                         -4*q(2),                    -4*q(3),                         0];
        end
                function [Fm,Jm] = objectiveM(obj)
            q = obj.Quaternion;

            % Reference direction of Earth's magnetic feild
            h = quaternProd(q, quaternProd([0 obj.Magnetometer], quaternConj(q)));
            b = [0 norm([h(2) h(3)]) 0 h(4)];
            %             Fa = quaternProd(quaternConj(q),quaternProd([0 0 0 1],q)) - [0 Accelerometer]
            %             Fm = quaternProd(quaternConj(q),quaternProd(b,q)) - [0 Magnetometer]
            % Gradient decent algorithm corrective step
            % (this is q-1 * vr * q -vm) --> F that needs to be minimised
                Fm = [2*b(2)*(0.5 - q(3)^2 - q(4)^2) + 2*b(4)*(q(2)*q(4) - q(1)*q(3)) - obj.Magnetometer(1)
                    2*b(2)*(q(2)*q(3) - q(1)*q(4)) + 2*b(4)*(q(1)*q(2) + q(3)*q(4)) - obj.Magnetometer(2)
                    2*b(2)*(q(1)*q(3) + q(2)*q(4)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2) - obj.Magnetometer(3)];

                Jm = [-2*b(4)*q(3),               2*b(4)*q(4),               -4*b(2)*q(3)-2*b(4)*q(1),       -4*b(2)*q(4)+2*b(4)*q(2)
                    -2*b(2)*q(4)+2*b(4)*q(2),	2*b(2)*q(3)+2*b(4)*q(1),	2*b(2)*q(2)+2*b(4)*q(4),       -2*b(2)*q(1)+2*b(4)*q(3)
                    2*b(2)*q(3),                2*b(2)*q(4)-4*b(4)*q(2),	2*b(2)*q(1)-4*b(4)*q(3),       2*b(2)*q(2)];

        end
        function [F,J] = objective(obj)
            q = obj.Quaternion;

            % Reference direction of Earth's magnetic feild
            h = quaternProd(q, quaternProd([0 obj.Magnetometer], quaternConj(q)));
            b = [0 norm([h(2) h(3)]) 0 h(4)];
            %             Fa = quaternProd(quaternConj(q),quaternProd([0 0 0 1],q)) - [0 Accelerometer]
            %             Fm = quaternProd(quaternConj(q),quaternProd(b,q)) - [0 Magnetometer]
            % Gradient decent algorithm corrective step
            % (this is q-1 * vr * q -vm) --> F that needs to be minimised
            Fa = [2*(q(2)*q(4) - q(1)*q(3)) - obj.Accelerometer(1)
                2*(q(1)*q(2) + q(3)*q(4)) - obj.Accelerometer(2)
                2*(0.5 - q(2)^2 - q(3)^2) - obj.Accelerometer(3)];

            % Jacobian are the partial derivatives of F (below is not the
            % actual partial derivatives)
            Ja = [-2*q(3),                 	2*q(4),                    -2*q(1),                         2*q(2)
                2*q(2),                 	2*q(1),                    	2*q(4),                         2*q(3)
                0,                         -4*q(2),                    -4*q(3),                         0];
            Fm = []; Jm = [];
            if obj.UseMagnetometer
                Fm = [2*b(2)*(0.5 - q(3)^2 - q(4)^2) + 2*b(4)*(q(2)*q(4) - q(1)*q(3)) - obj.Magnetometer(1)
                    2*b(2)*(q(2)*q(3) - q(1)*q(4)) + 2*b(4)*(q(1)*q(2) + q(3)*q(4)) - obj.Magnetometer(2)
                    2*b(2)*(q(1)*q(3) + q(2)*q(4)) + 2*b(4)*(0.5 - q(2)^2 - q(3)^2) - obj.Magnetometer(3)];

                Jm = [-2*b(4)*q(3),               2*b(4)*q(4),               -4*b(2)*q(3)-2*b(4)*q(1),       -4*b(2)*q(4)+2*b(4)*q(2)
                    -2*b(2)*q(4)+2*b(4)*q(2),	2*b(2)*q(3)+2*b(4)*q(1),	2*b(2)*q(2)+2*b(4)*q(4),       -2*b(2)*q(1)+2*b(4)*q(3)
                    2*b(2)*q(3),                2*b(2)*q(4)-4*b(4)*q(2),	2*b(2)*q(1)-4*b(4)*q(3),       2*b(2)*q(2)];

            end
            F = [Fa;Fm];
            J = [Ja;Jm].';
        end

        function qDot = estimateQderivative(obj,deltaF)
            % F is the evaluated cost function for the gradient descent
            % algorithm. This cost generally represents the error between
            % the predicted quaternion and the quaternion measured by the
            % accelerometer and magnetometer.
            % J is the jacobian for F.

            step = deltaF;
            if obj.normStepSize
                step = step / norm(step);
            end
            qDotMeasured = quaternProd(obj.Quaternion,[0 obj.Gyroscope]);
            % Gradient descent step for qDot
            qDot = 0.5 * qDotMeasured - obj.Beta * step.';
        end
        function q = predict(obj,qDot)
            % Integrate qdot to get the next q
            q = obj.Quaternion + obj.SamplePeriod * qDot;
            q = q / norm(q);
        end
    end
end