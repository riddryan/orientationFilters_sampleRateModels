classdef (Abstract) GradDescentFilter < AHRSfilter
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
        function obj = GradDescentFilter()
        end
    end

    methods (Abstract = true)
        F = objective(obj);
    end

    methods

        function P = allParams(obj)
            P = [obj.Beta];
        end
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
            [F,J] = objective(obj); % cost and cost jacobian
            deltaF = J * F; % gradient of F
            
            step = deltaF;
%             if obj.normStepSize
%                 step = step/norm(step);
%             end
%             qCorrected = obj.Quaternion - obj.Beta * step;

            % Estimate current time derivative of the quaternion by fusing
            % the gyroscope measurement with a gradient descent step w.r.t.
            % the accelerometer (and magnetometer) measurements.
            qDot = estimateQderivative(obj,deltaF);
            
            % Predict quaternion at next time step
            qPredicted = predict(obj,qDot);

            obj.Quaternion = qPredicted;

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