classdef JustaAHRS < AHRSfilter
%   JUSTA, Josef; ŠMÍDL, Václav; HAMÁÈEK, Aleš. Fast AHRS Filter for Accelerometer, Magnetometer, 
%   and Gyroscope Combination with Separated Sensor Corrections. Sensors, 2020, 20.14: 3824.
%
%   Date          Author          Notes
%   30/5/2020     Josef Justa     Initial release
% Wolly: added added wAcc and wMag gains as user input
    properties
        wAcc=0.00248;
        wMag=0.001;

        normStepSize = true;
        normAccelerometer = true;
        normMagnetometer = true;
    end
    
    methods (Access = public)
        function obj = JustaAHRSPure(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod'), obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Quaternion'), obj.Quaternion = varargin{i+1};
                elseif  strcmp(varargin{i}, 'wAcc'), obj.wAcc = varargin{i+1};
                elseif  strcmp(varargin{i}, 'wMag'), obj.wMag = varargin{i+1};
                else error('Invalid argument');
                end
            end
        end

        function P = allParams(obj)
            P = [obj.wAcc obj.wMag];
        end
        function quat = update(obj, Gyroscope, Accelerometer, Magnetometer)

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


            q = obj.Quaternion; % short name local variable for readability

            acc = obj.Accelerometer;
            gyro = obj.Gyroscope;
            mag = obj.Magnetometer;
            
%             qDot=0.5 * quaternProd(q, [0 Gyroscope(1) Gyroscope(2) Gyroscope(3)]);
            
            wt=Gyroscope*obj.SamplePeriod/2;
            qDotx=sin(wt(1));
            qDoty=sin(wt(2));
            qDotz=sin(wt(3));
            qDotw=sqrt(1-sumsqr([qDotx qDoty qDotz]));
            qDotw = real(qDotw);
            
            qDot=[qDotw qDotx qDoty qDotz];

            quatGyrPred= quaternProd(q,qDot);  %q + qDot*obj.SamplePeriod;
            qp=quatGyrPred;

            % Get predicted reference directions based on current
            % orientation for gravity and magnetic fields
            R =[2*(0.5 - qp(3)^2 - qp(4)^2)   0   2*(qp(2)*qp(4) - qp(1)*qp(3))
                2*(qp(2)*qp(3) - qp(1)*qp(4))  0  2*(qp(1)*qp(2) + qp(3)*qp(4))
                2*(qp(1)*qp(3) + qp(2)*qp(4))  0  2*(0.5 - qp(2)^2 - qp(3)^2)];

            ar=[0 0 1];
            accMesPred=(R*ar')';
            

            mr_z= dot(accMesPred,mag);
            mr_x= real(sqrt(1-mr_z^2));
            mr=[mr_x 0 mr_z];
         
            magMesPred=(R*mr')';

            % Compare to actual measurements
            ca=cross(acc,accMesPred);
            cm=cross(mag,magMesPred);

            if obj.normStepSize
                vecA=ca/norm(ca);
                vecB=cm/norm(cm);
            else
                vecA = ca;
                vecB = cm;
            end
            

            % Update quaternion based on the error and gains for mag and
            % acc
            im=vecA*obj.wAcc/2+vecB*obj.wMag/2;
            im2=im*sinc(norm(im)/pi);
           
            try qCor=[sqrt(1-sumsqr(im2)),im2];
            catch
                keyboard
            end
            
            quat=quaternProd(quatGyrPred,qCor);
  
%             quat=quatGyrPred;
            % Wolly: not sure why quaternion w is negated
            if(quat(1)<0)
                quat=-quat;
            end
            quat = quat/norm(quat);
            obj.Quaternion = quat;
        end
        
        function obj = UpdateIMU(obj, Gyroscope, Accelerometer)
            q = obj.Quaternion; % short name local variable for readability
            
            % Normalise accelerometer measurement
            if(norm(Accelerometer) == 0), return; end	% handle NaN
            acc = Accelerometer / norm(Accelerometer);	% normalise magnitude
            
            wt=Gyroscope*obj.SamplePeriod/2;
            qDotx=sin(wt(1));
            qDoty=sin(wt(2));
            qDotz=sin(wt(3));
            qDotw=sqrt(1-sumsqr([qDotx qDoty qDotz]));
            
            qDot=[qDotw qDotx qDoty qDotz];

            quatGyrPred= quaternProd(q,qDot);  %q + qDot*obj.SamplePeriod;
            
            qp=quatGyrPred;
            
            ar=[0 0 1];
            
            R=[0 0  2*(qp(2)*qp(4) - qp(1)*qp(3))
               0 0  2*(qp(1)*qp(2) + qp(3)*qp(4))
               0 0  2*(0.5 - qp(2)^2 - qp(3)^2)];
            
            accMesPred=(R*ar')';
            
            ca=cross(acc,accMesPred);
            veca=ca/norm(ca);
            
            qCor=[1 veca*obj.wAcc/2];
            
            quat=quaternProd(qp,qCor);
            
            if(quat(1)<0)
                quat=-quat;
            end
            
            obj.Quaternion = quat/norm(quat);
        end
        
    end
    
end

