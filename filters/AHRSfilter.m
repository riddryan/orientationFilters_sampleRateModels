classdef (Abstract) AHRSfilter < matlab.mixin.Copyable & matlab.mixin.Heterogeneous & matlab.mixin.CustomDisplay
    %AHRS Base structure for an Attitude and Heading Reference System
    %(AHRS) filter which estimates orientation from accelerometer,
    %gyroscope, and magnetometer data.
    % Abstract class - need a specific subclass to test the attitude
    % estimation.

    properties (Access=public)
        % Length of time per sample in seconds
        SamplePeriod = 1/190;
        % Current estimate of orientation in quaternion form
        Quaternion
        % Current gyroscope reading
        Gyroscope
        % Current accelerometer reading
        Accelerometer
        % Current magnetomer reading
        Magnetometer
        % Output estimate of orientation of filter in quaternion form
        Qest
        % Whether to use magnetometer data
        UseMagnetometer = true

%         % Length of time per sample in seconds
%         SamplePeriod(1,1) double = 1/190;
%         % Current estimate of orientation in quaternion form
%         Quaternion(1,4) double
%         % Current gyroscope reading
%         Gyroscope(1,3) double
%         % Current accelerometer reading
%         Accelerometer(1,3) double
%         % Current magnetomer reading
%         Magnetometer(1,3) double
%         % Output estimate of orientation of filter in quaternion form
%         Qest(:,4) double
%         % Whether to use magnetometer data
%         UseMagnetometer logical = true
    end
    methods (Abstract=true)
        q = update(obj,Gyroscope,Accelerometer,Magnetometer)
    end
    methods
        function obj = AHRSfilter(varargin)
            for i = 1:2:nargin
                if  strcmp(varargin{i}, 'SamplePeriod')
                    obj.SamplePeriod = varargin{i+1};
                elseif  strcmp(varargin{i}, 'Quaternion')
                    obj.Quaternion = varargin{i+1};
                else 
                    error('Invalid argument');
                end
            end
        end
        function initialize(obj)
        end

        function qEstimated = estimateOrientation(obj,q0,gyro,acc,mag,indexOffset)
            if nargin < 6
                indexOffset = 0;
            end

            obj.Quaternion = q0;
            % Number of samples
            NT = size(gyro,1);

            obj.Qest = zeros(NT,4);

            qEstimated = NaN(NT,4);
            for tt = 1:(NT - 1 - indexOffset)
                if tt == 1
                    initialize(obj);
                    qEstimated(tt,:) = q0;
                end

                if obj.UseMagnetometer
                    q = update(obj,gyro(tt,:),acc(tt,:),mag(tt,:));
                else
                    q = update(obj,gyro(tt,:),acc(tt,:));
                end
                % save estimated quaternion
                qEstimated(tt + 1 + indexOffset,:) = q;
                obj.Quaternion = q;
            end
            obj.Qest = qEstimated;
        end

%         function qEstimated = estimateOrientation_correctedIndices(obj,q0,gyro,acc,mag)
%             if nargin < 4
%                 obj.UseMagnetometer = false;
%             else
%                 obj.UseMagnetometer = true;
%             end
% 
%             obj.Quaternion = q0;
%             % Number of samples
%             NT = size(gyro,1);
% 
%             obj.Qest = zeros(NT,4);
% 
%             fFlag = false;
% %             oclass = class(obj);
% %             if strcmp(oclass,'RiddickAHRS')
% %                 fFlag = true;
% %             end
% 
%             qEstimated = NaN(NT,4);
%             for tt = 1:NT-1
%                 if tt == 1
%                     initialize(obj);
%                     qEstimated(tt,:) = q0;
%                 end
% 
%                 if ~fFlag
%                 if obj.UseMagnetometer
%                     q = update(obj,gyro(tt,:),acc(tt,:),mag(tt,:));
%                 else
%                     q = update(obj,gyro(tt,:),acc(tt,:));
%                 end
%                 % save estimated quaternion
%                 qEstimated(tt+1,:) = q;
%                 continue
%                 end
% 
%                 switch oclass
%                     case 'RiddickAHRS'
%                         gains = [obj.c_gyroscope obj.c_accelerometer obj.c_magnetometer];
%                         %                         qPredicted = RiddickAHRS_update(qEstimated(tt,:),gyro(tt,:),acc(tt,:),mag(tt,:),gains,obj.SamplePeriod,obj.normStepSize);
% 
%                         qPredicted = RiddickAHRS_update_mex(qEstimated(tt,:),gyro(tt,:),acc(tt,:),mag(tt,:),gains,obj.SamplePeriod,obj.normStepSize);
%                         qEstimated(tt+1,:) = qPredicted;
%                 end
% 
%             end
%             obj.Qest = qEstimated;
%         end
    end
end