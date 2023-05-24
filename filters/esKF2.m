classdef esKF2 < AHRSfilter
    properties
        % State estimation covariance
        P = 0.00*eye(3);
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
        function obj = esKF2(varargin)
        end
        function P = allParams(obj)
            P = [obj.Sigma_g(:);obj.Sigma_a(:);obj.Sigma_m(:)].';
        end

        function [qEstimated,PEstimated] = update(obj,Gyroscope,Accelerometer,Magnetometer)
            qEstimated = [];
            PEstimated = [];
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