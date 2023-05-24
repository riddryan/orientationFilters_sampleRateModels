classdef WilsonAHRS < GradDescentFilter

    methods
        function obj = WilsonAHRS()
        end
        function [F,J] = objective(obj)
            q = obj.Quaternion;



            Fa = [-2*q(1)*q(3) + 2*q(2)*q(4)                             - obj.Accelerometer(1)
                2*q(1)*q(2)   + 2*q(3)*q(4)                             - obj.Accelerometer(2)
                q(1)*q(1)     - q(2)*q(2) - q(3)*q(3) + q(4)*q(4)       - obj.Accelerometer(3)];
% Fa = quaternProd(quaternConj(q),quaternProd([0 0 0 1],q)) - [0 obj.Accelerometer]
%             Fm = quaternProd(quaternConj(q),quaternProd(b,q)) - [0 Magnetometer]

            Ja = 2* [ -q(3)  q(2)  q(1)
                q(4)  q(1) -q(2)
                -q(1)  q(4) -q(3)
                q(2)  q(3)  q(4)];



            
            Fm = []; Jm = [];
            if obj.UseMagnetometer
                Vm = cross(obj.Accelerometer, obj.Magnetometer);
                Vm = Vm / norm(Vm);

                Fm =                 [2*q(1)*q(4) + 2*q(2)*q(3)                               - Vm(1)
                    q(1)^2 - q(2)^2 + q(3)^2 - q(4)^2                       - Vm(2)
                    2*q(3)*q(4) - 2*q(1)*q(2)                               - Vm(3)];

                Jm = 2 * [q(4)  q(1) -q(2)
                    q(3) -q(2) -q(1)
                    q(2)  q(3)  q(4)
                    q(1) -q(4)  q(3)];

            end
            F = [Fa;Fm];
            J = [Ja Jm];
        end
    end
end