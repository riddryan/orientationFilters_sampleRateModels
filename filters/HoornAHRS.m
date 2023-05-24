classdef HoornAHRS < GradDescentFilter

    methods
        function obj = HoornAHRS()
        end
        function [F,J] = objective(obj)
            q = obj.Quaternion;
            qw = q(1);
            qx = q(2);
            qy = q(3);
            qz = q(4);

            % Expected gravity (q-1 [0 0 1] q)
            G_simulated = [2*qx*qz - 2*qw*qy
                2*qw*qx + 2*qy*qz
                qw^2 - qx^2 - qy^2 + qz^2];

            % Error between expected and measured gravity (3x1)
            Fa = G_simulated - obj.Accelerometer';


            % Jacobian
            Ja = 2* [-qy  qx  qw
                qz  qw -qx
                -qw  qz -qy
                qx  qy  qz];

            Fm = []; Jm = [];
            if obj.UseMagnetometer
                % Take cross product with expected rather than measured Acc, as
                % measured Acc could also measure movement related
                % accelerations. This isolates the magnetometer readings that
                % are perpendicular to the expected gravity direction.
                Vm = cross(G_simulated, obj.Magnetometer);
                Vm = Vm / norm(Vm);

                % Fm (error) based on perpendicular magnetic field q-1 vr q - vm
                % where vr = [1 0 0]. Error between expected and measured
                % magnetic field. (3x1)
                Fm = [2*qw*qz + 2*qx*qy         - Vm(1)
                    qw^2 - qx^2 + qy^2 - qz^2   - Vm(2)
                    2*qy*qz - 2*qw*qx           - Vm(3)    ];


                Jm = 2 * [qz  qw -qx
                    qy -qx -qw
                    qx  qy  qz
                    qw -qz  qy];
            end
            F = [Fa;Fm];
            J = [Ja Jm];
        end
    end
end