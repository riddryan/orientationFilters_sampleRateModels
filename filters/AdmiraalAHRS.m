classdef AdmiraalAHRS < GradDescentFilter

    methods
        function obj = AdmiraalAHRS()
        end
        function [F,J] = objective(obj)
            q = obj.Quaternion;

               % Reference direction of Earth's magnetic field
            h = quaternProd(q, quaternProd([0 obj.Magnetometer], quaternConj(q)));
            mr = [h(2) 0 h(4)]; % mr = Magnetic reference field
            
            % Jacobian for gravity and magnetic fields
            %            Gravity                                        
            %       -----------------   
            Ja = 2 * [-q(3) q(2)  q(1)   
                     q(4)  q(1) -q(2)   
                    -q(1)  q(4) -q(3)  
                     q(2)  q(3)  q(4)];
  
            % F function (q-1 * earth field ref * q - earth field measured)
            % The error that needs to be minimised
            Fa = [-2*q(1)*q(3) + 2*q(2)*q(4) - obj.Accelerometer(1)
                2*q(1)*q(2)   + 2*q(3)*q(4) - obj.Accelerometer(2)
                q(1)*q(1)     - q(2)*q(2) - q(3)*q(3) + q(4)*q(4) - obj.Accelerometer(3)];

            Fm = []; Jm = [];
            if obj.UseMagnetometer


                Fm = [mr(1)*(q(1)*q(1)    + q(2)*q(2)    - q(3)*q(3) - q(4)*q(4)) + mr(3)*(-2*q(1)*q(3) + 2*q(2)*q(4))                      - obj.Magnetometer(1)
                    mr(1)*(-2*q(1)*q(4) + 2*q(2)*q(3)) +                          mr(3)*(2*q(1)*q(2) + 2*q(3)*q(4))                       - obj.Magnetometer(2)
                    mr(1)*(2*q(1)*q(3)  + 2*q(2)*q(4)) +                          mr(3)*(q(1)*q(1) - q(2)*q(2) - q(3)*q(3) + q(4)*q(4))   - obj.Magnetometer(3)];
                %                                 Magnetic field
                % ----------------------------------------------------------------------------------
                Jm = 2*[mr(1)*q(1) - mr(3)*q(3)     -mr(1)*q(4) + mr(3)*q(2)      mr(1)*q(3) + mr(3)*q(1) 
                     mr(1)*q(2) + mr(3)*q(4)      mr(1)*q(3) + mr(3)*q(1)      mr(1)*q(4) - mr(3)*q(2)
                     -mr(1)*q(3) - mr(3)*q(1)      mr(1)*q(2) + mr(3)*q(4)      mr(1)*q(1) - mr(3)*q(3)
                     -mr(1)*q(4) + mr(3)*q(2)     -mr(1)*q(1) + mr(3)*q(3)      mr(1)*q(2) + mr(3)*q(4)];
            end
            F = [Fa;Fm];
            J = [Ja Jm];
        end
    end
end