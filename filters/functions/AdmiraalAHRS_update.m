function qPredicted = AdmiraalAHRS_update(q,gyro,acc,mag,gains,dt,normStepSize)
beta = gains(1);

%% Normalize
acc = acc / norm(acc);
mag = mag / norm(mag);
%% Measurement

h = quaternProd(q, quaternProd([0 mag], quaternConj(q)));


% Jacobian for gravity and magnetic fields
%            Gravity
%       -----------------
Ja = 2 * [-q(3) q(2)  q(1)
    q(4)  q(1) -q(2)
    -q(1)  q(4) -q(3)
    q(2)  q(3)  q(4)];

% F function (q-1 * earth field ref * q - earth field measured)
% The error that needs to be minimised
Fa = [-2*q(1)*q(3) + 2*q(2)*q(4) - acc(1)
    2*q(1)*q(2)   + 2*q(3)*q(4) - acc(2)
    q(1)*q(1)     - q(2)*q(2) - q(3)*q(3) + q(4)*q(4) - acc(3)];



Fm = []; Jm = [];
if ~any(isnan(mag))
    mr = [h(2) 0 h(4)]; % mr = Magnetic reference field

    Fm = [mr(1)*(q(1)*q(1)    + q(2)*q(2)    - q(3)*q(3) - q(4)*q(4)) + mr(3)*(-2*q(1)*q(3) + 2*q(2)*q(4))                      - mag(1)
        mr(1)*(-2*q(1)*q(4) + 2*q(2)*q(3)) +                          mr(3)*(2*q(1)*q(2) + 2*q(3)*q(4))                       - mag(2)
        mr(1)*(2*q(1)*q(3)  + 2*q(2)*q(4)) +                          mr(3)*(q(1)*q(1) - q(2)*q(2) - q(3)*q(3) + q(4)*q(4))   - mag(3)];
    %                                 Magnetic field
    % ----------------------------------------------------------------------------------
    Jm = 2*[mr(1)*q(1) - mr(3)*q(3)     -mr(1)*q(4) + mr(3)*q(2)      mr(1)*q(3) + mr(3)*q(1)
        mr(1)*q(2) + mr(3)*q(4)      mr(1)*q(3) + mr(3)*q(1)      mr(1)*q(4) - mr(3)*q(2)
        -mr(1)*q(3) - mr(3)*q(1)      mr(1)*q(2) + mr(3)*q(4)      mr(1)*q(1) - mr(3)*q(3)
        -mr(1)*q(4) + mr(3)*q(2)     -mr(1)*q(1) + mr(3)*q(3)      mr(1)*q(2) + mr(3)*q(4)];
end
F = [Fa;Fm];
J = [Ja Jm];

%%

qPredicted = gdfPredict(J,F,q,gyro,beta,dt,normStepSize);
