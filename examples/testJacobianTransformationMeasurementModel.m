syms q1 q2 q3 q4
syms ax ay az

dqAcc = [2*q1^3 + 2*q1*q2^2 + 2*q1*q3^2 + 2*q1*q4^2 - 2*az*q1 - 2*ay*q2 + 2*ax*q3
    2*q1^2*q2 - 2*ay*q1 + 2*q2^3 + 2*q2*q3^2 + 2*q2*q4^2 + 2*az*q2 - 2*ax*q4
    2*q1^2*q3 + 2*ax*q1 + 2*q2^2*q3 + 2*q3^3 + 2*q3*q4^2 + 2*az*q3 - 2*ay*q4
    2*q1^2*q4 + 2*q2^2*q4 - 2*ax*q2 + 2*q3^2*q4 - 2*ay*q3 + 2*q4^3 - 2*az*q4];

J = jacobian(dqAcc,[ax;ay;az]);

syms r
R = diag([r r r]);


RT = J * R * J.';