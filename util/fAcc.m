function qAcc = fAcc(acc,q)
q1 = q(1);q2 = q(2);q3 = q(3);q4 =q(4);
ax = acc(1); ay=acc(2); az= acc(3);


qAcc = [2*ay*q2 - 2*ax*q3 + q1*(2*az - 2)
2*ax*q4 + 2*ay*q1 - q2*(2*az + 2)
2*ay*q4 - 2*ax*q1 - q3*(2*az + 2)
2*ax*q2 + 2*ay*q3 + q4*(2*az - 2)];