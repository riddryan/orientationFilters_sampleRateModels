function J = JAcc(acc,q)
q1 = q(1);q2 = q(2);q3 = q(3);q4 =q(4);
ax = acc(1); ay=acc(2); az= acc(3);

J = [-2*q3, 2*q2,  2*q1
 2*q4, 2*q1, -2*q2
-2*q1, 2*q4, -2*q3
 2*q2, 2*q3,  2*q4];