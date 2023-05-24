function qMag = fMag(acc,mag,q)
q1 = q(1);q2 = q(2);q3 = q(3);q4 =q(4);
Vm = cross(acc, mag);
Vm = Vm / norm(Vm);
vm = Vm;


qMag = [2*vm(1)*q4 - 2*vm(3)*q2 + q1*(2*vm(2) - 2)
2*vm(1)*q3 - 2*vm(3)*q1 - q2*(2*vm(2) + 2)
2*vm(1)*q2 + 2*vm(3)*q4 + q3*(2*vm(2) - 2)
2*vm(1)*q1 + 2*vm(3)*q3 - q4*(2*vm(2) + 2)];
