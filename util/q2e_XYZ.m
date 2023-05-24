function e = q2e_XYZ(q)
% Taken from: 
% Quaternion to Euler angles conversion: A direct, general and
% computationally efficient method EvandroBernardes ID , Ste´phane Viollet
% 2022

qr = q(:,1);
qx = q(:,2);
qy = q(:,3);
qz = q(:,4);

e(:,1) = atan2(qx+qz,qr-qy) - atan2(qz-qx,qy+qr);
e(:,2) = acos((qr-qy).^2 + (qx+qz).^2 - 1) - pi/2;
e(:,3) = atan2(qx+qz,qr-qy) + atan2(qz-qx,qy+qr);
