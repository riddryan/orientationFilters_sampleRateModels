function Euler_angle=Euler_all(order,R1, R2)

if order=='yxz'
[y,z,x]=Euler_yzx_col(R1);
if exist('R2','var')
    [y,z,x]=Euler_yzx_col(R1,R2);
end
end

if order=='zxy'
[z,x,y]=Euler_zxy_col(R1);
if exist('R2','var')
    [z,x,y]=Euler_zxy_col(R1,R2);
end
end

Euler_angle=[x,y,z]*180/pi;
