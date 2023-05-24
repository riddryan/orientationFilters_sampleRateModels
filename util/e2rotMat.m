function R = e2rotMat(ang,seq)

if size(ang,1) == 3 && size(ang,2) ~=3
    ang = ang.';
end

N = size(ang,1);

a1=ang(:,1);a2 = ang(:,2);a3=ang(:,3);

a1 = reshape(a1,1,1,N);
a2 = reshape(a2,1,1,N);
a3 = reshape(a3,1,1,N);

c1 = cos(a1);c2=cos(a2);c3=cos(a3);
s1 = sin(a1);s2=sin(a2);s3=sin(a3);

switch seq
    case 'ZXY'
        R = [(c1.*c3 - s1.*s2.*s3)    (-c2.*s1)      (c1.*s3 + c3.*s1.*s2)
     (c3.*s1 + c1.*s2.*s3)    (c1.*c2)      (s1.*s3 - c1.*c3.*s2)
     (-c2.*s3)                 (s2)          (c2.*c3)];
    case 'ZYX'
R = [(c1.*c2)    (c1.*s2.*s3 - c3.*s1)      (s1.*s3 + c1.*c3.*s2)
     (c2.*s1)    (c1.*c3 + s1.*s2.*s3)      (c3.*s1.*s2 - c1.*s3)
     (-s2)                 (c2.*s3)          (c2.*c3)];
    otherwise
       error('Other seqs not supported')
end





