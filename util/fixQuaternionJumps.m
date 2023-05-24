function q = fixQuaternionJumps(q)

N = size(q,1);

for i = 2 : N
    qPrev = q(i-1,:);
    qCurrent = q(i,:);

    if norm(qPrev - qCurrent) > norm(qPrev + qCurrent)
        q(i,:) = -qCurrent;
    end
end