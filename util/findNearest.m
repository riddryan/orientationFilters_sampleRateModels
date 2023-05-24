function [closestValue,I] = findNearest(x,y,varargin)
% Find nearest element in x equal to y. Can be vectors

rflag = false;
if isrow(x)
    x = x.';
    rflag = true;
end
if isrow(y)
    y = y.';
end

z = repmat(y,[1 size(x,1)]);


[~,I] = min(abs(x-z.'));
closestValue = x(I);

if rflag
    closestValue = closestValue.';
end