% +------------------------------------------------------+
% |             Phase Difference Measurement             |
% |              with MATLAB Implementation              | 
% |                                                      |
% | Author: M.Sc. Eng. Hristo Zhivomirov        12/01/14 | 
% +------------------------------------------------------+
% 
% function: PhDiff = phdiffmeasure(x, y)
%
% Input:
% x - first signal in the time domain
% y - second signal in the time domain
% 
% Output:
% PhDiff - phase difference Y -> X, rad

function PhDiff = phdiffmeasure(x, y)

% represent the signals as column-vectors
x = x(:);
y = y(:);

% remove the DC component of the signals
x = x - mean(x);
y = y - mean(y);

% signals length calculation
xlen = length(x);
ylen = length(y);

% windows generation
xwin = hanning(xlen, 'periodic');
ywin = hanning(ylen, 'periodic');

% perform fft on the signals
X = fft(x.*xwin); 
Y = fft(y.*ywin);

% fundamental frequency detection
[~, indx] = max(abs(X));
[~, indy] = max(abs(Y));

% phase difference estimation
PhDiff = angle(Y(indy)) - angle(X(indx));

end