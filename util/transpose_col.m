function  [transp_R19] = transpose_col(R19)
% [transp_R19] = transpose_col(R19)
% -------------------------------------------------------------------------
% This function calculates the transpose for a nx9 matrix
% equivalent to R' of transpose(R) in case R is a 3x3 matrix. The advantage, 
% however is that no reshaping of the data is necesarry and that the 
% calculation is done over all time samples at once which prevents 
% loops over time samples.
% -------------------------------------------------------------------------
% % INPUT
% R19     : input matrix (n x 9)
%  
% % OUTPUT
% transp_R19: transposed matrix (n x 9)
%
% (n= number of samples) 
transp_R19=[R19(:,1:3:end) R19(:,2:3:end) R19(:,3:3:end)];