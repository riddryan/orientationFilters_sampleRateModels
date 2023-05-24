function  [Rproduct] = prod_col(Rfirst, Rsecond)
% function: [Rproduct] = prod_col(Rfirst, Rsecond)
% -------------------------------------------------------------------------
% This function calculates product of two nx9 matices in the same way a
% multiplication of two 3x3 matrices is done in by matlab (R*R), but than
% over all time samples at once. This is faster and prevents loops over
% time.
% It also calculates the product of a orientation matrix (nx9), and a
% vector (either 1x3 or nx3)
% -------------------------------------------------------------------------
% % INPUT
% Rfirst  : first orientation matrix (n x 9 or 1 x 9)
% Rsecond : second orientation matrix (n x 9 or 1 x 9) or vector (n x 3 or 1 x 3)
%  
% % OUTPUT
% Rproduct: product of Rfirst and Rsecond (n x 9)
%
% (n= number of samples) 

[m,n]  =size(Rfirst);
[m1,n1]=size(Rsecond);
if n==3
    error('first argument can not be a vector, only second argument can be a vector!!')
end
if m1==1
    Rsecond=ones(m,1)*Rsecond;
end
if m==1
    Rfirst=ones(m1,1)*Rfirst;
end

no_col=n1/3;

matrix_1=[ 1 4 7
    2 5 8
    3 6 9];
matrix_2=reshape(1:n1,3,no_col);

Rproduct=zeros(max([m m1]),n1);
for i_row=1:3
    row=matrix_1(i_row,:);
    for i_kol=1:no_col
        column=matrix_2(:,i_kol);
        Rproduct(:,matrix_2(i_row,i_kol))=sum    (Rfirst(:,row).*Rsecond(:,column),2);
    end
end

