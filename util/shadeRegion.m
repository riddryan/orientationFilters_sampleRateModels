function shadeRegion(ax,X,col,trans,edgeCol)
% X is Nx2 matrix denoting N regions to shade, where first column is
% starting point and second column is ending point (w.r.t. x axis of plot)

if nargin < 4
    trans = 0.2;
end
if nargin < 3
    col = [0.2 0.2 0.2];
end
if nargin < 5
    edgeCol = 'none';
end

hflag = false;
if ishold
    hflag = true;
end

hold on

% xlim = get(ax,'XLim');
ylim = get(ax,'YLim');

xPts = [X fliplr(X)].';

yPts = [ylim(1);ylim(1);ylim(2);ylim(2)];
yPts = repmat(yPts,1,size(xPts,2));

patch(ax,xPts,yPts,col,'FaceAlpha',trans,'EdgeColor',edgeCol)








if ~hflag
    hold off
end