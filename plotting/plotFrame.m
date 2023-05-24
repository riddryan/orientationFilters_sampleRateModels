function hOrigin = plotFrame(pos,R,L,hOrigin)

cols = {[0 0 1] [1 0 0] [0 1 0]};
for dim = 1 : 3
    ar = zeros(1,3);
    ar(dim) = 1*L;
    ar = (R*ar.').';
    ar = num2cell(ar);

    if nargin < 4
        hOrigin(dim) = quiver3(pos(1),pos(2),pos(3),...
            ar{:},'Color',cols{dim});
    else
        hOrigin(dim).XData = pos(1);
        hOrigin(dim).YData = pos(2);
        hOrigin(dim).ZData = pos(3);

        hOrigin(dim).UData = ar{1};
        hOrigin(dim).VData = ar{2};
        hOrigin(dim).WData = ar{3};
    end
end