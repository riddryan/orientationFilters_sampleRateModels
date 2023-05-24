function getBoundingPosition(src,event,hRect,actFileName)

dexStart = hRect.Position(1);
dexEnd = hRect.Position(1) + hRect.Position(3);

dex = dexStart:dexEnd;

save(actFileName,'dex');

delete(gcf);