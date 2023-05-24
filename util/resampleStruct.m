function Ddown = resampleStruct(D,Fold,Fnew)
fnames = fieldnames(D);
if Fold == Fnew
    Ddown = D;
    return
end
told = D.time;
Ddown.time = linspace(told(1),told(end),round((told(end)-told(1))*Fnew)).';
for i = 1 : length(fnames)
    if ~strcmp(fnames{i},'time')
    Ddown.(fnames{i}) = interp1(D.time,D.(fnames{i}),Ddown.time);
    end
end

% Fold = round(Fold);
% Fnew = round(Fnew);
% for i = 1 : length(fnames)
%     if ~strcmp(fnames{i},'time')
%     Ddown.(fnames{i}) = resample(D.(fnames{i}),Fnew,Fold);
%     end
% end
% Ddown.time = linspace(D.time(1),D.time(end),length(Ddown.gyro)).';