function Sc = combineStructs(S)
fnames = fieldnames(S(1));
for i = 1 : length(fnames)
    try    Sc.(fnames{i}) = vertcat(S.(fnames{i}));
    catch
        Sc.(fnames{i}) = {S.(fnames{i})}.';
    end
end