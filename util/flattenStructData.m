function Sflat = flattenStructData(S)
    snames = fieldnames(S);
    for i = 1 : length(snames)
        Sflat.(snames{i}) = vertcat(S.(snames{i})(:));
    end
end