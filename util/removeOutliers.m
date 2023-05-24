function T = removeOutliers(T,Toutlier)

for i = 1 : size(Toutlier,1)
    suDex = T.Subject == Toutlier.Subject(i);
    seDex = strcmp(T.Sensor,Toutlier.Sensor{i});

    

    CO= Toutlier.Condition{i};
    coDex = false(size(suDex));
    for j = 1 : length(CO)
        co = CO{j};
        if strcmp(co,'all')
            cDex = true(size(coDex));
        else
            cDex = strcmp(T.Condition,co);
        end
        coDex = coDex | cDex;
    end
    rmDex = suDex & seDex & coDex;

    T(rmDex,:) = [];
end