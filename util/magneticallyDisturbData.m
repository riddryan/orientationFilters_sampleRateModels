function D = magneticallyDisturbData(D,opts)

subjnames = fieldnames(D);
seed = 5;
count = 0;

pertMaxNorm = 200;

pertDimMag = pertMaxNorm/sqrt(3);

for ss = 1 : length(subjnames)
    for cc = 1 : length(opts.conditions)
        sensornames = fieldnames(D.(subjnames{ss}).(opts.conditions{cc}));
        for se = 1 : length(sensornames)
            count = count + 1;
            if count == 1
                FS = D.(subjnames{ss}).(opts.conditions{cc}).(sensornames{se}).SampleRate;
                T = linspace(0,2000,2000*FS);
                m0 = zeros(length(T),3);
                mPert = pertDimMag*simulateMagneticDisturbance(T,m0,seed);
            end

            morig = D.(subjnames{ss}).(opts.conditions{cc}).(sensornames{se}).ResampleData.mag;
            pertPortion = mPert(1:size(morig,1),:);
            D.(subjnames{ss}).(opts.conditions{cc}).(sensornames{se}).ResampleData.mag = morig + pertPortion;
%             D.(subjnames{ss}).(opts.conditions{cc}).(sensornames{se}).ResampleData.mag = simulateMagneticDisturbance(D.(subjnames{ss}).(opts.conditions{cc}).(sensornames{se}).ResampleData.time,...
%                 D.(subjnames{ss}).(opts.conditions{cc}).(sensornames{se}).ResampleData.mag,seed);
        end
    end
end