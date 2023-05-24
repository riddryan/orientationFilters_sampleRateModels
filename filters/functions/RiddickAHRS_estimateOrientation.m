function qEstimated = RiddickAHRS_estimateOrientation(q0,gyro,acc,mag,gains,dt,normStepSize,indexOffset)

NT = size(gyro,1);
qEstimated = NaN(NT,4);
for tt = 1:(NT-1-indexOffset)
    if tt == 1
        qEstimated(tt,:) = q0;
    end
    qPredicted = RiddickAHRS_update(qEstimated(tt,:),gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,normStepSize);
    %             qPredicted = RiddickAHRS_update_mex(qEstimated(tt,:),gyro(tt,:),acc(tt,:),mag(tt,:),gains,obj.SamplePeriod,obj.normStepSize);
    qEstimated(tt+1+indexOffset,:) = qPredicted;
end