function qEstimated = estimateOrientation_Fast(filter,q0,gyro,acc,mag,gains,dt,normStepSize,indexOffset,P)
if nargin < 10
    P = [];
end


NT = size(gyro,1);
qEstimated = NaN(NT,4);
for tt = 1:(NT-1-indexOffset)
    if tt == 1
        qEstimated(tt,:) = q0;
    end
    if indexOffset < 0 && tt > 1
        q = qEstimated(tt-1,:);
    else
        q = qEstimated(tt,:);
    end

    switch filter
        case 'RiddickAHRS'
            qPredicted = RiddickAHRS_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,normStepSize);
        case 'WilsonAHRS'
            qPredicted = WilsonAHRS_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,normStepSize);
        case 'AdmiraalAHRS'
            qPredicted = AdmiraalAHRS_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,normStepSize);
        case 'MadgwickAHRS3'
            qPredicted = MadgwickAHRS3_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,normStepSize);
        case 'JustaAHRS'
            qPredicted = JustaAHRS_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,normStepSize);
        case 'FastKF'
            [qPredicted,P] = FastKF_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,P);
        otherwise
            qPredicted = NaN(1,4);
    end

    qEstimated(tt+1+indexOffset,:) = qPredicted;
end