function [qEstimated,qa,qm] = estimateOrientation_Fast2(filter,q0,gyro,acc,mag,gains,dt,normStepSize,P,intOrder)%#codegen
if nargin < 8
    normStepSize = true;
end
if nargin < 9
    P = [];
end
if nargin < 10
    intOrder = 1;
end

NT = size(gyro,1);
qEstimated = NaN(NT,4);
qPredicted = q0;
dqdtPrev = zeros(intOrder,4);

qa = NaN(NT,4);
qm = NaN(NT,4);
% 
% qa = NaN(NT,4);
% qm = NaN(NT,4);

%% Loop
for tt = 1:NT
    q = qPredicted;

%     if tt == 39261
%         keyboard
%     end

    switch filter
        case 'RiddickAHRS'
            %             if intOrder == 1
            %                 [qPredicted] = RiddickAHRS_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,normStepSize);
            %                 qCorrected = qPredicted;
            %             else
            %                 [qPredicted,qCorrected,dqdtN] = RiddickAHRS_update_higherOrder(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,normStepSize,intOrder,dqdtPrev);
            %                 dqdtPrev = dqdtN;
            %             end

            [qPredicted,qCorrected,dqdtN,qa(tt,:),qm(tt,:)] = RiddickAHRS_update_higherOrder(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,normStepSize,intOrder,dqdtPrev);
            dqdtPrev = dqdtN;

        case 'WilsonAHRS'
            qPredicted = WilsonAHRS_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,normStepSize);
            qCorrected = qPredicted;
        case 'AdmiraalAHRS'
            qPredicted = AdmiraalAHRS_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,normStepSize);
            qCorrected = qPredicted;
        case 'MadgwickAHRS3'
            qPredicted = MadgwickAHRS3_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,normStepSize);
            qCorrected = qPredicted;
        case 'JustaAHRS'
            qPredicted = JustaAHRS_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,normStepSize);
            qCorrected = qPredicted;
        case 'FastKF'
            [qPredicted,P] = FastKF_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,P);
            qCorrected = qPredicted;
        case 'KF'
            [qPredicted,qCorrected,P] = KF_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,P);
%             qCorrected = qPredicted;
        case 'KF2'
            [qPredicted,qCorrected,P] = KF2_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,P);
        case 'ticcKF'
            [qPredicted,qCorrected] = ticcKF_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt);
        case 'ticcKFexp'
            [qPredicted,qCorrected] = ticcKFexp_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt);
        case 'eKF'
            [qPredicted,qCorrected,P] = eKF_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,P);
        case 'eKF2'
            [qPredicted,qCorrected,P] = eKF2_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,P);
        case 'esKF'
            [qPredicted,qCorrected,P] = esKF_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,P);
        case 'esKF2'
            [qPredicted,qCorrected,P] = esKF2_update(q,gyro(tt,:),acc(tt,:),mag(tt,:),gains,dt,P);
        otherwise
            qPredicted = NaN(1,4);
            qCorrected = NaN(1,4);
    end
    
    qEstimated(tt,:) = qCorrected;

end