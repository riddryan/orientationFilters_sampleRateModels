sensorVarianceAssumptions = 1;
stateCovarianceAssumption = 2;
assumeSeparable = false;
assumeAccMagSameVariance = false;
combinedAccAndMag = false;
%% Sensor Variance

switch sensorVarianceAssumptions
    case 1
        %% Define covariance matrices (1 variance term - same for each dimension)
        syms rg ra rm
        Q = diag([rg rg rg rg]);
        Ra = diag([ra ra ra ra]);
        Rm = diag([rm rm rm rm]);
        R = blkdiag(Ra,Rm);
%         syms P [4 4]
        
        
    case 2
        %% Define covariance matrices (1 variance term per dimension)

        Q = sym.zeros(4);
        Ra = sym.zeros(4);
        Rm = sym.zeros(4);
        for i = 1 : 4
            for j = i
                gstr = "rg" + i + j;
                syms(gstr);
                Q(i,j) = eval(gstr);

                astr = "ra" + i + j;
                syms(astr);
                Ra(i,j) = eval(astr);

                mstr = "rm" + i + j;
                syms(mstr);
                Rm(i,j) = eval(mstr);

            end
        end
        R = blkdiag(Ra,Rm);
%         syms P [4 4]
    case 3
        %% Define covariance matrices (with covariance terms)

        Q = sym.zeros(4);
        Ra = sym.zeros(4);
        Rm = sym.zeros(4);
        for i = 1 : 4
            for j = i:4
                gstr = "rg" + i + j;
                syms(gstr);
                Q(i,j) = eval(gstr);
                if i~=j
                    Q(j,i) = Q(i,j);
                end

                astr = "ra" + i + j;
                syms(astr);
                Ra(i,j) = eval(astr);
                if i~=j
                    Ra(j,i) = Ra(i,j);
                end

                mstr = "rm" + i + j;
                syms(mstr);
                Rm(i,j) = eval(mstr);
                if i~=j
                    Rm(j,i) = Rm(i,j);
                end
            end
        end
        R = blkdiag(Ra,Rm);
%         syms P [4 4]
end
syms dt
Q = dt.^2/4 * Q;
%% State Variance
switch stateCovarianceAssumption
    case 1
        P = sym.zeros(4);
    case 2
        syms p
        P = diag([p p p p]);
    case 3
        syms p11 p22 p33 p44
        P = diag([p11 p22 p33 p44]);
    case 4
end
%% Predict


syms wx wy wz
syms q [4 1]
% Gyroscope measurement
omega4=[0,-wx,-wy,-wz;
    wx,0,wz,-wy;
    wy,-wz,0,wx;
    wz,wy,-wx,0];

A = sym.eye(4) + dt/2*omega4; % State transition model

q_p = A*q;  % predicted state
P_p = A*P*A.' + Q; %PRedicted state covariance


%% Correct
syms qmag [4 1]
syms qacc [4 1]

    e = [qacc;qmag] - [q_p;q_p];


if assumeSeparable
    H = eye(4);
    R1 = Ra;
    S1 = H * P_p * H.' + R1; % Innovation covariance
    K1 = P_p * H.' / S1; % Optimal kalman gain

    if assumeAccMagSameVariance
        S2 = S1;
        K2 = K1;
    else
        R2 = Rm;
        S2 = H * P_p * H.' + R2; % Innovation covariance
        K2 = P_p * H.' / S2; % Optimal kalman gain
    end
%     K = K1 + K2;
%     e1 = qa - q_p;
%     e2 = qm - q_p;
    K = [K1 K2];
     H = [eye(4);eye(4)];
        
%     q_c = q_p + K1*e1 + K2*e2;
%     q_c = q_p + K1 * e; % Corrected State
%     P_c = (eye(4) - K*H)*P_p; % Corrected state covariance
else
    H = [eye(4);eye(4)]; % Observation Model

    S = H * P_p * H.' + R; % Innovation covariance
    K = P_p * H.' / S; % Optimal kalman gain


end

if combinedAccAndMag
    syms qmeas [4 1]
    e = qmeas - q_p;
    K = K(1:4,1:4) + K(1:4,5:8);
    H = eye(4);
end


q_c = q_p + K * e; % Corrected State
P_c = (eye(4) - K*H)*P_p; % Corrected state covariance

[n,d] = numden(K(1,1));

nc = collect(n,dt);
dc = collect(d,dt);

%% Set everthing besides dt is a constant
KS = K;

pVars = symvar(P);
% KS = subs(KS,pVars,ones(size(pVars)));

qpVars = symvar(q_p);
qpVars(qpVars == dt) = [];
% KS = subs(KS,qpVars,ones(size(qpVars)));

QVars = symvar(Q);
RVars = symvar(R);

repVars = [pVars qpVars QVars RVars];
repVars(repVars == dt) = [];
repVars = unique(repVars);
KS = subs(KS,repVars,ones(size(repVars)));



