function qPredicted = gdfPredict(J,F,q,gyro,beta,dt,normStepSize)
deltaF = J * F; % gradient of F
step = deltaF;

if normStepSize
    step = step / norm(step);
end

qDotMeasured = quaternProd(q,[0 gyro]);
% Gradient descent step for qDot
qDot = 0.5 * qDotMeasured - beta* step.';

q = q + dt * qDot;
qPredicted = q / norm(q);