function qPredicted = JustaAHRS_update(q,gyro,acc,mag,gains,dt,normStepSize)
wAcc = gains(1);
wMag = gains(2);


%% Normalize
acc = acc / norm(acc);
mag = mag / norm(mag);
%% 


            wt=gyro*dt/2;
            qDotx=sin(wt(1));
            qDoty=sin(wt(2));
            qDotz=sin(wt(3));
    
            qdiff = 1-sum([qDotx qDoty qDotz].^2);
            if qdiff >= 0
                qDotw=sqrt(qdiff);
            else
                qDotw = 0;
            end
            
            qDot=[qDotw qDotx qDoty qDotz];

            quatGyrPred= quaternProd(q,qDot);  %q + qDot*obj.SamplePeriod;


            qp=quatGyrPred;

            if ~any(isnan(mag))
                % Get predicted reference directions based on current
                % orientation for gravity and magnetic fields
                R =[2*(0.5 - qp(3)^2 - qp(4)^2)   0   2*(qp(2)*qp(4) - qp(1)*qp(3))
                    2*(qp(2)*qp(3) - qp(1)*qp(4))  0  2*(qp(1)*qp(2) + qp(3)*qp(4))
                    2*(qp(1)*qp(3) + qp(2)*qp(4))  0  2*(0.5 - qp(2)^2 - qp(3)^2)];

                ar=[0 0 1];
                accMesPred=(R*ar')';


                mr_z= dot(accMesPred,mag);
                mdiff = 1-mr_z^2;
                if mdiff >= 0
                    mr_x= sqrt(mdiff);
                else
                    mr_x = 0;
                end
                mr=[mr_x 0 mr_z];

                magMesPred=(R*mr')';

                % Compare to actual measurements
                ca=cross(acc,accMesPred);
                cm=cross(mag,magMesPred);

                if normStepSize
                    vecA=ca/norm(ca);
                    vecB=cm/norm(cm);
                else
                    vecA = ca;
                    vecB = cm;
                end


                % Update quaternion based on the error and gains for mag and
                % acc
                im=vecA*wAcc/2+vecB*wMag/2;
                im2=im*sinc(norm(im)/pi);

                qCor=[sqrt(1-sum(im2.^2)),im2];

            else

                ar=[0 0 1];

                R=[0 0  2*(qp(2)*qp(4) - qp(1)*qp(3))
                    0 0  2*(qp(1)*qp(2) + qp(3)*qp(4))
                    0 0  2*(0.5 - qp(2)^2 - qp(3)^2)];

                accMesPred=(R*ar')';

                ca=cross(acc,accMesPred);
                if normStepSize
                    veca=ca/norm(ca);
                else
                    veca = ca;
                end

                qCor=[1 veca*wAcc/2];

            end

            quat=quaternProd(qp,qCor);

            if(quat(1)<0)
                quat=-quat;
            end
            qPredicted = quat/norm(quat);