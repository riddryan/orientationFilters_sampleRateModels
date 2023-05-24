function [zStar,costStar,optim] = optimizeAHRSparameters(cost,algorithm,optimOpts,FS)
%% Problem definition

lb = optimOpts.(algorithm).parameters.lb;
ub = optimOpts.(algorithm).parameters.ub;


z0 = optimOpts.(algorithm).parameters.Initial;
zStar = [];
optim = struct();

% Display = 'iter-detailed';
Display = optimOpts.OptimDisplay;

optMethod = optimOpts.optimizationMethod;

if isfield(optimOpts.(algorithm),'optimizer')
    optMethod = optimOpts.(algorithm).optimizer;
end
if optimOpts.alwaysUseLocalOptimFor1D && length(lb) == 1
    optMethod = 'fmincon';
end

%% algorithm specific bounds based on sampling frequency
if strcmp(algorithm,'RiddickAHRS')
    pnames = optimOpts.(algorithm).parameters.Names;

%     dex = find(strcmp(pnames,'c_gyroscope'));
%      [~,dex] = intersect(pnames,{'c_gyroscope' 'c_gyroscope2' 'c_gyroscope3'});
     [~,dex] = intersect(pnames,{'c_gyroscope'});
    if ~isempty(dex)
        % Gyroscope gain should be near 1 at high freqs
        if FS > 30
            lb(dex) = 0.9;
        elseif FS > 5
            lb(dex) = 0.1;
        else
            lb(dex) = 0.0001;
        end
    end
elseif strcmp(algorithm,'JustaAHRS')
    if FS > 60
        optimOpts.JustaAHRS.parameters.ub = 0.01*ones(1,2);
    elseif FS > 30
        optimOpts.JustaAHRS.parameters.ub = 0.1*ones(1,2);
    else
        optimOpts.JustaAHRS.parameters.ub = 1*ones(1,2);
    end
end

%%
tic;
switch optMethod
    case 'fmincon'

        fopts = optimoptions('fmincon');
        fopts.OptimalityTolerance = 1e-2;
        fopts.StepTolerance = 1e-6;
        fopts.MaxIterations = optimOpts.fmincon.MaxIterations;
        fopts.MaxFunctionEvaluations = optimOpts.fmincon.MaxFunctionEvaluations;
%         fopts.ScaleProblem = 'obj-and-constr';
        % fopts.ConstraintTolerance = 1e-3;
        fopts.Display = Display;
        fopts.Algorithm = 'sqp';

        problem = createOptimProblem('fmincon','x0',z0,'objective',cost,...
            'lb',lb,'ub',ub,'options',fopts);

        [zStar,costStar,eflag,output] = fmincon(problem);

        optim.options = fopts;
        optim.output = output;
        optim.eflag = eflag;
    case 'surrogate'
        if strcmp(Display,'iter-detailed')

            Display = 'iter';
        end
        options = optimoptions('surrogateopt');
        options.Display = Display;
        % MP = rand(20,length(lb)).*(ub.'-lb.') + lb.';
        % options.InitialPoints = MP;
        options.MaxFunctionEvaluations = optimOpts.surrogate.Iterations;
        % options.UseParallel = true;
        options.MinSampleDistance = optimOpts.surrogate.MinSampleDistance;
        options.MinSurrogatePoints = optimOpts.surrogate.MinSurrogatePoints;
        options.PlotFcn = 'surrogateoptplot';
        options.UseParallel = optimOpts.surrogate.UseParallel;



        problem = struct('objective',cost,...
            'lb',lb,...
            'ub',ub,...
            'options',options,...
            'solver','surrogateopt');
        figure(3);
        [zStar,costStar,eflag,output,trials] = surrogateopt(problem);

        % figure;
        % for b = 1 : size(trials.X,2)
        %     if size(trials.X,2)>4
        %         subplot(2,3,b)
        %     elseif size(trials.X,2)>2
        %         subplot(2,2,b)
        %     elseif size(trials.X,2)>1
        %         subplot(1,2,b)
        %     end
        %
        %     plot(trials.X(:,b),trials.Fval,'.');
        %     legend(opts.(ahrsClass).parameters.Names{:});
        %     xlabel('Parameter Value')
        %     ylabel('MSE')
        %     title([ahrsClass ' ' opts.conditions{opts.condition} ' ' opts.(ahrsClass).parameters.Names{b}]);
        % end

        % Perform local optimization afterwards
        if optimOpts.surrogate.fminconAfter
            localoptions = optimoptions('fmincon');
            localoptions.Display = Display;
            localoptions.OptimalityTolerance = 1e-2;
            localoptions.StepTolerance = 1e-6;
            localoptions.MaxIterations = optimOpts.MultiStart.MaxIterations;
            localoptions.MaxFunctionEvaluations = 80;
            localoptions.ConstraintTolerance = 1e-3;
            localoptions.Algorithm = 'sqp';
            localProblem = struct('objective',cost,...
                'lb',lb,...
                'ub',ub,...
                'options',localoptions,'x0',zStar,...
                'solver','fmincon');
            [zStar,costStar,eflagL,outputL] = fmincon(localProblem);
            optim.localOptions = localoptions;
        end

        optim.options = options;
        optim.output = output;
        optim.eflag = eflag;
        optim.trials = trials;
    case 'MultiStart'
        ms = MultiStart;
        if strcmp(Display,'iter-detailed')

            Display = 'iter';
        end
ms.Display = Display;
ms.UseParallel = optimOpts.MultiStart.UseParallel;
ms.FunctionTolerance = 1e-3;
ms.XTolerance = 1e-3;

fopts = optimoptions('fmincon');
fopts.OptimalityTolerance = 1e-2;
fopts.StepTolerance = 1e-5;
fopts.MaxIterations = optimOpts.MultiStart.MaxIterations;
fopts.MaxFunctionEvaluations = optimOpts.MultiStart.MaxFunctionEvaluations;
fopts.ConstraintTolerance = 1e-3;
fopts.Algorithm = 'sqp';
problem = createOptimProblem('fmincon','x0',z0,'objective',cost,...
    'lb',lb,'ub',ub,'options',fopts);

% np = opts.MultiStart.Number;
% for i = 1 : length(lb)
%    MP(:,i) = linspace(lb(i),ub(i),np);
% end
% tpoints = CustomStartPointSet(MP);
% 
% [zStar,costStar,eflag,output,solutions] = run(ms,problem,tpoints);
[zStar,costStar,eflag,output,solutions] = run(ms,problem,optimOpts.MultiStart.Number);

optim.eflag = eflag;
optim.output = output;
optim.solutions = solutions;
optim.optimizer = ms;



% sx = vertcat(solutions.X);
% xx = sx(:,1);
% 
% fv = vertcat(solutions.Fval);
% 
% % Append cost of starting points as well
% x0 =  cell2mat(vertcat(solutions.X0));
% for i = 1 : length(x0)
%    C0(i,1) = cost(x0(i,:) );
% end
% xx = [xx;x0(:,1)];  fv = [fv;C0];
% 
% 
% if length(zStar) > 1
%     yy = sx(:,2);
%     yy = [yy;x0(:,2)];
% 
%     N = 250;
% xvec = linspace(min(xx), max(xx), N);
% yvec = linspace(min(yy), max(yy), N);
% [X, Y] = ndgrid(xvec, yvec);
% F = scatteredInterpolant(xx, yy, fv);
% Z = F(X, Y);
% figure;
% surf(X, Y, Z, 'edgecolor', 'none');
% hold on
% plot3(xx,yy,fv,'k.')
% else
% 
%     figure
%     plot(xx,fv,'.')
%     hold on
%     plot(zStar,costStar,'ro')
% end
% 
% NP = length(lb);
% 
% nsbplt = NP;
% if mod(nsbplt,2) && nsbplt ~= 1
%     nsbplt = nsbplt + 1;
% end
% if NP > 2
%     numRows = 2;
% else
%     numRows = 1;
% end
% figure(4)
% clf;
% 
% p = [vertcat(solutions.X);x0];
% 
% for pp = 1 : NP
% subplot(numRows,nsbplt/numRows,pp)
% plot(p(:,pp),fv,'.')
% title(opts.(ahrsClass).parameters.Names{pp})
% end

    case 'GlobalSearch'
        fopts = optimoptions('fmincon');
        fopts.OptimalityTolerance = 1e-2;
        fopts.StepTolerance = 1e-5;
        fopts.MaxIterations = optimOpts.MultiStart.MaxIterations;
        fopts.MaxFunctionEvaluations = 80;
        fopts.ConstraintTolerance = 1e-3;
%         fopts.Algorithm = 'sqp';

        problem = createOptimProblem('fmincon','x0',z0,'objective',cost,...
            'lb',lb,'ub',ub,'options',fopts);

        gs = GlobalSearch;
        gs.Display = Display;
        gs.NumTrialPoints = optimOpts.GlobalSearch.NumTrialPoints;
        gs.NumStageOnePoints = optimOpts.GlobalSearch.NumStageOnePoints;
        gs.FunctionTolerance = optimOpts.GlobalSearch.FunctionTolerance;
        gs.XTolerance = optimOpts.GlobalSearch.XTolerance;
        [zStar,costStar,eflag,output,solutions] = run(gs,problem);

        optim.output = output;
        optim.eflag = eflag;
        optim.solutions = solutions;
        optim.optimizer = gs;

    case 'Justa'

        maxiter = optimOpts.Justa.MaxIter;
        NP = length(lb);
        zStar = NaN(size(lb));
        for p = 1 : NP
            iter = 0;
            
            p0 = z0(p);
            while iter < maxiter
                iter = iter + 1;
                p0 = z0(p);

                localP = [0.3 0.6 0.9 0.95 0.999 1.001 1.05 1.1 1.4 1.7 rand(1,4)]*p0;
                E = NaN(1,length(localP));
                for i = 1 : length(localP)
                    z = z0;
                    z(p) = localP(i);
                    E(1,i) = cost(z);
                end
                [~, index] = min(E);
                p0 = localP(index);
            end
            pStar = p0;
            zStar(p) = pStar;

%             fprintf('%s = %.04f ',optimOpts.(algorithm).parameters.Names{p},pStar);
        end
%         fprintf('\n')
        costStar = cost(zStar);
%         fprintf('Cost = %.04f\n',costStar)
      
end
optim.time = toc;

for p = 1 : length(lb)
    fprintf('\n%s = %.04g ',optimOpts.(algorithm).parameters.Names{p},zStar(p));
end
fprintf('\nCost = %.04g\n',costStar');

%% Analyze result
if isempty(zStar)
    zStar = z0;
end



% qStar = estimateOrientation(AHRSstar,data,info,opts);
% q0 = estimateOrientation(AHRS0,data,info,opts);
% 
% eaStar = eulerd(quaternion(qStar),'ZYX','point');
% % ea0 = eulerd(quaternion(q0),'ZYX','point');
% eaGround = eulerd(quaternion(data.ground_truth),'ZYX','point');
% 
% eaGroundOrig = eulerd(quaternion(dataOrig.ground_truth),'ZYX','point');
% figure(1)
% clf;
% % plot(data.time,eaGround,'k');
% % hold on
% plot(data.time,eaStar,'r')
% hold on
% plot(dataOrig.time,eaGroundOrig,'b');
% % plot(ea0,'b');
% title([ahrsClass ' ' opts.conditions{opts.condition} ' ' num2str(round(opts.Fs)) ' Hz'],'Interpreter','none');
% % legend('ground truth','estimate')
% ylabel('Euler Angles');
% drawnow;
