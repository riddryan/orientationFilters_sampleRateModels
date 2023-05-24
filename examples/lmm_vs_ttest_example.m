% Generate data

ND = 8;

% Response variable Y
y1 = 10 + 5*rand(ND,1);
y2 = 12 + 5*rand(ND,1);
y3 = 14 + 5*rand(ND,1);
Y = vertcat(y1,y2,y3);

% Categorical variable with 3 levels G
g1 = ones(ND,1);
g2 = 2*ones(ND,1);
g3 = 3*ones(ND,1);
G = vertcat(g1,g2,g3);

% T-tests between groups
[h12,p12]=ttest2(y1,y2);
[h13,p13]=ttest2(y1,y3);
[h23,p23]=ttest2(y2,y3);

% Bonferonni correction
[pa,~,stats]=anova1(Y,G,"off");
[results,means,~,gnames] = multcompare(stats,"CType","bonferroni");
pBon = results(:,end);

pBon12 = pBon(1);
pBon13 = pBon(2);
pBon23 = pBon(3);

% linear mixed model
mdl = 'Y ~ G';

y4 = 15 + 5*rand(ND,1);
g4 = 4*ones(ND,1);
% T = table(Y,num2str(G));
% T = table([y1;y2],[num2str(g1);num2str(g2)]);
T = table([y1;y2;y3;y4],[num2str(g1);num2str(g2);num2str(g3);num2str(g4)]);
T.Properties.VariableNames = {'Y' 'G'};
lme = fitlme(T,mdl)

% 1.6904e-05 
% 4.5369e-05
% 1.8901e-05 