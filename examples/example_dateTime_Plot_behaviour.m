t1 = datetime();

tstart = t1;
tstart.Hour = 0;
tstart.Minute = 0;
tstart.Second = 0;

tend = tstart;
tend.Day = tend.Day+2;

time = linspace(t1,t1 + hours(5),1000);
x = rand(size(time));
figure;plot(time,x)

hold on
plot([tstart tend],[0 0],'k.')
% xline([tstart tend]);

% set(gca,'XLim',[tstart tend]);