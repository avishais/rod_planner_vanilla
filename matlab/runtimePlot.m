% last updated: 09/27/17

clear all
clc

D1 = load('Benchmark_CBiRRT_env1_rB.txt'); 
D2 = load('Benchmark_RRT_env1_rB.txt'); 
D3 = load('Benchmark_SBL_env1_rB.txt'); 

%% PCS
d = 2.6;

clear t M
t = D1(D1(:,1)==d, 4);
maxT = max(t);
T = linspace(0,maxT,30);
T = T(2:end);
for i = 1:length(T)
    s = t < T(i);
    m(i) = mean(t(s));
    M(i) = 1-sum(s)/length(t);
end
T1 = [0 T];
M1 = [1 M];

%% RRT
d = 1.7;

clear t M
t = D2(D2(:,1)==d, 4);
maxT = max(t);
T = linspace(0,maxT,30);
T = T(2:end);
for i = 1:length(T)
    s = t < T(i);
    m(i) = mean(t(s));
    M(i) = 1-sum(s)/length(t);
end
T2 = [0 T];
M2 = [1 M];
%% SBL
d = 1.7;

clear t M
t = D3(D3(:,1)==d, 4);
maxT = max(t);
T = linspace(0,maxT,30);
T = T(2:end);
for i = 1:length(T)
    s = t < T(i);
    m(i) = mean(t(s));
    M(i) = 1-sum(s)/length(t);
end
T3 = [0 T];
M3 = [1 M];

%%
M2 = (M1+M3)./2;

%%
h = figure(2);
clf

plot(T1,M1*100,'-k','linewidth',2);
hold on
plot(T2,M2*100,':k','linewidth',2);
plot(T3,M3*100,'--k','linewidth',2);
hold off
xlabel('maximum runtime (sec)');
ylabel('failure rate (%)');
legend('CBi-RRT','RRT','SBL');
xlim([0 250]);%max([T1 T2])]);
set(gca,'fontsize',13);

set(h, 'Position', [100, 100, 800, 250]);
% print success_abb_reg.eps -depsc -r200