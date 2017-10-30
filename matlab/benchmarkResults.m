% Benchmarkin rod planning with CBi-RRT for env. 1 (ABB with two poles)
% With joint limits of the ABB robots.
% last updated: 10/11/17

clear all
clc

%%
planners = {'CBiRRT','RRT','SBL','PRM'};
plannerType = planners{2};
switch plannerType
    case 'CBiRRT'
%         D = load('Benchmark_CBiRRT_env1_rB.txt'); D = D(D(:,2)==1,:);
        D = load('Benchmark_CBiRRT_env1_temp.txt'); D = [2.6*ones(size(D,1),1) D]; D = D(D(:,2)==1,:); 
    case 'RRT'
        D = load('Benchmark_RRT_env1_rB.txt'); D = D(D(:,2)==1,:);
    case 'SBL'
        D = load('Benchmark_SBL_env1_rB.txt'); D = D(D(:,2)==1,:);
%         D = load('Benchmark_SBL_env1_temp.txt'); D = [1.7*ones(size(D,1),1) D]; D = D(D(:,2)==1,:);
    case 'PRM'
        D = load('Benchmark_PRM_env1.txt'); D = D(D(:,2)==1,:);
end

%% 
disp(['Results for ' plannerType ':']);

%%
rd = sort(unique(D(:,1)));
for i = 1:length(rd)
    M = D(D(:,1)==rd(i), 2:end);
    td(i) = mean(M(:,3));
    td_ste(i) = std(M(:,3))/sqrt(size(M,1));
end

%%
disp(' ');
[tdmin, id] = min(td);
% id = 2;
% tdmin = td(id);

%%
h = figure(1);
clf
errorbar(rd,td,td_ste,'-k','linewidth',2);
ylabel('mean runtime [sec]');
xlabel('max. local-connection distance');
% xlim([0 6]);
% xlim([min(rd) max(rd)]);

%% 
sS = rd(id);
tmin = tdmin;
D = D(D(:,1)==sS, 2:end);
suc = D(:,1)==1;

disp('------------------------------------');
disp(['Results of ' num2str(size(D,1)) ' queries.']);
disp(['Minimum avg. runtime is ' num2str(tmin) 'sec with d = ' num2str(sS) ]);
disp(['Plan distance: ' num2str(D(1,2)) ]);
disp(['Avg. runtime: ' num2str(mean(D(:,3)))  ' +/- ' num2str(std(D(:,3))/sqrt(size(D,1))) ' sec ']);
disp(['Min. runtime: ' num2str(min(D(:,3))) ' msec ']);
disp(['Avg. nodes in path: ' num2str(floor(mean(D(:,9)))) ]);
disp(['Avg. nodes in trees: ' num2str(floor(mean(D(:,10)))) ]);
disp(['Avg. number of projections: ' num2str(floor(mean(D(:,4)))) ]);
disp(['Avg. local-connection time: ' num2str(mean(D(:,11)./D(:,12))*1e3)  ' +/- ' num2str(std(D(:,11)./D(:,12))/sqrt(size(D,1))*1e3) ' msec ']);
disp(['Avg. total local-connection time: ' num2str(mean(D(:,11)))  ' +/- ' num2str(std(D(:,11))/sqrt(size(D,1))*1e3) ' sec ']);
disp(['Avg. number of local connection checks: ' num2str(mean(D(:,12)))]);
disp(['Avg. collision check time: ' num2str(mean(D(:,7))) ' sec.']);
disp(['Percent of successful local connections: ' num2str(100*mean(D(:,13)./D(:,12))) '%.']);
disp(['Sampling time: ' num2str(mean(D(:,14))) ' sec']);
disp(['Sampling success rate: ' num2str(100*mean( D(:,15)./(D(:,15)+D(:,16)) )) '%']);
disp(['ODE time: ' num2str(mean(D(:,19))) ' sec which is ' num2str(100*mean(D(:,19)./D(:,3))) '% of runtime.']);
disp(['ODE checks: ' num2str(mean(D(:,17))) ' with ' num2str(100*mean( D(:,18)./D(:,17) )) '% successful.']);
%%
%%
%%
td = D(:,3);
maxT = max(td);
T1 = linspace(0,maxT,25);
T1 = T1(2:end);
for i = 1:length(T1)
    sd = td < T1(i);
    md(i) = mean(td(sd));
    Md(i) = 1-sum(sd)/length(td);
end
T1 = [0 T1];
Md = [1 Md];

%%
h = figure(2);
clf
plot(T1,Md*100,'-k','linewidth',2);
xlabel('maximum runtime [sec]');
ylabel('failure rate [%]');
xlim([0 max(T1)]);
title(plannerType);
set(h, 'Position', [100, 100, 800, 400]);
% print success_abb_reg.eps -depsc -r200

%% Runtime break
% D = load('Benchmark_CBiRRT_env1_rB.txt'); D = D(D(:,2)==1,:);
% M = D(D(:,1)==2.6, 2:end);
% v = mean(M(:,:));
% 
% t2(1) = v(3); % Total runtime
% t2(2) = v(5)-v(19)-v(7); % IK time
% t2(3) = v(7); % Collision time
% t2(4) = v(19); % ODE
% 
% n2(1) = v(4);
% n2(2) = v(6);
% n2(3) = v(17);