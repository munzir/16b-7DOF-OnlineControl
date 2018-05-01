clc; clear; close all;
dof   = 7;

onlineQ = tdfread('./data/onlineDataQ.txt', '\t');
onlineQ = onlineQ.onlineDataQ;

onlineQref = tdfread('./data/onlineDataQref.txt', '\t');
onlineQref = onlineQref.onlineDataQref;

time = tdfread('./data/onlineDataTime.txt');
time = time.onlineDataTime;

%%
rows = 3; cols = 3;
for i = 1:dof
    subplot(rows, cols, i);
    plot(time, onlineQ(:,i), 'r', time, onlineQref(:,i) ,'g'); grid on;
    xlabel('Time');
    ylabel('State Trajectories');
    leg1 = legend('$q(t)$','$q_{ref}(t)$');
    set(leg1,'Interpreter','latex');
    xlabel(['Joint ' num2str(i)])
end
title('Tracking Error');

% 
% q = tdfread('./data/dataQ.txt', '\t');
% q = q.dataQ;
% 
% qref = tdfread('./data/dataQref.txt', '\t');
% qref = qref.dataQref;
% 
% %%
% figure();
% for i = 1:dof
%     subplot(rows, cols, i);
%     plot(time, q(:,i), time, qref(:,i)); grid on;
%     xlabel('Time');
%     ylabel('State Trajectories');
%     leg1 = legend('$q(t)$','$q_{ref}(t)$');
%     set(leg1,'Interpreter','latex');
%     xlabel(['Joint ' num2str(i)])
% end
% title('Tracking Error');