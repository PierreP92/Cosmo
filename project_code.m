clear all
close all

T=500; % mvt time (in ms)
disturb=10; % prism magnitude

% Diagonal matrix for the change in state from one step to the other
A = diag([1 1 1]);

% Noise of each source of disturbance in real world (state-space)
sigma_r_vis=0.1.^2;
sigma_r_propr=0.1.^2;
sigma_r_mot=0.1.^2;
Qs = diag([sigma_r_vis sigma_r_propr sigma_r_mot]);
e_s = sample_gaussian(zeros(length(Qs),1), Qs, T)';

% Observation noise
sigma_vis = .5;
sigma_propr = .5;
Qobs = diag([sigma_vis sigma_propr]);
e_obs = sample_gaussian(zeros(length(Qobs),1), Qobs, T)';

% Observation matrix
H=[1 0 1;
   0 1 1];

% Initialises the system
r(:,1)=[disturb 0 0]';
z(:,1) = H*r(:,1) + e_obs(:,1);
y(:,1) = 0;

R=H*[sigma_vis sigma_propr 0]'*[sigma_vis sigma_propr 0]*H';

V=diag([0.01 0.01 0.01]);

for t = 2:T, 
    r(:,t)=A*r(:,t-1) + e_s(:,t); % Real space
    z(:,t)=H*r(:,t) + e_obs(:,t); % Observation
    
    
    
end
[x, V, VV, loglik,zpred]=kalman_filter(z, A, H, Qs, R, [0 0 0]',diag([0.1 0.1 0.1]));



figure(1)
subplot(4,1,1)
plot(1:T, r(1,:),'r', 1:T, z(1,:),'g', 1:T, zpred(1,:),'b');
title('Visual disturbances')
legend('True state','Observation','Estimate')

subplot(4,1,2)
plot(1:T, r(2,:),'r', 1:T, z(2,:),'g', 1:T, zpred(2,:),'b');
title('Proprioceptive disturbances')
legend('True state','Observation','Estimate')

subplot(4,1,3)
plot(1:T, r(3,:),'r', 1:T, zpred(3,:),'b');
title('Motor disturbances')
legend('True state','Estimate')

% subplot(4,1,4)
% plot(1:T, sum(r,1),'r', 1:T, sum(z,1),'g', 1:T, sum(zpred,1),'b');
% title('Total disturbances')
% legend('True state','Observation','Estimate')
% 







