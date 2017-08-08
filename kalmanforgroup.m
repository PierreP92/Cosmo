%% Update step
% compute innovation (error) e:
% r=[rv,rp,ry];
clear;
qv=.01;qp=.01;qy=0.01;av=1;ap=1;ay=1;sigv=.1;sigu=0.05;sigp=.5;
%% State space:
handReal % x and y
targetReal % x and y
visualPert
propPert
motorPert

%% Observation space
visualObsTarget % x and y
% propObsTarget
visualObsHand % x and y
propObsHand % x and y

%% Estimation space
visualEstTarget % Kalman filter
visualEstHand % Kalman filter
propEstHand
combEstHand
combEstTarget
visualEstPert = visualEstHand(t) - visualPredHand(t-1); % maybe we get this from the cue combination
propEstPert = propEstHand(t) - propPredHand(t-1); % maybe we get this from the cue combination

%% Prediction space
visualPredHand 
propPredHand

%% Motor command
% The motor command is a vector emerging from the multimodal 
% estimate of the hand and pointing at the multimodal estimate 
% of the target and of length 1

%% Insert code here:

A=


H=[1 0 1;0 1 1]; % observation
Q=diag([qv,qp,qy]);
A=diag([av,ap,ay]);
Sigma=[sigv;sigp;sigu];
R=H*Sigma*Sigma'*H';
xpos(:,1)=[0; 0; 0];xpri(:,1)=[0; 0; 0];ppri=diag([1,1,1]);

%% simulation
[x,z] = sample_lds(A, H, Q, R, xpri(:,1), 100);
%   x(t+1) = A*x(t) + w(t),  w ~ N(0, Q),  x(0) = init_state
%   y(t) =   C*x(t) + v(t),  v ~ N(0, R)
%%
%z=[zeros(1,20)+randn(1,20)*.05,exp(-.1*(1:1:80))+randn(1,80)*.05;zeros(1,100)+.2];
%%
for i=1:100

e(:,i)=z(:,i)-H*xpri(:,i); 
s=R+H*ppri*H';
k=ppri*H'*pinv(s);
xpos(:,i)=xpri(:,i)+k*e(:,i);
ppos=(eye(3)-k*H)*ppri;
xpri(:,i+1)=A*xpos(:,i);
ppri=A*ppos*A'+Q;

end
figure;plot(x');hold on;plot(xpos');legend('rv','rp','ry','erv','erp','ery');
