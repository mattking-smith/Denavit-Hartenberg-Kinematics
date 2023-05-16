% Script for reading in DH table and computing forward kinemitics
clear all;
close all;

% Add in functions
addpath('../functions');

% Load DH table
WAM_DH;

% joint angles
q = [1.58, 1.1, 0, 1.7, 0, -1.24, 1.57]';

% joint velocities
qdot = 0.1*ones(7,1);

% Compose homogeneous transformation matrices
basePose = eye(4);
T_B_n = FwdKin(DH,q,basePose);

% struct for describing inertial quantities of manipulator
link = struct('pos',{},'rot',{},'vel',{});

%% Plotting 
% construct triads for plotting
trds = 0.02*eye(3);
clrs = {'r','g','b'};

fig1 = figure(1);
set(fig1,'Name','Wire Frame Manipulator Configuration');
daspect([1 1 1]);
view([60,10]);

% plot manipulator frames
for ii = 1:1:size(T_B_n,3)
    link(ii).rot = T_B_n(1:3,1:3,ii);
    link(ii).pos = T_B_n(1:3,4,ii);

    for kk = 1:3
        trd_rot = link(ii).rot*trds(:,kk);
        line([0, trd_rot(1)] + link(ii).pos(1),...
             [0, trd_rot(2)] + link(ii).pos(2),...
             [0, trd_rot(3)] + link(ii).pos(3),'color',clrs{kk},'linewidth',2);
    end
    
    % line connecting origins
    if ii == 1
        line([0, link(ii).pos(1)],...
             [0, link(ii).pos(2)],...
             [0, link(ii).pos(3)],'color','k','LineStyle','--');
    else
        line([link(ii-1).pos(1), link(ii).pos(1)],...
             [link(ii-1).pos(2), link(ii).pos(2)],...
             [link(ii-1).pos(3), link(ii).pos(3)],'color','k','LineStyle','--');
    end
end
grid on;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');


% Compute Geometric Jacobian
J = GeometricJacobian(T_B_n);

% Linear and angular velocities per joint in work/configuration space
for kk = 1:1:size(J,3)
    link(kk).vel = J(:,:,kk)*qdot;
end
