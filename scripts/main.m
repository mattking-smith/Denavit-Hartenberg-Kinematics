% Script for reading in DH table and computing forward kinemitics
clear all;
close all;

% Add in functions
addpath('../functions');

% Load DH table
KinovaGen3_DH;

% joint angles
q = [pi;pi/4;pi/6;-pi/6;-pi/4;-pi/2;-pi];

% joint velocities
qdot = 0.1*ones(7,1);

% choose parameterization convention (DH, modDH)
conven = modDH;

% Compose homogeneous transformation matrices
T_B_n = FwdKin(conven,q);

% struct for describing inertial quantities of manipulator
joint = struct('pos',{},'rot',{},'linVel',{},'angVel',{});

% construct triads for plotting
trds = 0.02*eye(3);
clrs = {'r','g','b'};

fig1 = figure(1);
set(fig1,'Name','Manipulator Configuration');
daspect([1 1 1]);
view([60,10]);

% plot manipulator frames
for ii = 1:1:size(T_B_n,3)
    joint(ii).rot = T_B_n(1:3,1:3,ii);
    joint(ii).pos = T_B_n(1:3,4,ii);

    for kk = 1:3
        trd_rot = joint(ii).rot*trds(:,kk);
        line([0, trd_rot(1)] + joint(ii).pos(1),...
             [0, trd_rot(2)] + joint(ii).pos(2),...
             [0, trd_rot(3)] + joint(ii).pos(3),'color',clrs{kk},'linewidth',2);
    end
    
    % line connecting origins
    if ii == 1
        line([0, joint(ii).pos(1)],...
             [0, joint(ii).pos(2)],...
             [0, joint(ii).pos(3)],'color','k','LineStyle','--');
    else
        line([joint(ii-1).pos(1), joint(ii).pos(1)],...
             [joint(ii-1).pos(2), joint(ii).pos(2)],...
             [joint(ii-1).pos(3), joint(ii).pos(3)],'color','k','LineStyle','--');
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
    if strcmp(conven.param,'modDH')
        joint(kk).linVel = J(1:3,:,kk)*qdot;
        joint(kk).angVel = J(4:6,:,kk)*qdot;
    elseif strcmp(conven.param,'DH')
        joint(kk).linVel = J(1:3,:,kk)*[0;qdot];
        joint(kk).angVel = J(4:6,:,kk)*[0;qdot];
    end
end
