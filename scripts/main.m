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

T_B_n = FwdKin(modDH,q);

% construct triads for plotting
trds = 0.02*eye(3);
clrs = {'r','g','b'};

fig1 = figure(1);
set(fig1,'Name','Kinova Kinematics');
daspect([1 1 1]);
view([60,10]);
t_prev = [0;0;0];

% plot base frame
for kk = 1:3
    line([0, trds(1,kk)],[0, trds(2,kk)],[0, trds(3,kk)],'color',clrs{kk},'linewidth',2);
end

% plot manipulator frames
for ii = 1:1:size(T_B_n,3)
    R_B_n = T_B_n(1:3,1:3,ii);
    t = T_B_n(1:3,4,ii);
    for kk = 1:3
        trd_rot = R_B_n*trds(:,kk);
        line([0, trd_rot(1)] + t(1),...
             [0, trd_rot(2)] + t(2),...
             [0, trd_rot(3)] + t(3),'color',clrs{kk},'linewidth',2);
    end
    % line connecting origins
    line([t_prev(1), t(1)],...
         [t_prev(2), t(2)],...
         [t_prev(3), t(3)],'color','k','LineStyle','--');
    t_prev = t;
end
grid on;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');

% Compute Geometric Jacobian
J = GeometricJacobian(T_B_n);

% Linear and angular velocities per link in work/configuration space
for kk = 1:1:size(J,3)
    link(kk).LinVel = J(1:3,:,kk)*qdot;
    link(kk).AngVel = J(4:6,:,kk)*qdot;
end
