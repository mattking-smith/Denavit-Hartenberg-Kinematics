% Script for reading in DH table and computing forward kinemitics
% clear all;
% close all;

% Load DH table
KinovaGen3_DH;

% joint angles
q = zeros(7,1);

T_B_n = eye(4);
for ii = 1:1:numel(DH.a)
    if ii ~= 1
        T_nm1_n = HomTrans(DH.d(ii), DH.a(ii), DH.alpha(ii), DH.theta(ii) + q(ii-1),'DH'); 
    else 
        T_nm1_n = HomTrans(DH.d(ii), DH.a(ii), DH.alpha(ii), DH.theta(ii),'DH'); 
    end
    T_B_n = T_B_n*T_nm1_n;
end