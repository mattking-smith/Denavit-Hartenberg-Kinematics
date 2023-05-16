function T_B_n = FwdKin(DH,q,basePose)
% Compute transformation from base to end-effector recursively 
T_B_n = nan(4,4,numel(DH.a)+1);
T_B_n(:,:,1) = basePose;
if strcmp(DH.param,'DH')
    for ii = 1:1:numel(DH.a)
        if strcmp(DH.manipulator, 'KinovaGen3')
        if ii ~= 1
            T_nm1_n = HomTrans(DH.d(ii), DH.a(ii), DH.alpha(ii), DH.theta(ii) + q(ii),'DH'); 
        else 
            T_nm1_n = HomTrans(DH.d(ii), DH.a(ii), DH.alpha(ii), DH.theta(ii),'DH'); 
        end
        elseif strcmp(DH.manipulator, 'Planar2Link') || strcmp(DH.manipulator,'3Link')
            T_nm1_n = HomTrans(DH.d(ii), DH.a(ii), DH.alpha(ii), DH.theta(ii) + q(ii),'DH'); 
        end
        T_B_n(:,:,ii+1) = T_B_n(:,:,ii)*T_nm1_n;
    end
elseif strcmp(DH.param,'modDH')
    for ii = 1:1:numel(DH.a)
        T_nm1_n = HomTrans(DH.d(ii), DH.a(ii), DH.alpha(ii), q(ii),'modDH');
        T_B_n(:,:,ii+1) = T_B_n(:,:,ii)*T_nm1_n;
    end
else
    error('Input incorrect parameterization (neither DH or modDH)');
end

end

