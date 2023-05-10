function T_B_n = FwdKin(DH,q)
% Compute transformation from base to end-effector recursively 
T_B_n = nan(4,4,numel(DH.a));
if strcmp(DH.param,'DH')
    for ii = 1:1:numel(DH.a)
        if ii ~= 1
            T_nm1_n = HomTrans(DH.d(ii), DH.a(ii), DH.alpha(ii), DH.theta(ii) + q(ii-1),'DH'); 
        else 
            T_nm1_n = HomTrans(DH.d(ii), DH.a(ii), DH.alpha(ii), DH.theta(ii),'DH'); 
        end
        if ii == 1
            T_B_n(:,:,ii) = T_nm1_n;
        else
            T_B_n(:,:,ii) = T_B_n(:,:,ii-1)*T_nm1_n;
        end
    end
elseif strcmp(DH.param,'modDH')
    for ii = 1:1:numel(DH.a)
        T_nm1_n = HomTrans(DH.d(ii), DH.a(ii), DH.alpha(ii), q(ii),'modDH');
        if ii == 1
            T_B_n(:,:,ii) = T_nm1_n;
        else
            T_B_n(:,:,ii) = T_B_n(:,:,ii-1)*T_nm1_n;
        end
    end
else
    error('Input incorrect parameterization (neither DH or modDH)');
end

end

