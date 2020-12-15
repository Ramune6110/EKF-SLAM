function [x,P] = KF_cholesky_update(x,P,v,R,H)
    % Calculate the KF (or EKF) update given the prior state [x,P]
    % the innovation [v,R] and the (linearised) observation model H.
    % The result is calculated using Cholesky factorisation, which
    % is more numerically stable than a naive implementation.
      
    S= H * P * H' + R;
    S= (S+S')*0.5; % make symmetric
    SChol= chol(S);
    G = P * H' * inv(SChol);
    K = G * (inv(SChol))';
    x = x + K * v; % update 
    P = P - G * G';

%     S = H * P * H' + R;
%     S = (S + S') * 0.5; % make symmetric
%     SChol= chol(S);
%     K = P * H' * pinv(SChol) * pinv(SChol)';
%     x = x + K*v;
%     P = (eye(size(x,1)) - K * H) * P;
end
