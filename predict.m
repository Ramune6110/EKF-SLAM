function [x,P]= predict (x,P,v,g,Q,WB,dt)
    % Inputs:
    %   x, P - SLAM state and covariance
    %   v, g - control inputs: velocity and gamma (steer angle)
    %   Q - covariance matrix for velocity and gamma
    %   WB - vehicle wheelbase
    %   dt - timestep
    %
    % Outputs: 
    %   xn, Pn - predicted state and covariance

    s= sin(g+x(3)); c= cos(g+x(3));
    vts= v*dt*s; vtc= v*dt*c;

    % jacobians   
    Gv= [1 0 -vts;
         0 1  vtc;
         0 0 1];
    Gu= [dt*c -vts;
         dt*s  vtc;
         dt*sin(g)/WB v*dt*cos(g)/WB];

    % predict covariance
    P(1:3,1:3)= Gv*P(1:3,1:3)*Gv' + Gu*Q*Gu';
    if size(P,1)>3
        P(1:3,4:end)= Gv*P(1:3,4:end);
        P(4:end,1:3)= P(1:3,4:end)';
    end    
    
%     R=diag([0.2 0.2 1.0 * pi / 180]).^2;
%     Fx = horzcat(eye(3),zeros(3, 2 * GetnLM(x)));
% 
%     jF = [0 0 -(v / g) * cos(x(3)) + (v / g) * cos(x(3) + g * dt);
%           0 0 -(v / g) * sin(x(3)) + (v / g) * sin(x(3) + g * dt);
%           0 0 0];
%       
%     G = eye(length(x)) + Fx' * jF * Fx;
%     
%     P = G' * P * G + Fx' * R * Fx;

    % predict state
    x(1:3) = motion_model(x, v, g, WB,dt);
end

function n = GetnLM(xEst)
    %ランドマークの数を計算する関数
    n = (length(xEst)-3)/2;
end
