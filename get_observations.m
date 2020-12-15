function [z,idf]= get_observations(x, lm, idf, rmax, R)
    % INPUTS:
    %   x - vehicle pose [x;y;phi]
    %   lm - set of all landmarks
    %   idf - index tags for each landmark
    %   rmax - maximum range of range-bearing sensor 
    %
    % OUTPUTS:
    %   z - set of range-bearing observations
    %   idf - landmark index tag for each observation

    [lm,idf] = get_visible_landmarks(x,lm,idf,rmax);
    z = compute_range_bearing(x, lm, R);
end

function [lm,idf]= get_visible_landmarks(x,lm,idf,rmax)
    % Select set of landmarks that are visible within vehicle's semi-circular field-of-view
    dx  = lm(1,:) - x(1);
    dy  = lm(2,:) - x(2);
    phi = x(3);

    % incremental tests for bounding semi-circle
    % ŠÏ‘ª”ÍˆÍ“à‚É‚ ‚éLM‚Ìindex‚ğ’Tõ‚µ‚Ä‚¢‚é
    ii= find(abs(dx) < rmax & abs(dy) < rmax ... % bounding box
          & (dx*cos(phi) + dy*sin(phi)) > 0 ...  % bounding line
          & (dx.^2 + dy.^2) < rmax^2);           % bounding circle
    
    % ŠÏ‘ª‰Â”\‚ÈLM
    lm  = lm(:,ii);
    % ŠÏ‘ª‰Â”\‚ÈLM‚Ìindex
    idf = idf(ii);
end

function z = compute_range_bearing(x, lm, R)
    % Compute exact observation
    dx  = lm(1,:) - x(1);
    dy  = lm(2,:) - x(2);
    phi = x(3);
    z   = [sqrt(dx.^2 + dy.^2);
           atan2(dy,dx) - phi];
       
    len= size(z,2);
    if len > 0
        z(1,:)= z(1,:) + randn(1,len)*sqrt(R(1,1));
        z(2,:)= z(2,:) + randn(1,len)*sqrt(R(2,2));
    end
end