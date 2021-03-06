function [zf,idf, zn] = data_associate(x,P,z,R, gate1, gate2)
    % Simple gated nearest-neighbour data-association. No clever feature
    % caching tricks to speed up association, so computation is O(N), where
    % N is the number of features in the state.
    
    % 既に観測したLMを格納する配列
    zf  = []; 
    % 始めて観測したLMを格納する配列
    zn  = [];
    % 既に観測したLMのindexを格納する配列
    idf = []; 

    Nxv= 3; % number of vehicle pose states
    Nf= (length(x) - Nxv)/2; % number of features already in map

    % linear search for nearest-neighbour, no clever tricks (like a quick
    % bounding-box threshold to remove distant features; or, better yet,
    % a balanced k-d tree lookup). TODO: implement clever tricks.
    for i=1:size(z,2)
        jbest = 0;
        nbest = inf;
        outer = inf;

        % search for neighbours
        for j=1:Nf
            [nis, nd] = compute_association(x,P,z(:,i),R, j);
            if nis < gate1 & nd < nbest % if within gate, store nearest-neighbour
                nbest = nd;
                jbest = j;
            elseif nis < outer % else store best nis value
                outer = nis;
            end
        end

        % add nearest-neighbour to association list
        if jbest ~= 0
            zf  =  [zf  z(:,i)];
            idf = [idf jbest];
        elseif outer > gate2 % z too far to associate, but far enough to be a new feature
            zn= [zn z(:,i)];
        end
    end
end

% マハラノビス距離計算
function [nis, nd]= compute_association(x,P,z,R,idf)
    % return normalised innovation squared (ie, Mahalanobis distance) and normalised distance
    [zp,H] = observe_model(x, idf);
    v = z-zp; 
    v(2) = pi_to_pi(v(2));
    S = H*P*H' + R;

    nis = v'*inv(S)*v;
    nd  = nis + log(det(S));
end
