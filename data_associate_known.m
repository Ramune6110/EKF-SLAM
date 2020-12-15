function [zf, idf, zn, table] = data_associate_known(x, z, idz, table)
    % For simulations with known data-associations, this function maintains
    % a feature/observation lookup table. It returns the updated table, the
    % set of associated observations and the set of observations to new features.
    
    % LM‚Ìƒf[ƒ^‘Î‰•t‚¯‚ªŠù’m‚Ìê‡‚Ìˆ—
    
    % INPUTS:
    %   x - vehicle pose estimate [x;y;phi]
    %   z - set of range-bearing observations
    %   idz - landmark index tag for each observation
    %   table - data association table 
    %
    % OUTPUTS:
    %   z - set of range-bearing observations
    %   idf - landmark index tag for each observation
    
    % Šù‚ÉŠÏ‘ª‚µ‚½LM‚ğŠi”[‚·‚é”z—ñ
    zf  = []; 
    % n‚ß‚ÄŠÏ‘ª‚µ‚½LM‚ğŠi”[‚·‚é”z—ñ
    zn  = [];
    % Šù‚ÉŠÏ‘ª‚µ‚½LM‚Ìindex‚ğŠi”[‚·‚é”z—ñ
    idf = []; 
    % n‚ß‚ÄŠÏ‘ª‚µ‚½LM‚Ìindex‚ğŠi”[
    idn = [];

    % find associations (zf) and new features (zn)
    for i=1:length(idz)
        ii= idz(i);
        if table(ii) == 0 % new feature
            zn= [zn z(:,i)];
            idn= [idn ii];
        else
            zf= [zf z(:,i)];
            idf= [idf table(ii)];
        end
    end

    % add new feature IDs to lookup table
    Nxv = 3; % number of vehicle pose states
    Nf  = (length(x) - Nxv)/2; % number of features already in map
    table(idn) = Nf + (1:size(zn,2)); % add new feature positions to lookup table
end
