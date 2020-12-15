function [zf, idf, zn, table] = data_associate_known(x, z, idz, table)
    % For simulations with known data-associations, this function maintains
    % a feature/observation lookup table. It returns the updated table, the
    % set of associated observations and the set of observations to new features.
    
    % LM�̃f�[�^�Ή��t�������m�̏ꍇ�̏���
    
    % INPUTS:
    %   x - vehicle pose estimate [x;y;phi]
    %   z - set of range-bearing observations
    %   idz - landmark index tag for each observation
    %   table - data association table 
    %
    % OUTPUTS:
    %   z - set of range-bearing observations
    %   idf - landmark index tag for each observation
    
    % ���Ɋϑ�����LM���i�[����z��
    zf  = []; 
    % �n�߂Ċϑ�����LM���i�[����z��
    zn  = [];
    % ���Ɋϑ�����LM��index���i�[����z��
    idf = []; 
    % �n�߂Ċϑ�����LM��index���i�[
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
