function Q_SOM = formSOM(Q_SOM, F, H, n, Q, OMEGA, dt)
       
    Qj = formQj(F, H, n, Q, OMEGA, dt); 
    Q_SOM = [Q_SOM; Qj];
    
    % Get "rid" of ill condition matrix
    Q_SOM(isnan(Q_SOM)) = 0;
    Q_SOM(isinf(Q_SOM)) = 0;
    
    Q_SOM = sparse(Q_SOM);
    
    
end

