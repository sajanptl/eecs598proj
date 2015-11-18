function Q_SOM = formSOM_viaReplacement(Q_SOM, H, n, Q, OMEGA, dt, oldestTimeSegment)

    global numLocalFrames   
    
    Q_new = formQj(0, H, n, Q, OMEGA, dt);
    
    [r,c] = size(Q_new);
    
    % computeReplacementIndex Row position
    replacementIndex = mod(oldestTimeSegment, numLocalFrames);
    if replacementIndex == 0
        replacementIndex = numLocalFrames;
    end
    
    Q_SOM((replacementIndex-1)*r +1: (replacementIndex-1)*r + r,:) = Q_new;  
    
end