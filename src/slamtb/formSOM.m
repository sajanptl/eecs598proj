function Q_SOM = formSOM(F, H, n, timeSegment, Q, OMEGA, dt)
    
    global numLocalFrames

    Q_SOM = [];
    
    start = max(1,timeSegment-numLocalFrames);
    for j = start:timeSegment
       Qj = formQj(F, H, n, Q, OMEGA, dt); 
       Q_SOM = [Q_SOM Qj'];
    end
    
    % Get "rid" of ill condition matrix
    Q_SOM(isnan(Q_SOM)) = 0;
    Q_SOM(isinf(Q_SOM)) = 0;
    
    Q_SOM = sparse(Q_SOM);
end

function Qj = formQj(F, H, n, Q, OMEGA, dt)
    Qj = [];
    
    for i = 0:n
        
        t = (Q^0)*OMEGA;
        for k = 1:i
            t = t + (Q^k)*OMEGA;
        end
        
       HF_n = [H(:,1:3) H(:,4:7)*(Q^n) H(:,1:3)*n*dt H(:,4:7)*t];
       
       Qj = [Qj HF_n'];
    end
end

