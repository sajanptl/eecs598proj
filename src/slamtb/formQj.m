% H is a function of j => Qj is a function of j
function Qj = formQj(F, H, n, Q, OMEGA, dt)
    Qj = [];
    t = zeros(size(OMEGA));
    for i = 0:n-1
        
       t = t + (Q^i)*OMEGA;
        
       HF_j = [H(:,1:3) H(:,4:7)*(Q^i) H(:,1:3)*i*dt H(:,4:7)*t];
       
       Qj = [Qj HF_j'];
    end
    
    % Should I transpose?
    Qj = Qj.';
    
    % Necessary?
    Qj(isnan(Qj)) = 0;
    Qj(isinf(Qj)) = 0;
    
end