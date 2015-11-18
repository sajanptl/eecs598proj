function [H, F, Q, OMEGA] = SysObsMeasure(q,  q2, angVel, angVel2, dt, Nf, Na, h, h2, r, r2, p, p2)
    qR = q(1); qx = q(2); qy = q(3); qz = q(4);
    
    Q = [ qR -qx -qy -qz
          qx  qR  qz -qy
          qy -qz  qR  qx
          qz qy  -qx  qR];
      
    OMEGA = ...
        [ qR -qx -qy -qz 
          qx  qR -qz  qy
          qy  qz  qR -qx 
          qz -qy  qx qR];
    
    dq = reshape(q - q2,[],1);
    dw = reshape(angVel-angVel2,[],1);
    
    dqdw = [dq./dw(1) dq./dw(2) dq./dw(3)];

%     dqdw = [dq dq dq];
    
    OMEGA = OMEGA * dqdw * dt;
    
%     %% Formulate F_x
%     tmp1 = [eye(3,3);
%             zeros(4,3);
%             zeros(6,3)];
%     tmp2 = [zeros(3,4);
%             Q;
%             zeros(6,4)];
%     tmp3 = [dt*eye(3,3);
%             zeros(4,3)];
%     tmp4 = [zeros(3,3);
%             OMEGA];
%         
%     tmp5 = [tmp3 tmp4];
%     tmp5 = [tmp5; eye(6,6)];
%     
%     F_x = [tmp1 tmp2 tmp5];
%     
%     %% Formulate F    
%     tmp1 = [F_x zeros(13,3*Nf)];
%     tmp2 = [zeros(3*Nf,13) eye(3*Nf,3*Nf)];
    
    F = 0;;
    
    %% Form H
    delH = h - h2;
    delR = reshape(r - r2, 1, []);
    delQ = reshape(q - q2, 1, []);
    delP = p - p2;
    
    H = [];
    
    tmp1 = [];
    % Normal features
    for i = 1:Nf
       tmp1 = ...
           [ tmp1;
           
             delH(i,1)./delR delH(i,1)./delQ zeros(1,3);
             delH(i,2)./delR delH(i,2)./delQ zeros(1,3);           
           ];       
    end
    % Anchor Features
    for i = Nf+1:Nf+Na
       tmp1 = ...
           [ tmp1;
           
             delH(i,1)./delR delH(i,1)./delQ zeros(1,3);
             delH(i,2)./delR delH(i,2)./delQ zeros(1,3);           
           ];       
    end
    
    tmp2 = zeros(2*Nf, 3*Nf);
    for i = 1:Nf
       tmp2(2*i-1,3*i-2:3*i) = delH(i,1)./delP(i,:);
       tmp2(2*i  ,3*i-2:3*i) = delH(i,2)./delP(i,:);
    end
    
    %% Formulate final H
    
    tmp3 = [tmp2; zeros(2*Na,3*Nf)];
    H = [tmp1 tmp3];
    
    % Is this supposed to happen?
    H(isnan(H)) = 0;
    H(isinf(H)) = 0;
    
    
    
            

end

