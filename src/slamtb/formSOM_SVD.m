% H is changing as a function of j
function Q_SOM_new = formSOM_SVD(Q_SOM, H, n, Q, OMEGA, dt)
    forceFullRank = rand(size(Q_SOM))*1e-10;
    [U,S,V] = svds(Q_SOM+ forceFullRank,13);
    a = formQj(0, H, n, Q, OMEGA, dt);
    [ma,mn] = size(a);
    m = U(1:ma,:)'*a;
    p = a(1:13,1:13)' - U(1:13,:)*m;
    P = p/norm(p);
    
    K = [S                  m;
         zeros(size(S))     norm(p)*ones(size(m))];
     
    [Uprime, Sprime, Vprime] = svd(K);
    
    U_new = [U P]*Uprime';
     
     
end