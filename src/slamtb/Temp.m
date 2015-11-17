%% Random measurements to test code

angVel = rand(3,1);
angVel2 = rand(3,1);

dt = .1;

p = rand(3,1);
p2 = rand(3,1);

r = rand(3,1);
r2 = rand(3,1);

q = rand(4,1);
q2 = rand(4,1);

Nf = 10;
Na = 5;

n = Nf+Na;

h = rand(Nf+Na,2);
h2 = rand(Nf+Na,2);


[H, F, Q, OMEGA] = SysObsMeasure(q,  q2, angVel, angVel2, dt, Nf, Na, h, h2, r, r2, p, p2);

timeSegment = 1;
Q_SOM = formSOM(F, H, n, timeSegment, Q, OMEGA, dt);

size(Q_SOM)

%% Extract a matrix corresponding to feature i



