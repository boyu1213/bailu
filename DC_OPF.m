% electricity demand MW
P_D1 = 300;
P_D2 = 300;
P_D3 = 400;
%parameter about 5 generators
% G1
P_G1max = 40;
P_G1min = 0;
C1 = 14;

% G2
P_G2max = 170;
P_G2min = 0;
C2 = 15;

% G3
P_G3max = 520;
P_G3min = 0;
C3 = 30;

% G4
P_G4max = 200;
P_G4min = 0;
C4 = 40;

% G5
P_G5max = 600;
P_G5min = 0;
C5 = 10;
% constraints on lines
%line1-2
x12 = 0.0281;
b12 = 1/x12;
P_12max = 400;

%line2-3
x23 = 0.0108;
b23 = 1/x23;
P_23max = 400;

%line3-4
x34 = 0.0297;
b34 = 1/x34;
P_34max = 400;

%line4-5
x45 = 0.0297;
b45 = 1/x45;
P_45max = 240;

%line1-5
x15 = 0.0064;
b15 = 1/x15;
P_15max = 400;

%line1-4
x14 = 0.035;
b14 = 1/x14;
P_14max = 400;

%MW to p.u
pbase = 100;                                %base in MW
P_D = P_D/pbase;
P_D1 = P_D1/pbase;
P_D2 = P_D2/pbase;
P_D3 = P_D3/pbase;

P_G1max = P_G1max/pbase;
P_G1min = P_G1min/pbase;
P_G2max = P_G2max/pbase;
P_G2min = P_G2min/pbase;
P_G3max = P_G3max/pbase;
P_G3min = P_G3min/pbase;
P_G4max = P_G4max/pbase;
P_G4min = P_G4min/pbase;
P_G5max = P_G5max/pbase;
P_G5min = P_G5min/pbase;

P_12max = P_12max/pbase;
P_23max = P_23max/pbase;
P_34max = P_34max/pbase;
P_45max = P_45max/pbase;
P_15max = P_15max/pbase;
P_14max = P_14max/pbase;

% optimization 

A = [1  0  0  0  0  0    0      0    0   0;               % max generator power
     0  1  0  0  0  0    0      0    0   0;
     0  0  1  0  0  0    0      0    0   0;
     0  0  0  1  0  0    0      0    0   0;
     0  0  0  0  1  0    0      0    0   0;
     -1 0  0  0  0  0    0      0    0   0;               % min generator power
     0 -1  0  0  0  0    0      0    0   0;
     0  0 -1  0  0  0    0      0    0   0;
     0  0  0 -1  0  0    0      0    0   0; 
     0  0  0  0 -1  0    0      0    0   0;              
     0  0  0  0  0  b12 -b12    0    0   0;               % line power limit
     0  0  0  0  0  0    b23  -b23   0   0;
     0  0  0  0  0  0    0    b34  -b34  0;
     0  0  0  0  0  0    0      0   b45 -b45;
     0  0  0  0  0  b15  0      0    0  -b15;
     0  0  0  0  0  b14  0      0   -b14  0;
     0  0  0  0  0  -b12 b12    0    0   0;
     0  0  0  0  0  0    -b23  b23   0   0;
     0  0  0  0  0  0    0    -b34  b34  0;
     0  0  0  0  0  0    0      0   -b45 b45;
     0  0  0  0  0  -b15  0      0    0   b15;
     0  0  0  0  0  -b14  0      0   b14  0;
     
    ];
b = [P_G1max P_G2max P_G3max P_G4max P_G5max P_G1min P_G2min P_G3min P_G4min P_G5min P_12max P_23max P_34max P_45max P_15max P_14max P_12max P_23max P_34max P_45max P_15max P_14max]';

Aeq = [ -1  -1  0  0  0  (b12+b14+b15)  -b12         0         -b14         -b15;
        0    0  0  0  0  -b12         (b12+b23)    -b23          0           0;
        0    0 -1  0  0   0              -b23     (b34+b23)    -b34          0;
        0    0  0 -1  0  -b14             0       -b34     (b34+b14+b45)   -b45;
        0    0  0  0 -1  -b15             0          0           -b45    (b45+b15);
        0    0  0  0  0   1               0          0           0           0   ;
        ];
    
beq = [0 -P_D1 -P_D2 -P_D3 0 0]';

%objective function
f = [C1  C2  C3  C4  C5 0 0 0 0 0 ]';

 
%call matlab optimization function
[x,fval] = linprog(f, A, b, Aeq, beq);

G1 = pbase*x(1)
G2 = pbase*x(2)
G3 = pbase*x(3)
G4 = pbase*x(4)
G5 = pbase*x(5)
Cost = (G1*C1+G2*C2+G3*C3+G4*C4+G5*C5)
