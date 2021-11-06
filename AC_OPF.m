clear;clc
y=0;
% Input the original data and calculate the node admittance matrix
y(1,2)=1/(0.0281+0.0143i); 
y(1,4)=1/(0.0304+0.015i);
y(1,5)=1/(0.0064+0.0045i); 
y(2,3)=1/(0.0108+0.0093i);
y(3,4)=1/(0.0297+0.0981i);
y(4,5)=1/(0.0297+0.0151i); 
for	i=1:5
    for	j=i:5 
    y(j,i)=y(i,j); 
    end
end 
Y=0;
% Finding mutual admittance
for	i=1:5
    for	j=1:5
        if	i~=j 
            Y(i,j)=-y(i,j); 
        end
    end
end
% Self admittance
for	i=1:5
    Y(i,i)=sum(y(i,:)); 
end
Y 
%Y is the admittance matrix
G=real(Y);
B=imag(Y);
% Original node power
S(1)=-40+27i;
S(2)=170+57i;
S(3)=-520+347i;
S(4)=200+89i;
S(5)=0;
P=real(S);
Q=imag(S);
% Initial value assignment
U=ones(1,5);
U(5)=1.33; % Initial voltage 1.33
e=zeros(1,5); % Initial value of phase angle
ox=ones(6,1); 
fx=ones(6,1); % Unbalance of PQ nodes
count=0	% Calculate the number of iterations
while	max(fx)>1e-5
for	i=1:4
    for	j=1:4 
        H(i,j)=0;N(i,j)=0;M(i,j)=0;L(i,j)=0;oP(i)=0;oQ(i)=0;
    end
end
for	i=1:4
    for	j=1:5
        oP(i)=oP(i)-U(i)*U(j)*(G(i,j)*cos(e(i)-e(j))+B(i,j)*sin(e(i)-e(j)));
        oQ(i)=oQ(i)-U(i)*U(j)*(G(i,j)*sin(e(i)-e(j))-B(i,j)*cos(e(i)-e(j))); 
    end
    oP(i)=oP(i)+P(i); oQ(i)=oQ(i)+Q(i);
end
fx=[oP,oQ]'; % Unbalance of PQ nodes
% Calculate Jacobian matrix
% When i~=j, h, N, m, l are as follows:
for	i=1:4
    for	j=1:4
        if	i~=j 
            H(i,j)=-U(i)*U(j)*(G(i,j)*sin(e(i)-e(j))-B(i,j)*cos(e(i)-e(j)));
            N(i,j)=-U(i)*U(j)*(G(i,j)*cos(e(i)-e(j))+B(i,j)*sin(e(i)-e(j))); 
            L(i,j)=H(i,j);
            M(i,j)=-N(i,j);
        end
    end
end
% When i=j, h, N, m, l are as follows:
for	i=1:4
    for	j=1:5
        if	i~=j
            H(i,i)=H(i,i)+U(i)*U(j)*(G(i,j)*sin(e(i)-e(j))-B(i, j)*cos (e(i)-e(j)));
            N(i,i)=N(i,i)-U(i)*U(j)*(G(i, j)*cos(e(i)-e(j))+B(i,j)*sin(e(i)-e(j)));
            M(i,i)=M(i,i)-U(i)*U(j)*(G(i,j)*cos(e(i)-e(j))+B(i,j)*sin(e(i)-e(j)));
            L(i,i)=L(i,i)-U(i)*U(j)*(G(i,j)*sin(e(i)-e(j))-B(i,j)*cos(e(i)-e(j))); 
        end
    end
    N(i,i)=N(i,i)-2*(U(i))^2*G(i,i);
    L(i,i)=L(i,i)+2*(U(i))^2*B(i,i); 
end
J=[H,N;M,L];	% J is Jacobian matrix
ox=-((inv(J))*fx); 
for	i=1:4
    oe(i)=ox(i); oU(i)=ox(i+4)*U(i); 
end
for	i=1:4
    e(i)=e(i)+oe(i); % Modified equation
    U(i)=U(i)+oU(i); 
end
count=count+1; 
end
J,ox,U,e,count
% Calculate the net power injected by the node
i=5;
for	j=1:5
    P(i)=U(i)*U(j)*(G(i,j)*cos(e(i)-e(j))+B(i,j)*sin(e(i)-e(j)))+P(i);
    Q(i)=U(i)*U(j)*(G(i,j)*sin(e(i)-e(j))-B(i,j)*cos(e(i)-e(j)))+Q(i); 
end
S(5)=P(5)+Q(5)*sqrt(-1); 
S
% Calculate node injection current
I= Y*U'
% reference: https://wenku.baidu.com/view/866e1a2833687e21af45a994.html