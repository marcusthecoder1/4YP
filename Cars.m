%Replicating cars safety system 


clear; close all;



u_M=sdpvar(2,1);


%Parameters

g=9.81;
m=1650;
f0=0.1;
f1=5;
f2=0.25;
vd=24;
v0=13.89;
e=10;
v=1;
p_sc=0.00001;

%Time

dt=0.1;
time=20;
t=0:dt:time;


%Initialization

z=100; % D between cars
x1=900; % Position
x2=10; % Speed


i=1;


u=u_M(1);

  x2_values=zeros(size(t));
  u_values=zeros(size(t));
  z_values=zeros(size(t));
  x1_values=zeros(size(t));
  h_values=zeros(size(t));

% Optimization for each time step
  
for t1=0:dt:time

  
    
Fr=f0+f1*x2+f2*x2^2;

y1=2*(x2-vd)/m;
y0=-2*((x2-vd)./m).*Fr+ e.*(x2-vd).^2;

LgB=1.8/(m*(1-1.8*x2+z)*(-1.8*x2+z));
LfB=-(1.8*Fr+m*(v0-x2))/(m*(1-1.8*x2+z)*(-1.8*x2+z));

H_acc=2.*[1/m^2 0 ; 0 p_sc];
F_acc=-2.*[Fr/m^2 ; 0];
    

u_M=sdpvar(2,1);

 h = z - 1.8 * x2;
 B = -log(h/(1+h));

 h_values(i)=h;

A_clf=[y1 -1];
b_clf=-y0;
A_cbf=[LgB 0];
b_cbf=-LfB +v/B;

model=[A_clf*u_M <= b_clf, A_cbf*u_M <= b_cbf];


f=0.5.* u_M'*H_acc*u_M+F_acc'*u_M;



optimize(model, f);

u_optimal= value(u_M);
u_values(i)=value(u_optimal(1));

x1=x2.*dt+x1;
z= z+ (v0-x2).*dt;
x2=x2+((u_values(i)-Fr)/m).*dt;
    

  

  
  x2_values(i)=value(x2);
  
  z_values(i)=value(z);
  x1_values(i)=value(x1);
  i=i+1;

   
end



figure;

subplot(1,3,1)
 plot(t,double(x2_values))
 xlabel('t (s)');
ylabel('x2 (m/s)');
title('Speed vs time');

subplot(1,3,2)
plot(t,double(u_values)/(m*g))
 xlabel('t (s)');
ylabel('u/mg ');
title('Normalised Fw(N) vs time');

subplot(1,3,3)
plot(t,double(h_values))
 xlabel('t (s)');
ylabel('h');
title('Hard Constraint vs time');
