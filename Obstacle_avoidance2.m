 %Replicating cars safety system 


clear; close all;



u_M=sdpvar(2,1);




e=5;
v=1;
%Time

dt=0.01;
time=2;
t=0:dt:time;

L=animatedline;

axis([0,25,-50,50])
%Ellipse equation 0 centre

% Define ellipse parameters
a = 15;  % Semi-major axis
b = 5;  % Semi-minor axis

% Generate points on the ellipse
theta = linspace(0, 2*pi, 1000);
x = a * cos(theta);
y_e = b * sin(theta);


%Initialization


x1=4; % Position x 
x2=0; % Speed x
x3=7; % Position y 
x4=0; % Speed y
xd=[0,-13]; %Desired position


i=1;


u1=u_M(1);
u2=u_M(2);

  x1_values=zeros(size(t));
  u1_values=zeros(size(t));
  u2_values=zeros(size(t));
  d_values=zeros(size(t));
  x3_values=zeros(size(t));
   x4_values=zeros(size(t));
  h_values=zeros(size(t));

% Optimization for each time step
  
for t1=0:dt:time

d=sqrt(x1^2+x3^2)-sqrt(a^2*b^2*(x1^2+x3^2))/sqrt(b^2*x1^2+a^2*x3^2);
  h = d; % Distance from ellipse
  
y1=x1-xd(1);
y2=x3-xd(2);
y= [y1 ; y2];
%Lyapunov function


dBdx1= (-1/(h+h^2))*x1*((b^2*x1^2+a^2*x3^2)^1.5+(a*b^3-a^3*b)*x3^2)/(sqrt(x1^2+x3^2)*(b^2*x1^2+a^2*x3^2)^1.5);
dBdx3= (-1/(h+h^2))*x3*((b^2*x1^2+a^2*x3^2)^1.5+(a^3*b-a*b^3)*x3^2)/(sqrt(x1^2+x3^2)*(b^2*x1^2+a^2*x3^2)^1.5);



LgB1= dBdx1;
LgB2= dBdx3;
LfB= 0;
    

u_M=sdpvar(2,1);

u_M(1)=u1;
u_M(2)=u2;


 B = -log(h/(1+h));
 
 h_values(i)=h;
 
 psi_0 = e*y'*y;
 
 psi_1 = 2*y';

 A_clf=[psi_1];
 

 
 b_clf=-psi_0;


A_cbf=[LgB1 LgB2];
b_cbf=-LfB + v/B;


model=[
    A_clf*u_M <= b_clf  
    
    A_cbf*u_M <= b_cbf
    
  ];

w1=0.8;
w2=0.2;
w3=0.1;
w4=0.1;
f=u_M(1)^2+u_M(2)^2;



optimize(model, f);

u_optimal= value(u_M);
u1_values(i)=value(u_optimal(1));
u2_values(i)=value(u_optimal(2));


x1=x2.*dt+x1;
x2=u1_values(i);
x3=x4.*dt+x3;
x4=u2_values(i);
    

  

  x1_values(i)=value(x1);
  x3_values(i)=value(x3);
   x4_values(i)=value(x4);
  d_values(i)=d;

  i=i+1;

   addpoints(L,x1_values(i),x3_values(i));
   drawnow
end



figure;

subplot(1,3,1)
plot(x,y_e) 
hold on
 plot(double(x1_values),double(x3_values))
 xlabel('x');
ylabel('y');
title('x vs y');

subplot(1,3,2)
plot(t,double(u2_values))
 xlabel('t (s)');
ylabel('u2 (speed y) ');
title(' speed vs time');

subplot(1,3,3)
plot(t,double(h_values))
 xlabel('t (s)');
ylabel('h');
title('Hard Constraint vs time');



