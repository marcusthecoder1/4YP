%Replicating cars safety system with expanding ellipse

clear; close all;

u_M=sdpvar(2,1);
u1=u_M(1);
u2=u_M(2);

%Barrier/Lyapunov parameters
e=4;
v=1;

%obstacle speed
m1=0.7;
m2=0.9;

%Time

dt=0.02;
time=2;
t=0:dt:time;

%Initialization

x1=20; % Position x 
x2=20; % Position y 
x3=0; % Speed x
x4=0; % Speed y
xd=[-6,-6]; %Desired position

% Define center coordinates
x_c0 = 2;  % x-coordinate of the center
y_c0 = 3;  % y-coordinate of the center

% Define ellipse parameters
a = 3;    % Semi-major axis
b = 3;    % Semi-minor axis

% Generate points on the ellipse
theta = linspace(0, 2*pi, 1000);
x_e0 = x_c0 + a * cos(theta);
y_e0 = y_c0 + b * sin(theta);



figure;

% Initialize an empty plot


%Vectors for plots
x1_values=zeros(size(t));
u1_values=zeros(size(t));
u2_values=zeros(size(t));
d_values=zeros(size(t));
x2_values=zeros(size(t));
h_values=zeros(size(t));


% Preallocate data for animation (optional)


h1=plot(xd(1),xd(2),'x');
title('Animated Plot');

% Set axis limits
xlim([-20, 50]);
ylim([-20, 50]);

% Optimization for each time step
  u_optimal=0;
  
  
  
for i=1:length(t)
    
    %Logging values
    x1_values(i)=value(x1);
    x2_values(i)=value(x2);
    
    
        % Define center coordinates
    x_c = x_c0+m1*(i-1);  % x-coordinate of the center
    y_c = y_c0+m2*(i-1);  % y-coordinate of the center

    % Define ellipse parameters
    a = 5;    % Semi-major axis
    b = 5;    % Semi-minor axis

    % Generate points on the ellipse
    theta = linspace(0, 2*pi, 1000);
    x_e = x_c + a * cos(theta);
    y_e = y_c + b * sin(theta);


    % Distance from ellipse
    d=sqrt((x1-x_c)^2+((x2-y_c))^2)-sqrt(a^2*b^2*((x1-x_c)^2+((x2-y_c))^2))/sqrt(b^2*(x1-x_c)^2+a^2*((x2-y_c))^2);
    h = d;
    h_values(i)=h;
    
    %Lyapunov Function
    y1=x1-xd(1);
    y2=x2-xd(2);
    y= [y1 ; y2];

    %Update animation 
    xplot=[xd(1), x_e,x1_values(i)];
    
    yplot=[xd(2), y_e,x2_values(i)];
    
    set(h1, 'XData', xplot, 'YData', yplot);
    
    %Barrier function 
    B = -log(h/(1+h));

    dBdx1= (-1/(h+h^2))*(x1-x_c)*((b^2*(x1-x_c)^2+a^2*(x2-y_c)^2)^1.5+(a*b^3-a^3*b)*(x2-y_c)^2)/(sqrt((x1-x_c)^2+((x2-y_c))^2)*(b^2*(x1-x_c)^2+a^2*((x2-y_c))^2)^1.5);
    dBdx2= (-1/(h+h^2))*(x2-y_c)*((b^2*(x1-x_c)^2+a^2*(x2-y_c)^2)^1.5+(a^3*b-a*b^3)*(x1-x_c)^2)/(sqrt((x1-x_c)^2+((x2-y_c))^2)*(b^2*(x1-x_c)^2+a^2*((x2-y_c))^2)^1.5);

    %Barrier Constraint setup
    LgB1= dBdx1;
    LgB2= dBdx2;
    LfB= -dBdx1*(m1/dt)-dBdx2*(m2/dt);
   
    A_cbf=[LgB1 LgB2];
    b_cbf=-LfB + v/B;

    %Lyapunov constraint setup
    psi_0 = e*y'*y;
    psi_1 = 2*y';
    
    A_clf=[psi_1];
    b_clf=-psi_0;

    u_M=sdpvar(2,1);
    u_M(1)=u1;
    u_M(2)=u2;
    

    %Constraints
    model=[
        A_clf*u_M <= b_clf  
    
        A_cbf*u_M <= b_cbf
    
          ];

     
    %Objective function
    %w1=0.5;
    %w2=0.5;

    f=u_M(1)^2+u_M(2)^2;

    optimize(model, f);
    
    
    %Logging optimal value
    u_optimal= value(u_M);
    u1_values(i)=value(u_optimal(1));
    u2_values(i)=value(u_optimal(2));
  
    %Linear Dynamics
    x3=u1_values(i);
    x1=x3.*dt+x1;
    x4=u2_values(i);
    x2=x4.*dt+x2;
    
    
     
end



figure;
%Plot position evolution
subplot(1,3,1)
plot(x_e0,y_e0) 
hold on
plot(x_e,y_e) 
hold on
 plot(double(x1_values),double(x2_values))
 xlabel('x');
ylabel('y');
title('x vs y');

%Speed plot
subplot(1,3,2)
plot(t,double(u2_values))
 xlabel('t (s)');
ylabel('u2 (speed y) ');
title(' speed(y) vs time');

%Barrier plot
subplot(1,3,3)
plot(t,double(h_values))
 xlabel('t (s)');
ylabel('h');
title('Hard Constraint vs time');