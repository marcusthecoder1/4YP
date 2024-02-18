%Replicating cars safety system with expanding ellipse

clear; close all;

%N vehicles
N=4;
u_M=sdpvar(2,N);

%Barrier/Lyapunov parameters
e=4;
v=1;

%obstacle speed
m1=1;
m2=1;

%Time

dt=0.01;
time=2;
t=0:dt:time;

%Initialization

x1=[10;18;30;50]; % x Positions  
x2=[20;28.3;25;50]; % y Positions  
x3=[0;0;1;2]; % x Speeds 
x4=[0;0;1;2]; % y Speeds 
xd=[0,-20;0,-3;-10,-10;20,30]; %Desired position

% Define center coordinates
x_c0 = 2;  % x-coordinate of the center
y_c0 = 3;  % y-coordinate of the center


% Initialize an empty plot
figure;
Line1=animatedline('Color','k');
%Line2=animatedline('Color','r');
Line3=animatedline('Color','b');
Line4 = animatedline('Color', 'b', 'Marker', 'o');
Line5=animatedline('Color','r');
Line6 = animatedline('Color', 'r', 'Marker', 'o');
Line7=animatedline('Color','g');
Line8 = animatedline('Color', 'g', 'Marker', 'o');
Line9=animatedline('Color','m');
Line10 = animatedline('Color', 'm', 'Marker', 'o');


%Vectors for plots
x1_values=zeros(N,length(t));

u1_values=zeros(length(t),N);
u2_values=zeros(length(t),N);

x2_values=zeros(N,length(t));

h_values=zeros(N,length(t));

% Set axis limits
xlim([-20, 50]);
ylim([-20, 50]);

% Optimization for each time step
  u_optimal=0;
  
  % Define a range of x values
x_road = -30:0.1:30;

% Calculate corresponding y values for each line
y_road1 = x_road + 15;
y_road2 = x_road - 20;
  
  
for i=1:length(t)
    
    %Logging values
    x1_values(:,i)=value(x1);
    x2_values(:,i)=value(x2);
    
    
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

    
    % Distance from ellipse and road function
    d=sqrt((x1-x_c).^2+((x2-y_c)).^2)-sqrt(a^2*b^2*((x1-x_c).^2+((x2-y_c)).^2))./sqrt(b^2*(x1-x_c).^2+a^2*((x2-y_c)).^2);
    h=d;
    h_values(:,i)=h;

   
    
    % Distance from ellipse and road functions
    
%     y_road1 = x_road + 15;
%     y_road2 = x_road - 20;
%   
%     h_road = x1-x2+15
%    
    
%Lyapunov Function
    y1=x1-xd(:,1);
    y2=x2-xd(:,2);
    y= [y1' ; y2'];


    %Update animation 
    clearpoints(Line1);
    %clearpoints(Line2);
    clearpoints(Line3);
    clearpoints(Line4);
    clearpoints(Line5);
    clearpoints(Line6);
    clearpoints(Line7);
    clearpoints(Line8);
    clearpoints(Line9);
    clearpoints(Line10);
    
    
    addpoints(Line1,x_e,y_e);
    %addpoints(Line2,x_road,y_road1);
    addpoints(Line3,x1_values(1,1:i),x2_values(1,1:i));
    addpoints(Line4,xd(1,1),xd(1,2));
    addpoints(Line5,x1_values(N-1,1:i),x2_values(N-1,1:i));
    addpoints(Line6,xd(N-1,1),xd(N-1,2));
    addpoints(Line7,x1_values(N,1:i),x2_values(N,1:i));
    addpoints(Line8,xd(N,1),xd(N,2));
    addpoints(Line9,x1_values(N-2,1:i),x2_values(N-2,1:i));
    addpoints(Line10,xd(N-2,1),xd(N-2,2))

    drawnow
    


    

    
    %Barrier functions 
    B = -log(h./(1+h));
    %B_road= -log(h_road(1)./(1+h_road(1)));

    dBdx1= (-1./(h+h.^2)).*(x1-x_c).*((b^2*(x1-x_c).^2+a^2*(x2-y_c).^2).^1.5+(a*b^3-a^3*b)*(x2-y_c).^2)./(sqrt((x1-x_c).^2+((x2-y_c)).^2).*(b^2*(x1-x_c).^2+a^2*((x2-y_c)).^2).^1.5);
    dBdx2= (-1./(h+h.^2)).*(x2-y_c).*((b^2*(x1-x_c).^2+a^2*(x2-y_c).^2).^1.5+(a^3*b-a*b^3)*(x1-x_c).^2)./(sqrt((x1-x_c).^2+((x2-y_c)).^2).*(b^2*(x1-x_c).^2+a^2*((x2-y_c)).^2).^1.5);
    
%     dBdx1_road= (-1/(h_road(1)+h_road(1)^2))*(-1);
%     
%     dBdx2_road= (-1/(h_road(1)+h_road(1)^2))*(1);
    
   
    
    %Barrier Constraint setup
    LgB1= dBdx1;
    %LgB1_road= dBdx1_road;
    LgB2= dBdx2;
    %LgB2_road= dBdx2_road;
    LfB= -dBdx1.*(m1/dt)-dBdx2.*(m2/dt);
    
   
    A_cbf=[LgB1 LgB2];
    %A_cbf_road=[LgB1_road LgB2_road];
    b_cbf=-LfB + v./B;
    
    %b_cbf_road = 10/B_road2;
    
    
    
 
    
    %Lyapunov constraint setup
    psi_0 = e.*y'*y;
    psi_1 = 2.*y';
    
    A_clf=psi_1;
    b_clf=-diag(psi_0) ;
    
   

    u_M=sdpvar(2,N,'full');
    
      
    %Constraints
    model=[
        diag(A_clf*u_M) <= b_clf  
    
        diag(A_cbf*u_M) <= b_cbf
        
        %A_cbf_road*u_M <= b_cbf_road
        
        u_M<=150
        
        
    
          ];

     
    %Objective function
    %w1=0.5;
    %w2=0.5;

    f=trace(u_M'*u_M);

    optimize(model, f);
    
    
    %Logging optimal value
    u_optimal= value(u_M);
    u1_values(i,:)=value(u_optimal(1,:));
    u2_values(i,:)=value(u_optimal(2,:));

  
    %Linear Dynamics
    x3=u1_values(i,:);
    x1=x3'.*dt+x1;
    x4=u2_values(i,:);
    x2=x4'.*dt+x2;
    
 
    
    
     
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