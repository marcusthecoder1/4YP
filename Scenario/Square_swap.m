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

x1=[20;-20;20;-20]; % x Positions  
x2=[20;-20;-20;20]; % y Positions  
x3=[0;0;1;2]; % x Speeds 
x4=[0;0;1;2]; % y Speeds 
xd=[-20.1,-20;20,20.2;-20,20.4;20.5,-20]; %Desired position

D= 6; % min vehicle separation

% Define center coordinates
x_c0 = 2;  % x-coordinate of the center
y_c0 = 3;  % y-coordinate of the center


% Initialize plots
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
Line11=animatedline('Color','c');
Line12 = animatedline('Color', 'c', 'Marker', 'o');


%Matrices for plots/calculations
x1_values=zeros(N,length(t));
u1_values=zeros(length(t),N);
u2_values=zeros(length(t),N);
x2_values=zeros(N,length(t));
h_values=zeros(N,length(t));
h_agent_values=zeros(N*(N-1)/2,length(t));
d_norms=zeros(N*(N-1)/2,1);
dist2=zeros(2,N*(N-1)/2);


% Set axis limits
xlim([-40, 40]);
ylim([-40, 40]);
    
  
for i=1:length(t)
    
    %Logging values
    x1_values(:,i)=value(x1);
    x2_values(:,i)=value(x2);
    
    %Matrix of position vectors
    positions=[x1' ; x2'];
        
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
    d=sqrt((x1-x_c).^2+((x2-y_c)).^2)-sqrt(a^2*b^2*((x1-x_c).^2+((x2-y_c)).^2))./sqrt(b^2*(x1-x_c).^2+a^2*((x2-y_c)).^2);
    h=d;
    h_values(:,i)=h;

    % All separation distances of agents(magnitudes, vectors and input difference)
    u_M=sdpvar(2,N,'full');
    count=1;
   
    for j=1:N-1
        for k=j:N-1
            d_norms(count)= abs(norm(positions(:,j)-positions(:,k+1)));
            dist2(:,count)= positions(:,j)-positions(:,k+1);
            u_difference(:,count)=u_M(:,j)-u_M(:,k+1);
            count =count+1;
        end
    end
    
    h_agents=d_norms-D;
    h_agent_values(:,i)=h_agents;
    
    
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
    clearpoints(Line11);
    clearpoints(Line12);
    
    
    %addpoints(Line1,x_e,y_e);
    %addpoints(Line2,x_road,y_road1);
    addpoints(Line3,x1_values(1,1:i),x2_values(1,1:i));
    addpoints(Line4,xd(1,1),xd(1,2));
    addpoints(Line5,x1_values(N-1,1:i),x2_values(N-1,1:i));
    addpoints(Line6,xd(N-1,1),xd(N-1,2));
    addpoints(Line7,x1_values(N,1:i),x2_values(N,1:i));
    addpoints(Line8,xd(N,1),xd(N,2));
    addpoints(Line9,x1_values(N-2,1:i),x2_values(N-2,1:i));
    addpoints(Line10,xd(N-2,1),xd(N-2,2))
    addpoints(Line11,x1_values(N-3,1:i),x2_values(N-3,1:i));
    addpoints(Line12,xd(N-3,1),xd(N-3,2))

    drawnow
    
    %Barrier functions of ellipse distance
    B = -log(h./(1+h));
    
    dBdx1= (-1./(h+h.^2)).*(x1-x_c).*((b^2*(x1-x_c).^2+a^2*(x2-y_c).^2).^1.5+(a*b^3-a^3*b)*(x2-y_c).^2)./(sqrt((x1-x_c).^2+((x2-y_c)).^2).*(b^2*(x1-x_c).^2+a^2*((x2-y_c)).^2).^1.5);
    dBdx2= (-1./(h+h.^2)).*(x2-y_c).*((b^2*(x1-x_c).^2+a^2*(x2-y_c).^2).^1.5+(a^3*b-a*b^3)*(x1-x_c).^2)./(sqrt((x1-x_c).^2+((x2-y_c)).^2).*(b^2*(x1-x_c).^2+a^2*((x2-y_c)).^2).^1.5);
    
    %Barrier functions for agent separations
    
    B_agents=-log(h_agents./(1+h_agents));
    
    dBdx1_agents= (-1./(h_agents+h_agents.^2)).*(dist2(1,:)'./(d_norms));
    dBdx2_agents= (-1./(h_agents+h_agents.^2)).*(dist2(2,:)'./(d_norms));
    
    %Barrier Constraint setup for ellipse distance
    LgB1= dBdx1;
    LgB2= dBdx2;
    
    LfB= -dBdx1.*(m1/dt)-dBdx2.*(m2/dt);

    A_cbf=[LgB1 LgB2];
    b_cbf=-LfB + v./B;
    
   
    %Barrier Constraint setup for agent distance
    
    LgB1_agents= dBdx1_agents;
    LgB2_agents= dBdx2_agents;
    
    LfB_agents=0;
    
    A_cbf_agents=[LgB1_agents  LgB2_agents];
    
    b_cbf_agents=-LfB_agents + v./B_agents;
 
    
    %Lyapunov constraint setup
    psi_0 = e.*y'*y;
    psi_1 = 2.*y';
    
    A_clf=psi_1;
    b_clf=-diag(psi_0);
    

    %Constraints
    model=[
        diag(A_clf*u_M) <= b_clf  
    
        %diag(A_cbf*u_M) <= b_cbf

        diag(A_cbf_agents*u_difference) <= b_cbf_agents
       
        u_M<=500
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
% %Plot position evolution
% subplot(1,3,1)
% plot(x_e0,y_e0) 
% hold on
% plot(x_e,y_e) 
% hold on
%  plot(double(x1_values),double(x2_values))
%  xlabel('x');
% ylabel('y');
% title('x vs y');

%Speed plot
%subplot(1,3,2)
% plot(t,double(u2_values))
%  xlabel('t (s)');
% ylabel('u2 (speed y) ');
% title(' speed(y) vs time');

%Barrier plot
%subplot(1,3,3)
plot(t,double(h_agent_values))
 xlabel('t (s)');
ylabel('h');
title('Hard Constraint vs time');