function [] = plotRobot( traj,t, param )
%plot position, given Pose
%parameters contains: [e,f,re,rf]

% initialize
%coords
x=traj(1); y=traj(2); z=traj(3);
t1=t(1); t2=t(2); t3=t(3);
%rod lengths in m:
rf=param(4); 
re=param(3);

%triangular side lengths in m:
f=param(2); 
e=param(1);

%% plot
%init
grid on
%to adjust the view
l=(f+2*rf);
axis([-l l -l l -l*2 l/2])
cla;

%calc and plot robot f plate
P_f1=[f/2,-f/(2*sqrt(3)),0];
P_f2=[0,f/cos(30*pi/180)/2,0];
P_f3=[-f/2,-f/(2*sqrt(3)),0];

line([P_f1(1) P_f2(1)],[P_f1(2) P_f2(2)],[P_f1(3) P_f2(3)])
line([P_f2(1) P_f3(1)],[P_f2(2) P_f3(2)],[P_f2(3) P_f3(3)])
line([P_f3(1) P_f1(1)],[P_f3(2) P_f1(2)],[P_f3(3) P_f1(3)])

hold on
%calc and plot robot end-effector pose
P_e1=traj'+[e/2,-e/(2*sqrt(3)),0];
P_e2=traj'+[0,e/cos(30*pi/180)/2,0];
P_e3=traj'+[-e/2,-e/(2*sqrt(3)),0];

line([P_e1(1) P_e2(1)],[P_e1(2) P_e2(2)],[P_e1(3) P_e2(3)])
line([P_e2(1) P_e3(1)],[P_e2(2) P_e3(2)],[P_e2(3) P_e3(3)])
line([P_e3(1) P_e1(1)],[P_e3(2) P_e1(2)],[P_e3(3) P_e1(3)])

hold on

plot3(x,y,z,'o','color','green')%plotting the end-effector point

%calc and plot joints f
P_F1=[0,-f/(2*sqrt(3)),0];
%use rotation matrix to calc other points
deg=120;
R=[cosd(deg),-sind(deg),0;sind(deg),cosd(deg),0;0,0,1];
P_F2=(R*P_F1')';
P_F3=(R*P_F2')';
%Plot F1,F2,F3
hold on
plot3(P_F1(1),P_F1(2),P_F1(3),'o','color','green')
hold on
plot3(P_F2(1),P_F2(2),P_F2(3),'o','color','green')
hold on
plot3(P_F3(1),P_F3(2),P_F3(3),'o','color','green')
hold on

P_J1=[0,-f/(2*sqrt(3))-rf*cos(t1),-rf*sin(t1)];
P_J2=(R*[0,-f/(2*sqrt(3))-rf*cos(t2),-rf*sin(t2)]')';
deg=-120;
R=[cosd(deg),-sind(deg),0;sind(deg),cosd(deg),0;0,0,1];
P_J3=(R*[0,-f/(2*sqrt(3))-rf*cos(t3),-rf*sin(t3)]')';
%rf link plot
line([P_F1(1) P_J1(1)],[P_F1(2) P_J1(2)],[P_F1(3) P_J1(3)],'color','red')
line([P_F2(1) P_J2(1)],[P_F2(2) P_J2(2)],[P_F2(3) P_J2(3)],'color','red')
line([P_F3(1) P_J3(1)],[P_F3(2) P_J3(2)],[P_F3(3) P_J3(3)],'color','red')
%plot J1, J2, J3
hold on
plot3(P_J1(1),P_J1(2),P_J1(3),'o','color','green')
hold on
plot3(P_J2(1),P_J2(2),P_J2(3),'o','color','green')
hold on
plot3(P_J3(1),P_J3(2),P_J3(3),'o','color','green')
hold on

%calc and plot joint e
Transl=[0,-e/(2*sqrt(3)),0];
P_E1=Transl+[x,y,z];
deg=120;
R=[cosd(deg),-sind(deg),0;sind(deg),cosd(deg),0;0,0,1];
P_E2=((R*Transl')+traj)';
deg=-120;
R=[cosd(deg),-sind(deg),0;sind(deg),cosd(deg),0;0,0,1];
P_E3=((R*Transl')+traj)';
%re link plot
line([P_J1(1) P_E1(1)],[P_J1(2) P_E1(2)],[P_J1(3) P_E1(3)],'color','red')
line([P_J2(1) P_E2(1)],[P_J2(2) P_E2(2)],[P_J2(3) P_E2(3)],'color','red')
line([P_J3(1) P_E3(1)],[P_J3(2) P_E3(2)],[P_J3(3) P_E3(3)],'color','red')

hold on
xlabel(['x = ' num2str(x)]);
ylabel(['y = ' num2str(y)]);
zlabel(['z = ' num2str(z)]);

end







