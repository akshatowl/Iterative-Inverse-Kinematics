%Inverse Kinematics of 3R arm with Iterative Newton Raphson
%Inverse Jacobian Approach
%Script by Akshat Pandey
%CSV file was used for Coppelia Sim simulation
clear all
clc

syms x y gamma l1 l2 l3 jacobian theta1 theta2 theta3 thetabuffer; %Symbolic Toolbox

l1=2  %Defining Length of Individual Links
l2=2;
l3=2;
thetabuffer=[theta1;theta2;theta3];

initstore(:,1)=[0,0,0];  %Initial 6 iterations, Not a necessary step
initstore(:,2)=[0,0,0];
initstore(:,3)=[0,0,0];
initstore(:,4)=[0,0,0];
initstore(:,5)=[0,0,0];


f1=subs(x-(l1*cos(theta1)+l2*cos(theta1+theta2)+l3*cos(theta1+theta2+theta3))); %Function formulation
f2=subs(y-(l1*sin(theta1)+l2*sin(theta1+theta2)+l3*sin(theta1+theta2+theta3)));
f3=subs(gamma-(theta1+theta2+theta3));
f=subs([f1;f2;f3]);

%Jacobian matrix
jacobian=([diff(f1,theta1),diff(f1,theta2),diff(f1,theta3);diff(f2,theta1),diff(f2,theta2),diff(f2,theta3);diff(f3,theta1),diff(f3,theta2),diff(f3,theta3)]);



x=0; %X-end effector
y=6; %Y-end effector
gamma=pi/2; %Gamma angle/ Angle third link makes with X-axis
thetaprev=[0.5;0.5;0.5]; %Initialize
thetabuffer=thetaprev;
theta1=0.5;
theta2=0.5;
theta3=0.5;

for i=6:27  % iterations till 6 were pre stored for better output in CSV file
count=1;
    thetabuffer=double((subs(thetabuffer)))-double(subs(inv(jacobian)))*double(subs(f));%Main Equation
    theta1=thetabuffer(1); %buffer matrix to another variable for convenience
    theta2=thetabuffer(2);
    theta3=thetabuffer(3);
    theta11=atan2(sin(theta1),cos(theta1)); %mapping angles -pi to pi, final angle
    theta22=atan2(sin(theta2),cos(theta2));
    theta33=atan2(sin(theta3),cos(theta3));
    thetabuffermain=[theta11;theta22;theta33]; %organizing angles in matrix
    disp(thetabuffermain');
    
    %Plotting
    %Forward Kinematics of Simple 3-R Robotic Arm
    x1=l1*cos(theta11);
    y1=l1*sin(theta11);
    
    x2=x1+(l2*cos(theta11+theta22));
    y2=y1+(l2*sin(theta11+theta22));
    
    x3=x2+(l3*cos(theta11+theta22+theta33));
    y3=y2+(l3*sin(theta11+theta22+theta33));
    plot([0,x1],[0,y1],[x1,x2],[y1,y2],[x2,x3],[y2,y3], 'linewidth',3)
    grid on
    axis([-0.1 8 -0.1 8])
    pause(0.1)
    m(count) =getframe(gcf);
    count=count+1;
    
    
    %CSV format convenience
    initstore(:,i)=thetabuffermain';
    error=abs(thetabuffer-thetaprev);
    if(error<10^-6)
        break;
    end
    
end
    %error=abs(thetabuffermain-thetaprev);
    csvwrite('plsgetdata',initstore');
    mat=initstore';
    xf=l1*cos(theta11)+l2*cos(theta11+theta22)+l3*cos(theta11+theta22+theta33); %final end effector X
    yf=l1*sin(theta11)+l2*sin(theta11+theta22)+l3*sin(theta11+theta22+theta33); %final end effector Y
    error=abs([x;y]-[xf;yf]);
  