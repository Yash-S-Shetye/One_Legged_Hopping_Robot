clear;close all;

% Array of different values of m1
m1 = [10;20;50;100];
time_of_flight  = zeros(4,1); % For measuring the time of flight of the robot

% Loop for each value of m1 array
for a = 1:4

%% Set up
% Variables for the simulation
dim=[10,3]; % [bodyheight,bodywidth]
t=0.02; % Time step
time_span=60; % (in seconds)
j=1; % initializing increment operator for errors

% Variables for the model
x0=10; %length of spring
m2 = 1; % Mass of base of robot
g = 9.8; % Acceleration due to gravity
k = 80; % Spring coefficient
c1 = 5; % Damper coefficient for mass1
c2 = 0.5; % Damper coefficient for mass2
z=[3,0,0,0]; % Initial state
h=25; % Goal Height (should be greater than x0)

% Variables for the controller
K=1;
Kp=0.05;Ki=0.06;Kd=0.0001; % PID Coefficients
F=[0 0];I=[0 0];D=[0 0];error=[0 0]; % Initializing the force and all PID Errors
max_height=[0 0];kc=0; % Initializing the max_height and the total PID error

tic % To start measuring the time elapsed since the code starts running
%% Calculate the kinematics
for i=1:time_span/t

    z1=z(i,1);
    z2=z(i,2);
    z3=z(i,3);
    z4=z(i,4);
    znew(1)=t*z2+z1;
    f_ex=0;
    F(i+1)=0;
    
    if z3==0 && z2>=0
        hc=h*(1+kc);
        missing_energy=(m1(a)+m2)*g*hc-m1(a)*g*z1-.5*z2^2*m1(a)-.5*sign(x0-z1)*k*(z1-x0)^2;
        f_ex=K*missing_energy;
        znew(2)=z2+t*(-m1(a)*g-k*(z1-z3-x0)-c2*z2+f_ex)/m1(a);
        F(i+1)=f_ex;
    else
        znew(2)=z2+t*(-m1(a)*g-k*(z1-z3-x0)-c1*(z2-z4)-c2*z2)/m1(a);
    end
    znew(3)=t*z4+z3;
    znew(4)=z(i,4)+t*(-m2*g+k*(z1-z3-x0)+c1*(z2-z4)-c2*z4)/m2;
    if znew(3)<=0
        znew(3)=0;
        if znew(4)<0
            znew(4)=0;
        end
    end
    z(i+1,:)=znew;

    % controller design
    if i>3 && z(i-1,1)<z(i,1)&&z(i+1)<z(i)
        max_height(j)=z(i,1);
        error(j)=h-max_height(j);
        if j>1
            D(j)=error(j)-error(j-1);
            I(j)=I(j-1)+error(j-1);
        end
        kc=Kp*error(j)+Ki*I(j)+Kd*D(j);  %PID
        j=j+1;
    end

end
toc % To stop measuring the time elapses since the code starts running

% Time of flight measurement calculation
for i = 1:length(z)
    % The lenght of the fully stretched spring is 10 and at that instant the base of the robot touches
    % touches the ground so for any value of z greater than 10 the bot will
    % be above the ground and the time will get incremented by the sample
    % rate
    if z(i,1) > 10  
        time_of_flight(a) = time_of_flight(a) + 0.02;
    end
end

% Display the result
figure (a);
% Display the Height and veloity of the robot
subplot(3,1,1)
t_=0:t:time_span;
plot(t_,z);
title(['m1 = ',num2str(m1(a))]);
subtitle('Height and Velocity');
xlabel('time');
ylabel('Magnitude');
legend('Height 1','Velocity 1','Height 2','Velocity 2');

% Display the external force acting on the robot
subplot(3,1,2)
plot(t_,F);
subtitle('External Force');
xlabel('time');
ylabel('N-m');
legend('Force');

% Display the maximum height attained by the robot
subplot(3,1,3);
plot(1:j-1,max_height);
subtitle('Maximum Height');
xlabel('time');
ylabel('Metres(s)');
legend('Height');

% Animation of the hopping robot can be played by uncommenting the following part of the code
 v=VideoWriter('vid1.avi');
 open(v);
 figure(5)
 for k=1:3:length(z)
     draw(z(k,:),dim,h);
     pause(0.01);
     frame=getframe(gcf);
     writeVideo(v,frame);
 end
 close(v);

end
