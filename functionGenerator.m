%Leg Function Generator
%Converts a given series of x,y coordinates into alpha,beta coordinates.
%This path is a semicircle. The matrix "master" stores pairs of servo
%angles that correspond to the two motors on each leg.
clear
clc

fig = figure();
fig1 = figure();
fig2 = figure();
fig3 = figure();
fig4 = figure();

T=2;        %leg motion period, seconds
f=.02;      %time step size
L=.05;      %semicircle diameter, meters
h=-.08;     %bottom edge y coordinate

% More Constants
A=.015;     %Linkage lengths in m
B=.050;
C=.035;

%Generate the leg path's x and y coordinates

n=floor(T/f);
txy=zeros(3,n);     %row 1: time, row 2: x, row 3: y
for i=1:n
    txy(1,i)=(i-1)*f;
end

syms t
xFunction=piecewise(t<3*T/4,-(L/(3*T/4))*t+(L/2),t>=3*T/4,(L/2)*cos((4*pi/T)*t));
yFunction=piecewise(t<3*T/4,h,t>=3*T/4,h+(-L/2)*sin((4*pi/T)*t));

for i=1:n
    txy(2,i)=subs(xFunction,t,txy(1,i));
    txy(3,i)=subs(yFunction,t,txy(1,i));
end

%Convert those to angles alpha and beta

tab=zeros(3,n);
tab(1,:)=txy(1,:);

for i=1:n
    angles=xyToAngles([txy(2,i);txy(3,i)]);
    tab(2,i)=angles(1);
    tab(3,i)=angles(2);
end

%Package for exporting, include angles for all 8 servos

%Master is a 5xn matrix. Row 1 is time. Rows 2-3 are alpha and beta for leg
%one and leg two. Rows 4-5 are alpha and beta for leg three and leg four

master=zeros(5,n);
master(1:3,:)=tab;
%Phase offset T/4 the remaining pairs of motors

shifted=circshift(master(2:3,:), floor(n/2), 2);
master(4:5,:)=shifted;

%Motor offsets and unit conversions
for i=1:2
    for j=1:n
        master(2*i,j)=(180/pi)*(master(2*i,j)-(7*pi/4));      %alpha motors
        master(2*i+1,j)=(180/pi)*-1*(master(2*i+1,j)-(5*pi/4));  %beta motors
        %That factor of -1 is because the alpha and beta servos face
        %opposite directions.
    end
end

%% Plot for troubleshooting

figure(fig)
plot(txy(2,:),txy(3,:))

figure(fig1)
hold on
plot(master(1,:),master(2,:))
plot(master(1,:),master(3,:))
plot(master(1,:),master(4,:))
plot(master(1,:),master(5,:))
legend('alpha12','beta12','alpha34','beta34')
hold off

%% Angles to x and y again (Sanity check)
xy_master = master;
for i = 2:2:4
    xy_master(i,:) = A*cos(master(i,:)*(pi/180)+7*pi/4)+B*cos(-1*(master(i+1,:)*(pi/180)-5*pi/4))+C*cos(master(i,:)*(pi/180)+7*pi/4);
    xy_master(i+1,:) = A*sin(master(i,:)*(pi/180)+7*pi/4)+B*sin(-1*(master(i+1,:)*(pi/180)-5*pi/4))+C*sin(master(i,:)*(pi/180)+7*pi/4);
end

% Plot y
figure(fig2)
hold on
for y = 3:2:5
    plot(xy_master(1,:), xy_master(y,:))
end
legend('Leg 1 and 2', 'Leg 3 and 4')
title('Y position vs time for each leg')
hold off

% Plot x
figure(fig3)
hold on
for y = 2:2:4
    plot(xy_master(1,:), xy_master(y,:))
end
legend('Leg 1 and 2', 'Leg 3 and 4')
title('X position vs time for each leg')
hold off


figure(fig4)
hold on
% Leg trajectory animation
tCirclePlot=linspace(0,2*pi,100);
xCirclePlot=.1*cos(tCirclePlot);
yCirclePlot=.1*sin(tCirclePlot);
plot(xCirclePlot,yCirclePlot)

for i=1:n
    plot(txy(2,i),txy(3,i),'ko')
    pause(0.1)
end

% Convert Master from angles to pulse duration values and then to duty
% cycle values
% 2ms = +50 degrees, 1ms = -50 degrees, 1.5ms = 0 degrees

us_per_degree = 1000/100;

center_value = 1500;

% Values are now in microseconeds
master(2:end, :) = (us_per_degree)*master(2:end, :) + center_value;

% Output CSV file
writematrix(round(master(2:end, :)), 'angles.csv')




