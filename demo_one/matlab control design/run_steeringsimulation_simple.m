% run_steeringsimulation_simple
%
% runs a simulation of a two wheeled robot with feedback control
% of the position of each wheel, and motion prescribed by setting the
% desired wheel position vs time. This script sets up
% the parameters of the robot, and the input voltages as timeseries
% The simulink model 'steeringsimulation_simple' is run, and the results plotted


% conversions from angle to counts and back
rad_to_counts = 3600/(2*pi);
counts_to_rad = 1/rad_to_counts;
r_measured_ft = .5; % measured wheel radius;
b_measured_ft = 1; % measured robot width;
r_actual_ft = .5; % actual wheel radius
b_actual_ft = 1; % actual robot width
Ts=.01; % sample time in seconds
%
% right wheel parameters
%
K_r=1;
sigma_r=10;
%K_r=1.0;
%sigma_r=10;

%
% left wheel parameters
%
K_l=1;
sigma_l=10;

%
% rotate, then move
left_distance_desired=timeseries([-b_measured_ft*pi/4 -b_measured_ft*pi/4 -b_measured_ft*pi/4+2 -b_measured_ft*pi/4+2],[0 4.9 5 10]);
right_distance_desired=timeseries([b_measured_ft*pi/4 b_measured_ft*pi/4 r_measured_ft*pi/4+2 r_measured_ft*pi/4+2],[0 4.9 5 10]);


%
% immediate change of position setpoint
%left_distance_desired=timeseries([0 0 2 2],[0 .9 1 10]);
%right_distance_desired=timeseries([0 0 2 2],[0 .9 1 10]);

%
% ramped change of position setpoint
%left_distance_desired=timeseries([0 0 2 2],[0 1  2 10]);
%right_distance_desired=timeseries([0 0 2 2],[0 1 2 10]);



out=sim('steeringsimulation_simple.slx')
figure(1)
clf
plot(left_distance_desired)
hold on
plot(right_distance_desired)
plot(out.Wheel_Pos)
set(gca,'fontsize',14)
legend('left setpoint','right setpoint','left','right','location','northwest')
xlabel('Time (s)')
ylabel('Position (ft)')
title('Wheel position')
figure(2)
clf
plot(out.Pos)
set(gca,'fontsize',14)
xlabel('Time (s)')
ylabel('Position (ft)')
legend('X','Y','Phi')
title('Robot Position')
figure(3)
animate

