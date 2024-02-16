K = 1.7;
sigma = 16;
Kp = 2;
open_system('motor_control');
out = sim('motor_control');

%load('stepData.mat')
% convert from ms to s
%data(:, 1) = data(:, 1) ./ 1000;

figure
subplot(3, 1, 1)
plot(out.voltage, 'LineWidth', 2)
xlabel('Time (s)')
ylabel('Voltage (V)')
title('Voltage')
legend('Simulated', 'Experimental Motor 1', 'Experimental Motor 2')
subplot(3, 1, 2)
plot(out.desired_velocity, 'LineWidth', 2)
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
title('Velocity')
legend('Simulated', 'Desired', 'Experimental Motor 1', 'Experimental Motor 2')
subplot(3, 1, 3)
plot(out.desired_position, 'LineWidth', 2)
hold on
plot(out.position, '--', 'LineWidth', 2)
hold off
xlabel('Time (s)')
ylabel('Position (rad)')
title('Position')
legend('Desired', 'Simulated')