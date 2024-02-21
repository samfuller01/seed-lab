% modify motor constants to match experimental values to simulated values in plots
K = 1.7;
sigma = 16;
Kp = 2;
open_system('motor_control');
out = sim('motor_control');

load('stepData.mat') % 'stepData.mat' is generated by 'ReadFromArduino.mlx'
data(:, 1) = data(:, 1) ./ 1000; % convert from ms to s

figure % generate figure
% plot voltage vs time
subplot(3, 1, 1)
plot(out.voltage, 'LineWidth', 2)
xlabel('Time (s)')
ylabel('Voltage (V)')
title('Voltage')
legend('Simulated', 'Experimental Motor 1', 'Experimental Motor 2')

% plot angular velocity vs time
subplot(3, 1, 2)
plot(out.desired_velocity, 'LineWidth', 2)
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
title('Velocity')
legend('Simulated', 'Desired', 'Experimental Motor 1', 'Experimental Motor 2')

% plot position vs time
subplot(3, 1, 3)
plot(out.desired_position, 'LineWidth', 2)
hold on
plot(out.position, '--', 'LineWidth', 2)
hold off
xlabel('Time (s)')
ylabel('Position (rad)')
title('Position')
legend('Desired', 'Simulated')
