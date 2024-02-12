K = 1.7;
sigma = 16;
Kp = 4;
open_system('motor_control');
out = sim('motor_control');

load('stepData.mat')
% convert from ms to s
data(:, 1) = data(:, 1) ./ 1000;

figure
subplot(2, 1, 1)
plot(out.voltage, '--', 'LineWidth', 2)
hold on
% graph both motor voltages
plot(data(:, 1), data(:, 2), ':', 'LineWidth', 2)
%plot(data(:, 1), data(:, 3), 'LineWidth', 2)
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')
legend('Simulated', 'Experimental Motor 1', 'Experimental Motor 2')
subplot(2, 1, 2)
plot(out.velocity, '--', 'LineWidth', 2)
hold on
plot(out.desired_velocity, ':', 'LineWidth', 2)
% graph both motor velocities
plot(data(:, 1), data(:, 4), '-.', 'LineWidth', 2)
%plot(data(:, 1), data(:, 5), 'LineWidth', 2)
hold off
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')
legend('Simulated', 'Desired', 'Experimental Motor 1', 'Experimental Motor 2')
