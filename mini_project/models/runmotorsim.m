K = 1.7;
sigma = 16;
open_system('motor_sim');
out = sim('motor_sim');
load('stepData.mat')

data(:, 1) = data(:, 1) ./ 1000; % convert from ms to s

figure
subplot(2, 1, 1)
plot(out.voltage, '--', 'LineWidth', 2)
hold on
plot(data(:, 1), data(:, 2), 'LineWidth', 2)
hold off
xlabel('Time (s)')
ylabel('Voltage (V)')
legend('Simulated', 'Experimental')
subplot(2, 1, 2)
plot(out.velocity,  '--', 'LineWidth', 2)
hold on
plot(data(:, 1), data(:, 3), 'LineWidth', 2)
hold off
legend('Simulated', 'Experimental')
xlabel('Time (s)')
ylabel('Angular Velocity (rad/s)')