data = readmatrix('data.csv'); % Replace 'data.csv' with the actual file path
time = data(:,1);
x = data(:,2);
y = data(:,3);
yaw_error = data(:,4);
u = data(:,5);

%Plotting our Path
figure;
plot(x, y, '-o');
xlabel('X Position (meters)');
ylabel('Y Position (meters)');
title('Logged Path of the AUV');
grid on;
axis equal;


%Subplots for Yaw Error  & Control Effort Over Time
figure;
subplot(2,1,1); % Yaw error plot
plot(time, yaw_error);
xlabel('Time (seconds)');
ylabel('Yaw Error (radians)');
title('Yaw Error Over Time');
grid on;

subplot(2,1,2); % Control effort plot
plot(time, u);
xlabel('Time (seconds)');
ylabel('Control Effort (u)');
title('Control Effort Over Time');
grid on;
