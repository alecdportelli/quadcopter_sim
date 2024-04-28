figure;
plot(out.time_s, out.horz_speed_mps, 'b', 'LineWidth', 2);
hold on;
plot(out.time_s, out.horz_speed_cmd_mps, 'r--', 'LineWidth', 2);
legend('Horizontal Speed', 'HZS CMD');
xlabel('Time (s)'); % Label for the x-axis
ylabel('M/s'); % Label for the y-axis
title('Horizontal Controller Response'); % Title for the plot

% figure;
% plot(out.time_s, out.pitch_deg, 'b', 'LineWidth', 2);
% hold on;
% plot(out.time_s, out.pitch_cmd_deg, 'r--', 'LineWidth', 2);
% legend('Quad Pitch', 'Pitch CMD');
% xlabel('Time (s)'); % Label for the x-axis
% ylabel('Pitch degrees'); % Label for the y-axis
% title('Quad Pitch Controller Response'); % Title for the plot

% figure;
% plot(out.time_s, out.yaw_deg, 'b', 'LineWidth', 2);
% hold on;
% plot(out.time_s, out.yaw_cmd_deg, 'r--', 'LineWidth', 2);
% legend('Quad Yaw', 'Yaw CMD');
% xlabel('Time (s)'); % Label for the x-axis
% ylabel('Yaw (s)'); % Label for the y-axis
% title('Quad Controller Response'); % Title for the plot
% 
% figure;
% plot(out.time_s, out.alt_m, 'b', 'LineWidth', 2);
% hold on;
% plot(out.time_s, out.alt_cmd_m, 'r--', 'LineWidth', 2);
% legend('Altitude', 'Altitude CMD');
% xlabel('Time (s)'); % Label for the x-axis
% ylabel('Altitude (m)'); % Label for the y-axis
% title('Altitude Controller Response'); % Title for the plot