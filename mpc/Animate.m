%% setting parameters
lane_width = 10;
% t = 0:0.02:10;
% x_ref = [0:0.4:200]; % from c++
% y_ref = 9*tanh(t-t(end)/2); % from c++
lf = 2; % from c++
lr = 3; % from c++
dim1 = [0.18 0.2 .1 .1];
data = states;
input = states(:, 5);
x_ref = x_ref';
y_ref = y_ref';
%% reading matrices
% data = readmatrix('total_states'); % from c++
% input = readmatrix('inputs'); % from c++
%% setting figure and objects

% setting lanes
fig_x = 8; fig_y = 6
t_len = 0.7
vehicle_traj = figure();
% figure();
set(vehicle_traj, 'position', [20, 50, 1500, 700]);
for j = 1:500
    plot([x_ref(1), x_ref(end)], [-lane_width, -lane_width], 'k', 'LineWidth', 0.5, 'LineStyle', '-');
    hold on
    plot([x_ref(1), x_ref(end)], [0, 0], 'k', 'LineWidth', 0.5, 'LineStyle', '--');
    hold on
    plot([x_ref(1), x_ref(end)], [lane_width, lane_width], 'k', 'LineWidth', 0.5, 'LineStyle', '-');
    ylim([-20 20])
    hold on
    % setting reference trajectory
    plot(x_ref, y_ref, 'y', 'LineWidth', 1.2);
    hold on
    plot([x_ref(1, j)-lr*cos(data(j, 2)),x_ref(1, j)+lf*cos(data(j,2))], [data(j,4)-lr*sin(data(j,2)),data(j,4)+lf*sin(data(j,2))], 'r', 'LineWidth', 2.5);
    hold on 
    plot([x_ref(1, j) + lf*cos(data(j,2))-t_len*cos(data(j,2)+input(j)),x_ref(1, j) + lf*cos(data(j,2))+t_len*cos(data(j,2)+input(j))], ...
        [data(j, 4) + lf*sin(data(j,2))-t_len*sin(data(j,2)+input(j)), data(j, 4) + lf*sin(data(j,2))+t_len*sin(data(j,2)+input(j))], 'k', 'LineWidth', 2);
    hold on 
    plot([x_ref(1, j) - (lr+t_len)*cos(data(j,2)), x_ref(1, j) - (lr-t_len)*cos(data(j,2))], ...
        [data(j, 4) - (lr+t_len)*sin(data(j,2)), data(j, 4) - (lr-t_len)*sin(data(j,2))], 'k', 'LineWidth', 2)
    axis equal
    annotation('textbox',dim1,'string',sprintf('angle %.2f degrees', input(j)*180/pi));
    drawnow
%     pause(0.05);
    clf(vehicle_traj);
end

%%
% lf = 2;
% lr = 3;
% % syms j
% j = 1:1:500;
% k = 1:1:500;
% % t = 0:0.02:10;
% % f = @(t) plot([t*20-2, t*20+2],[9*tanh(t- 10/2)-2, 9*tanh(t- 10/2)+2],'-');
% % car = @(t) plot([X_ref[num]-lr*np.cos(statesTotal[num,1]),X_ref[num]+lf*np.cos(statesTotal[num,1])],
% %         [statesTotal[num,3]-lr*np.sin(statesTotal[num,1]),statesTotal[num,3]+lf*np.sin(statesTotal[num,1])])
% figure
% for j = 1:500
%     plot([x_ref(1, j)-lr*cos(data(j, 2)),x_ref(1, j)+lf*cos(data(j,2))], [data(j,4)-lr*sin(data(j,2)),data(j,4)+lf*sin(data(j,2))], '-');
%     pause(0.1);
% end
% % car = @(j) plot([x_ref(1, j)-lr*cos(data(j, 2)),x_ref(1, j)+lf*cos(data(j,2))], [data(j,4)-lr*sin(data(j,2)),data(j,4)+lf*sin(data(j,2))], '-');
% % fanimator(car)
% % % fanimator(@fplot,[x_ref(1, j)-lr*cos(data(j, 2)),x_ref(1, j)+lf*cos(data(j,2))], [data(j,4)-lr*sin(data(j,2)),data(j,4)+lf*sin(data(j,2))])
% % % axis equal
% % playAnimation
% 



