% clc; clear; close all;
% 
% % Setup environment
% bounds.x = [0, 40];
% bounds.y = [0, 40];
% bounds.z = [0, 20];
% 
% % Drone initial STATES (pos and vel)
% % Assume drones start and want to meet at rest (vel = 0)
% posR = [10, 10, 5];
% posS = [30, 30, 5];
% start_state_R = [posR, 0, 0, 0];
% goal_state_S = [posS, 0, 0, 0]; % RRT* plans from R to S
% 
% % Define NFZs
% NFZ_centers = [20 20 10; 15 15 8];
% NFZ_radii = [4; 5];
% 
% % System dynamics parameters
% u_max = 2.0; % Max acceleration in m/s^2
% 
% % --- Run Kinodynamic RRT* to find a path from R to S ---
% disp('Running Kinodynamic RRT*...');
% [path_kino, time_cost, success] = run_kinodynamic_rrt_star(start_state_R, goal_state_S, NFZ_centers, NFZ_radii, bounds, u_max);
% 
% if ~success
%     warning('Kinodynamic RRT* failed to find a solution.');
%     return;
% end
% 
% % --- Find Docking Point from the Kinodynamic Path ---
% % The logic from getDockingPoint_RRT_star can be adapted here.
% % For simplicity, let's find the mid-point in time along the path.
% mid_time_index = floor(size(path_kino, 1) / 2);
% docking_point_state = path_kino(mid_time_index, :);
% docking_point = docking_point_state(1:3);
% docking_cost = time_cost; % The total time is the cost
% 
% % Split the path for the two drones
% pathR_states = path_kino(1:mid_time_index, :);
% pathS_states_rev = path_kino(mid_time_index:end, :);
% % For supplier, need to reverse the trajectory in time
% pathS_states = flipud(pathS_states_rev);
% pathS_states(:, 4:6) = -pathS_states(:, 4:6); % Reverse velocities
% 
% pathR = pathR_states(:, 1:3);
% pathS = pathS_states(:, 1:3);
% 
% 
% % --- Visualization ---
% figure; hold on; grid on;
% xlabel('X'); ylabel('Y'); zlabel('Z'); view(3);
% axis equal;
% xlim(bounds.x); ylim(bounds.y); zlim(bounds.z);
% 
% % Plot drones and docking point
% scatter3(posR(1), posR(2), posR(3), 100, 'b', 'filled', 'DisplayName', 'Receiver Start');
% scatter3(posS(1), posS(2), posS(3), 100, 'g', 'filled', 'DisplayName', 'Supplier Start');
% scatter3(docking_point(1), docking_point(2), docking_point(3), 150, 'k', '*', 'DisplayName', 'Docking Point');
% 
% % Plot path
% plot3(pathR(:,1), pathR(:,2), pathR(:,3), 'b--', 'LineWidth', 2, 'DisplayName', 'Receiver Path');
% plot3(pathS(:,1), pathS(:,2), pathS(:,3), 'g--', 'LineWidth', 2, 'DisplayName', 'Supplier Path');
% 
% % Plot full RRT* path
% plot3(path_kino(:,1), path_kino(:,2), path_kino(:,3), 'm-', 'LineWidth', 1, 'DisplayName', 'Full RRT* Path');
% 
% % Plot NFZ
% [xs, ys, zs] = sphere(20);
% for i = 1:length(NFZ_radii)
%     surf(NFZ_centers(i,1) + NFZ_radii(i)*xs, ...
%          NFZ_centers(i,2) + NFZ_radii(i)*ys, ...
%          NFZ_centers(i,3) + NFZ_radii(i)*zs, ...
%          'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', 'r', 'HandleVisibility', 'off');
% end
% 
% legend show;
% title(sprintf('Kinodynamic Docking Point Found. Total Time = %.2fs', docking_cost));


clc; clear; close all;

% Setup environment
bounds.x = [0, 40];
bounds.y = [0, 40];
bounds.z = [0, 20];

% Drone initial STATES (pos and vel)
posR = [10, 10, 5];
posS = [30, 30, 5];
start_state_R = [posR, 0, 0, 0];
goal_state_S = [posS, 0, 0, 0];

% Define NFZs
NFZ_centers = [20 20 10; 15 15 8];
NFZ_radii = [4; 5];

% System dynamics parameters
u_max = 3.0; % Max acceleration in m/s^2

% --- Run Robust Kinodynamic RRT* ---
[path_kino, time_cost, success] = run_kinodynamic_rrt_star_V2(start_state_R, goal_state_S, NFZ_centers, NFZ_radii, bounds, u_max);

if ~success
    warning('Kinodynamic RRT* failed to find a solution.');
    % Here you could implement your fallback grid search logic if needed
    return;
end

% --- Find Docking Point from the Kinodynamic Path ---
% Simple approach: find the point closest to the mid-point in time.
mid_time = time_cost / 2;
time_so_far = 0;
docking_idx = 1;
% This requires knowing the time evolution, which we can approximate
% by the number of segments, since each has a fixed dt.
docking_idx = floor(size(path_kino, 1) / 2);
docking_point = path_kino(docking_idx, 1:3);

% Split paths
pathR = path_kino(1:docking_idx, 1:3);
% For the supplier, we must reverse the path
pathS_full_rev = path_kino(docking_idx:end, 1:3);
pathS = flipud(pathS_full_rev);


% --- Visualization ---
% (Visualization code remains the same as in the previous response)
figure; hold on; grid on;
xlabel('X'); ylabel('Y'); zlabel('Z'); view(3);
axis equal;
xlim(bounds.x); ylim(bounds.y); zlim(bounds.z);
scatter3(posR(1), posR(2), posR(3), 100, 'b', 'filled', 'DisplayName', 'Receiver Start');
scatter3(posS(1), posS(2), posS(3), 100, 'g', 'filled', 'DisplayName', 'Supplier Start');
scatter3(docking_point(1), docking_point(2), docking_point(3), 150, 'k', '*', 'DisplayName', 'Docking Point');
plot3(pathR(:,1), pathR(:,2), pathR(:,3), 'b--', 'LineWidth', 2, 'DisplayName', 'Receiver Path');
plot3(pathS(:,1), pathS(:,2), pathS(:,3), 'g--', 'LineWidth', 2, 'DisplayName', 'Supplier Path');
[xs, ys, zs] = sphere(20);
for i = 1:length(NFZ_radii)
    surf(NFZ_centers(i,1) + NFZ_radii(i)*xs, ...
         NFZ_centers(i,2) + NFZ_radii(i)*ys, ...
         NFZ_centers(i,3) + NFZ_radii(i)*zs, ...
         'FaceAlpha', 0.2, 'EdgeColor', 'none', 'FaceColor', 'r', 'HandleVisibility', 'off');
end
legend show;
title(sprintf('Robust Kinodynamic Docking Found. Total Time = %.2fs', time_cost));