function [final_path, cost, success] = run_kinodynamic_rrt_star_V2(start_state, goal_state, NFZ_centers, NFZ_radii, bounds, u_max)
% run_kinodynamic_rrt_star_V2.m
% A more robust implementation of Kinodynamic RRT*.
% Uses a fixed-time extension steering function instead of a BVP solver.

% Parameters
max_iter = 30000;          % Increased iterations for better exploration
goal_bias = 0.1;          % Bias towards the goal
dt_steer = 0.1;           % Fixed time duration for each steering extension
goal_threshold_pos = 2.0; % Position tolerance for reaching the goal
goal_threshold_vel = 0.5; % Velocity tolerance for reaching the goal
search_radius = 5.0;      % Search radius for rewiring (cost-based)

% Heuristic weights for nearest neighbor search (Position vs. Velocity)
w_pos = 1.0; % Weight for position distance
w_vel = 0.5; % Weight for velocity distance

% Node structure: {state, parent_idx, cost, trajectory_from_parent}
nodes = { {start_state, 0, 0, []} };
success = false;

fprintf('Running Robust Kinodynamic RRT*...\n');
for k = 1:max_iter
    % 1. Sample a random state
    if rand < goal_bias
        rand_state = goal_state;
    else
        rand_pos = [rand_range(bounds.x), rand_range(bounds.y), rand_range(bounds.z)];
        v_max_sample = 5.0; % Max velocity to sample
        rand_vel = (2*rand(1,3) - 1) * v_max_sample;
        rand_state = [rand_pos, rand_vel];
    end

    % 2. Find the nearest neighbor in the tree
    min_dist = inf;
    idx_nearest = -1;
    for i = 1:length(nodes)
        node_state = nodes{i}{1};
        % Use a weighted distance metric for a more meaningful "nearness"
        dist = sqrt(w_pos * norm(node_state(1:3) - rand_state(1:3))^2 + ...
                    w_vel * norm(node_state(4:6) - rand_state(4:6))^2);
        if dist < min_dist
            min_dist = dist;
            idx_nearest = i;
        end
    end
    nearest_node = nodes{idx_nearest};
    
    % 3. Steer from the nearest node towards the random state
    [new_state, traj_segment, control] = steer_kinodynamic(nearest_node{1}, rand_state, u_max, dt_steer);

    % 4. Collision Check
    if ~is_trajectory_collision_free(traj_segment, NFZ_centers, NFZ_radii)
        continue;
    end
    
    % 5. Choose Parent (Rewire within a radius)
    parent_node = nearest_node;
    parent_idx = idx_nearest;
    min_cost = parent_node{3} + dt_steer; % Cost is time

    neighbor_indices = find_neighbors_in_radius(nodes, new_state, search_radius, w_pos, w_vel);

    for i = 1:length(neighbor_indices)
        idx = neighbor_indices(i);
        potential_parent = nodes{idx};
        
        [temp_state, temp_traj, ~] = steer_kinodynamic(potential_parent{1}, new_state, u_max, dt_steer);
        
        if is_trajectory_collision_free(temp_traj, NFZ_centers, NFZ_radii)
            cost_via_neighbor = potential_parent{3} + dt_steer;
            if cost_via_neighbor < min_cost
                min_cost = cost_via_neighbor;
                parent_idx = idx;
                traj_segment = temp_traj;
                new_state = temp_state; % Update state to be the one from the better parent
            end
        end
    end
    
    % 6. Add the new node to the tree
    new_node_idx = length(nodes) + 1;
    new_node = {new_state, parent_idx, min_cost, traj_segment};
    nodes{new_node_idx} = new_node;
    
    % 7. Rewire the tree
    for i = 1:length(neighbor_indices)
        idx = neighbor_indices(i);
        neighbor_node = nodes{idx};

        [rewired_state, rewired_traj, ~] = steer_kinodynamic(new_state, neighbor_node{1}, u_max, dt_steer);
        
        if is_trajectory_collision_free(rewired_traj, NFZ_centers, NFZ_radii)
            cost_via_new_node = min_cost + dt_steer;
            if cost_via_new_node < neighbor_node{3}
                % Update parent and cost of the neighbor
                nodes{idx}{2} = new_node_idx;
                nodes{idx}{3} = cost_via_new_node;
                nodes{idx}{4} = rewired_traj;
            end
        end
    end

    % 8. Check for Goal
    pos_err = norm(new_state(1:3) - goal_state(1:3));
    vel_err = norm(new_state(4:6) - goal_state(4:6));
    if pos_err < goal_threshold_pos && vel_err < goal_threshold_vel
        fprintf('Goal reached at iteration %d!\n', k);
        success = true;
        break;
    end
end

% 9. Reconstruct Path
final_path = [];
cost = inf;
if success
    goal_idx = length(nodes);
    cost = nodes{goal_idx}{3};
    curr_idx = goal_idx;
    while curr_idx ~= 0
        node = nodes{curr_idx};
        % Prepend the trajectory segment from its parent
        final_path = [node{4}; final_path];
        curr_idx = node{2};
    end
    % Add the start state itself
    final_path = [nodes{1}{1}; final_path];
else
    fprintf('Failed to find a solution after %d iterations.\n', max_iter);
end

end

% --- Helper Functions ---
function val = rand_range(rng)
    val = rng(1) + rand() * (rng(2) - rng(1));
end

function collision = is_trajectory_collision_free(traj, centers, radii)
    if isempty(traj)
        collision = false;
        return;
    end
    for i = 1:size(traj, 1)
        p = traj(i, 1:3);
        for j = 1:size(centers, 1)
            if norm(p - centers(j,:)) <= radii(j)
                collision = false;
                return;
            end
        end
    end
    collision = true;
end

function neighbor_indices = find_neighbors_in_radius(nodes, state, radius, w_pos, w_vel)
    neighbor_indices = [];
    for i = 1:length(nodes)
        dist = sqrt(w_pos * norm(nodes{i}{1}(1:3) - state(1:3))^2 + ...
                    w_vel * norm(nodes{i}{1}(4:6) - state(4:6))^2);
        if dist < radius
            neighbor_indices = [neighbor_indices, i];
        end
    end
end