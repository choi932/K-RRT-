function [final_state, trajectory, control] = steer_kinodynamic(start_state, target_state, u_max, dt)
% steer_kinodynamic
% Steers from start_state TOWARDS target_state for a fixed duration dt.
% This is a simple, robust forward propagation method.

% Proportional controller to determine desired acceleration
kp_pos = 2.0; % Proportional gain for position error
kp_vel = 1.5; % Proportional gain for velocity error

pos_err = target_state(1:3) - start_state(1:3);
vel_err = target_state(4:6) - start_state(4:6);

% Calculate desired acceleration
desired_accel = kp_pos * pos_err + kp_vel * vel_err;

% Saturate control input to system limits
control_norm = norm(desired_accel);
if control_norm > u_max
    control = (desired_accel / control_norm) * u_max;
else
    control = desired_accel;
end

% Discretize the trajectory for collision checking
num_steps = 10;
time_steps = linspace(0, dt, num_steps);
trajectory = zeros(num_steps, 6);
current_state = start_state;

for i = 1:num_steps
    t = time_steps(i);
    % Simple Euler integration
    p_prev = current_state(1:3);
    v_prev = current_state(4:6);
    
    v_new = v_prev + control * (dt/num_steps);
    p_new = p_prev + v_prev * (dt/num_steps) + 0.5 * control * (dt/num_steps)^2;
    
    current_state = [p_new, v_new];
    trajectory(i, :) = current_state;
end

final_state = trajectory(end, :);

end