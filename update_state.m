function x_next = update_state(x, u, dt, L)
% UPDATE_STATE  Integrate vehicle kinematics using forward Euler
% Inputs:
%   x: state [X; Y; phi; v]
%   u: control [a; delta]
%   dt: time step
%   L: wheelbase
% Output:
%   x_next: state at next time step

% extract state and control for readability
phi = x(3);
v = x(4);

a = u(1);
delta = u(2);

% state derivative (dx/dt)
x_dot = [v * cos(phi);
    v * sin(phi);
    v / L * tan(delta);
    a];
% forward Euler integration
x_next = x + dt * x_dot;
end
