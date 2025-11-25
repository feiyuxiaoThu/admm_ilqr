function A = get_A_matrix(x, u, dt, L)
    % GET_A_MATRIX  Compute discrete-time state Jacobian A = df/dx
    % Inputs:
    %   x: state [X; Y; phi; v]
    %   u: control [a; delta]
    %   dt: time step
    %   L: wheelbase
    % Output:
    %   A: 4x4 state Jacobian

    % extract state and control
    phi = x(3);
    v = x(4);
    delta = u(2);

    % build Jacobian A
    A = eye(4);
    A(1, 3) = -v * sin(phi) * dt;
    A(1, 4) = cos(phi) * dt;
    A(2, 3) = v * cos(phi) * dt;
    A(2, 4) = sin(phi) * dt;
    A(3, 4) = tan(delta) / L * dt;
    
    % Example form:
    % A = [1, 0, -v*sin(phi)*dt, cos(phi)*dt;
    %      0, 1,  v*cos(phi)*dt, sin(phi)*dt;
    %      0, 0,  1,             tan(delta)/L*dt;
    %      0, 0,  0,             1];
end
