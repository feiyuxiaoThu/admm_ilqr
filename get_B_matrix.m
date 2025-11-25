function B = get_B_matrix(x, u, dt, L)
    % GET_B_MATRIX  Compute discrete-time control Jacobian B = df/du
    % Inputs:
    %   x: state [X; Y; phi; v]
    %   u: control [a; delta]
    %   dt: time step
    %   L: wheelbase
    % Output:
    %   B: 4x2 control Jacobian

    % extract variables
    v = x(4);
    delta = u(2);

    % build Jacobian B
    B = zeros(4, 2);
    B(3, 2) = v / (L * cos(delta)^2) * dt;
    B(4, 1) = dt;
    
    % B = [0, 0;
    %      0, 0;
    %      0, v / (L * cos(delta)^2) * dt;
    %      dt, 0];
end
