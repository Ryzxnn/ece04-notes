function U = periodicFastSweepingSolver(theta1, theta2, target_i, target_j, unsafe, tol, maxIter)
arguments (Input)
    theta1
    theta2
    target_i
    target_j
    unsafe
    tol = 1e-8
    maxIter = 50
end

arguments (Output)
    U
end

BIG = 3000;

N = length(theta1);

% h = 2*pi/N;
h = theta1(2) - theta2(1);

U = BIG * ones(N, N);
U(target_i, target_j) = 0;
U(unsafe) = inf;

if isinf(U(target_i,target_j))
    error('Target was marked as unsafe.');
end

for iter = 1:maxIter
    U_old = U;
    dirs_x = {1:N; N:-1:1; 1:N; N:-1:1};
    dirs_y = {1:N; 1:N; N:-1:1; N:-1:1};
    for d = 1:height(dirs_x)
        for i = dirs_y{d}
            for j = dirs_x{d}
                if i == target_i && j == target_j
                    continue;
                end
                if unsafe(i,j)
                    continue;
                end
                [i_up,i_down,j_left,j_right] = getPeriodicNeighbors(i, j, N-1);
                u_up = U(i_up, j);
                u_down = U(i_down, j);
                u_left = U(i, j_left);
                u_right = U(i, j_right);
                u_candidate = localEikonalUpdate(min(u_left, u_right), min(u_up, u_down), h, 1);
                U(i,j) = min(U(i,j), u_candidate);
            end
        end
    end

    % Compute change only where both old and new values are finite.
    finite_mask = isfinite(U) & isfinite(U_old);
    change = max(abs(U(finite_mask) - U_old(finite_mask)));
    disp(['iter = ', num2str(iter), ', change = ', num2str(change)]);
    if change <= tol
        break;
    end
end

end
