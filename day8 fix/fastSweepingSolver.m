function [U,iter] = fastSweepingSolver(N,target_i,target_j,obstacle,maxIter,tol)
% fastSweepingSolver
%
% INPUTS:
% N = number of grid points in each direction
% target_i,target_j = target indices
% obstacle = true/false array for obstacle points
% maxIter = maximum number of iterations
% tol = convergence tolerance
%
% OUTPUTS:
% U = computed value function
% iter = number of iterations used
% 1. Compute grid spacing h for [0,1] x [0,1].
h = 1/(N-1);
% 2. Initialize U to infinity.
U = Inf(N, N);
% 3. Set the target value to zero.
U(target_i, target_j) = 0;
% 4. Main iteration loop.
for iter = 1:maxIter
    U_old = U;
    % 5. Perform one full set of four sweeps.
    U = fastSweepIteration(U, h, target_i, target_j, obstacle);
    % 6. Compute largest change.
    change = abs(U_old-U);
    % 7. Stop if converged.
    if change < tol
        break;
    end
end
end