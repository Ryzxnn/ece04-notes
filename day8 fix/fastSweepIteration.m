function U = fastSweepIteration(U,h,target_i,target_j,obstacle)
% fastSweepIteration
%
% INPUTS:
% U = current value function
% h = grid spacing
% target_i,target_j = target indices
% obstacle = true/false array for obstacle points
%
% OUTPUT:
% U = updated value function after four sweep directions

function U = sweepEik2(U, range_i, range_j)
    for i = range_i
        for j = range_j
            if i == target_i && j == target_j
                continue;
            end
            if obstacle(i,j)
                continue;
            end
            a = min(U(i, j+1), U(i, j-1));
            b = min(U(i+1, j), U(i-1, j));
            u_candidate = localEikonalUpdate(a,b,h, 1);
            U(i,j) = min(U(i,j), u_candidate);
        end
    end
end

N = size(U,1);

dirs_x = [2:N-1; N-1:-1:2; 2:N-1; N-1:-1:2];
dirs_y = [2:N-1; 2:N-1; N-1:-1:2; N-1:-1:2];

for d = 1:height(dirs_x)
    U = sweepEik2(U, dirs_x(d, :), dirs_y(d, :));
end

end
