function [i_up,i_down,j_left,j_right] = getPeriodicNeighbors(i,j,N)
% getPeriodicNeighbors
%
% INPUTS:
% i, j = current grid indices
% N = number of grid points in each direction
%
% OUTPUTS:
% i_up, i_down, j_left, j_right = wrapped neighbor indices
i_up = i-1;
i_down = i+1;
j_left = j-1;
j_right = j+1;
% Wrap row indices.
if i_up < 1
    i_up = N;
end
if i_down > N
    i_down = 1;
end
% Wrap column indices.
if j_left < 1
    j_left = N;
end
if j_right > N
    j_right = 1;
end
end