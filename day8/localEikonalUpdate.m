function u_new = localEikonalUpdate(a,b,h)
% localEikonalUpdate
%
% INPUTS:
% a = best horizontal neighbor value
% b = best vertical neighbor value
% h = grid spacing
%
% OUTPUT:
% u_new = updated value computed from the local Eikonal rule
% 1. Make sure a <= b.
% Hint: if a > b, swap them.
if a > b
    [b, a] = deal(a, b);
end
% 2. Case 1:
% If b-a is larger than or equal to h, information mainly
% comes from the smaller neighbor.
if b-a > h
    %u_new = min(a, b)+h; %a+h
    u_new = a+h;
% 3. Case 2:
% Otherwise, information comes from both directions.
else
    u_new = (a+b+sqrt(2*h^2-(a-b)^2))/2;
end
end