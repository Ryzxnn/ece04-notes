function u_new = localEikonalUpdate(a, b, h1, h2, f)
%LOCALEIKONALUPDATE Upwind update for |grad U| = f on a rectangular grid.
%
% a is the best theta1-neighbor value and b is the best theta2-neighbor
% value. h1 and h2 are the corresponding grid spacings. f is the local
% right-hand side value.

if nargin < 4
    h2 = h1;
end
if nargin < 5 || isempty(f)
    f = 1;
end

if f < 0
    error('The Eikonal right-hand side f must be nonnegative.');
end

u_new = Inf;

if isfinite(a)
    u_new = min(u_new, a + h1*f);
end
if isfinite(b)
    u_new = min(u_new, b + h2*f);
end

if ~(isfinite(a) && isfinite(b))
    return;
end

A = 1/h1^2 + 1/h2^2;
B = -2*(a/h1^2 + b/h2^2);
C = a^2/h1^2 + b^2/h2^2 - f^2;
disc = max(B^2 - 4*A*C, 0);
u_two_sided = (-B + sqrt(disc))/(2*A);

if u_two_sided >= max(a, b)
    u_new = min(u_new, u_two_sided);
end
end
