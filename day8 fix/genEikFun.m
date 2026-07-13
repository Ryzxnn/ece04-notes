function f = genEikFun(N, unsafe, eps, sig)

f = ones(N, N);

for i = 1:N
    for j = 1:N
        f(i, j) = ( ...
            1+(1+eps*(i+j)) ...
            * exp(-unsafeDist(N, unsafe, j, i)/sig) ...
        );
    end
end

end

function d = unsafeDist(N, unsafe, x, y)
ds = [];
for i = 1:N
    for j = 1:N
        if unsafe(i, j) == 0
            continue
        end
        ds(end+1) = norm([x-j, y-i]);
    end
end
d = min(ds);
end

