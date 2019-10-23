tm=simulateMultiVehicle;
n = numel(tm.aas);
d = zeros(n,n)+100;
for i = 1:n
    for j = i+1:n
        xi = tm.aas{i}.x;
        xj = tm.aas{j}.x;
        d(i,j)= norm(xi([1,3])-xj([1,3]));
    end
end
r = min(d(:));
for i = 1:n
    xi = tm.aas{i}.x;
    t = 0:0.01:2*pi;
    plot(xi(1)+(r/2)*cos(t),xi(3)+(r/2)*sin(t));
end
