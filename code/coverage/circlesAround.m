n = numel(a.aas);
d = 100*ones(n,n);
for i = 1:n
    for j = i+1:n
        d(i,j) = norm(a.aas{i}.x([1,3])-a.aas{j}.x([1,3]));
    end
end
r = min(d(:))/2;
t = [0:0.05:2*pi];
c = r*[cos(t)', sin(t)'];

for i=1:n
    p = a.aas{i}.x([1,3]);
    plot(c(:,1)+p(1),c(:,2)+p(2));
end