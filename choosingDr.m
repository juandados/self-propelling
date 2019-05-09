A = (3^0.5)/4;
nm = 15;
nm = [6:40]
rd = (4/(3^0.25))*(A.^0.5./((8*nm+1).^0.5-3))
rde = (A^0.5./(nm.^0.5-1))
plot(nm, [rd;rde])
legend('rd3','rd4')
%% square
d = [];
j = 1;
K = 0.85:0.0125:1.15;
rng(10)
for k= K
    X = selfPropelling(struct('n',16,'n_s',4),'regular_polygon', k);
    n = size(X,1);
    for i= 1:n
        ds = repmat(X(i,:),n,1)-X;
        ds(i,:) = [1000,1000];
        min_ds(i) = min(sqrt(ds(:,1).^2+ds(:,2).^2));
    end
    max_d(j) = max(min_ds);
    j = j + 1;
end
plot(K,max_d);
title('square case');
xlabel('rd k');
ylabel('max min');
%% pentagon
d = [];
j = 1;
K = 0.85:0.0125:1.15;
rng(10)
for k= K
    X = selfPropelling(struct('n',16,'n_s',5),'regular_polygon', k);
    n = size(X,1);
    for i= 1:n
        ds = repmat(X(i,:),n,1)-X;
        ds(i,:) = [1000,1000];
        min_ds(i) = min(sqrt(ds(:,1).^2+ds(:,2).^2));
    end
    max_d(j) = max(min_ds);
    j = j + 1;
end
plot(K,max_d);
title('pentagon case');
xlabel('rd k');
ylabel('max min');
%% triangle
d = [];
j = 1;
K = 0.85:0.0125:1.15;
rng(10)
for k= K
    X = selfPropelling(struct('n',15,'n_s',3),'regular_polygon', k);
    n = size(X,1);
    for i= 1:n
        ds = repmat(X(i,:),n,1)-X;
        ds(i,:) = [1000,1000];
        min_ds(i) = min(sqrt(ds(:,1).^2+ds(:,2).^2));
    end
    max_d(j) = max(min_ds);
    j = j + 1;
end
plot(K,max_d);
title('triangle case');
xlabel('rd k');
ylabel('max min');
%% rectangle
d = [];
j = 1;
K = 0.85:0.0125:1.15;
rng(10)
for k= K
    X = selfPropelling(struct('n',24,'n_s',3),'rectangle', k);
    n = size(X,1);
    for i= 1:n
        ds = repmat(X(i,:),n,1)-X;
        ds(i,:) = [1000,1000];
        min_ds(i) = min(sqrt(ds(:,1).^2+ds(:,2).^2));
    end
    max_d(j) = max(min_ds);
    j = j + 1;
end
plot(K,max_d);
title('rectangle case');
xlabel('rd k');
ylabel('max min');