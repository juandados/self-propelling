function [] = selfPropelling(inputStruct, type)
if ~isstruct(inputStruct)
    error('Input must be struct type');
end
% Defining Default Parameters
dt = 0.1;
T = 100;

if type == "pull_in_square"
    % type specific variables
    n = 9;
    nw = 5;
    % Reading InputStruct:
    names = fieldnames(inputStruct);
    for i=1:length(names)
        eval([names{i} '=inputStruct.' names{i} ';' ]);
    end
    % Defining Initial Values
    X = 0.001*rand(n,2)+2*ones(n,2);
    V = 0.001*rand(n,2);
    % Defining Boundary
    x = linspace(0,1,nw)'; y = linspace(0,1,nw)';
    neg_leaders = [zeros(nw,1),y;...
        x(2:end-1),ones(nw-2,1);...
        ones(nw,1),y(end:-1:1);...
        x(end-1:-1:2),zeros(nw-2,1)];
    f = @f_pull_in_square;
end

if type == "polygon"
    n = 9;
    % Reading InputStruct:
    names = fieldnames(inputStruct);
    neg_leaders = [-1,0;-1,1;0,0.4;1.5,0];
    % Defining Boundary
    for i=1:length(names)
        eval([names{i} '=inputStruct.' names{i} ';' ]);
    end
    % Defining Initial Values
    X = 0.001*rand(n,2)+2*ones(n,2);
    V = 0.001*rand(n,2);
    % Defining Regular Polygon
    f = @f_polygon;
end

if type == "star"
    n = 9;
    % Reading InputStruct:
    names = fieldnames(inputStruct);
    neg_leaders = [-1,0;-1,1;0,1.5;1.5,0];
    n_s = 5;
    n_s = 2*n_s;
    t = [-2*pi/n_s:2*pi/n_s:2*pi-4*pi/n_s]';
    mask = 0.5*repmat(2+(-1).^([1:size(t)]'),1,2);
    neg_leaders = mask.*[sin(t),cos(t)];
    % Defining Boundary
    for i=1:length(names)
        eval([names{i} '=inputStruct.' names{i} ';' ]);
    end
    % Defining Initial Values
    X = 0.001*rand(n,2)+2*ones(n,2);
    V = 0.001*rand(n,2);
    % Defining Regular Polygon
    f = @f_polygon;
end

if type == "regular_polygon"
    n = 9;
    % Reading InputStruct:
    names = fieldnames(inputStruct);
    n_s = 3;
    % Defining Boundary
    for i=1:length(names)
        eval([names{i} '=inputStruct.' names{i} ';' ]);
    end
    % Defining Initial Values
    X = 0.001*rand(n,2)+2*ones(n,2);
    V = 0.001*rand(n,2);
    % Defining Regular Polygon
    t = [-2*pi/n_s:2*pi/n_s:2*pi-4*pi/n_s]';
    neg_leaders = [sin(t),cos(t)];
    f = @f_polygon;
end


if type == "square"
    % type specific variables
    n = 9;
    nw = 5;
    % Reading InputStruct:
    names = fieldnames(inputStruct);
    for i=1:length(names)
        eval([names{i} '=inputStruct.' names{i} ';' ]);
    end
    % Defining Initial Values
    X = 0.1*rand(n,2)+0.5*ones(n,2);
    V = 0.001*rand(n,2);
    % Defining Boundary
    x = linspace(0,1,nw)'; y = linspace(0,1,nw)';
    neg_leaders = [zeros(nw,1),y;...
        x(2:end-1),ones(nw-2,1);...
        ones(nw,1),y(end:-1:1);...
        x(end-1:-1:2),zeros(nw-2,1)];
    f = @f_square;
end

if type == "gravity"
    n = 2;
    % Reading InputStruct:
    names = fieldnames(inputStruct);
    neg_leaders = [0,0];
    % Defining Boundary
    for i=1:length(names)
        eval([names{i} '=inputStruct.' names{i} ';' ]);
    end
    % Defining Initial Values
    X = [0.5, 0.5; 0.4, 0.4];
    X = [-0.5, 0.5];
    %V = 0.1*[X(:,2), -X(:,1)]*2/sqrt(2);
    V = 0.1*[X(:,2), -X(:,1)]*2/sqrt(2);
    % Defining Regular Polygon
    f = @f_gravity;
end

% Time evolution
for t = 0:dt:T
    [X,V] = runge_kutta_evolution(X, V, dt, f, neg_leaders);
    figure(1);
    plot(X(:,1),X(:,2),'b.');
    hold on;
    plot(neg_leaders([1:end,1],1), neg_leaders([1:end,1],2),'r-*');
    %quiver(X(:,1),X(:,2),V(:,1),V(:,2));
    axis([-2,2,-2,2]);
    axis square;
    hold off;
end

end

function dY = f_gravity(Y, leaders)
    p = 1;
    rd = 1/1.5;
    fI = @(r)0*0.00016./(r.^2);
    %fh = @(r)0.01/(r.^2); % elliptic orbit;
    fh = @(r)0.00707/(r.^2); % circular orbit;
    vd = 0; a = 0;
    d = size(Y,2); n = size(Y,1)/2; l=size(leaders,1);
    X = Y(1:n,:);
    V = Y(n+1:2*n,:);
    dX = V;
    dV2 = zeros(size(V));
    for i = 1:n
        R = ones(n,1)*X(i,:) - X;
        r = apply(@(v)norm(v,p), R);
        DUsI = (fI(r)./r)*ones(1,d).*R;
        DUsI(i,:) = zeros(1,d);
        H = ones(l,1)*X(i,:) - leaders;
        h = apply(@(v)norm(v,p), H);
        DUsH = (fh(h)./h)*ones(1,d).*H;
        dV2(i,:) = sum(DUsI,1) + sum(DUsH,1);
    end
    v = apply(@(v)norm(v,p), V);
    dV1 = -a*((v-vd)./v)*ones(1,d).*V;
    dV = dV1 - dV2;
    dY = [dX; dV];
end

function dY = f_polygon(Y, vertices)
    p = 8;
    d = size(Y,2); n = size(Y,1)/2; l=size(vertices,1);
    area = polyarea(vertices(:,1),vertices(:,2));
    rd = 2*sqrt(area/n);
    fI = @(r)2*(r-rd).*(r-rd<0);
    fh = @(r)(10*r-2).*(r<0);
    vd = 0; a = 1.5;
    X = Y(1:n,:);
    V = Y(n+1:2*n,:);
    shape_vecs = circshift(vertices,-1) - vertices;
    normal_vecs = [shape_vecs(:,2), -shape_vecs(:,1)];
    normal_vecs = normal_vecs./apply(@(v)norm(v,2), normal_vecs);
    dX = V;
    dV2 = zeros(size(V));
    for i = 1:n
        R = ones(n,1)*X(i,:) - X;
        r = apply(@(v)norm(v,p), R);
        DUsI = (fI(r)./r)*ones(1,d).*R;
        DUsI(i,:) = zeros(1,d);
        H = repmat(X(i,:),l,1) - vertices;
        h = sum(H.*normal_vecs,2);
        mask = inpolygon(X(i,1),X(i,2),vertices(:,1), vertices(:,2));
        DUsH = (1-mask)*repmat(fh(h),1,d).*normal_vecs;
        hold on;
        %quiver(repmat(X(i,1),3,1), repmat(X(i,2),3,1), DUsH(:,1),DUsH(:,2),0);
        %quiver(repmat(X(i,1),3,1), repmat(X(i,2),3,1), -H(:,1),-H(:,2),0);
        hold off;
        dV2(i,:) = sum(DUsI,1) + sum(DUsH,1);
    end
    v = apply(@(v)norm(v,p), V);
    dV1 = -a*((v-vd)./v)*ones(1,d).*V;
    dV = dV1 - dV2;
    dY = [dX; dV];
end


function dY = f_pull_in_square(Y, leaders)
    p = 10;
    rd = 1/1.5;
    fI = @(r)2*(r-rd);
    fh = @(r)2*(r-rd);
    vd = 0; a = 1.5;
    d = size(Y,2); n = size(Y,1)/2; l=size(leaders,1);
    X = Y(1:n,:);
    V = Y(n+1:2*n,:);
    dX = V;
    dV2 = zeros(size(V));
    for i = 1:n
        R = ones(n,1)*X(i,:) - X;
        r = apply(@(v)norm(v,p), R);
        DUsI = (fI(r)./r)*ones(1,d).*R;
        DUsI(i,:) = zeros(1,d);
        H = ones(l,1)*X(i,:) - leaders;
        h = apply(@norm, H);
        DUsH = (fh(h)./h)*ones(1,d).*H;
        dV2(i,:) = sum(DUsI,1) + sum(DUsH,1);
    end
    v = apply(@(v)norm(v,p), V);
    dV1 = -a*((v-vd)./v)*ones(1,d).*V;
    dV = dV1 - dV2;
    dY = [dX; dV];
end

function dY = f_square(Y, leaders)
    d = size(Y,2); n = size(Y,1)/2; l=size(leaders,1);
    p = 20; rd = 1/(sqrt(n)+1);
    vd = 0; a = 1;
    fI = @(r)(0.3*(r-rd).*(r<rd));
    fh = @(r)(0.3*(r-rd).*(r<rd));
    X = Y(1:n,:);
    V = Y(n+1:2*n,:);
    dX = V;
    dV2 = zeros(size(V));
    for i = 1:n
        R = ones(n,1)*X(i,:) - X;
        r = apply(@(v)norm(v,p), R);
        DUsI = (fI(r)./r)*ones(1,d).*R;
        DUsI(i,:) = zeros(1,d);
        H = ones(l,1)*X(i,:) - leaders;
        h = apply(@norm, H);
        DUsH = (fh(h)./h)*ones(1,d).*H;
        dV2(i,:) = sum(DUsI,1) + sum(DUsH,1);
    end
    v = apply(@(v)norm(v,p), V);
    dV1 = -a*((v-vd)./v)*ones(1,d).*V;
    dV = dV1 - dV2;
    dY = [dX; dV];
end