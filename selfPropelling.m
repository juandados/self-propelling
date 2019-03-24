function [] = selfPropelling(inputStruct, type)
if ~isstruct(inputStruct)
    error('Input must be struct type');
end
% Defining Default Parameters
dt = 0.1;
T = 100;
path_flag = false;

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
    f = @f_polygon_1;
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
    f = @f_polygon_1;
end

if type == "regular_polygon"
    n = 9;
    % Reading InputStruct:
    names = fieldnames(inputStruct);
    n_s = 3;
    for i=1:length(names)
        eval([names{i} '=inputStruct.' names{i} ';' ]);
    end
    % Defining Initial Values
    X = 0.001*rand(n,2)+2*ones(n,2);
    V = 0.001*rand(n,2);
    % Defining Regular Polygon
    t = [-2*pi/n_s:2*pi/n_s:2*pi-4*pi/n_s]';
    neg_leaders = [sin(t),cos(t)];
    f = @f_polygon_1;
end

% Time evolution
for t = 0:dt:T
    path = path_flag*(t>10)*(t<30)*[0.1*(t-10),-0.1*(t-10)]; % choose n = 20
    reposition = repmat(path,size(neg_leaders,1),1);
    neg_leaders_t = neg_leaders + reposition;
    [X,V] = runge_kutta_evolution(X, V, dt, f, neg_leaders_t);
    if (apply(@(v)max(abs(v)), V(:))<1e-4) & false
        disp('closed due to slow spead');
        break
    end
    figure(1);
    plot(X(:,1),X(:,2),'b.');
    hold on;
    plot(neg_leaders_t([1:end,1],1), neg_leaders_t([1:end,1],2),'r:');
    title(['t',num2str(t)])
    %quiver(X(:,1),X(:,2),V(:,1),V(:,2),0);
    axis([-5,5,-5,5]);
    axis square;
    hold off;
end

end

function dY = f_polygon_2(Y, vertices)
    % fH depending on the distance from every individual to single point in
    % the polygon interior
    p = 8;
    interior_point = [0,0];
    d = size(Y,2); n = size(Y,1)/2; l=size(vertices,1);
    area = polyarea(vertices(:,1),vertices(:,2));
    rd = 2*sqrt(area/n);
    fI = @(r)2*(r-rd).*(r-rd<0);
    fh = @(r)(5*r).*(r>0);
    vd = 0; a = 1.5;
    X = Y(1:n,:);
    V = Y(n+1:2*n,:);
    dX = V;
    dV2 = zeros(size(V));
    for i = 1:n
        R = ones(n,1)*X(i,:) - X;
        r = apply(@(v)norm(v,p), R);
        DUsI = (fI(r)./r)*ones(1,d).*R;
        DUsI(i,:) = zeros(1,d);
        h = norm(X(i,:)-interior_point,2);
        mask = inpolygon(X(i,1),X(i,2),vertices(:,1), vertices(:,2));
        DUsH = (1-mask).*repmat(fh(h),1,d).*(X(i,:)-interior_point);
        hold on;
%        quiver(-DUsH(:,1)',-DUsH(:,2)',0);
%        quiver(repmat(X(i,1),3,1), repmat(X(i,2),3,1), -H(:,1),-H(:,2),0);
        hold off;
        dV2(i,:) = sum(DUsI,1) + sum(DUsH,1);
    end
    v = apply(@(v)norm(v,p), V);
    dV1 = -a*((v-vd)./v)*ones(1,d).*V;
    dV = dV1 - dV2;
    dY = [dX; dV];
end

function dY = f_polygon_1(Y, vertices)
    % fH depending on the distance from every individual to the polygon
    p = 8;
    d = size(Y,2); n = size(Y,1)/2; l=size(vertices,1);
    area = polyarea(vertices(:,1),vertices(:,2));
    rd = 2*sqrt(area/n);
    fI = @(r)2*(r-rd).*(r-rd<0);
    fh = @(r)(10*r+2*l).*(r>0);
    vd = 0; a = 1.5;
    X = Y(1:n,:);
    V = Y(n+1:2*n,:);
    dX = V;
    dV2 = zeros(size(V));
    for i = 1:n
        R = ones(n,1)*X(i,:) - X;
        r = apply(@(v)norm(v,p), R);
        DUsI = (fI(r)./r)*ones(1,d).*R;
        DUsI(i,:) = zeros(1,d);
        [h, x_poly,y_poly] = poly_dist(X(i,1),X(i,2),vertices(:,1),vertices(:,2));
        DUsH = repmat(fh(h),1,d).*(X(i,:)-[x_poly, y_poly]);
        hold on;
%        quiver(-DUsH(:,1)',-DUsH(:,2)',0);
%        quiver(repmat(X(i,1),3,1), repmat(X(i,2),3,1), -H(:,1),-H(:,2),0);
        hold off;
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