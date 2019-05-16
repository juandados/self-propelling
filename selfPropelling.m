function [X] = selfPropelling(inputStruct, type)
% Examples:
% selfPropelling(struct("n",30),"polygon")

if ~isstruct(inputStruct)
    error('Input must be struct type');
end
% Defining Default Parameters
dt = 0.1;
T = 100;

if type == "L"
    T = 20;
    n = 16;
    % Reading InputStruct:
    names = fieldnames(inputStruct);
    neg_leaders = @(t)[0,0;0,1;0.5,1;0.5,0.5;1,0.5;1,0];
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

if type == "polygon"
    T = 20;
    n = 16;
    % Reading InputStruct:
    names = fieldnames(inputStruct);
    neg_leaders = @(t)[0,0;0.5,1;0.5,0.5;1,0.5];
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

if type == "rectangle"
    T = 24;
    n = 16;
    % Reading InputStruct:
    names = fieldnames(inputStruct);
    neg_leaders = @(t)[0,0;0,1;4,1;4,0]/4;
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

if type == "surveying"
    T = 30;
    n = 16;
    % Reading InputStruct:
    names = fieldnames(inputStruct);
    unit_square = [0,0;0,1;1,1;1,0]/100;
    tl = 15;
    neg_leaders = @(t) (t<tl)*100*unit_square + (t>=tl)*20*unit_square ;
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

if type == "moving"
    T = 45;
    n = 16;
    tl1 = 15;
    tl2 = 30;
    % Reading InputStruct:
    names = fieldnames(inputStruct);
    unit_triangle = [0,0;0.5,sqrt(3)/2;1,0]/100;
    neg_leaders = @(t) 40*unit_triangle + (t>=tl1)*(t<=tl2)*[t-tl1,t-tl1]/20 ;
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
    s = [-2*pi/n_s:2*pi/n_s:2*pi-4*pi/n_s]';
    mask = 0.5*repmat(2+(-1).^([1:size(s)]'),1,2);
    neg_leaders = @(t) 0.5*mask.*[sin(s),cos(s)];
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
    T = 50;
    n = 9;
    % Reading InputStruct:
    names = fieldnames(inputStruct);
    n_s = 3;
    for i=1:length(names)
        eval([names{i} '=inputStruct.' names{i} ';' ]);
    end
    % Defining Initial Values
    X = 0.001*rand(n,2)+0*ones(n,2);
    V = 0.001*rand(n,2);
    % Defining Regular Polygon
    s = [-2*pi/n_s:2*pi/n_s:2*pi-4*pi/n_s]';
    neg_leaders = @(t) 0.5*[sin(s),cos(s)];
    f = @f_polygon;
end

% Figure
figure(1), set(gcf, 'Color', 'white');
axis tight;

% Create AVI object
nFrames = 20;
vidObj = VideoWriter([type,'.avi']);
vidObj.Quality = 100;
vidObj.FrameRate = 10;
open(vidObj);

% Time evolution, Create movie
for t = 0:dt:T
    neg_leaders_t = neg_leaders(t);
    [X,V] = runge_kutta_evolution(X, V, dt, f, neg_leaders_t);
    if (apply(@(v)max(abs(v)), V(:))<1e-4) & false
        disp('closed due to slow spead');
        break
    end    
    plot(X(:,1),X(:,2),'b.');
    hold on;
    %quiver(X(:,1),X(:,2),V(:,1),V(:,2),0);
    %voronoi(X(:,1),X(:,2))
    %triplot(delaunayTriangulation(X),'-g');
    plot_voronoi_centroids(X);
    plot(neg_leaders_t([1:end,1],1), neg_leaders_t([1:end,1],2),'r:');
    title(['Time: ',num2str(t)])
    
    axis([-1,2,-1,2]);
    axis square;
    hold off;
    writeVideo(vidObj, getframe(gca));
end
%close(gcf);

% Save as AVI file.
close(vidObj);

end

function dY = f_polygon(Y, vertices)
    % fH depending on the distance from every individual to the polygon
    p = 2;
    d = size(Y,2); n = size(Y,1)/2; l=size(vertices,1);
    area = polyarea(vertices(:,1),vertices(:,2));
    rd = 1*sqrt(area/n);
    fI = @(r)10*(r-rd).*(r-rd<0);
    fh = @(h)10*(h+rd/2).*(h+rd/2>0);
    %fh = @(h)80*(h).*(h>0); %borders
    pw = rd;
    vd = 0; a = 1.5; a = 0.5;
    X = Y(1:n,:);
    V = Y(n+1:2*n,:);
    dX = V;
    dV2 = zeros(size(V));
    for i = 1:n
        R = ones(n,1)*X(i,:) - X;
        r = apply(@(v)norm(v,p), R);
        DUsI = (fI(r)./r)*ones(1,d).*R;
        DUsI(i,:) = zeros(1,d);
        DUsI = DUsI;
        [h, x_poly,y_poly] = poly_dist(X(i,1),X(i,2),vertices(:,1),vertices(:,2));
        H = X(i,:)-[x_poly, y_poly];
        DUsH = repmat(fh(h),1,d).*H/h;
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

function [] = plot_voronoi_centroids(X)
    try
        dt = delaunayTriangulation(X(:,1),X(:,2));
        [V,C] = voronoiDiagram(dt);
        centroids=[];
        for i=1:size(C,1)
            try
                polygon = polyshape(V(C{i},1),V(C{i},2));
                [cx,cy] = centroid(polygon);
                centroids(i,:)=[cx,cy];
            catch
            end
        end
        voronoi(X(:,1),X(:,2));
        hold on;
        %plot(V(:,1),V(:,2), '*');
        plot(centroids(:,1),centroids(:,2),'o');
    catch
    end
end
