load('/home/juan/self-propelling/code/coverage/RS_core/saved/qr_qr_safe_V_circle_Radius_2_Speed_10_SafetyTime_1.mat')
load('/home/juan/self-propelling/code/coverage/RS_core/saved/analytical_qr_qr_safe_V_circle_Radius_2_Speed_10_SafetyTime_1')
%% evaluate analytical ttr
i = 0; 
ATTR = single(zeros(size(ttr)));
for v1 = g.vs{1}'
    i = i+1;
    j = 0;
    for v2 = g.vs{2}'
        j = j+1;
        k = 0;
        for v3 = g.vs{3}'
            k = k+1;
            l = 0;
            for v4 = g.vs{4}'
                l = l+1;
                ATTR(i,j,k,l) = single(TTR([v1, v2, v3, v4]));
                disp([i,j,k,l])
            end
        end
    end
end

%%
visSetIm(g,ttr,'red',0);
visSetIm(g,ttr,'blue',1);
visSetIm(g,ATTR,'green',1);
axis square
%%
diff = ttr-ATTR;
Th = 1000;
maskAT = ATTR < Th;
maskT = ttr < Th;
mask = maskAT .* maskT;
mask_diff = mask .* diff;
e2 = norm(mask_diff(:))/nnz(mask(:));
em = max(mask_diff(:));
%%

function phi = TTR(x)
    px = x(1); vx = x(2); py = x(3); vy = x(4);
    cr = 2;
    a = vx.^2+vy.^2;
    b = 2*px.*vx+2*py.*vy;
    c = px.^2+py.^2-cr.^2;
    t1 =(-b-sqrt(b^2-4*a.*c))/(2*a);
    t2 =(-b+sqrt(b^2-4*a.*c))/(2*a);
    phi = t1.*(t1>=0) + 100*(t2<=0);
    phi(isnan(phi))=100;
    phi((imag(phi)~=0))=100;
end