function r=find_r(P,K)
%P is matrix defining Lyapunov function V=x^TPx
%K is linear state feedback gain matrix
%r is the radius of the largest circle in which dot{V} is neg. def. 
%see the function mVdot below
Rmax=10;
Rmin=.1;
r1=1;
while mVdot(r1,P,K)>0 & r1>Rmin
    r1=r1/2;
end
r2=r1;
while mVdot(r2,P,K)<0 & r2<Rmax
    r2=2*r2;
end
while r2-r1>.001
    rr=(r1+r2)/2;
    if mVdot(rr,P,K)>0
        r2=rr;
    else
        r1=rr;
    end
end
r=r1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function x=mVdot(r,P,K)
% Computes the largest value of dot{V} on a circle of radius r
%P is matrix defining Lyapunov function V=x^TPx
%K is linear state feedback gain matrix
theta=linspace(0,2*pi);
x1=r*cos(theta);
x2=r*sin(theta);
alpha = 0.018;
x=-1e16;
for k=1:length(theta)
    u=-K*[x1(k);x2(k)];
    xdot=[x2(k);22.8*sin(x1(k))+(22.8/9.8)*cos(x1(k))*u-alpha*(x2(k)^2)*sign(x2(k))];
    Vdot=2*xdot'*P*[x1(k);x2(k)];
    if Vdot>x;x=Vdot;end
end



