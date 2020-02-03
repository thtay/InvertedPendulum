function  plot_ellipse(P,c)
% PLOT_ELLIPSE  Plots the ellipse defined by x^T P x = c
%   where P is a symmetric, positive definite matrix   
n = 100;   % number of points
[u,e]=eig(P);
phi=angle(u(1,1)+j*u(2,1));
a=sqrt(c/e(1,1));
b=sqrt(c/e(2,2));
th = linspace(0,2*pi,n+1);
x = a*cos(th);
y = b*sin(th);
c = cos(phi);
s = sin(phi);
th = x*c-y*s;
y = x*s+y*c;
x = th;
plot(x,y);
%xlim([-5 5]);
%ylim([-5 5]);


