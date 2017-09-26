%-------------------------------------------------------------------------
%compare.m
%
%
%
%-------------------------------------------------------------------------

clc
clear
close all

load('analytical.mat')
load('numerical.mat')
theta = 0:0.05:2*pi;
r = 5;
xs = r*cos(theta);
ys = r*sin(theta);

hold on
quiver(Xa,Ya,ua,va,'r','linewidth',3)
quiver(Xn,Yn,un,vn,'k')
plot(xs,ys,'b','linewidth',2);
xlabel 'x'
ylabel 'y'
legend({'Analytical','Numerical'})
axis equal
grid on