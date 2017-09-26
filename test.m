%=========================================================================
%test.m
%
%
%--------------------------------------------------------------------------

clc
clear
close all

topx = linspace(-1,1,5);
topy = ones(1,5);

bottomx = topx;
bottomy = -topy;

rightx = ones(1,5);
righty = linspace(-1,1,5);

leftx = -rightx;
lefty = righty;

x = [topx,bottomx,rightx,leftx,0];
y = [topy,bottomy,righty,lefty,0];


hold on
xy = [x;y];
z = [zeros(1,length(x)-1),1];

plot3(x,y,z,'r*')

s = tpaps(xy,z,1);
fnplt(s)




