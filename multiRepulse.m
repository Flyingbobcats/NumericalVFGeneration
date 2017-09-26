%=========================================================================
%multiRepulse.m
%
%Generate repulsive VF for two circles and plot resulting guidance
%--------------------------------------------------------------------------

clc
clear
close all

G1 = -1;
H = 0;
L = 0;
dt = 0.1;

G2 = -1;


%Field 1
theta = 0:0.05:2*pi;
r = .1;
x1 = [r*cos(theta),0]+5;
y1 = [r*sin(theta),0];
z1 = zeros(1,length(x1)-1);
z1 = [z1,10];

%Field 2
x2 = [r*cos(theta),0]-5;
y2 = [r*sin(theta),0];
z2 = zeros(1,length(x2)-1);
z2 = [z2,10];

%Ellipse
% a=5; % horizontal radius
% b=10; % vertical radius
% x0=0; % x0,y0 ellipse centre coordinates
% y0=0;
% t=-pi:0.01:pi;
% x=[x0+a*cos(t),0];
% y=[y0+b*sin(t),0];
% z = [zeros(1,length(x)-1),1];

% m = 2;
% topx = linspace(-1,1,m);
% topy = ones(1,m);
% 
% bottomx = topx;
% bottomy = -topy;
% 
% rightx = ones(1,m);
% righty = linspace(-1,1,m);
% 
% leftx = -rightx;
% lefty = righty;
% 
% x = [topx,bottomx,rightx,leftx,0];
% y = [topy,bottomy,righty,lefty,0];
% xy = [x;y];
% z = [zeros(1,length(x)-1),1];

%Get Points
% fig = figure;
% axis([-10,10,-10,10]);
% grid on
% [x,y] = getpts(fig);
% 
% x = [x',0];
% y = [y',0];
% z = [zeros(1,length(x)-1),1];


xy1 = [x1;y1];
xy2 = [x2;y2];

this.s1 = tpaps(xy1,z1,1);
this.s3 = tpaps(xy2,z2,1);

% hold on
% fnplt(this.s1)
% fnplt(this.s3)



this.s1.dx = fnder(this.s1,[1,0]);
this.s1.dy = fnder(this.s1,[0,1]);

this.s3.dx = fnder(this.s3,[1,0]);
this.s3.dy = fnder(this.s3,[0,1]);

XY = [x1+dt;y1];
this.s2 = tpaps(XY,z1,1);

n = 35;
range = 21/2;
X = linspace(-range,range,n);
Y = linspace(-range,range,n);

for i = 1:length(X)
    for j = 1:length(Y)
        if X(i)==0 && Y(j) == 0
            u(i,j) = NaN;
            v(i,j) = NaN;
            xs(i,j) = NaN;
            ys(i,j) = NaN;
            
        else
            %Repulsive Field 1
            dx = fnval(this.s1.dx,[X(i),Y(j)]');
            dy = fnval(this.s1.dy,[X(i),Y(j)]');
            dz = 0;
            crs = cross([dx;dy;0],[0;0;1]);
            Vconv = -fnval(this.s1,[X(i),Y(j)]')*[dx;dy;dz];
            Vcirc = -crs;
            a1 = diff([fnval(this.s2,[X(i),Y(j)]'),fnval(this.s1,[X(i),Y(j)]')])/dt;
            a2 = 0;
            a3 = 0;
            a = [a1;a2;a3];
            M = [dx',dy',0;0,0,1;crs'];
            Minv = inv(M);
            Vtv = Minv*a;
         
             Vt1 = G1*Vconv/norm(Vconv)+H*Vcirc/norm(Vcirc);
            
%             Vt1 = G*Vconv+H*Vcirc;
            
            %Repulsive Field 2
            dx2 = fnval(this.s3.dx,[X(i),Y(j)]');
            dy2 = fnval(this.s3.dy,[X(i),Y(j)]');
            dz2 = 0;
            crs = cross([dx2;dy2;0],[0;0;1]);
            Vconv = -fnval(this.s3,[X(i),Y(j)]')*[dx2;dy2;dz2];
            Vcirc = -crs;
            a1 = diff([fnval(this.s3,[X(i),Y(j)]'),fnval(this.s3,[X(i),Y(j)]')])/dt;
            a2 = 0;
            a3 = 0;
            a = [a1;a2;a3];
            M = [dx2',dy2',0;0,0,1;crs'];
            Minv = inv(M);
            Vtv = Minv*a;
            
             Vt2 = G2*Vconv/norm(Vconv)+H*Vcirc/norm(Vcirc);
%             Vt2 = G*Vconv+H*Vcirc;
            
            Vt = Vt1+Vt2;
           
            
        
            u(i,j) = Vt(1)/norm(Vt);
            v(i,j) = Vt(2)/norm(Vt);
            u(i,j) = Vt(1);
            v(i,j) = Vt(2);
            xs(i,j) = X(i);
            ys(i,j) = Y(j);
        end
        

    end
end



hold on
quiver(xs,ys,u,v,'k','linewidth',1)
plot(x1(1:end-1),y1(1:end-1),'r','linewidth',2)
plot(x2(1:end-1),y2(1:end-1),'r','linewidth',2)
axis equal

Xn = xs;
Yn = ys;
un = u;
vn = v;

save('numerical','Xn','Yn','un','vn')