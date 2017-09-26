%=========================================================================
%main.m
%
%Generate VF from a surface numerically 
%--------------------------------------------------------------------------

clc
clear
close all

G = 1;
H = 1;
L = 0;
dt = 0.1;


theta = 0:0.05:2*pi;
r = .1;

x = [r*cos(theta),0];
y = [r*sin(theta),0];
z = zeros(1,length(x)-1);
z = [z,10];

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
% % x = [x',0];
% % y = [y',0];
% % z = [0,-1];
% 
% x = [x'];
% y = [y'];
% z = [0,-100];
% z = repmat(z,1,length(x)/2);
% 
% 
% x = [x,0];
% y = [y,0];
% z = [z,100];
% 
% 
xy = [x;y];

hold on
this.s1 = tpaps(xy,z,1);
% fnplt(this.s1)
% axis equal
% grid on
% view(45,25)


this.s1.dx = fnder(this.s1,[1,0]);
this.s1.dy = fnder(this.s1,[0,1]);

XY = [x+dt;y];
this.s2 = tpaps(XY,z,1);
fnplt(this.s2)


n = 55;
range = 10;
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
            
            
            Vt = G*Vconv/norm(Vconv)+H*Vcirc/norm(Vcirc)+L*Vtv/norm(Vtv);
            
        
            u(i,j) = Vt(1)/norm(Vt);
            v(i,j) = Vt(2)/norm(Vt);
            xs(i,j) = X(i);
            ys(i,j) = Y(j);
        end
        

    end
end



hold on
quiver(xs,ys,u,v,'k','linewidth',1)
% plot(x(1:end-1),y(1:end-1),'r','linewidth',2)
axis equal

Xn = xs;
Yn = ys;
un = u;
vn = v;

% save('numerical','Xn','Yn','un','vn')
e=x'*1e3;
n=y'*1e3;
save('testtrack','e','n');


