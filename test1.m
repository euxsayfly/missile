clc;clear all;close;
xm0 = 0; ym0 = 0 ; zm0 = 20000;
V0 = 1400; psim0 = 0; gammam0 = 0;

xt0 = 2000; yt0 = 3000; zt0 = 15000;
vt0 = 500; psit0 = 0; gammat0 = 0.5;

rx0 = xt0 - xm0; ry0 = yt0 - ym0; rz0 = zt0 - zm0;

belta0 = atan(ry0 / rx0); epsilon0 = atan(rz0 / sqrt(rx0^2 + ry0^2));
%[t,y] = ode45('dynamics',[0,20],[xm0,ym0,zm0,V0,psi0,gamma0,belta0,epsilon0,xt0,yt0,zt0]);
[t,y] = ode15s('dynamics1',[0,50],[xm0,ym0,zm0,V0,psim0,gammam0,belta0,epsilon0,xt0,yt0,zt0,vt0,psit0,gammat0]);

k = length(t);
for i = 1:k
    if (y(i,1) - y(i,9))^2 + (y(i,2) - y(i,10))^2 + (y(i,3) - y(i,11))^2 < 100
        break
    else
        i = i+1 ;
    end
end

y = y(1:i-1,:);

xc = y(i-1,1); yc = y(i-1,2); zc = y(i-1,3);



figure(1)

plot3(y(:,1),y(:,2),y(:,3))
hold on
plot3(y(:,9),y(:,10),y(:,11))
text(xm0,ym0,zm0,['(',num2str(xm0),',',num2str(ym0),',',num2str(zm0),')'],'color','b');
text(xt0,yt0,zt0,['(',num2str(xt0),',',num2str(yt0),',',num2str(zt0),')'],'color','r');
text(xc,yc,zc,['(',num2str(round(xc)),',',num2str(round(yc)),',',num2str(round(zc)),')']);

grid on
xlabel('x轴/m')
ylabel('y轴/m')
zlabel('z轴/m')
legend('导弹轨迹','飞机轨迹')
%axis([0,10000,0,10000,0,30000])
