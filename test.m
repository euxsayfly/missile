clc;clear ;close;
xm0 = 0; ym0 = 0 ; zm0 = 20000;
V0 = 1000; psi0 = 0; gamma0 = 0;

xt0 = 2000; yt0 = 2000; zt0 = 16000;

rx0 = xt0 - xm0; ry0 = yt0 - ym0; rz0 = zt0 - zm0;

belta0 = atan(ry0 / rx0); epsilon0 = atan(rz0 / sqrt(rx0^2 + ry0^2));
%[t,y] = ode45('dynamics',[0,20],[xm0,ym0,zm0,V0,psi0,gamma0,belta0,epsilon0,xt0,yt0,zt0]);
[t,y] = ode15s('dynamics',[0,30],[xm0,ym0,zm0,V0,psi0,gamma0,belta0,epsilon0,xt0,yt0,zt0]);

k = length(t);
xc = y(k,1); yc = y(k,2); zc = y(k,3);



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

figure(2)
plot(t,y(:,5) * 180 / pi)
hold on
plot(t,y(:,6) * 180 / pi)

xlabel('$t/s$','FontName','Time New Roman','Fontsize',14,'Interpreter','latex')
ylabel('$\gamma,\psi$','FontName','Time New Roman','Fontsize',14,'Interpreter','latex')
legend('$\gamma$','$\psi$','FontName','Time New Roman','Fontsize',14,'Interpreter','latex')

axis([0,12,-180,180])