%比例导引验证
clc;clear;
N=6000;  %时域N*dt
dbelta=zeros(1,N); depsilon=zeros(1,N);rx=zeros(1,N);ry=zeros(1,N);rz=zeros(1,N);R=zeros(1,N);
drx=zeros(1,N);dry=zeros(1,N);drz=zeros(1,N);dgamam=zeros(1,N);dvm=zeros(1,N);dpusaim=zeros(1,N);
pusaim=zeros(1,N);gamam=zeros(1,N);vm=zeros(1,N);xm=zeros(1,N);ym=zeros(1,N);zm=zeros(1,N);
xt=zeros(1,N);yt=zeros(1,N);zt=zeros(1,N);nmy=zeros(1,N);nmp=zeros(1,N);nx=zeros(1,N);
vxm=zeros(1,N);vym=zeros(1,N);vzm=zeros(1,N);vm=zeros(1,N);
dt=0.01; % 时间间隔

%飞机参数
xt(1)=0;  %位置
yt(1)=7000;
zt(1)=0;
st=800; % 速度
gamat=-11.98/360*pi;  %俯仰角 计算时用正值
pusait=-6.082/360*pi;  %偏航角 
vxt=st*cos(gamat)*sin(pusait);% 速度分量
vyt=st*sin(-gamat);
vzt=st*cos(gamat)*cos(pusait);
%导弹参数
xm(1)=1000;
ym(1)=6000;
zm(1)=-6000;
vm(1)=1400; %初始速度
nmy(1)=0; %初始过载
nmp(1)=0;
K=10;%比例系数
g=9.81;

m = 85; %导弹质量
T = 0 ;% 导弹推力
rho = 0.0888;%空气密度
Sm = pi * 0.06 * 0.06;
Cdm = 0.5;

for k=1:N-1
 rx(k) = xt(k) - xm(k); ry(k) = yt(k) - ym(k); rz(k) = zt(k) - zm(k);
 drx(k) = vxt - vxm(k); dry(k) = vyt - vym(k); drz(k) = vzt - vzm(k);
 R(k) = sqrt(rx(k)^2 + ry(k)^2 + rz(k)^2); 
 
 dbelta(k) = (drx(k) * rz(k) - drz(k) * rx(k)) / (rz(k)^2 + rx(k)^2);
 depsilon(k) = ((rz(k)^2 + rx(k)^2) * dry(k) - ry(k) * (drx(k) * rz(k) + drz(k) * rx(k))) / (R(k)^2 * sqrt(rx(k)^2 + rz(k)^2));
    
 nmy(k) = K * vm(k) / g * dbelta(k); %法向过载
 nmp(k) = vm(k) * K / g * depsilon(k);
 nx(k) = (T- 0.5 * rho * vm(k)^2 * Sm * Cdm)/(m*g); %气动阻力
 
 dvm(k)  =  g * (nx(k) + sin(gamam(k)));
 dpusaim(k) =  g * nmy(k) / (vm(k) * cos(gamam(k)));
 dgamam(k) =  g / vm(k) * (nmp(k) + cos(gamam(k)));
 %位置角度更新
 vm(k+1)=vm(k)+dt*dvm(k);
 vxm(k+1)=vm(k)*cos(gamam(k))*sin(pusaim(k));
 vym(k+1)=-vm(k)*sin(gamam(k));
 vzm(k+1)=vm(k)*cos(gamam(k))*cos(pusaim(k));
 xm(k+1)=xm(k)+dt*vxm(k);
 ym(k+1)=ym(k)+dt*vym(k);
 zm(k+1)=zm(k)+dt*vzm(k);
 xt(k+1)=xt(k)+dt*vxt;
 yt(k+1)=yt(k)+dt*vyt;
 zt(k+1)=zt(k)+dt*vzt;
 pusaim(k+1)=pusaim(k)+dt*dpusaim(k); 
 gamam(k+1)=gamam(k)+dt*dgamam(k); 
end

figure(1); 
plot3(xm(1,:),ym(1,:),zm(1,:),'k',xt(1,:),yt(1,:),zt(1,:)); 
% axis([0 25 0 5 0 25]); ,'k',xt(1,:),yt(1,:),zt(1,:)
% text(x(180),y(180),z(180),'\rightarrow 比例导引律制导下的导弹运动轨迹'); 
% text(ptr(1,280),ptr(2,280),ptr(3,280),'\rightarrow 目标运动轨迹'); 
grid on 