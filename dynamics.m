function dy = dynamics(~,y)
% y(1) = xm; y(2) = ym; y(3) = zm;
% y(4) = V; y(5) = psi; y(6) = gamma; V为速度，psi为弹道偏角，gamma为弹道倾角
% y(7) = belta; y(8) = epsilon;
% y(9) = xt; y(10) = yt; y(11) = zt;

% nmy = K * (V * cos(gammma)) / g * (dbeta + tan(epsilon) * tan(belta + ...
% epsilon) * depsilon);
% nmp = V * K * depsilon / (g * cos(epsilon + belta));    K为比例导引系数

% L为升力，D为阻力，T为发动机推力，m为质量，alpha为迎角
% D = 0.5 * rho * vm^2 * Sm * Cdm 
% rho为当地空气密度，vm为速度，Sm为导弹参考截面积，Cdm为导弹阻力系数
       

m = 85;
g = 9.81;% 重力加速度
T = 0 ;% 发动机推力
rho = 0.0888;%https://wenku.baidu.com/view/4ac49351a66e58fafab069dc5022aaea998f41da.html
vm = y(4);
Sm = pi * 0.06 * 0.06;
Cdm = 0.5;
D = 0.5 * rho * vm^2 * Sm * Cdm;%阻力

K = 3;%导引系数

dxm = y(4) * cos(y(6)) * cos(y(5));
dym = y(4) * cos(y(6)) * sin(y(5));
dzm = y(4) * sin(y(6));%导弹运动学方程

dxt = 300; dyt = 400; dzt = 300;%目标运动学方程

belta = y(7); epsilon = y(8);%视线偏角及视线倾角

rx = y(9) - y(1); ry = y(10) - y(2); rz = y(11) - y(3);%相对矢量
R = sqrt(rx^2 + ry^2 + rz^2);%相对距离

drx = dxt - dxm; dry = dyt - dym; drz = dzt - dzm;%相对运动学方程

dbelta = (dry * rx - drx * ry) / (rx^2 + ry^2);
depsilon = ((rx^2 + ry^2) * drz - rz * (drx * rx + dry * ry)) / (R^2 * sqrt(rx^2 + ry^2));%视线偏角及视线倾角变化率

nmy = K * (y(4) * cos(y(6))) / g * (dbelta + tan(epsilon) * tan(belta + epsilon) * depsilon); 
nmp = y(4) * K * depsilon / (g * cos(epsilon + belta)); 
nx = (T - D) / (m * g);%控制过载

dvm  =  g * (nx - sin(y(6)));
dpsi =  g * nmy / (y(4) * cos(y(6)));
dgamma =  g / y(4) * (nmp - cos(y(6)));%导弹动力学方程


dy = [ dxm;
       dym;
       dzm;
       dvm;
       dpsi;
       dgamma;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
       dbelta;
       depsilon;
       dxt;
       dyt;
       dzt;];
end