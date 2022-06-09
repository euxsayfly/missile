function dy =  missile_dynamics(~,y)


dxm = v * cos(theta) * cos(psi);
dym = v * sin(theta);
dzm = -v * cos(theta) * sin(psi);% 导弹三自由度运动学方程
dv = P / m * cos(alpha) - X / m - g * sin(theta);
dtheta = P / (m * v) * sin(alpha) * cos(gamma) + Y / (m * v) - g / v * cos(theta);
dpsi = -P / (m * v * cos(theta)) * sin(alpha) * sin(gamma) - Y / (m * v * cos(theta)) * sin(gamma);% 导弹三自由度动力学方程

xr = xt - xm; 
yr = yt - ym;
zr = zt - zm;
dxr = dxt - dxm;
dyr = dyt - dym;
dzr = dzt - dzm;% 导弹与目标相对运动方程

dq_gamma = ((xr^2 + zr^2) * dyr - yr * (xr * dxr + zr * dzr)) / ((xr^2 + yr^2 + zr^2) * sqrt(xr^2 + zr^2));
dq_lambda = (zr * dxr - xr * dzr) / (xr^2 + zr^2);% 惯性视线角速率

Nyc = K1 * dq_gamma * v / g + cos(theta);
Nzc = K2 * dq_lambda * v / g;% 导弹在两个方向上的需用过载

gamma = atan(Nzc / Nyc);
2





end