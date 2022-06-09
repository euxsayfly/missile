function [value,isterminal,direction] = events1(t,y)
value = (y(1) - y(9))^2 + (y(2) - y(10))^2 + (y(3) - y(11))^2;
isterminal= 1;
direction = 0;
end