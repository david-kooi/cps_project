function [X, Y] = drawretangle(x)
%DRAWRETANGLE Summary of this function goes here
%   Detailed explanation goes here
alpha = 30/180*pi;
beta = 30/180*pi;
a = 2;
b = 2;
x_ret(1) = a*cos(x(3)-alpha) + x(1);
x_ret(2) = a*cos(x(3)+alpha) + x(1);
x_ret(3) = -b*cos(x(3)-beta) + x(1);
x_ret(4) = -b*cos(x(3)+beta) + x(1);
y_ret(1) = b*sin(x(3)-alpha) + x(2);
y_ret(2) = b*sin(x(3)+alpha) + x(2);
y_ret(3) = -b*sin(x(3)-beta) + x(2);
y_ret(4) = -b*sin(x(3)+beta) + x(2);
X = [x_ret x_ret(1)]';
Y = [y_ret y_ret(1)]';

end

