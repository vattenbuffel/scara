clc; clear all; close all;

x = 3/sqrt(2)
y = -3/sqrt(2)
z = 0

L1 = 2;
L2 = 1;

% Inverse kinematics
theta2 = pi - acos((-x^2 - y^2 + L1^2 + L2^2) / (2*L1*L2)) % Ska det vara  +-?? Ja för att få båda lösningarna

s2 = sin(theta2);
c2 = cos(theta2);
% theta1 = atan2(L2*s2*x + (L1+L2*c2)*y, (L1 + L2*c2)*x - L2*s2*y) % Taget  från pdf
theta1 = atan2(-L2*s2*x + (L1+L2*c2)*y, -(L1 + L2*c2)*x + L2*s2*y);
a = L1+L2*c2;
b = L2*s2;
s1 = (y*a-b*x)/(a*b^2+a^2)
c1 = (x*a+b*y)/(a*b^2+a^2)
theta1 = atan2(s1, c1)


% Forward kinematics
x_fk = cos(theta1)*L1 + cos(theta1 + theta2)*L2 % PDF thinks it should be theta1-theta2, because of his theta2 going the wrong way
y_fk = sin(theta1)*L1 + sin(theta1 + theta2)*L2

