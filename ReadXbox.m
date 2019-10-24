function [A, B, X, Y, LB, RB, T, L, R] = ReadXbox(joy)
[axes, buttons, ~] = read(joy);
A = buttons(1);
B = buttons(2);
X = buttons(3);
Y = buttons(4);
LB = buttons(5);
RB = buttons(6);
T = axes(3);
L = [axes(1), axes(2)];
R = [axes(4), axes(5)];

deadband = 0.2;
L(1) = setDeadband(L(1),deadband);
L(2) = setDeadband(L(2),deadband);
R(1) = setDeadband(R(1),deadband);
R(2) = setDeadband(R(2),deadband);
end

function axis = setDeadband(axis, deadband)
if abs(axis) < deadband
    axis = 0;
end
end