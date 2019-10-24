function out = IK_2D(x,y)
h_B = 146.5;
l_B = 280;
l_F = 255;

x_2 = 0;
y_2 = h_B;

x_4 = x;
y_4 = y;

delta_x = x_4 - x_2;
delta_y = y_4 - y_2;

l_H = sqrt(delta_x^2+delta_y^2);
k1 = l_B^2 + l_F^2 - (delta_x)^2 - (delta_y)^2;
k2 = 1/(2*l_B*l_F);
k3 = k1*k2;
theta_3 = acos(k3);
theta_1 = asin(l_F/l_H*sin(theta_3));

phi_1 = asin(abs(delta_x)/l_H);

if y < h_B
    theta_R = pi - phi_1 - theta_1;
else
    theta_R = phi_1 - theta_1;
end

offset_R = 0.27;

theta_L = theta_R + pi - offset_R - theta_3;

out = [theta_R*180/pi (theta_L)*180/pi rad2step(theta_R), rad2step(theta_L)];
end

function step = rad2step(theta)
step = 56*1600*theta/(2*pi);
end