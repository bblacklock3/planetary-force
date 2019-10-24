function TeleopControl
s = serialport("COM4", 9600);
xbox = InitXbox;
[A, B, X, Y, LB, RB, T, L, R] = ReadXbox(xbox);

w_min = 0;
w_max = 100;
TeleopState = 'a';
IKState = 'b';
GoToState = 'c';
state = TeleopState;
w_pos = 50;
g_pos = 50;
rec_joints = '0';
joints = '0';

w_pos_1 = 50;
g_pos_1 = 50;
w_pos_2 = 50;
g_pos_2 = 50;

switch (state)
    case TeleopState
        while ~A
            [A, B, X, Y, LB, RB, T, L, R] = ReadXbox(xbox);
            w_1 = round(map(abs(L(2)),0.2,1,w_min,w_max));
            w_2 = round(map(abs(R(2)),0.2,1,w_min,w_max));
            w_3 = round(map(abs(L(1)),0.2,1,w_min,w_max));
            if L(2)~=0
                dir_1 = num2str(L(2)/abs(L(2)));
            else
                w_1 = w_min;
                dir_1 = '0';
            end
            if R(2)~=0
                dir_2 = num2str(R(2)/abs(R(2)));
            else
                w_2 = w_min;
                dir_2 = '0';
            end
            if L(1)~=0
                dir_3 = num2str(L(1)/abs(L(1)));
            else
                w_3 = w_min;
                dir_3 = '0';
            end
            
            w_1 = num2str(round(w_1));
            w_2 = num2str(round(w_2));
            w_3 = num2str(round(w_3));
            
            if length(w_1) < length(num2str(w_max))
                while length(w_1) < length(num2str(w_max))
                    w_1 = ['0' w_1];
                end
            end
            
            if length(w_2) < length(num2str(w_max))
                while length(w_2) < length(num2str(w_max))
                    w_2 = ['0' w_2];
                end
            end
            
            if length(w_3) < length(num2str(w_max))
                while length(w_3) < length(num2str(w_max))
                    w_3 = ['0' w_3];
                end
            end
            
            if (abs(T)-0.1)>0
                w_pos = w_pos + 5*T;
            end
            
            delta_g_pos = R(1);
            
            if delta_g_pos~=0
                g_pos = g_pos + 10*delta_g_pos;
            end
            
            if w_pos < 0
                w_pos = 0;
            elseif w_pos > 100
                w_pos = 100;
            end
            
            if g_pos < 0
                g_pos = 0;
            elseif g_pos > 100
                g_pos = 100;
            end
            
            if LB
                if X
                    rec_joints = '1';
                    w_pos_1 = w_pos;
                elseif Y
                    rec_joints = '2';
                    w_pos_2 = w_pos;
                else
                    rec_joints = '0';
                end
                data = ['<000,0,000,0,000,0,' num2str(round(w_pos)) ',' num2str(round(g_pos)) ',' rec_joints ',' joints '>']
            elseif RB
                if X
                    joints = '1';
                    w_pos = w_pos_1;
                elseif Y
                    joints = '2';
                    w_pos = w_pos_2;
                else
                    joints = '0';
                end
                data = ['<000,0,000,0,000,0,' num2str(round(w_pos)) ',' num2str(round(g_pos)) ',' rec_joints ',' joints '>']
            else 
                rec_joints = '0';
                joints = '0';
                data = ['<' w_1 ',' dir_1 ',' w_2 ',' dir_2 ',' w_3 ',' dir_3 ',' num2str(round(w_pos)) ',' num2str(round(g_pos)) ',' rec_joints ',' joints '>']
            end
            writeline(s,data);
            pause(0.05);
        end
        
    case IKState
        while ~A
            [A, B, X, Y, LB, RB, T, L, R] = ReadXbox(xbox);
            
            target_1 = 0;
            target_2 = 0;
            
            if LB
                [target_1, target_2] = IK_2D(430, 355);
            elseif  RB
                [target_1, target_2] = IK_2D(200, 355);
            elseif Y
                [target_1, target_2] = IK_2D(485, 60);
            elseif X
                [target_1, target_2] = IK_2D(200, 80);
            elseif B
                [target_1, target_2] = IK_2D(200, 355);
            end
            
            data = ['<' num2str(target_1) ',' num2str(target_2) '>']
            writeline(s,data);
            pause(0.05);
        end
        
    case GoToState
        while ~A
            [A, B, X, Y, LB, RB, T, L, R] = ReadXbox(xbox);
            data = [num2str(200),num2str(300)]
            writeline(s,data);
            pause(0.05);
        end
end


end

function [target_step_1, target_step_2] = IK_2D(x,y)
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

deg_1 = theta_R*180/pi;
deg_2 = (theta_L)*180/pi;

target_step_1 = rad2step(theta_R);
target_step_2 = rad2step(theta_L);
end

function step = rad2step(theta)
step = round(56*1600*theta/(2*pi));
end

function out = IK_3D(x, y, z, theta_G)
h_B = 150;
l_B = 240;
l_F = 200;
l_G = 100;

x_2 = 0;
y_2 = h_B;

x_4 = x - l_G * cos(theta_G);
y_4 = y + l_G * sin(theta_G);

delta_x = x_4 - x_2;
delta_y = y_4 - y_2;

l_H = sqrt(delta_x^2+delta_y^2);
k1 = l_B^2 + l_F^2 - (delta_x)^2 - (delta_y)^2;
k2 = 1/(2*l_B*l_F);
k3 = k1*k2;
theta_3 = acos(k3);
theta_1 = asin(l_F/l_H*sin(theta_3));
beta = asin(delta_x/l_H);

theta_R = beta - theta_1;
theta_L = theta_R + theta_3;
theta_W = pi/2 + theta_G + beta - theta_1 - theta_3;
theta_Y = tan(z/x);
out = [theta_Y, theta_R, theta_L, theta_W];
end

function out = map(val, vmin, vmax, outmin, outmax)
out = (val-vmin)*(outmax-outmin)/(vmax-vmin)+outmin;
end
