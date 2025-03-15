clear;
close all;
clc;

%建立机器人模型
%           关节角   关节偏距  连杆长度   连杆转角   旋转关节   偏差
%           theta    d        a         alpha     type      offset
L(1) = Link([0       0        1         0         0          0]);
L(2) = Link([0       0        1    0         0          0]);
L(3) = Link([0       0        1       pi/2         0          0]);
L(4) = Link([0       0        1       pi/2         0          0]);

L(1).qlim = [-180, 180] * pi / 180;
L(2).qlim = [-180, 180] * pi / 180;
L(3).qlim = [-180, 180] * pi / 180;
L(4).qlim = [-180, 180] * pi / 180;

robot = SerialLink(L, 'name', 'test');
%view(3)
%robot.plot([-pi/3,pi/4,pi/2,pi/6]);
%robot.teach
a=robot.fkine([pi/3,-pi/4,-pi/2,-pi/6])*[0;0;0];

L1 = 1; % 第一段臂长
L2 = 1; % 第二段臂长
L3 = 1; % 第三段臂长
L4 = 1; % 第四段臂长

timely_position = [1.9489;0.6776;0.5];
final_position = [1.9489;0.6776;0.5];
anticipate_position = [1.9489;0.6776;0.5];
timely_speed = [0;0;0];
anticipate_speed = [0;0;0];

theta = [-pi/3,pi/4,pi/2,pi/6];

Fe = [0;0;0];

control_time = 12;
control_T = 0.1;

M = diag([0.5 0.5 0.5]);
B = diag([1 1 1]);
K = diag([20 20 20]);

kp = 1;

e = [0;0;0];
de = [0;0;0];
dde = [0;0;0];

for i =1:control_time/control_T
    if i<40
        Fe = [25;0;0];
    end
    if 40<=i
        Fe = [0;0;0];
    end
    %if 60<=i &&i<80
    %    Fe = [0;20;0];
    %end
    %if 80<=i &&i<120
    %    Fe = [0;0;0];
    %end
    e = anticipate_position-timely_position;
    anticipate_speed = kp*e;
    de = anticipate_speed - timely_speed;
    timely_speed = anticipate_speed;
    dde = inv(M)*(Fe-B*de-K*e);
    de = de + dde*control_T;
    e = e + de*control_T;
    if (i<40)
        anticipate_position = final_position+e;
    else
        anticipate_position = final_position;
    end
    timely_position = timely_position + timely_speed*control_T;
    J = [-L1*sin(theta(1)) - L2*sin(theta(1)+theta(2)) - L3*sin(theta(1)+theta(2)+theta(3)), ...
         -L2*sin(theta(1)+theta(2)) - L3*sin(theta(1)+theta(2)+theta(3)), ...
         -(L3+ L4*cos(theta(4)))*sin(theta(1)+theta(2)+theta(3)),...
         -L4*sin(theta(4))*cos(theta(1)+theta(2)+theta(3));
         L1*cos(theta(1)) + L2*cos(theta(1)+theta(2)) + L3*cos(theta(1)+theta(2)+theta(3)), ...
         L2*cos(theta(1)+theta(2)) + L3*cos(theta(1)+theta(2)+theta(3)), ...
         (L3+ L4*cos(theta(4)))*cos(theta(1)+theta(2)+theta(3)),...
         -L4*sin(theta(4))*sin(theta(1)+theta(2)+theta(3));
         0, 0, 0, L4*cos(theta(4))];
    motor_speed = pinv(J)*timely_speed;
    theta = theta+transpose(motor_speed)*control_T;
    qq(i,:) = theta;
end
robot.plot(qq)
a=robot.fkine(theta)*[0;0;0]