%% create inverse kinematics function
% d - link length
% Coordinates
x = 0.8575;
y = 0;
z = 0.3859;
O = [x; y; z];
% RPY/xyz
roll = deg2rad(90);
pitch = deg2rad(90);
yaw = deg2rad(90);
R = rotx(roll) * roty(pitch) * rotz(yaw)

O_c = O - d(6)*R*[0; 0; 1]

x_c = O_c(1);
y_c = O_c(2);
z_c = O_c(3);

q1_front = atan2(y_c, x_c);
q1_behind = pi + atan2(y_c, x_c);

% front case
r = sqrt(x_c^2 + y_c^2) - a(1);
s = z_c - d(1);
D = (r^2 + s^2 - d(4)^2 - a(3)^2 - a(2)^2) / (2 * a(2) * sqrt(d(4)^2 + a(3)^2)); % cos(phi)
gamma = atan2(d(4), a(3));

q3_up = atan2(sqrt(1 - D^2), D) - gamma;
q3_down = atan2(-sqrt(1 - D^2), D) - gamma;


B = sqrt(1 - D^2) * sqrt((d(4)^2 + a(3)^2 ) / (r^2 + s^2)); % sin(alpha) - can be positive (q3_up) or negative (q3_down)
beta = atan2(s, r);

q2_up = -(atan2(B, sqrt(1 - B^2)) + beta);
q2_down = -(atan2(-B, sqrt(1 - B^2)) + beta);


q4 = atan2(R(2,3), R(1,3));
q5 = atan2(R(2,3), -R(3,3) * sin(q4));
q6 = atan2( -R(3,2), R(3,1));



q_fu = [q1_front, q2_up, q3_up q4 q5 q6]
q_fd = [q1_front, q2_down, q3_down q4 q5 q6]