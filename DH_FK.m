function [fk, f_R03, f_R36] = forward_kinematics(theta, d, a, alpha)
% This function create forward kinematics matrix using standard DH paramter matrices

% T = trotz(theta)*transl(0,0,d)*transl(a,0,0)*trotx(alpha);

T_1 = trotz(theta(1)) * transl(0,0,d(1)) * transl(a(1),0,0) * trotx(alpha(1));

T_2 = trotz(theta(2)) * transl(0,0,d(2)) * transl(a(2),0,0) * trotx(alpha(2));

T_3 = trotz(theta(3)) * transl(0,0,d(3)) * transl(a(3),0,0) * trotx(alpha(3));

T_4 = trotz(theta(4)) * transl(0,0,d(4)) * transl(a(4),0,0) * trotx(alpha(4));

T_5 = trotz(theta(5)) * transl(0,0,d(5)) * transl(a(5),0,0) * trotx(alpha(5));

T_6 = trotz(theta(6)) * transl(0,0,d(6)) * transl(a(6),0,0) * trotx(alpha(6));


% Foward matrix of the arm
T03 = simplify(T_1 * T_2 * T_3);
R03 = T03(1:3,1:3);
f_R03 = matlabFunction(R03,'Vars', {theta});

% Foward matrix of the wrist
T36 = simplify(T_4 * T_5 * T_6);
R36 = T36(1:3,1:3);
f_R36 = matlabFunction(R36, 'Vars', {theta});

% Total foward matrix
H = simplify(T03 * T36);

% Create function for forward kinematics
fk = matlabFunction(H,'Vars', {theta});

end

