%MDL_KR10_R1100_2 Create model of KUKA KR10 R1100-2 manipulator

clear L

% all parameters are in SI units: m, radians, kg, kg.m2, N.m, N.m.s etc.
% d - link length (Dennavit-Hartenberg notation)
% a - link offset (Dennavit-Hartenberg notation)
% alpha - link twist (Dennavit-Hartenberg notation)
% qlim - minimum and maximum joint angle

L(1) = Revolute('d', 0.4, 'a', 0.025, 'alpha', deg2rad(-90), 'qlim', deg2rad([-170 170])); 

L(2) = Revolute('d', 0, 'a', 0.56, 'alpha', 0, 'qlim', deg2rad([-100 135]));

L(3) = Revolute('d', 0, 'a', 0.025, 'alpha', deg2rad(-90), 'qlim', deg2rad([-210 46]));

L(4) = Revolute('d', 0.515, 'a', 0, 'alpha', deg2rad(90), 'qlim', deg2rad([-185 185]));

L(5) = Revolute('d', 0, 'a', 0, 'alpha', deg2rad(-90), 'qlim', deg2rad([-120 120]));

L(6) = Revolute('d', 0.09, 'a', 0, 'alpha', 0, 'qlim', deg2rad([-350 350]));


% some useful poses
qz = [0 0 0 0 0 0]; % zero angles
qr = [0 deg2rad(-90) 0 0 0 0]; % ready pose, arm up
qh = [0 0 deg2rad(-90) 0 0 0]; % horizontal pose
qt = [0 pi/4 pi 0 pi/4  0]; % ???


kr10_r1100_2 = SerialLink(L, 'name', 'KR10 R1100-2', ...
    'configs', {'qz', qz, 'qr', qr, 'qs', qh, 'qt', qt}, ...
    'manufacturer', 'KUKA');

clear L
