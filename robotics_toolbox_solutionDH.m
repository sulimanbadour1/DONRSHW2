clear; clc;

% create KUKA KR10 R1100-2 model 
  mdl_kr10_r1100_2

% plot arm ready pose
  kr10_r1100_2.plot(qt)
  
% Solve forward kinematics using fkine() function
  T = kr10_r1100_2.fkine(qt)
  print(T)
  
% Solve inverse kinematics using ikine() function
  joint_variables = kr10_r1100_2.ikine(T)

  kr10_r1100_2.plot(joint_variables, 'jaxes')