% Test Rotx function
theta_x = pi/4; % 45 degrees
R_x = Rotx(theta_x);
disp('Rotation matrix about the X-axis (Rotx):');
disp(R_x);

% Test Roty function
theta_y = pi/4; % 45 degrees
R_y = Roty(theta_y);
disp('Rotation matrix about the Y-axis (Roty):');
disp(R_y);

% Test Rotz function
theta_z = pi/4; % 45 degrees
R_z = Rotz(theta_z);
disp('Rotation matrix about the Z-axis (Rotz):');
disp(R_z);

% Test transformation function
R = [0.809, -0.294, 0.509;
     0.588,  0.405, -0.706;
     0,  0.866, 0.5];
p = [1; 2; -1];
T = transformation(R, p);
disp('Homogeneous transformation matrix (transformation):');
disp(T);

% Test invtrans function
T_inv = invtrans(T);
disp('Inverse of the homogeneous transformation matrix (invtrans):');
disp(T_inv);