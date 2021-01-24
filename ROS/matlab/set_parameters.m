% setenv('ROS_MASTER_URI','http://192.168.1.2:11311')
% setenv('ROS_IP','192.168.1.1') 
% 
% rosinit('192.168.1.2')
ptree = rosparam;
pval.Kp = 0.3;
pval.Kpy = 0.3;
pval.Kppsi = 0.3;
pval.Kd = 0.5;
pval.ref.x = 0.2;
pval.ref.y = 0.3;
pval.ref.psi = pi/4;
set(ptree,'CSEI',pval)

% Inspect parameters by using the get() function
% get(ptree,'my_params')