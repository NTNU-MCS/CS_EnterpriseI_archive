% setenv('ROS_MASTER_URI','http://192.168.1.2:11311')
% setenv('ROS_IP','192.168.1.1') 
% 
% rosinit('192.168.1.2')
ptree = rosparam;
pval.Kpy = 0.3;
pval.Kppsi = 0.3;
pval.Kd = 0.5;
pval.ref.x = 0.2;
pval.ref.y = 0.3;
pval.ref.psi = pi/4;

pval.gains.L11 = 10;
pval.gains.L12 = 10;
pval.gains.L13 = 10;

pval.gains.L21 = 100;
pval.gains.L22 = 100;
pval.gains.L23 = 100;

pval.gains.L31 = 10;
pval.gains.L32 = 10;
pval.gains.L33 = 10;


set(ptree,'CSEI',pval)

% Inspect parameters by using the get() function
% get(ptree,'my_params') /CSEI/gains/L12