% setenv('ROS_MASTER_URI','http://192.168.1.2:11311')
% setenv('ROS_IP','192.168.1.1') 
% % 
% rosinit('192.168.1.2')
ptree = rosparam;

% 
% pval.gains.L11 = 10;
% pval.gains.L12 = 10;
% pval.gains.L13 = 10;
% 
% pval.gains.L21 = 50;
% pval.gains.L22 = 50;
% pval.gains.L23 = 50;
% 
% pval.gains.L31 = 1;
% pval.gains.L32 = 1;
% pval.gains.L33 = 1;


pval.gains.L11 = 7.9058;
pval.gains.L12 = 7.9058;
pval.gains.L13 = 7.9058;

pval.gains.L21 = 50;
pval.gains.L22 = 50;
pval.gains.L23 = 10;

pval.gains.L31 = .1;
pval.gains.L32 = .1;
pval.gains.L33 = 0.01;

pval.ref.x = 2;
pval.ref.y = 0.0;
pval.ref.z = 60*pi/180;

% pval.gains.K11 = 0.36;
% pval.gains.K12 = 0.36;
pval.gains.K11 = 0.5;
pval.gains.K12 = 1;
pval.gains.K13 = 0.5;

pval.gains.K21 = 1;
pval.gains.K22 = 1;
pval.gains.K23 = 2;

pval.gains.k = 0.1;

set(ptree,'CSEI',pval)

% Inspect parameters by using the get() function
% get(ptree,'my_params') /CSEI/gains/L12