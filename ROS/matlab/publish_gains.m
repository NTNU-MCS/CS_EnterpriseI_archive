%% Run this block ONCE

rosinit('192.168.1.2')

[pub_obs,obs_msg] = rospublisher('CSEI/observer/gains','std_msgs/Float64MultiArray');
[pub_ctrl,ctrl_msg] = rospublisher('CSEI/controller/gains','std_msgs/Float64MultiArray');



%% Run this block every time you want to publish new data for the controller and/or observer gains.


obs_msg.Data =[1.0 2.0 3.0]';
ctrl_msg.Data = [2.0 4.0 13.37 420.0]';


send(pub_ctrl, ctrl_msg);
send(pub_obs,obs_msg);
