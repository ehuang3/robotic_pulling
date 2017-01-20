%% Generate custom messages from ROS workspace.
import data.*

root_path = fileparts(getDataPath);
msg_path = fullfile(root_path,'matlab','custom_msgs');
mkdire(msg_path)

% robot_msgs_path = fullfile(getenv('HOME'),'mlab_ws','src','robot');
% rosgenmsg(robot_msgs_path)

catkin_src_path = fullfile(getenv('HOME'),'mlab_ws','src');
rosgenmsg(catkin_src_path)