%% 

rosmsg('show','robot_comm/robot_GetCartesianRequest')
rosmsg('show','robot_comm/robot_GetCartesianResponse')

%%
msgs = rosmsg('list');

k = strfind(msgs,'robot_comm','ForceCellOutput',false);
I = find(~cellfun(@isempty,k));
robot_msgs = msgs(I);

k = strfind(msgs,'mocap','ForceCellOutput',false);
I = find(~cellfun(@isempty,k));
mocap_msgs = msgs(I);

mocap_frame_msg = rosmessage('mocap/mocap_frame')
rosmessage('robot_comm/robot_GetCartesianResponse')

rosmessage('robot_comm/robot_SetCartesianRequest')


