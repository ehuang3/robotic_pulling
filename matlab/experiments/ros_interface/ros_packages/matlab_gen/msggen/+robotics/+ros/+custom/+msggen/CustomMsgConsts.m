classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2017 The MathWorks, Inc.
    
    properties (Constant)
        mocap_marker_set = 'mocap/marker_set'
        mocap_mocap_GetMocapFrame = 'mocap/mocap_GetMocapFrame'
        mocap_mocap_GetMocapFrameRequest = 'mocap/mocap_GetMocapFrameRequest'
        mocap_mocap_GetMocapFrameResponse = 'mocap/mocap_GetMocapFrameResponse'
        mocap_mocap_GetMocapTransformation = 'mocap/mocap_GetMocapTransformation'
        mocap_mocap_GetMocapTransformationRequest = 'mocap/mocap_GetMocapTransformationRequest'
        mocap_mocap_GetMocapTransformationResponse = 'mocap/mocap_GetMocapTransformationResponse'
        mocap_mocap_SetMocapTransformation = 'mocap/mocap_SetMocapTransformation'
        mocap_mocap_SetMocapTransformationRequest = 'mocap/mocap_SetMocapTransformationRequest'
        mocap_mocap_SetMocapTransformationResponse = 'mocap/mocap_SetMocapTransformationResponse'
        mocap_mocap_SetObjTransformation = 'mocap/mocap_SetObjTransformation'
        mocap_mocap_SetObjTransformationRequest = 'mocap/mocap_SetObjTransformationRequest'
        mocap_mocap_SetObjTransformationResponse = 'mocap/mocap_SetObjTransformationResponse'
        mocap_mocap_frame = 'mocap/mocap_frame'
        robot_comm_robot_AddTrajectoryPoint = 'robot_comm/robot_AddTrajectoryPoint'
        robot_comm_robot_AddTrajectoryPointRequest = 'robot_comm/robot_AddTrajectoryPointRequest'
        robot_comm_robot_AddTrajectoryPointResponse = 'robot_comm/robot_AddTrajectoryPointResponse'
        robot_comm_robot_Approach = 'robot_comm/robot_Approach'
        robot_comm_robot_ApproachRequest = 'robot_comm/robot_ApproachRequest'
        robot_comm_robot_ApproachResponse = 'robot_comm/robot_ApproachResponse'
        robot_comm_robot_CartesianLog = 'robot_comm/robot_CartesianLog'
        robot_comm_robot_ClearTrajectory = 'robot_comm/robot_ClearTrajectory'
        robot_comm_robot_ClearTrajectoryRequest = 'robot_comm/robot_ClearTrajectoryRequest'
        robot_comm_robot_ClearTrajectoryResponse = 'robot_comm/robot_ClearTrajectoryResponse'
        robot_comm_robot_ExecuteTrajectory = 'robot_comm/robot_ExecuteTrajectory'
        robot_comm_robot_ExecuteTrajectoryRequest = 'robot_comm/robot_ExecuteTrajectoryRequest'
        robot_comm_robot_ExecuteTrajectoryResponse = 'robot_comm/robot_ExecuteTrajectoryResponse'
        robot_comm_robot_ForceLog = 'robot_comm/robot_ForceLog'
        robot_comm_robot_GetCartesian = 'robot_comm/robot_GetCartesian'
        robot_comm_robot_GetCartesianRequest = 'robot_comm/robot_GetCartesianRequest'
        robot_comm_robot_GetCartesianResponse = 'robot_comm/robot_GetCartesianResponse'
        robot_comm_robot_GetFK = 'robot_comm/robot_GetFK'
        robot_comm_robot_GetFKRequest = 'robot_comm/robot_GetFKRequest'
        robot_comm_robot_GetFKResponse = 'robot_comm/robot_GetFKResponse'
        robot_comm_robot_GetIK = 'robot_comm/robot_GetIK'
        robot_comm_robot_GetIKRequest = 'robot_comm/robot_GetIKRequest'
        robot_comm_robot_GetIKResponse = 'robot_comm/robot_GetIKResponse'
        robot_comm_robot_GetJoints = 'robot_comm/robot_GetJoints'
        robot_comm_robot_GetJointsRequest = 'robot_comm/robot_GetJointsRequest'
        robot_comm_robot_GetJointsResponse = 'robot_comm/robot_GetJointsResponse'
        robot_comm_robot_GetState = 'robot_comm/robot_GetState'
        robot_comm_robot_GetStateRequest = 'robot_comm/robot_GetStateRequest'
        robot_comm_robot_GetStateResponse = 'robot_comm/robot_GetStateResponse'
        robot_comm_robot_IsMoving = 'robot_comm/robot_IsMoving'
        robot_comm_robot_IsMovingRequest = 'robot_comm/robot_IsMovingRequest'
        robot_comm_robot_IsMovingResponse = 'robot_comm/robot_IsMovingResponse'
        robot_comm_robot_JointsLog = 'robot_comm/robot_JointsLog'
        robot_comm_robot_Ping = 'robot_comm/robot_Ping'
        robot_comm_robot_PingRequest = 'robot_comm/robot_PingRequest'
        robot_comm_robot_PingResponse = 'robot_comm/robot_PingResponse'
        robot_comm_robot_SetCartesian = 'robot_comm/robot_SetCartesian'
        robot_comm_robot_SetCartesianJ = 'robot_comm/robot_SetCartesianJ'
        robot_comm_robot_SetCartesianJRequest = 'robot_comm/robot_SetCartesianJRequest'
        robot_comm_robot_SetCartesianJResponse = 'robot_comm/robot_SetCartesianJResponse'
        robot_comm_robot_SetCartesianRequest = 'robot_comm/robot_SetCartesianRequest'
        robot_comm_robot_SetCartesianResponse = 'robot_comm/robot_SetCartesianResponse'
        robot_comm_robot_SetComm = 'robot_comm/robot_SetComm'
        robot_comm_robot_SetCommRequest = 'robot_comm/robot_SetCommRequest'
        robot_comm_robot_SetCommResponse = 'robot_comm/robot_SetCommResponse'
        robot_comm_robot_SetDefaults = 'robot_comm/robot_SetDefaults'
        robot_comm_robot_SetDefaultsRequest = 'robot_comm/robot_SetDefaultsRequest'
        robot_comm_robot_SetDefaultsResponse = 'robot_comm/robot_SetDefaultsResponse'
        robot_comm_robot_SetJoints = 'robot_comm/robot_SetJoints'
        robot_comm_robot_SetJointsRequest = 'robot_comm/robot_SetJointsRequest'
        robot_comm_robot_SetJointsResponse = 'robot_comm/robot_SetJointsResponse'
        robot_comm_robot_SetSpeed = 'robot_comm/robot_SetSpeed'
        robot_comm_robot_SetSpeedRequest = 'robot_comm/robot_SetSpeedRequest'
        robot_comm_robot_SetSpeedResponse = 'robot_comm/robot_SetSpeedResponse'
        robot_comm_robot_SetTool = 'robot_comm/robot_SetTool'
        robot_comm_robot_SetToolRequest = 'robot_comm/robot_SetToolRequest'
        robot_comm_robot_SetToolResponse = 'robot_comm/robot_SetToolResponse'
        robot_comm_robot_SetTrackDist = 'robot_comm/robot_SetTrackDist'
        robot_comm_robot_SetTrackDistRequest = 'robot_comm/robot_SetTrackDistRequest'
        robot_comm_robot_SetTrackDistResponse = 'robot_comm/robot_SetTrackDistResponse'
        robot_comm_robot_SetVacuum = 'robot_comm/robot_SetVacuum'
        robot_comm_robot_SetVacuumRequest = 'robot_comm/robot_SetVacuumRequest'
        robot_comm_robot_SetVacuumResponse = 'robot_comm/robot_SetVacuumResponse'
        robot_comm_robot_SetWorkObject = 'robot_comm/robot_SetWorkObject'
        robot_comm_robot_SetWorkObjectRequest = 'robot_comm/robot_SetWorkObjectRequest'
        robot_comm_robot_SetWorkObjectResponse = 'robot_comm/robot_SetWorkObjectResponse'
        robot_comm_robot_SetZone = 'robot_comm/robot_SetZone'
        robot_comm_robot_SetZoneRequest = 'robot_comm/robot_SetZoneRequest'
        robot_comm_robot_SetZoneResponse = 'robot_comm/robot_SetZoneResponse'
        robot_comm_robot_SpecialCommand = 'robot_comm/robot_SpecialCommand'
        robot_comm_robot_SpecialCommandRequest = 'robot_comm/robot_SpecialCommandRequest'
        robot_comm_robot_SpecialCommandResponse = 'robot_comm/robot_SpecialCommandResponse'
        robot_comm_robot_Stop = 'robot_comm/robot_Stop'
        robot_comm_robot_StopRequest = 'robot_comm/robot_StopRequest'
        robot_comm_robot_StopResponse = 'robot_comm/robot_StopResponse'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(89, 1);
                msgList{1} = 'mocap/marker_set';
                msgList{2} = 'mocap/mocap_GetMocapFrame';
                msgList{3} = 'mocap/mocap_GetMocapFrameRequest';
                msgList{4} = 'mocap/mocap_GetMocapFrameResponse';
                msgList{5} = 'mocap/mocap_GetMocapTransformation';
                msgList{6} = 'mocap/mocap_GetMocapTransformationRequest';
                msgList{7} = 'mocap/mocap_GetMocapTransformationResponse';
                msgList{8} = 'mocap/mocap_SetMocapTransformation';
                msgList{9} = 'mocap/mocap_SetMocapTransformationRequest';
                msgList{10} = 'mocap/mocap_SetMocapTransformationResponse';
                msgList{11} = 'mocap/mocap_SetObjTransformation';
                msgList{12} = 'mocap/mocap_SetObjTransformationRequest';
                msgList{13} = 'mocap/mocap_SetObjTransformationResponse';
                msgList{14} = 'mocap/mocap_frame';
                msgList{15} = 'robot_comm/robot_AddTrajectoryPoint';
                msgList{16} = 'robot_comm/robot_AddTrajectoryPointRequest';
                msgList{17} = 'robot_comm/robot_AddTrajectoryPointResponse';
                msgList{18} = 'robot_comm/robot_Approach';
                msgList{19} = 'robot_comm/robot_ApproachRequest';
                msgList{20} = 'robot_comm/robot_ApproachResponse';
                msgList{21} = 'robot_comm/robot_CartesianLog';
                msgList{22} = 'robot_comm/robot_ClearTrajectory';
                msgList{23} = 'robot_comm/robot_ClearTrajectoryRequest';
                msgList{24} = 'robot_comm/robot_ClearTrajectoryResponse';
                msgList{25} = 'robot_comm/robot_ExecuteTrajectory';
                msgList{26} = 'robot_comm/robot_ExecuteTrajectoryRequest';
                msgList{27} = 'robot_comm/robot_ExecuteTrajectoryResponse';
                msgList{28} = 'robot_comm/robot_ForceLog';
                msgList{29} = 'robot_comm/robot_GetCartesian';
                msgList{30} = 'robot_comm/robot_GetCartesianRequest';
                msgList{31} = 'robot_comm/robot_GetCartesianResponse';
                msgList{32} = 'robot_comm/robot_GetFK';
                msgList{33} = 'robot_comm/robot_GetFKRequest';
                msgList{34} = 'robot_comm/robot_GetFKResponse';
                msgList{35} = 'robot_comm/robot_GetIK';
                msgList{36} = 'robot_comm/robot_GetIKRequest';
                msgList{37} = 'robot_comm/robot_GetIKResponse';
                msgList{38} = 'robot_comm/robot_GetJoints';
                msgList{39} = 'robot_comm/robot_GetJointsRequest';
                msgList{40} = 'robot_comm/robot_GetJointsResponse';
                msgList{41} = 'robot_comm/robot_GetState';
                msgList{42} = 'robot_comm/robot_GetStateRequest';
                msgList{43} = 'robot_comm/robot_GetStateResponse';
                msgList{44} = 'robot_comm/robot_IsMoving';
                msgList{45} = 'robot_comm/robot_IsMovingRequest';
                msgList{46} = 'robot_comm/robot_IsMovingResponse';
                msgList{47} = 'robot_comm/robot_JointsLog';
                msgList{48} = 'robot_comm/robot_Ping';
                msgList{49} = 'robot_comm/robot_PingRequest';
                msgList{50} = 'robot_comm/robot_PingResponse';
                msgList{51} = 'robot_comm/robot_SetCartesian';
                msgList{52} = 'robot_comm/robot_SetCartesianJ';
                msgList{53} = 'robot_comm/robot_SetCartesianJRequest';
                msgList{54} = 'robot_comm/robot_SetCartesianJResponse';
                msgList{55} = 'robot_comm/robot_SetCartesianRequest';
                msgList{56} = 'robot_comm/robot_SetCartesianResponse';
                msgList{57} = 'robot_comm/robot_SetComm';
                msgList{58} = 'robot_comm/robot_SetCommRequest';
                msgList{59} = 'robot_comm/robot_SetCommResponse';
                msgList{60} = 'robot_comm/robot_SetDefaults';
                msgList{61} = 'robot_comm/robot_SetDefaultsRequest';
                msgList{62} = 'robot_comm/robot_SetDefaultsResponse';
                msgList{63} = 'robot_comm/robot_SetJoints';
                msgList{64} = 'robot_comm/robot_SetJointsRequest';
                msgList{65} = 'robot_comm/robot_SetJointsResponse';
                msgList{66} = 'robot_comm/robot_SetSpeed';
                msgList{67} = 'robot_comm/robot_SetSpeedRequest';
                msgList{68} = 'robot_comm/robot_SetSpeedResponse';
                msgList{69} = 'robot_comm/robot_SetTool';
                msgList{70} = 'robot_comm/robot_SetToolRequest';
                msgList{71} = 'robot_comm/robot_SetToolResponse';
                msgList{72} = 'robot_comm/robot_SetTrackDist';
                msgList{73} = 'robot_comm/robot_SetTrackDistRequest';
                msgList{74} = 'robot_comm/robot_SetTrackDistResponse';
                msgList{75} = 'robot_comm/robot_SetVacuum';
                msgList{76} = 'robot_comm/robot_SetVacuumRequest';
                msgList{77} = 'robot_comm/robot_SetVacuumResponse';
                msgList{78} = 'robot_comm/robot_SetWorkObject';
                msgList{79} = 'robot_comm/robot_SetWorkObjectRequest';
                msgList{80} = 'robot_comm/robot_SetWorkObjectResponse';
                msgList{81} = 'robot_comm/robot_SetZone';
                msgList{82} = 'robot_comm/robot_SetZoneRequest';
                msgList{83} = 'robot_comm/robot_SetZoneResponse';
                msgList{84} = 'robot_comm/robot_SpecialCommand';
                msgList{85} = 'robot_comm/robot_SpecialCommandRequest';
                msgList{86} = 'robot_comm/robot_SpecialCommandResponse';
                msgList{87} = 'robot_comm/robot_Stop';
                msgList{88} = 'robot_comm/robot_StopRequest';
                msgList{89} = 'robot_comm/robot_StopResponse';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(28, 1);
                svcList{1} = 'mocap/mocap_GetMocapFrame';
                svcList{2} = 'mocap/mocap_GetMocapTransformation';
                svcList{3} = 'mocap/mocap_SetMocapTransformation';
                svcList{4} = 'mocap/mocap_SetObjTransformation';
                svcList{5} = 'robot_comm/robot_AddTrajectoryPoint';
                svcList{6} = 'robot_comm/robot_Approach';
                svcList{7} = 'robot_comm/robot_ClearTrajectory';
                svcList{8} = 'robot_comm/robot_ExecuteTrajectory';
                svcList{9} = 'robot_comm/robot_GetCartesian';
                svcList{10} = 'robot_comm/robot_GetFK';
                svcList{11} = 'robot_comm/robot_GetIK';
                svcList{12} = 'robot_comm/robot_GetJoints';
                svcList{13} = 'robot_comm/robot_GetState';
                svcList{14} = 'robot_comm/robot_IsMoving';
                svcList{15} = 'robot_comm/robot_Ping';
                svcList{16} = 'robot_comm/robot_SetCartesian';
                svcList{17} = 'robot_comm/robot_SetCartesianJ';
                svcList{18} = 'robot_comm/robot_SetComm';
                svcList{19} = 'robot_comm/robot_SetDefaults';
                svcList{20} = 'robot_comm/robot_SetJoints';
                svcList{21} = 'robot_comm/robot_SetSpeed';
                svcList{22} = 'robot_comm/robot_SetTool';
                svcList{23} = 'robot_comm/robot_SetTrackDist';
                svcList{24} = 'robot_comm/robot_SetVacuum';
                svcList{25} = 'robot_comm/robot_SetWorkObject';
                svcList{26} = 'robot_comm/robot_SetZone';
                svcList{27} = 'robot_comm/robot_SpecialCommand';
                svcList{28} = 'robot_comm/robot_Stop';
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
    end
end
