classdef robot_GetJointsResponse < robotics.ros.Message
    %robot_GetJointsResponse MATLAB implementation of robot_comm/robot_GetJointsResponse
    %   This class was automatically generated by
    %   robotics.ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2017 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'robot_comm/robot_GetJointsResponse' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '927b82d08077d046362055bd87d33b65' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Dependent)
        J1
        J2
        J3
        J4
        J5
        J6
        Ret
        Msg
    end
    
    properties (Constant, Hidden)
        PropertyList = {'J1', 'J2', 'J3', 'J4', 'J5', 'J6', 'Msg', 'Ret'} % List of non-constant message properties
        ROSPropertyList = {'j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'msg', 'ret'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = robot_GetJointsResponse(msg)
            %robot_GetJointsResponse Construct the message object robot_GetJointsResponse
            import com.mathworks.toolbox.robotics.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('robotics:ros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('robotics:ros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('robotics:ros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function j1 = get.J1(obj)
            %get.J1 Get the value for property J1
            j1 = double(obj.JavaMessage.getJ1);
        end
        
        function set.J1(obj, j1)
            %set.J1 Set the value for property J1
            validateattributes(j1, {'numeric'}, {'nonempty', 'scalar'}, 'robot_GetJointsResponse', 'J1');
            
            obj.JavaMessage.setJ1(j1);
        end
        
        function j2 = get.J2(obj)
            %get.J2 Get the value for property J2
            j2 = double(obj.JavaMessage.getJ2);
        end
        
        function set.J2(obj, j2)
            %set.J2 Set the value for property J2
            validateattributes(j2, {'numeric'}, {'nonempty', 'scalar'}, 'robot_GetJointsResponse', 'J2');
            
            obj.JavaMessage.setJ2(j2);
        end
        
        function j3 = get.J3(obj)
            %get.J3 Get the value for property J3
            j3 = double(obj.JavaMessage.getJ3);
        end
        
        function set.J3(obj, j3)
            %set.J3 Set the value for property J3
            validateattributes(j3, {'numeric'}, {'nonempty', 'scalar'}, 'robot_GetJointsResponse', 'J3');
            
            obj.JavaMessage.setJ3(j3);
        end
        
        function j4 = get.J4(obj)
            %get.J4 Get the value for property J4
            j4 = double(obj.JavaMessage.getJ4);
        end
        
        function set.J4(obj, j4)
            %set.J4 Set the value for property J4
            validateattributes(j4, {'numeric'}, {'nonempty', 'scalar'}, 'robot_GetJointsResponse', 'J4');
            
            obj.JavaMessage.setJ4(j4);
        end
        
        function j5 = get.J5(obj)
            %get.J5 Get the value for property J5
            j5 = double(obj.JavaMessage.getJ5);
        end
        
        function set.J5(obj, j5)
            %set.J5 Set the value for property J5
            validateattributes(j5, {'numeric'}, {'nonempty', 'scalar'}, 'robot_GetJointsResponse', 'J5');
            
            obj.JavaMessage.setJ5(j5);
        end
        
        function j6 = get.J6(obj)
            %get.J6 Get the value for property J6
            j6 = double(obj.JavaMessage.getJ6);
        end
        
        function set.J6(obj, j6)
            %set.J6 Set the value for property J6
            validateattributes(j6, {'numeric'}, {'nonempty', 'scalar'}, 'robot_GetJointsResponse', 'J6');
            
            obj.JavaMessage.setJ6(j6);
        end
        
        function ret = get.Ret(obj)
            %get.Ret Get the value for property Ret
            ret = int64(obj.JavaMessage.getRet);
        end
        
        function set.Ret(obj, ret)
            %set.Ret Set the value for property Ret
            validateattributes(ret, {'numeric'}, {'nonempty', 'scalar'}, 'robot_GetJointsResponse', 'Ret');
            
            obj.JavaMessage.setRet(ret);
        end
        
        function msg = get.Msg(obj)
            %get.Msg Get the value for property Msg
            msg = char(obj.JavaMessage.getMsg);
        end
        
        function set.Msg(obj, msg)
            %set.Msg Set the value for property Msg
            validateattributes(msg, {'char'}, {}, 'robot_GetJointsResponse', 'Msg');
            
            obj.JavaMessage.setMsg(msg);
        end
    end
    
    methods (Access = protected)
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@robotics.ros.Message(obj);
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.J1 = obj.J1;
            cpObj.J2 = obj.J2;
            cpObj.J3 = obj.J3;
            cpObj.J4 = obj.J4;
            cpObj.J5 = obj.J5;
            cpObj.J6 = obj.J6;
            cpObj.Ret = obj.Ret;
            cpObj.Msg = obj.Msg;
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.J1 = strObj.J1;
            obj.J2 = strObj.J2;
            obj.J3 = strObj.J3;
            obj.J4 = strObj.J4;
            obj.J5 = strObj.J5;
            obj.J6 = strObj.J6;
            obj.Ret = strObj.Ret;
            obj.Msg = strObj.Msg;
        end
    end
    
    methods (Access = ?robotics.ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.J1 = obj.J1;
            strObj.J2 = obj.J2;
            strObj.J3 = obj.J3;
            strObj.J4 = obj.J4;
            strObj.J5 = obj.J5;
            strObj.J6 = obj.J6;
            strObj.Ret = obj.Ret;
            strObj.Msg = obj.Msg;
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?robotics.ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = robotics.ros.custom.msggen.robot_comm.robot_GetJointsResponse.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = robotics.ros.custom.msggen.robot_comm.robot_GetJointsResponse;
            obj.reload(strObj);
        end
    end
end
