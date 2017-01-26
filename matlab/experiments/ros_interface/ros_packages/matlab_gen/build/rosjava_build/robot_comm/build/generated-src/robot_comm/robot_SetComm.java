package robot_comm;

public interface robot_SetComm extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetComm";
  static final java.lang.String _DEFINITION = "#Service to set the communication mode of the robot\n\nint64 mode  #1-Blocking; 0-Nonblocking\n---\nint64 ret\nstring msg";
}
