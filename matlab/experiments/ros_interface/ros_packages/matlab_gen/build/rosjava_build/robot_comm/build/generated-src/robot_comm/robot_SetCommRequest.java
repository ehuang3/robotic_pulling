package robot_comm;

public interface robot_SetCommRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetCommRequest";
  static final java.lang.String _DEFINITION = "#Service to set the communication mode of the robot\n\nint64 mode  #1-Blocking; 0-Nonblocking\n";
  long getMode();
  void setMode(long value);
}
