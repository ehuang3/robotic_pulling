package robot_comm;

public interface robot_SetVacuumRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetVacuumRequest";
  static final java.lang.String _DEFINITION = "#Service to set vacuum on/off\n\nint64 vacuum  #1-on; 0-off\n";
  long getVacuum();
  void setVacuum(long value);
}
