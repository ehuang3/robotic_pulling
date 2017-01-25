package robot_comm;

public interface robot_IsMovingResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_IsMovingResponse";
  static final java.lang.String _DEFINITION = "bool moving\nint64 ret\nstring msg";
  boolean getMoving();
  void setMoving(boolean value);
  long getRet();
  void setRet(long value);
  java.lang.String getMsg();
  void setMsg(java.lang.String value);
}
