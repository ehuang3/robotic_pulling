package robot_comm;

public interface robot_ExecuteTrajectoryResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_ExecuteTrajectoryResponse";
  static final java.lang.String _DEFINITION = "int64 ret\nstring msg";
  long getRet();
  void setRet(long value);
  java.lang.String getMsg();
  void setMsg(java.lang.String value);
}
