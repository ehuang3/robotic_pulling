package robot_comm;

public interface robot_SetJointsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetJointsRequest";
  static final java.lang.String _DEFINITION = "# Service to Set Joints\n\nfloat64 j1\nfloat64 j2\nfloat64 j3\nfloat64 j4\nfloat64 j5\nfloat64 j6\n";
  double getJ1();
  void setJ1(double value);
  double getJ2();
  void setJ2(double value);
  double getJ3();
  void setJ3(double value);
  double getJ4();
  void setJ4(double value);
  double getJ5();
  void setJ5(double value);
  double getJ6();
  void setJ6(double value);
}
