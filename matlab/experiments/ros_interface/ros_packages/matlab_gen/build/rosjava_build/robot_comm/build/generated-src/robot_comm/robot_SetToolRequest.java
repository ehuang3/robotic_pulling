package robot_comm;

public interface robot_SetToolRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetToolRequest";
  static final java.lang.String _DEFINITION = "# Service to Set the Tool of the robot in cartesian coordinates\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 q0\nfloat64 qx\nfloat64 qy\nfloat64 qz\n";
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  double getZ();
  void setZ(double value);
  double getQ0();
  void setQ0(double value);
  double getQx();
  void setQx(double value);
  double getQy();
  void setQy(double value);
  double getQz();
  void setQz(double value);
}
