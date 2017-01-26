package robot_comm;

public interface robot_AddTrajectoryPointRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_AddTrajectoryPointRequest";
  static final java.lang.String _DEFINITION = "# Service to add a trajectory point to the robot\'s buffer\n\n# Cartesian Pose\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 q0\nfloat64 qx\nfloat64 qy\nfloat64 qz\n\n# Time in seconds for the robot to execute this part of the path\nfloat64 time\n\n# The zone number, as defined in robot_comm\nint64 zone\n";
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
  double getTime();
  void setTime(double value);
  long getZone();
  void setZone(long value);
}
