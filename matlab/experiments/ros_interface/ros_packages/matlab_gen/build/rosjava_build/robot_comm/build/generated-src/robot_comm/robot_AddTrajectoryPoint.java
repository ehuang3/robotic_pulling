package robot_comm;

public interface robot_AddTrajectoryPoint extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_AddTrajectoryPoint";
  static final java.lang.String _DEFINITION = "# Service to add a trajectory point to the robot\'s buffer\n\n# Cartesian Pose\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 q0\nfloat64 qx\nfloat64 qy\nfloat64 qz\n\n# Time in seconds for the robot to execute this part of the path\nfloat64 time\n\n# The zone number, as defined in robot_comm\nint64 zone\n---\nint64 ret\nstring msg\n";
}
