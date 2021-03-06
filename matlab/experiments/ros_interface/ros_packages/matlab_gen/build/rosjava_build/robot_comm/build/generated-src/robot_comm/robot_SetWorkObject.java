package robot_comm;

public interface robot_SetWorkObject extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetWorkObject";
  static final java.lang.String _DEFINITION = "# Service to Set the Work Object of the robot\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 q0\nfloat64 qx\nfloat64 qy\nfloat64 qz\n---\nint64 ret\nstring msg\n";
}
