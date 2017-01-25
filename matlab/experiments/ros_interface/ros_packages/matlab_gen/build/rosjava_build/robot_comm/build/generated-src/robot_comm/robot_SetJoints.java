package robot_comm;

public interface robot_SetJoints extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetJoints";
  static final java.lang.String _DEFINITION = "# Service to Set Joints\n\nfloat64 j1\nfloat64 j2\nfloat64 j3\nfloat64 j4\nfloat64 j5\nfloat64 j6\n---\nint64 ret\nstring msg";
}
