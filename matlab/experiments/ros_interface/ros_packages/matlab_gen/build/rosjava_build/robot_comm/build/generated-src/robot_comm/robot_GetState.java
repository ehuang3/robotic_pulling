package robot_comm;

public interface robot_GetState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_GetState";
  static final java.lang.String _DEFINITION = "# Service to get the current state of the robot.\n\n---\nfloat64 tcp  # mm/s\nfloat64 ori  # deg/s\nint64 zone\nint64 vacuum\nfloat64 workx\nfloat64 worky\nfloat64 workz\nfloat64 workq0\nfloat64 workqx\nfloat64 workqy\nfloat64 workqz\nfloat64 toolx\nfloat64 tooly\nfloat64 toolz\nfloat64 toolq0\nfloat64 toolqx\nfloat64 toolqy\nfloat64 toolqz\nint64 ret\nstring msg\n";
}
