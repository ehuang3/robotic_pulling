package robot_comm;

public interface robot_SetTrackDist extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SetTrackDist";
  static final java.lang.String _DEFINITION = "# Service to Set the tracking distance of the robot while in non-blocking mode\n\nfloat64 pos_dist  # mm\nfloat64 ang_dist  # deg\n---\nint64 ret\nstring msg\n";
}
