package mocap;

public interface mocap_frame extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mocap/mocap_frame";
  static final java.lang.String _DEFINITION = "Header header\nint64 number\ngeometry_msgs/PoseArray body_poses\n# Unidenfied markers. (E.g., the single marker used for calibration).\t\nmocap/marker_set uid_markers\n# The set of identified markers.\nmocap/marker_set[] id_marker_sets\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  long getNumber();
  void setNumber(long value);
  geometry_msgs.PoseArray getBodyPoses();
  void setBodyPoses(geometry_msgs.PoseArray value);
  mocap.marker_set getUidMarkers();
  void setUidMarkers(mocap.marker_set value);
  java.util.List<mocap.marker_set> getIdMarkerSets();
  void setIdMarkerSets(java.util.List<mocap.marker_set> value);
}
