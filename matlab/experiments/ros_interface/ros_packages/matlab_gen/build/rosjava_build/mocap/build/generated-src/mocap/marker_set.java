package mocap;

public interface marker_set extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mocap/marker_set";
  static final java.lang.String _DEFINITION = "geometry_msgs/Point[] markers";
  java.util.List<geometry_msgs.Point> getMarkers();
  void setMarkers(java.util.List<geometry_msgs.Point> value);
}
