package mocap;

public interface mocap_GetMocapTransformation extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mocap/mocap_GetMocapTransformation";
  static final java.lang.String _DEFINITION = "# Service to Get Transformation of Mocap w.r.t Robot Frame.\n\n---\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 q0\nfloat64 qx\nfloat64 qy\nfloat64 qz\nint64 ret\nstring msg\n";
}
