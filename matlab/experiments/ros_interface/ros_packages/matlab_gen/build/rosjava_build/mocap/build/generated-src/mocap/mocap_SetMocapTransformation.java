package mocap;

public interface mocap_SetMocapTransformation extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mocap/mocap_SetMocapTransformation";
  static final java.lang.String _DEFINITION = "# Service to Set Transformation of Mocap w.r.t Robot Frame.\n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 q0\nfloat64 qx\nfloat64 qy\nfloat64 qz\n---\nint64 ret\nstring msg\n";
}
