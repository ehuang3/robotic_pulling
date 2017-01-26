package mocap;

public interface mocap_SetObjTransformation extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mocap/mocap_SetObjTransformation";
  static final java.lang.String _DEFINITION = "# Service to Set Transformation of Object Local Frame w.r.t Object Rigid Body Frame in the mocap system. \n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 q0\nfloat64 qx\nfloat64 qy\nfloat64 qz\n---\nint64 ret\nstring msg\n";
}
