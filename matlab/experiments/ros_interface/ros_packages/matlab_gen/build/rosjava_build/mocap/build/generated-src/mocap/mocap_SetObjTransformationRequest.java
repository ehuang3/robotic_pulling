package mocap;

public interface mocap_SetObjTransformationRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mocap/mocap_SetObjTransformationRequest";
  static final java.lang.String _DEFINITION = "# Service to Set Transformation of Object Local Frame w.r.t Object Rigid Body Frame in the mocap system. \n\nfloat64 x\nfloat64 y\nfloat64 z\nfloat64 q0\nfloat64 qx\nfloat64 qy\nfloat64 qz\n";
  double getX();
  void setX(double value);
  double getY();
  void setY(double value);
  double getZ();
  void setZ(double value);
  double getQ0();
  void setQ0(double value);
  double getQx();
  void setQx(double value);
  double getQy();
  void setQy(double value);
  double getQz();
  void setQz(double value);
}
