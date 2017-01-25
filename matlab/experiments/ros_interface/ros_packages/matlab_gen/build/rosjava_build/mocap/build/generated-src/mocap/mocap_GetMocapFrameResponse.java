package mocap;

public interface mocap_GetMocapFrameResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "mocap/mocap_GetMocapFrameResponse";
  static final java.lang.String _DEFINITION = "mocap_frame mf\nint64 ret\nstring msg";
  mocap.mocap_frame getMf();
  void setMf(mocap.mocap_frame value);
  long getRet();
  void setRet(long value);
  java.lang.String getMsg();
  void setMsg(java.lang.String value);
}
