package robot_comm;

public interface robot_SpecialCommand extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SpecialCommand";
  static final java.lang.String _DEFINITION = "#Service to run spcial command\n\nint64 command  #integer identificating the command to run\nfloat64 param1 #Special purpose parameter number 1\nfloat64 param2 #Special purpose parameter number 2\nfloat64 param3 #Special purpose parameter number 3\nfloat64 param4 #Special purpose parameter number 4\nfloat64 param5 #Special purpose parameter number 5\n---\nint64 ret\nstring msg\n";
}
