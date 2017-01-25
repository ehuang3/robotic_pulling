package robot_comm;

public interface robot_SpecialCommandRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "robot_comm/robot_SpecialCommandRequest";
  static final java.lang.String _DEFINITION = "#Service to run spcial command\n\nint64 command  #integer identificating the command to run\nfloat64 param1 #Special purpose parameter number 1\nfloat64 param2 #Special purpose parameter number 2\nfloat64 param3 #Special purpose parameter number 3\nfloat64 param4 #Special purpose parameter number 4\nfloat64 param5 #Special purpose parameter number 5\n";
  long getCommand();
  void setCommand(long value);
  double getParam1();
  void setParam1(double value);
  double getParam2();
  void setParam2(double value);
  double getParam3();
  void setParam3(double value);
  double getParam4();
  void setParam4(double value);
  double getParam5();
  void setParam5(double value);
}
