#ifndef ROBOT_COMM_H
#define ROBOT_COMM_H

#include <ros/ros.h>
#include <matvec/matvec.h>
#include <geometry_msgs/Pose.h>

#include <robot_comm/robot_Ping.h>
#include <robot_comm/robot_SetCartesian.h>
#include <robot_comm/robot_SetCartesianJ.h>
#include <robot_comm/robot_GetCartesian.h>
#include <robot_comm/robot_SetWorkObject.h>
#include <robot_comm/robot_SetZone.h>
#include <robot_comm/robot_SetTool.h>
#include <robot_comm/robot_SetComm.h>
#include <robot_comm/robot_SetJoints.h>
#include <robot_comm/robot_GetJoints.h>
#include <robot_comm/robot_SetSpeed.h>
#include <robot_comm/robot_GetState.h>
#include <robot_comm/robot_SetVacuum.h>
#include <robot_comm/robot_SpecialCommand.h>
#include <robot_comm/robot_Stop.h>
#include <robot_comm/robot_SetTrackDist.h>
#include <robot_comm/robot_IsMoving.h>
#include <robot_comm/robot_SetDefaults.h>
#include <robot_comm/robot_GetIK.h>
#include <robot_comm/robot_GetFK.h>
#include <robot_comm/robot_Approach.h>
#include <robot_comm/robot_AddTrajectoryPoint.h>
#include <robot_comm/robot_ExecuteTrajectory.h>
#include <robot_comm/robot_ClearTrajectory.h>

#include <robot_comm/robot_CartesianLog.h>
#include <robot_comm/robot_JointsLog.h>
#include <robot_comm/robot_ForceLog.h>

#include <geometry_msgs/Pose.h>

#define NUM_JOINTS 6
#define NUM_FORCES 6

#define BLOCKING 1
#define NON_BLOCKING 0

#define VACUUM_OPEN 0
#define VACUUM_CLOSE 1

typedef enum
{
  ZONE_FINE = 0,
  ZONE_1,
  ZONE_2,
  ZONE_3,
  ZONE_4,
  ZONE_5,
  NUM_ZONES
} ZONE_TYPE;

typedef struct
{
  double p_tcp; // TCP path zone (mm)
  double p_ori; // Zone size for orientation (mm)
  double ori;   // Tool orientation (degrees)
} zone_vals;

static const zone_vals zone_data[NUM_ZONES] = 
{
  // p_tcp (mm), p_ori (mm), ori (deg)
  {0.0,   0.0,  0.0},   // ZONE_FINE
  {0.3,   0.3,  0.03},  // ZONE_1
  {1.0,   1.0,  0.1},   // ZONE_2
  {5.0,   8.0,  0.8},   // ZONE_3
  {10.0,  15.0, 1.5},   // ZONE_4
  {20.0,  30.0, 3.0}    // ZONE_5
};

class RobotComm
{
  public:
    RobotComm();
    RobotComm(ros::NodeHandle* np);
    ~RobotComm();

    // Subscription
    void subscribe(ros::NodeHandle* np);

    // Subscribe to Topics
    void subscribeCartesian(ros::NodeHandle* np, int q_len, 
        void (*funcPtr)(const robot_comm::robot_CartesianLogConstPtr&));
    void subscribeJoints(ros::NodeHandle* np, int q_len, 
        void (*funcPtr)(const robot_comm::robot_JointsLogConstPtr&));
    void subscribeForce(ros::NodeHandle* np, int q_len, 
        void (*funcPtr)(const robot_comm::robot_ForceLogConstPtr&));

    // Call this before program exits so we don't have double freeing issues
    void shutdown();

    // User functions
    bool Ping();
    bool SetCartesian(const double x, const double y, const double z, 
        const double q0, const double qx, const double qy, const double qz);
    bool SetCartesian(const HomogTransf pose);
    bool SetCartesian(const double cart[7]);
    bool SetCartesianJ(const HomogTransf pose);
    bool SetCartesianJ(const double cart[7]);
    bool SetCartesianJ(const double x, const double y, const double z, 
        const double q0, const double qx, const double qy, const double qz);
    bool SetJoints(const double j[NUM_JOINTS]); 
    bool SetJoints(const double j1, const double j2, const double j3, 
        const double j4, const double j5, const double j6);
    bool SetWorkObject(const double x, const double y, const double z, 
        const double q0, const double qx, const double qy, const double qz);
    bool SetWorkObject(const HomogTransf workObject);
    bool SetZone(const int z);
    bool SetTool(const HomogTransf toolframe);
    bool SetTool(const double x, const double y, const double z, 
        const double q0, const double qx, const double qy, const double qz);
    bool SetComm(const int mode);
    bool SetSpeed(const double tcp, const double ori);
    bool GetSpeed(double &tcp, double &ori);
    bool GetWorkObject(HomogTransf &workObject);
    bool GetWorkObject(double workObject[7]);
    bool GetTool(HomogTransf &tool);
    bool GetTool(double tool[7]);
    bool GetZone(int &zone);
    bool GetState(double &tcp, double &ori, int &zone, HomogTransf &workObject, HomogTransf &tool);
    bool GetState(double &tcp, double &ori, int &zone, double workObject[7], double tool[7]);
    bool SetTrackDist(const double pos_dist, const double ang_dist);
    bool Stop();
    bool SetVacuum(const int mode);
    bool SpecialCommand(int command, double param1=0, double param2=0, double param3=0, double param4=0, double param5=0);
    bool GetCartesian(double cart[7]);
    bool GetCartesian(double trans[3], double quat[4]);
    bool GetCartesian(Vec &trans, Quaternion &quat);
    bool GetCartesian(HomogTransf &t);
    bool GetCartesian(double &x, double &y, double &z, 
        double &q0, double &qx, double &qy, double &qz);
    bool GetJoints(double j[NUM_JOINTS]); 
    bool GetJoints(double &j1, double &j2, double &j3, 
        double &j4, double &j5, double &j6); 
    bool IsMoving();

    //bool AddTrajectoryPoint();
    //bool ExecuteTrajectory();
    //bool ClearTrajectory();


    bool GetIK(const HomogTransf pose, double joints[NUM_JOINTS]);
    bool GetFK(const double joints[NUM_JOINTS], HomogTransf &pose);

    bool Approach(geometry_msgs::Pose pose);
    bool moveArm(geometry_msgs::Pose pose);
    bool moveArm(double j[NUM_JOINTS]);
    bool relativeMoveArm(double x_off, double y_off, double z_off);
    bool relativeMoveArm(double x_off, double y_off, double z_off, 
                         geometry_msgs::Pose pose);
    bool invertHand(void);
    bool vibrate(void);
    bool moveToTop(void);
    bool moveReset(void);
    bool setupRobot(double tcp, double ori, int zone, double joints[NUM_JOINTS], geometry_msgs::Pose pose);

    bool SetDefaults();

  private:
    // Subscribers
    ros::Subscriber robot_cartesian_sub;
    ros::Subscriber robot_joints_sub;
    ros::Subscriber robot_force_sub;

    // ROS Service Clients
    ros::ServiceClient handle_robot_Ping;
    ros::ServiceClient handle_robot_SetCartesian;
    ros::ServiceClient handle_robot_SetCartesianJ;
    ros::ServiceClient handle_robot_GetCartesian;
    ros::ServiceClient handle_robot_SetWorkObject;
    ros::ServiceClient handle_robot_SetZone;
    ros::ServiceClient handle_robot_SetTool;
    ros::ServiceClient handle_robot_SetComm;
    ros::ServiceClient handle_robot_SetJoints;
    ros::ServiceClient handle_robot_GetJoints;
    ros::ServiceClient handle_robot_SetSpeed;
    ros::ServiceClient handle_robot_GetState;
    ros::ServiceClient handle_robot_SetTrackDist;
    ros::ServiceClient handle_robot_SpecialCommand;
    ros::ServiceClient handle_robot_SetVacuum;
    ros::ServiceClient handle_robot_Stop;
    ros::ServiceClient handle_robot_IsMoving;
    ros::ServiceClient handle_robot_SetDefaults;
    ros::ServiceClient handle_robot_GetIK;
    ros::ServiceClient handle_robot_GetFK;
    ros::ServiceClient handle_robot_Approach;

    // ROS services
    robot_comm::robot_Ping robot_Ping_srv;
    robot_comm::robot_SetCartesian robot_SetCartesian_srv;
    robot_comm::robot_SetCartesianJ robot_SetCartesianJ_srv;
    robot_comm::robot_GetCartesian robot_GetCartesian_srv;
    robot_comm::robot_SetWorkObject robot_SetWorkObject_srv;
    robot_comm::robot_SetZone robot_SetZone_srv;
    robot_comm::robot_SetTool robot_SetTool_srv;
    robot_comm::robot_SetComm robot_SetComm_srv;
    robot_comm::robot_SetJoints robot_SetJoints_srv;
    robot_comm::robot_GetJoints robot_GetJoints_srv;
    robot_comm::robot_SetSpeed robot_SetSpeed_srv;
    robot_comm::robot_GetState robot_GetState_srv;
    robot_comm::robot_SetTrackDist robot_SetTrackDist_srv;
    robot_comm::robot_SpecialCommand robot_SpecialCommand_srv;
    robot_comm::robot_SetVacuum robot_SetVacuum_srv;
    robot_comm::robot_Stop robot_Stop_srv;
    robot_comm::robot_IsMoving robot_IsMoving_srv;
    robot_comm::robot_SetDefaults robot_SetDefaults_srv;
    robot_comm::robot_GetIK robot_GetIK_srv;
    robot_comm::robot_GetFK robot_GetFK_srv;
    robot_comm::robot_Approach robot_Approach_srv;
};

#endif //ROBOT_COMM_H
