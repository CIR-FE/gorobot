#ifndef KUKA_ROBOT_DRIVER_H
#define KUKA_ROBOT_DRIVER_H

#include <GoRobot/GoRobot.h>
#include <GoRobot/GoRobotDriverInterface.h>

#define KUKA_MAX_POINTS_PER_MSG 512
#define KUKA_NUMBER_OF_AXIS 6

/**
 * @struct  KukaJoints
 * @brief   Angle position for each joint of the robot
 */
struct KukaJoints {
    double A[KUKA_NUMBER_OF_AXIS];
};

/**
 * @struct  RobotMsg
 * @brief   A message received from the Kuka robot
 *          Consists of the positions of the TCP and the Flange.
 */
struct RobotMsg {
    GoRobot::KukaPose actPose;
    GoRobot::KukaPose flangePose;
    KukaJoints joints;
    kBool error;

    RobotMsg();
    RobotMsg(kXml xmlReceive);
};

/**
 * @class        KukaRobotDriver
 * @brief        A driver for a Kuka robot, implementing the GoRobotDriver interface.
 *               Used to communicate with the robot from a PC (could be run on the Kuka Controller
 * as well) The robot must run the XmlLinearFollower program, see project resources for details.
 */
class KukaRobotDriver : GoRobot::Driver
{
 public:
    KukaRobotDriver();
    KukaRobotDriver(const char *ip, k32u port, kAlloc alloc = kNULL);
    ~KukaRobotDriver();

    /**
     * Sets the alloc object for kObject contstruction
     *
     * @param    alloc    The kAlloc object
     */
    void setAlloc(kAlloc alloc);

    /**
     * Connects to the Kuka robot
     * Robot must be running with the XmlLinearFollower program. See project resources for details.
     * @param    ip      The ip of the robot
     * @param    port    The port on which the server is listening
     */
    void connect(const char *ip, k32u port);

    // Virtual function implementations

    void set_base(GoRobot::Matrix base);
    void set_tcp(GoRobot::Matrix tcp);
    void move(GoRobot::Matrix *poseList, size_t poseCount = 1,
              GoRobot::MoveMode moveMode = GoRobot::MoveMode::CONTINOUS_LINEAR);
    void set_acc_speed(double acceleration, double speed,
                       GoRobot::MoveMode moveMode = GoRobot::MoveMode::CONTINOUS_LINEAR);
    GoRobot::Matrix get_tcp_pose();
    GoRobot::Matrix get_flange_pose();

 private:
    kSocket _socket;
    kAlloc _alloc;

    void sendXml(kXml xml);
    kXml receiveXml(kAlloc alloc);

    RobotMsg set_base(GoRobot::KukaPose base);
    RobotMsg set_tcp(GoRobot::KukaPose tcp);
    RobotMsg move(GoRobot::KukaPose *pList, kSize pCount);
    RobotMsg sendMessage(kXml *xmlSend);
    RobotMsg sendReset();
    RobotMsg receiveMsg();
};

#endif  // KUKA_ROBOT_DRIVER_H