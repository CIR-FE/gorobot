#include <RobotDrivers/KukaRobotDriver.h>
#include <kApi/Data/kString.h>
#include <kApi/kApi.h>
#include <process.h>

#include <format>
#include <iostream>
#include <vector>


static kXml KukaRobotDriver_startNewMessage(kAlloc alloc)
{
    kXml xmlSend = kNULL;
    kXmlItem root = kNULL, itemStatus = kNULL, itemCommand = kNULL;
    kXmlItem itemGoto = kNULL, itemSet = kNULL;

    kXml_Construct(&xmlSend, alloc);
    kXml_AddItem(xmlSend, kNULL, "Sensor", &root);
    kXml_AddItem(xmlSend, root, "Status", &itemStatus);
    kXml_AddItem(xmlSend, root, "Command", &itemCommand);
    kXml_AddItem(xmlSend, itemCommand, "Goto", &itemGoto);
    kXml_AddItem(xmlSend, itemCommand, "Set", &itemSet);

    kXml_SetAttrSize(xmlSend, itemStatus, "Count", 0);
    kXml_SetAttrBool(xmlSend, itemStatus, "Reset", false);
    kXml_SetAttrBool(xmlSend, itemSet, "base", false);
    kXml_SetAttrBool(xmlSend, itemSet, "tcp", false);
    kXml_SetAttrBool(xmlSend, itemSet, "joints", false);
    kXml_SetAttrBool(xmlSend, itemSet, "movement", false);
    kXml_SetAttrBool(xmlSend, itemGoto, "stop", false);

    return xmlSend;
}

KukaRobotDriver::KukaRobotDriver()
    : _alloc(kNULL),
      _socket(kNULL)
{
}

KukaRobotDriver::KukaRobotDriver(const char *ip, k32u port, kAlloc alloc)
{
    setAlloc(alloc);
    connect(ip, port);
}

// Virtual Class Implementation
void KukaRobotDriver::set_base(GoRobot::Matrix base)
{
    GoRobot::KukaPose basePose(base);
    set_base(basePose);
}
void KukaRobotDriver::set_tcp(GoRobot::Matrix tcp)
{
    GoRobot::KukaPose tcpPose(tcp);
    set_tcp(tcpPose);
}
void KukaRobotDriver::set_acc_speed(double acceleration, double speed, GoRobot::MoveMode moveMode)
{
    kXmlItem itemSet = kNULL, itemMovement = kNULL;
    kXml xmlSend = KukaRobotDriver_startNewMessage(_alloc);

    double speed_meters = speed / 1000;
    double acceleration_meters = acceleration / 1000;

    kXml_FindChild(xmlSend, kNULL, "Sensor/Command/Set", &itemSet);

    kXml_SetAttrBool(xmlSend, itemSet, "movement", true);
    kXml_AddItem(xmlSend, itemSet, "movement", &itemMovement);
    kXml_SetAttr64f(xmlSend, itemMovement, "speed", speed_meters);
    kXml_SetAttr64f(xmlSend, itemMovement, "acceleration", acceleration_meters);

    std::cout << std::format("Setting speed to: {} mm/s and acceleration to: {} mm/s^2", speed,
                             acceleration)
              << "\n";
    sendMessage(&xmlSend);
}
void KukaRobotDriver::move(GoRobot::Matrix *poseList, size_t poseCount, GoRobot::MoveMode moveMode)
{
    switch (moveMode) {
        case GoRobot::MoveMode::LINEAR:
            for (size_t i = 0; i < poseCount; i++) {
                GoRobot::KukaPose pose(poseList[i]);
                move(&pose, 1);
            }
            break;
        case GoRobot::MoveMode::CONTINOUS_LINEAR: {
            std::vector<GoRobot::KukaPose> kukaPoses;
            for (size_t i = 0; i < poseCount; i++)
                kukaPoses.push_back(GoRobot::KukaPose(poseList[i]));
            move(kukaPoses.data(), kukaPoses.size());
        } break;

        default:
            throw "This driver only supports linear movements";
            break;
    }
}
GoRobot::Matrix KukaRobotDriver::get_tcp_pose()
{
    RobotMsg msg = receiveMsg();
    return msg.actPose.toMatrix();
}
GoRobot::Matrix KukaRobotDriver::get_flange_pose()
{
    RobotMsg msg = receiveMsg();
    return msg.flangePose.toMatrix();
}

void KukaRobotDriver::setAlloc(kAlloc alloc)
{
    _alloc = alloc;
}

void KukaRobotDriver::connect(const char *ip, k32u port)
{
    if (kOK != kSocket_Construct(&_socket, kIP_VERSION_4, kSOCKET_TYPE_TCP, _alloc)) {
        _socket = kNULL;
        throw std::runtime_error("Connection Error");
    }

    kIpAddress ipAddress;
    kIpAddress_Parse(&ipAddress, ip);
    if (kOK != kSocket_Connect(_socket, ipAddress, port, 20000)) {
        _socket = kNULL;
        kDestroyRef(&_socket);
        throw std::runtime_error("Connection Error");
    }
}
KukaRobotDriver::~KukaRobotDriver()
{
    kDestroyRef(&_socket);
}
kXml KukaRobotDriver::receiveXml(kAlloc alloc)
{
    kXml xmlReceive = kNULL;

    std::string strReceive;
    std::string root, tag;
    while (1) {
        char r;
        kSize c;
        if (kOK != kSocket_Read(_socket, &r, 1, &c)) break;

        strReceive += r;
        tag += r;
        if (r == '<') tag = "";
        if (r == '>') {
            if (root == "")
                root = tag;
            else if (tag == "/" + root)
                break;
        }
    }

    kXml_FromText(&xmlReceive, strReceive.c_str(), alloc);
    if (kIsNull(xmlReceive)) throw std::runtime_error("Empty Message");
    return xmlReceive;
}
void KukaRobotDriver::sendXml(kXml xmlSend)
{
    kSize c;
    kString strContent = kNULL;
    kString_Construct(&strContent, "", kObject_Alloc(xmlSend));
    kXml_ToString(xmlSend, strContent);
    if (kOK != kSocket_Write(_socket, kString_Chars(strContent), kString_Length(strContent), &c))
        throw std::runtime_error("Error Sending");
    kDestroyRef(&strContent);
}

void Krd_kXmlSetAttr(kXml xml, kXmlItem item, const char *name, bool value)
{
    kXml_SetAttrBool(xml, item, name, value);
}
void Krd_kXmlSetAttr(kXml xml, kXmlItem item, const char *name, kSize value)
{
    kXml_SetAttrSize(xml, item, name, value);
}
void Krd_kXmlSetAttr(kXml xml, kXmlItem item, const char *name, k64f value)
{
    kXml_SetAttr64f(xml, item, name, value);
}
template <class T>
static void KukaRobotDriver_SetAttribute(kXml xml, const char *path, const char *name, T value)
{
    kXmlItem item = kNULL;
    kXml_FindChild(xml, kNULL, path, &item);
    Krd_kXmlSetAttr(xml, item, name, value);
}

void KukaRobotDriver_addPose(kXml &xmlSend, GoRobot::KukaPose pt)
{
    kXmlItem itemGoto = kNULL, itemFrame = kNULL;
    kXmlItem itemStatus = kNULL;

    kXml_FindChild(xmlSend, kNULL, "Sensor/Command/Goto", &itemGoto);
    kXml_FindChild(xmlSend, kNULL, "Sensor/Status", &itemStatus);

    kXml_AddItem(xmlSend, itemGoto, "xyzabc", &itemFrame);
    kXml_SetAttr64f(xmlSend, itemFrame, "X", pt.x);
    kXml_SetAttr64f(xmlSend, itemFrame, "Y", pt.y);
    kXml_SetAttr64f(xmlSend, itemFrame, "Z", pt.z);
    kXml_SetAttr64f(xmlSend, itemFrame, "A", pt.rz);
    kXml_SetAttr64f(xmlSend, itemFrame, "B", pt.ry);
    kXml_SetAttr64f(xmlSend, itemFrame, "C", pt.rx);

    kSize count = 0;
    if (kOK != kXml_AttrSize(xmlSend, itemStatus, "Count", &count)) count = 0;

    kXml_SetAttrSize(xmlSend, itemStatus, "Count", count + 1);
}
RobotMsg KukaRobotDriver::set_tcp(GoRobot::KukaPose tcp)
{
    kXmlItem itemSet = kNULL, itemFrame = kNULL;
    kXml xmlSend = KukaRobotDriver_startNewMessage(_alloc);

    kXml_FindChild(xmlSend, kNULL, "Sensor/Command/Set", &itemSet);

    kXml_SetAttrBool(xmlSend, itemSet, "tcp", true);
    kXml_AddItem(xmlSend, itemSet, "tcp", &itemFrame);
    kXml_SetAttr64f(xmlSend, itemFrame, "X", tcp.x);
    kXml_SetAttr64f(xmlSend, itemFrame, "Y", tcp.y);
    kXml_SetAttr64f(xmlSend, itemFrame, "Z", tcp.z);
    kXml_SetAttr64f(xmlSend, itemFrame, "A", tcp.rz);
    kXml_SetAttr64f(xmlSend, itemFrame, "B", tcp.ry);
    kXml_SetAttr64f(xmlSend, itemFrame, "C", tcp.rx);

    std::cout << std::format("Setting tcp to: X={}, Y={}, Z={}, A={}, B={}, C={}", tcp.x, tcp.y,
                             tcp.z, tcp.rz, tcp.ry, tcp.rx)
              << "\n";
    return sendMessage(&xmlSend);
}
RobotMsg KukaRobotDriver::set_base(GoRobot::KukaPose base)
{
    kXmlItem itemSet = kNULL, itemFrame = kNULL;
    kXml xmlSend = KukaRobotDriver_startNewMessage(_alloc);

    kXml_FindChild(xmlSend, kNULL, "Sensor/Command/Set", &itemSet);

    kXml_SetAttrBool(xmlSend, itemSet, "base", true);
    kXml_AddItem(xmlSend, itemSet, "base", &itemFrame);
    kXml_SetAttr64f(xmlSend, itemFrame, "X", base.x);
    kXml_SetAttr64f(xmlSend, itemFrame, "Y", base.y);
    kXml_SetAttr64f(xmlSend, itemFrame, "Z", base.z);
    kXml_SetAttr64f(xmlSend, itemFrame, "A", base.rz);
    kXml_SetAttr64f(xmlSend, itemFrame, "B", base.ry);
    kXml_SetAttr64f(xmlSend, itemFrame, "C", base.rx);

    std::cout << std::format("Setting base to: X={}, Y={}, Z={}, A={}, B={}, C={}", base.x, base.y,
                             base.z, base.rz, base.ry, base.rx)
              << "\n";
    return sendMessage(&xmlSend);
}
RobotMsg KukaRobotDriver::move(GoRobot::KukaPose *pList, kSize pCount)
{
    RobotMsg ret;
    kSize msgPoseCount = 0;

    kXml xmlSend = KukaRobotDriver_startNewMessage(_alloc);
    for (kSize i = 0; i < pCount; i++) {
        if (msgPoseCount >= KUKA_MAX_POINTS_PER_MSG) {
            ret = sendMessage(&xmlSend);
            xmlSend = KukaRobotDriver_startNewMessage(_alloc);
            KukaRobotDriver_SetAttribute(xmlSend, "Sensor/Command/Goto", "stop", false);
            msgPoseCount = 0;
        }

        std::cout << std::format("Moving to: X={}, Y={}, Z={}, A={}, B={}, C={}", pList[i].x,
                                 pList[i].y, pList[i].z, pList[i].rz, pList[i].ry, pList[i].rx)
                  << "\n";
        KukaRobotDriver_addPose(xmlSend, pList[i]);
        msgPoseCount++;
    }

    if (kIsNull(xmlSend)) xmlSend = KukaRobotDriver_startNewMessage(_alloc);
    KukaRobotDriver_SetAttribute(xmlSend, "Sensor/Command/Goto", "stop", true);
    ret = sendMessage(&xmlSend);

    return ret;
}

// This function also destroys xmlSend
RobotMsg KukaRobotDriver::sendMessage(kXml *xmlSend)
{
    sendXml(*xmlSend);
    kDestroyRef(xmlSend);

    // get an answer
    return RobotMsg(receiveXml(_alloc));
}
RobotMsg KukaRobotDriver::sendReset()
{
    kXmlItem itemStatus = kNULL;
    kXml xmlSend = KukaRobotDriver_startNewMessage(_alloc);
    kXml_FindChild(xmlSend, kNULL, "Sensor/Status", &itemStatus);
    kXml_SetAttrBool(xmlSend, itemStatus, "Reset", kTRUE);

    return sendMessage(&xmlSend);
}
RobotMsg KukaRobotDriver::receiveMsg()
{
    // Send an empty message to receive one
    kXml xmlSend = KukaRobotDriver_startNewMessage(_alloc);
    return sendMessage(&xmlSend);
}

RobotMsg::RobotMsg()
{
}
RobotMsg::RobotMsg(kXml xmlReceive)
{
    kXmlItem itemActPos = kNULL, itemStatus = kNULL, itemJoints = kNULL;

    kXml_FindChild(xmlReceive, kNULL, "Robot/Data/ActPos", &itemActPos);
    kXml_Attr64f(xmlReceive, itemActPos, "X", &actPose.x);
    kXml_Attr64f(xmlReceive, itemActPos, "Y", &actPose.y);
    kXml_Attr64f(xmlReceive, itemActPos, "Z", &actPose.z);
    kXml_Attr64f(xmlReceive, itemActPos, "C", &actPose.rx);
    kXml_Attr64f(xmlReceive, itemActPos, "B", &actPose.ry);
    kXml_Attr64f(xmlReceive, itemActPos, "A", &actPose.rz);

    kXml_FindChild(xmlReceive, kNULL, "Robot/Data/FlangePos", &itemActPos);
    kXml_Attr64f(xmlReceive, itemActPos, "X", &flangePose.x);
    kXml_Attr64f(xmlReceive, itemActPos, "Y", &flangePose.y);
    kXml_Attr64f(xmlReceive, itemActPos, "Z", &flangePose.z);
    kXml_Attr64f(xmlReceive, itemActPos, "C", &flangePose.rx);
    kXml_Attr64f(xmlReceive, itemActPos, "B", &flangePose.ry);
    kXml_Attr64f(xmlReceive, itemActPos, "A", &flangePose.rz);

    kXml_FindChild(xmlReceive, kNULL, "Robot/Status", &itemStatus);
    kXml_AttrBool(xmlReceive, itemStatus, "error", &error);
}
