/**
 * 通信代理节点，每台 Glider 一个实例
 * 仿真水声通信收发和通信决策，通过 mission/mode 指挥本机 mission_node 发布操作码，发指令切换状态
 * mission_node 只负责执行运动，不做通信决策
 * 决策映射逻辑为，
 *   收到 sensor/alert，本机切 LOITERING，广播 TARGET_FOUND，目标坐标，点对点 REQ_SURFACE
 *   收到 REQ_SURFACE，本机切 SURFACING，回复 ACK
 *   上浮到水面，广播 MISSION_COMPLETE
 *   每个周期 10s，广播 HEARTBEAT，自身坐标，作为通信握手
 * 参数：~namespace              (string)  本机命名空间，如 ug_glider_0，ID 从话题末尾数字推断
 * 
 * 订阅话题
 *   /{ns}/sensor/alert      (PointStamped)  异常场报警，point 为目标 NED 坐标
 *   /{ns}/glider_state      (GliderState)   本机状态，用于 HEARTBEAT 和水面判定
 *   /swarm/acoustic_channel (SwarmComm)     水声通信总线，所有的通信消息都发到这个topic里
 *
 * 发布话题
 *   /swarm/acoustic_channel (SwarmComm)     水声通信总线
 *   /{ns}/mission/mode      (UInt8)         模式指令，4=LOITERING, 5=SURFACING
 */

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/UInt8.h>
#include <ug_msgs/GliderState.h>
#include <ug_msgs/SwarmComm.h>
#include <clocale>
#include <string>

class CommAgentNode
{
public:
  CommAgentNode()
  {
    ros::NodeHandle nh("~");
    ns_ = nh.param<std::string>("namespace", "ug_glider");

    // 从 namespace 推断 ID
    gliderId_ = 0;
    auto pos = ns_.rfind('_');//从命名空间中查找下划线
    if (pos != std::string::npos)
    {
      try { gliderId_ = std::stoi(ns_.substr(pos + 1)); }
      catch (...) {}
    }

    ros::NodeHandle nhNs(ns_);
    ros::NodeHandle nhGlobal;//全局总线发布通讯消息

    // 订阅
    alertSub_ = nhNs.subscribe("sensor/alert", 1,
                                &CommAgentNode::onAlert, this);//是否找到target
    stateSub_ = nhNs.subscribe("glider_state", 1,
                                &CommAgentNode::onGliderState, this);
    commSub_ = nhGlobal.subscribe("/swarm/acoustic_channel", 10,
                                   &CommAgentNode::onAcousticMsg, this);//订阅总线上的消息

    // 发布
    commPub_ = nhGlobal.advertise<ug_msgs::SwarmComm>("/swarm/acoustic_channel", 10);
    modePub_ = nhNs.advertise<std_msgs::UInt8>("mission/mode", 1, true);

    // HEARTBEAT定时器，每 10 秒发布一次
    heartbeatTimer_ = nhGlobal.createTimer(ros::Duration(10.0),
                                            &CommAgentNode::onHeartbeat, this);

    ROS_INFO("[CommAgent] 启动 ns=%s, id=%d", ns_.c_str(), gliderId_);
  }

private:
  void onGliderState(const ug_msgs::GliderState::ConstPtr &msg)
  {
    lastState_ = *msg;
    hasState_ = true;

    // 上浮到水面判定
    if (surfacing_ && !surfaceReported_ && msg->depth < 0.5)
    {
      surfaceReported_ = true;
      ROS_WARN("[CommAgent-%d] 到达水面depth=%.2fm, 卫星链路可用", gliderId_, msg->depth);

      ug_msgs::SwarmComm done;
      done.header.stamp = ros::Time::now();
      done.sender_id = gliderId_;
      done.receiver_id = 255;
      done.opcode = ug_msgs::SwarmComm::MISSION_COMPLETE;
      done.x = msg->north;
      done.y = msg->east;
      done.z = msg->depth;
      commPub_.publish(done);
    }
  }

  void onAlert(const geometry_msgs::PointStamped::ConstPtr &msg)
  {
    if (alertTriggered_)
      return;

    alertTriggered_ = true;
    // 记录目标坐标（NED）
    targetNorth_ = msg->point.x;
    targetEast_ = msg->point.y;
    targetDepth_ = msg->point.z;
    ROS_WARN("[CommAgent-%d] 传感器发现目标 at (%.1f, %.1f, %.1f)",
             gliderId_, targetNorth_, targetEast_, targetDepth_);

    // 指挥本机 mission_node 切换 LOITERING
    std_msgs::UInt8 mode;
    mode.data = 4;  // LOITERING
    modePub_.publish(mode);

    // 广播 TARGET_FOUND（携带目标坐标）
    ug_msgs::SwarmComm comm;
    comm.header.stamp = ros::Time::now();
    comm.sender_id = gliderId_;
    comm.receiver_id = 255;
    comm.opcode = ug_msgs::SwarmComm::TARGET_FOUND;
    comm.x = targetNorth_;
    comm.y = targetEast_;
    comm.z = targetDepth_;
    commPub_.publish(comm);

    // 点对点 REQ_SURFACE（同样携带目标坐标）
    comm.opcode = ug_msgs::SwarmComm::REQ_SURFACE;
    comm.receiver_id = (gliderId_ == 0) ? 1 : 0;
    commPub_.publish(comm);

    ROS_WARN("[CommAgent-%d] 广播 TARGET_FOUND + REQ_SURFACE -> G%d",
             gliderId_, comm.receiver_id);
  }

  void onAcousticMsg(const ug_msgs::SwarmComm::ConstPtr &msg)
  {
    // 忽略自己发的
    if (msg->sender_id == gliderId_)
      return;

    // 不是发给自己也不是广播的，忽略
    if (msg->receiver_id != gliderId_ && msg->receiver_id != 255)
      return;

    switch (msg->opcode)
    {
    case ug_msgs::SwarmComm::REQ_SURFACE:
      if (!surfacing_)
      {
        surfacing_ = true;
        ROS_WARN("[CommAgent-%d] 收到 REQ_SURFACE from G%d, 切换 SURFACING",
                 gliderId_, msg->sender_id);

        // 指挥本机 mission_node 上浮
        std_msgs::UInt8 mode;
        mode.data = 5;  // SURFACING
        modePub_.publish(mode);

        // 回复 ACK
        ug_msgs::SwarmComm ack;
        ack.header.stamp = ros::Time::now();
        ack.sender_id = gliderId_;
        ack.receiver_id = msg->sender_id;
        ack.opcode = ug_msgs::SwarmComm::ACK;
        if (hasState_)
        {
          ack.x = lastState_.north;
          ack.y = lastState_.east;
          ack.z = lastState_.depth;
        }
        commPub_.publish(ack);
      }
      break;

    case ug_msgs::SwarmComm::TARGET_FOUND:
      ROS_INFO("[CommAgent-%d] 收到 TARGET_FOUND from G%d at (%.1f, %.1f, %.1f)",
               gliderId_, msg->sender_id, msg->x, msg->y, msg->z);
      break;

    case ug_msgs::SwarmComm::ACK:
      ROS_INFO("[CommAgent-%d] 收到 ACK from G%d", gliderId_, msg->sender_id);
      break;

    case ug_msgs::SwarmComm::MISSION_COMPLETE:
      ROS_WARN("[CommAgent-%d] 收到 MISSION_COMPLETE from G%d", gliderId_, msg->sender_id);
      break;

    default:
      break;
    }
  }

  void onHeartbeat(const ros::TimerEvent &)
  {
    if (!hasState_)
      return;

    ug_msgs::SwarmComm hb;
    hb.header.stamp = ros::Time::now();
    hb.sender_id = gliderId_;
    hb.receiver_id = 255;
    hb.opcode = ug_msgs::SwarmComm::HEARTBEAT;
    hb.x = lastState_.north;
    hb.y = lastState_.east;
    hb.z = lastState_.depth;
    commPub_.publish(hb);
  }

  std::string ns_;
  int gliderId_ = 0;

  ros::Subscriber alertSub_, stateSub_, commSub_;
  ros::Publisher commPub_, modePub_;
  ros::Timer heartbeatTimer_;

  ug_msgs::GliderState lastState_;
  bool hasState_ = false;
  bool alertTriggered_ = false;
  bool surfacing_ = false;
  bool surfaceReported_ = false;
  double targetNorth_ = 0, targetEast_ = 0, targetDepth_ = 0;
};

int main(int argc, char **argv)
{
  setlocale(LC_ALL, "");
  ros::init(argc, argv, "comm_agent_node");
  CommAgentNode node;
  ros::spin();
  return 0;
}
