#include <iostream>
#include <arpa/inet.h>
#include <stdint.h>
#include <mutex>

typedef uint8_t __u8;
typedef uint32_t canid_t;
typedef float float32;
typedef double float64;
struct can_frame {
  canid_t can_id;  // 32 bit CAN_ID + EFF/RTR/ERR flags
  __u8    can_dlc; // frame payload length in byte (0 .. 8)
  __u8    __pad;   // padding
  __u8    __res0;  // reserved / padding
  __u8    __res1;  // reserved / padding
  __u8    data[8] __attribute__((aligned(8)));
};

enum



int socket_fd;



void write(const can_frame * frame){
  //write(socket_fd, &frame, sizeof(can_frame));
  printf("%u %u %0x %0x %0x %0x %0x %0x %0x %0x\n",frame->can_id,frame->can_dlc,frame->data[0],frame->data[1],frame->data[2],frame->data[3],frame->data[4],frame->data[5],frame->data[6],frame->data[7]);
}

template<typename T>
class MessageSender{
  can_frame frame;
public:
  T* struct_ptr=(T*)frame.data;
  MessageSender() {
    frame.can_id=T::id;
    frame.can_dlc=T::size;
  }
  void sendFrame() const{
    write(&frame);
  }
};

void cmd_velCallback(const geometry_msgs::Twist* cmd){
  static MessageSender<cmd_vel_struct> sender;
  sender.struct_ptr->velX=(float32)cmd.linear.x;
  sender.struct_ptr->velTheta=(float32)cmd.angular.z;
  sender.sendFrame();
};

ros::Time universalNTimes[256];
uint16_t latestN;
uint16_t timeStampGeneratorNow(){
  ros::Duration timeStep=ros::Time::now()-universalTimeTimes[latestN];
  return latestN<<8 | timeStep.toNSec()/1000;
}

class InputPublisher{
  //is all this map stuff necesary? no. But it makes it very general and generalizes to as many odometry publishers as I want
  void odom_dPos_pub(__u8* data, uint16_t fromPriority){
    static std::map<uint16_t, ros::Time> priorityTimeMap;
    const odom_struct * recv=(odom_struct *)data;
    const ros::Time ts=recv->ts;


    //if insertion occured, do nothing, we don't have an old timestamp to compare to
    //if insertion didn't occur, then we simply use our result and then update our value
    if(auto [it, insertionOccured]=priorityTimeMap.insert({fromPriority, ts}); !insertionOccured){
      float64 dT=(it->second-ts).toSec();
      float64 dX=static_cast<float64>(recv->dX);
      float64 dY=static_cast<float64>(recv->dY);
      float64 dTheta=static_cast<float64>(recv->dTheta);
      float64 vX=dX/dT;
      float64 vY=dY/dT;
      float64 vTheta=dTheta/dT;
      geometry_msgs::TwistWithCovarianceStamped odom0;
      odom0_pub.publish(odom0);
      it->second=ts;
    }
  }

  void light_sensor_pub(__u8* data){
     light_sensor_struct * recv=(light_sensor_struct *)data;
  }

  void receiveAllAvailableMessagesOnBus(){
    while(read(s, &frame, sizeof(struct can_frame))>=0){
      switch(uint16_t priority=idToPriority(frame->can_dlc)){
        case cmd_vel_priority:
        break;
        case odom_mouse_priority:
        case odom_odrive_priority:
          odom_dPos_pub(frame->data, priority);
        break;
        case light_sensor_priority:
          light_sensor_pub(frame->data);
        case synchronised_time_priority:
          timeval tv;
          ioctl(s, SIOCGSTAMP, &tv);
          timestamp::processNMessage(frame->data[0], &tv)
        break;
        default:
          ROS_ERROR("message of unknown id received");
        break;
      }
    }
    perror(errno);
  }
};

int main(){
  struct sockaddr_can addr;
  struct ifreq ifr;

  socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  strcpy(ifr.ifr_name, "can0" );
  ioctl(s, SIOCGIFINDEX, &ifr);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr));

  InputPublisher pubber;
  //create thread that runs receiveAllAvailableMessagesOnBus continuously


  //now just process the callbacks and send the frames as necesary
  ros::spin();
}
