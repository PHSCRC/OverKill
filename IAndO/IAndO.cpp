#include <iostream>
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

void cmd_vel_callback(const geometry_msgs::Twist* cmd){
  static MessageSender<cmd_vel_struct> sender;
  sender.struct_ptr->velX=(float32)cmd.linear.x;
  sender.struct_ptr->velTheta=(float32)cmd.angular.z;
  sender.sendFrame();
};

class InputPublisher{
  Nodehandle nh;
  ros::Publisher baby_position_pubber=nh.adverstise<overkill_msgs::PoseStampedWithId>("baby_position", 10);
  ros::Publisher candle_position_pubber=nh.adverstise<overkill_msgs::PoseStampedWithId>("candle_position", 10);
  ros::Publisher odom0_pubber=nh.adverstise<geometry_msgs::PoseStamped>("odom0", 10);
  ros::Publisher odom1_pubber=nh.adverstise<geometry_msgs::PoseStamped>("odom1", 10);
  ros::Publisher light_sensor_pubber=nh.adverstise<overkill_msgs::BoolStamped>("odom1", 10);
  ros::Publisher global_state_pubber=nh.adverstise<std_msgs::Byte>("global_state", 10);


  GlobalState global_state;

  void pub_global_state(GlobalState in){
    global_state=in;
    pub_global_state(in);
  }

  void maze_number_callback(std_msgs::Byte* in){
    switch(global_state){
      case 2:
      case 5:
      case 7:
        pub_global_state(in+1);
      case 4:
      break;
      default:
        ROS_WARN("switched maze unexpectedly")
      break;
    }
  }

  ros::Subscriber maze_number_subber=nh.subscribe("maze_number", 3, &InputPublisher::maze_number_callback, this);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);


  //TODO create all 10 static publishers which camera0 to camera 9 which are 36 deg apart relate cameraPOS to rotate_base

  //TODO relate rotate_base to base_link, cause they're probably not exactly lined up




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

      //TODO build covariance estimate using measured data

      geometry_msgs::TwistWithCovarianceStamped odom_obj{};

      if(fromPriority==odom_odrive_priority){
        odom0_pubber.publish(odom_obj);
      }
      if(fromPriority==odom_mouse_priority){
        odom1_pubber.publish(odom_obj);
      }
      //update last sent data
      it->second=ts;
    }
  }

  void light_sensor_pub(__u8* data){
     light_sensor_struct * recv=(light_sensor_struct *)data;
     light_sensor_pubber.publish({data->ts, data->light});
  }

  inline geometry_msgs::PoseStamped getPoseStampedFromCameraInfo(uint8_t rotation, float64 distance, float64 angle, float64 targetFacingAngle, std::string base_id){
    //transform from camera frame (in whatever rotation it is, if applicable to the map frame)
    return tfBuffer.transform(geometry_msgs::PoseStamped
        { //PoseStamped
          { //stamp
            0, //publish number (should be empty)
            data->ts, //timestamp
            base_id + std::to_string(rotation) //frameid of pose
          },
          { //Pose
            { //Position
              distance*std::cos(angle), //X
              distance*std::sin(angle), //Y
              0 //Z
            },
            tf2_ros::quaternion_from_euler(0, 0, 3.14159265+targetFacingAngle) //Angle
          }
        },
        "map" //target frameid
      ); //convert the pose data from the camera frame to the map frame with a correct timestamp as to where base_link was
  }

  void baby_found_pub(__u8* data){
    //we can ignore these messages if we're not in the correct spot. These are checked on the send side, but it doesn't hurt to double check
    if(globalState==LOOKING_FOR_BABY){
      baby_found_struct * recv=(baby_found_struct *)data;
      baby_position_pubber.publish(getPoseStampedFromCameraInfo(static_cast<uint8_t>(recv->rotation), recv->combinedDistanceData.distance(), recv->combinedDistanceData.angle(), recv->combinedDistanceData.targetFacingAngle(), "candle_camera"));
    }
  }
  void candle_found_pub(__u8* data){
    //we can ignore these messages if we're not in the correct spot. These are checked on the send side, but it doesn't hurt to double check
    if(globalState==LOOKING_FOR_BABY || globalState==PICKED_UP_BABY || globalState==CANDLE_SEARCH_MAZE_2 || globalState==CANDLE_SEARCH_MAZE_1 || globalState==EXT_CANDLE_2){
      candle_found_struct * recv=(candle_found_struct *)data;
      candle_position_pubber.publish(getPoseStampedFromCameraInfo(static_cast<uint8_t>(recv->rotation), id, recv->combinedDistanceData.distance(), recv->combinedDistanceData.angle(), recv->combinedDistanceData.targetFacingAngle(), "candle_camera"));
    }
  }
  void pickup_stat_baby_pub(__u8* data){
    pickup_stat_baby_struct * recv =(pickup_stat_baby_struct *)data;
    if(recv->update==1){
      pub_global_state(PICKED_UP_BABY);
    }
  }
  void ext_stat_candle_pub(__u8* data){
    pickup_stat_baby_struct * recv =(pickup_stat_baby_struct *)data;
    if(recv->update==1){
      switch(globalState){
        case CANDLE_SEARCH_MAZE_2:
          pub_global_state(EXT_CANDLE_1)
        break;
        case CANDLE_SEARCH_MAZE_1:
          pub_global_state(EXT_CANDLE_2);
        break;
        case EXT_CANDLE_2:
          pub_global_state(EXT_CANDLE_3);
        break;
      }
    }
  }
  void microphone_on_pub(__u8* data){
    microphone_on_struct * recv=(microphone_on_struct *) data;
    pub_global_state(DRIVING_TO_MAZE_2);
  }

  void receiveAllAvailableMessagesOnBus(){
    //blocking call to read, that's when the thread gets a break
    while(read(s, &frame, sizeof(struct can_frame))>=0 && ros::ok()){
      switch(uint16_t priority=idToPriority(frame->can_dlc); priority){
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
          //static call so that all automatic conversions from now on update
          timestamp::processNMessage(frame->data[0], &tv)
        break;
        case baby_found_priority:
          baby_found_pub(frame->data);
        break;
        case candle_found_priority:
          candle_found_pub(frame->data);
        break;
        case pickup_stat_baby_priority:
          pickup_stat_baby_pub(frame->data);
        break;
        case ext_stat_candle_priority:
          ext_stat_candle_pub(frame->data);
        break;
        case microphone_on_priority:
          microphone_on_pub(frame->data);
        break;
        default:
          ROS_ERROR("message of unknown id received, make sure you've copied IAndOStructs over to all parties");
        break;
      }
      ros::spinOnce();
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

  Nodehandle nh;
  ros::init(argc, argv, "IAndO");

  InputPublisher pubber;
  //continuously read all can messages and publish them
  std::thread pub_thread(&InputPublisher::receiveAllAvailableMessagesOnBus, &pubber);

  ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 2, cmd_vel_callback);
  //now just process the callbacks and send off messages in this main thread
  ros::spin();

  pub_thread.join();
}
