#pragma pack(push, 1)

//htons/l overloads
uint32_t hton(uint32_t t){
  return htonl(t);
}
uint16_t hton(uint16_t t){
  return htons(t);
}
uint16_t hton(uint8_t t){
  return t;
}
uint32_t ntoh(uint32_t t){
  return ntohl(t);
}
uint16_t ntoh(uint16_t t){
  return ntohs(t);
}
uint16_t ntoh(uint8_t t){
  return t;
}

//base types
struct timestamp {
  static std::array<ros::Time, 255> timeArray;
  static void processNMessage(uint8_t n, timeval* tv) {
    timeArray[n]=kernelTimeToTime(tv);
  }

  uint8_t n;
  uint8_t msSinceN;
  operator ros::Time() const{
    return timeArray[n]+ros::Duration(0, (uint32_t)msSinceN*1000000);
  }
};

//uint32_t if size=4, uint16_t if size==2 and a transparent if size==1
template<typename T, typename U=typename std::conditional<sizeof(T)==4, uint32_t,
                        typename std::conditional<sizeof(T)==2, uint16_t,
                            typename std::conditional<sizeof(T)==1, uint8_t,
                             void>::type
                        >::type
                     >::type
        >
class flipped{
  static_assert(!std::is_void<U>::value, "No available types of that size");
  U t;

  union switcher {
      T f;
      U i;
      //hack to make fixed_points flippable
      switcher(){new(&f) T();}
  };
public:
  flipped(const T& in){
    switcher v;
    v.f=in;
    t=hton(v.i);
  }
  operator T() const{
    switcher v;
    v.i=ntoh(t);
    return v.f;
  }
};

template<int max>
class fixed_point{
    int16_t representation;
public:
    fixed_point(float a){
        representation=a/max*65535;
    }
    fixed_point(){
        representation=0;
    }
    operator float32() const{
        return (float32)representation*max/65535;
    }
    operator float64() const{
        return (float64)representation*max/65535;
    }
};

//id calculation
constexpr uint16_t MAX_CAN_ID=0x7FF;

enum {
  synchronised_time_priority,
  cmd_vel_priority,
  odom_odrive_priority,
  light_sensor_priority,
  _last_message,
  GREATEST_MESSAGE_PRIORITY=_last_message-1
};

constexpr uint16_t ID_PER_PRIORITY=MAX_CAN_ID/GREATEST_MESSAGE_PRIORITY;

constexpr uint16_t idProvider(uint16_t priority){
  uint16_t id=priority*ID_PER_PRIORITY;
}

uint16_t idToPriority(uint16_t id){
  if(id%ID_PER_PRIORITY != 0){
    return -1;
  }
  return id/ID_PER_PRIORITY;
}


//message definitions
struct light_sensor_struct{
  static constexpr uint16_t id=idProvider(light_sensor_priority);
  static constexpr uint8_t size=1;

  flipped<bool> light;
  timestamp ts;
};

struct cmd_vel_struct{
  static constexpr uint16_t id=idProvider(cmd_vel_priority);
  static constexpr uint8_t size=4;

  flipped<fixed_point<2>> vX;
  flipped<fixed_point<14>> vTheta;
};

struct odom_struct{
  static constexpr uint8_t size=4;

  flipped<fixed_point<1>> dX;
  flipped<fixed_point<1>> dY;
  flipped<fixed_point<7>> dTheta;
};

struct synchronised_time_struct{
  static constexpr uint16_t id=idProvider(synchronised_time_priority);
  static constexpr uint8_t size=1;

  //this just gets sent 10 times a second, and numMessageSent is incremented every time
  //loops back around every 2.56 seconds, so messages can't reference events older than this. This allows for a 16bit timestamp,
  //where the first number is the numMessageSent and the second number is the number of between that n was recieved and the timestamp target
  flipped<uint8_t> numMessageSent;
};
#pragma pack(pop)
