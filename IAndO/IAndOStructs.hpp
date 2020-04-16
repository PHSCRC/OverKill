#include <arpa/inet.h>

#pragma pack(push, 1)

enum GlobalState{
  STARTING_UP=0,
  WAITING_FOR_SOUND,
  DRIVING_TO_MAZE_2,
  LOOKING_FOR_BABY,
  PICKED_UP_BABY,
  DROPPED_BABY,
  CANDLE_SEARCH_MAZE_2,
  EXT_CANDLE_1,
  CANDLE_SEARCH_MAZE_1,
  EXT_CANDLE_2,
  EXT_CANDLE_3
};

enum ledBitSetEnum{
  SOUND_HANDLE_BITSET=0,
  FIRE_HANDLE_BITSET,
  BABY_HANDLE_BITSET
};

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


float toStdPos(float angle){
  while(angle<0){angle+=2*M_PI;}
  while(angle>=2*M_PI){angle-=2*M_PI;}
  return angle;
}

//base types
struct timestamp{
  static std::array<ros::Time, 255> timeArray;
  static void processNMessage(uint8_t n, timeval* tv) {
    timeArray[n]=kernelTimeToTime(tv);
  }
  operator ros::Time() const{
    return timeArray[n]+ros::Duration(0, (uint32_t)msSinceN*1000000);
  }

  uint8_t n;
  uint8_t msSinceN;
}

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

template<int max, bool isSigned, bool doubleWidth=true>
class fixed_point;

template<int max>
class fixed_point<max, true, true>{
    int16_t representation;
public:
    fixed_point(float a){
        representation=a/max*32767;
    }
    fixed_point(){
        representation=0;
    }
    operator float32() const{
        return (float32)representation*max/32767;
    }
    operator float64() const{
        return (float64)representation*max/32767;
    }
};

template<int max>
class fixed_point<max, false, true>{
    uint16_t representation;
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

template<int max>
class fixed_point<max, true, false>{
    int8_t representation;
public:
    fixed_point(float a){
        representation=a/max*127;
    }
    fixed_point(){
        representation=0;
    }
    operator float32() const{
        return (float32)representation*max/127;
    }
    operator float64() const{
        return (float64)representation*max/127;
    }
};

template<int max>
class fixed_point<max, false, false>{
    uint8_t representation;
public:
    fixed_point(float a){
        representation=a/max*255;
    }
    fixed_point(){
        representation=0;
    }
    operator float32() const{
        return (float32)representation*max/255;
    }
    operator float64() const{
        return (float64)representation*max/255;
    }
};

//fits 3 fixed point numbers into 32 bits. Distance is stored in a 10 bit number from 0-2, then the two angle are stored in an 11 bit number from 0-7 (0-2pi)
class CombinedDistanceData{
  uint32_t data;
  static constexpr uint32_t elevenBitMask=0x7FF;
  static constexpr uint32_t tenBitMask=0x3FF;
public:
  CombinedDistanceData(): data(0) {}
  CombinedDistanceData(float dist, float ang, float targetFacingAng){
    setDistance(dist);
    setAngle(ang);
    setTargetFacingAngle(targetFacingAng);
  }
  float64 distance() const{
    return float64(tenBitMask & data>>22)/1023*2;
  }
  float64 angle() const{
    return float64(elevenBitMask & data>>11)/2047*7;
  }
  float64 targetFacingAngle() const{
    return float64(elevenBitMask & data)/2047*7;
  }
  void setDistance(float64 distF){
    uint16_t dist=distF/2*1023;
    data=(data & ~(tenBitMask<<22)) | (dist & tenBitMask)<<22;
  }
  void setAngle(float64 angF){
    uint16_t ang=angF/7*2047;
    data=(data & ~(elevenBitMask<<11)) | (ang & elevenBitMask)<<11;
  }
  void setTargetFacingAngle(float64 angF){
    uint16_t ang=angF/7*2047;
    data=(data & ~elevenBitMask) | (ang & elevenBitMask);
  }
};

//id calculation
constexpr uint16_t MAX_CAN_ID=0x7FF;

enum {
  synchronised_time_priority=0,
  cmd_vel_priority,
  odom_odrive_priority,
  odom_mouse_priority,
  light_sensor_priority,
  global_state_priority,
  baby_found_priority,
  candle_found_priority,
  move_stat_baby_priority,
  move_stat_candle_priority,
  pickup_stat_baby_priority,
  ext_stat_candle_priority,
  set_handle_priority,
  microphone_on_priority,
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

  flipped<fixed_point<2, true>> vX;
  flipped<fixed_point<14, false>> vTheta;
};

struct odom_struct{
  static constexpr uint8_t size=4;

  flipped<fixed_point<1, true>> dX;
  flipped<fixed_point<1, true>> dY;
  flipped<fixed_point<7, false>> dTheta;
};

struct global_state_struct{
  static constexpr uint16_t id=idProvider(global_state_priority);
  static constexpr uint8_t size=1;

  flipped<uint8_t> globalState;
};

struct baby_found_struct{
  static constexpr uint16_t id=idProvider(baby_found_priority);
  static constexpr uint8_t size=8;

  //this gets us 0.35cm resolution, probably better than what we're measuring anyway
  flipped<CombinedDistanceData> combinedDistanceData;
  timestamp ts;
  flipped<uint8_t> rotation;
};
struct candle_found_struct{
  static constexpr uint16_t id=idProvider(candle_found_priority);
  static constexpr uint8_t size=8;

  flipped<CombinedDistanceData> combinedDistanceData;
  timestamp ts;
  flipped<uint8_t> rotation;
  flipped<uint8_t> certainty;
};

struct move_stat_baby_struct{
  static constexpr uint16_t id=idProvider(move_stat_baby_priority);
  static constexpr uint8_t size=1;

  flipped<uint8_t> response;
};
struct move_stat_candle_struct{
  static constexpr uint16_t id=idProvider(move_stat_candle_priority);
  static constexpr uint8_t size=1;

  flipped<uint8_t> response;
};

struct pickup_stat_baby_struct{
  static constexpr uint16_t id=idProvider(pickup_stat_baby_priority);
  static constexpr uint8_t size=1;

  flipped<uint8_t> update;
};
struct ext_stat_candle_struct{
  static constexpr uint16_t id=idProvider(ext_stat_candle_priority);
  static constexpr uint8_t size=1;

  flipped<uint8_t> update;
};

struct set_handle_struct{
  static constexpr uint16_t id=idProvider(set_handle_priority);
  static constexpr uint8_t size=1;

  flipped<uint8_t> ledBitSet;
};

struct microphone_on_struct{
  static constexpr uint16_t id=idProvider(microphone_on_priority);
  static constexpr uint8_t size=1;

  flipped<uint8_t> unused;
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
