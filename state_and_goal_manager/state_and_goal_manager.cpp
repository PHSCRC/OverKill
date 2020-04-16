#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class AbsPoseDifference{
  float64 dx;
  float64 dy;
  float64 dz;
  float64 angleDiff;
  AbsPoseDifference(const geometry_msgs::PoseStamped& a, const geometry_msgs::PoseStamped& b){
    dx = std::abs(a.pose.position.x-b.pose.position.x);
    dy = std::abs(a.pose.position.y-b.pose.position.y);
    dz = std::abs(a.pose.position.z-b.pose.position.z);

    const geometry_msgs::Quaternion& aQuat=a.pose.orientation;
    const geometry_msgs::Quaternion& bQuat=b.pose.orientation;
    angleDiff = 2 * std::cos(aQuat.w*bQuat.w + aQuat.z*bQuat.z + aQuat.y*bQuat.y + aQuat.x*bQuat.x);
  }

  AbsPoseDifference(const geometry_msgs::TransformStamped& a, const geometry_msgs::TransformStamped& b){
    dx = std::abs(a.transform.translation.x-b.transform.translation.x);
    dy = std::abs(a.transform.translation.y-b.transform.translation.y);
    dz = std::abs(a.transform.translation.z-b.transform.translation.z);

    const geometry_msgs::Quaternion& aQuat=a.transform.rotation;
    const geometry_msgs::Quaternion& bQuat=b.transform.rotation;
    angleDiff = 2 * std::cos(aQuat.w*bQuat.w + aQuat.z*bQuat.z + aQuat.y*bQuat.y + aQuat.x*bQuat.x);
  }

  float64 dist(){
    return std::sqrt(dx*dx + dy*dy + dz*dz)
  }
  float64 distSquared(){
    return dx*dx + dy*dy + dz*dz;
  }
  float64 absAngle(){
    return angleDiff;
  }
};


struct State{
  MoveBaseClient mbac("move_base", false);
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  tf2_ros::TransformBroadcaster tfBroadcaster;

  ros::Publisher baby_move_stat_pubber;
  ros::Publisher candle_move_stat_pubber;
  ros::Publisher maze_publisher;



  ros::Nodehandle& nh;



  State(ros::Nodehandle& n){
    nh=n;
    baby_move_stat_pubber=nh.adverstise<std_msgs::Byte>("baby_move_stat", 10);
    candle_move_stat_pubber=nh.adverstise<std_msgs::Byte>("candle_move_stat", 10);
    maze_publisher=nh.adverstise<std_msgs::Byte>("maze_number",10);
  }

  const bool rampedHallway=false;
  const geometry_msgs::Quaternion facingUp={0,0,0,1};
  const geometry_msgs::Quaternion facingDown={0,0,1,0};
  const geometry_msgs::Quaternion facingSixty={0,0,0.5,0.866025404};

  const geometry_msgs::Pose maze1RampStart={{1.22,2.45,0}, facingUp};
  //the bottom of the static_map
  const geometry_msgs::Pose maze2RampStart={{1.22,0,0}, facingDown};

x

  //TODO all of these things
  //30 back
  const geometry_msgs::TransformStamped goalPoseFromBaby={{0, ros::Time(0), "map"}, "map",
    {{-0.3, 0, 0}, geometry_msgs::Quaternion()}
  };
  //20 back
  const geometry_msgs::TransformStamped pickupPoseFromBaby={{0, ros::Time(0), "map"}, "map",
    {{-0.2, 0, 0}, geometry_msgs::Quaternion()}
  };

  //20cm back
  const geometry_msgs::TransformStamped extinguishPoseFromCandle={{0, ros::Time(0), "map"}, "map",
   {{-0.2, 0, 0}, geometry_msgs::Quaternion()}
  };


  //this doesn't change between mazes TODO add these
  //double braces because cpp stupidity
  const std::array<geometry_msgs::Pose, 4> roomSearchLocations={{
    {{}, },
    {{}, },
    {{}, },
    {{}, }
  }};

  //messages that need to be stored
  sensor_msgs::LaserScan latest_rawScan;
  bool lightSensor;
  GlobalState globalState; //0= starting up; 1=waiting for sound //2=driving to maze 2 //3=looking for baby //4=picked up baby //5=dropped baby //6=searching for candle in maze 2 //7=extinguished candle 1 //8=searching for candle in maze 1 //9=extinguishedCandle 2 //10=extinguished candle 3 (done)
  geometry_msgs::PoseStamped latestBabyGoalPose;
  geometry_msgs::PoseStamped latestBabyPickupPose;
  geometry_msgs::PoseStamped latestCandleGoalPose;

  int whichMazeAreWeIn=1;

  bool haveSeenCandle=false;
  geometry_msgs::PoseStamped possibleCandleLocation;

  int lastRoomSearchedBeforeReturn;

  //what goals have we completed?
  bool gottenBaby=false;
  bool placedBabyAtStart=false;
  bool extinguishedFirstCandle=false;


  bool publishStaticMapToMap1=false;
  geometry_msgs::TransformStamped map_to_staticMap1;
  bool publishStaticMapToMap2=false;
  geometry_msgs::TransformStamped map_to_staticMap2;

  void temporaryMapToStaticMapBroadcaster(const ros::TimerEvent& event){
    if(publishStaticMapToMap1){
      geometry_msgs::TransformStamped odom_to_staticMap1;
      map_to_staticMap1.header.stamp=ros::Time::now();
      this->tfBuffer->transform(map_to_staticMap1, odom_to_staticMap1, "odom");
      tfBroadcaster.sendTransform(odom_to_staticMap1);
    }
    if(publishStaticMapToMap2){
      geometry_msgs::TransformStamped odom_to_staticMap2;
      map_to_staticMap2.header.stamp=ros::Time::now();
      this->tfBuffer->transform(map_to_staticMap2, odom_to_staticMap2, "odom");
      tfBroadcaster.sendTransform(odom_to_staticMap2);
    }
  }

  void rawLaserScanCallback(const sensor_msgs::LaserScanConstPtr& scan){
    latest_rawScan=scan;
  }

  void globalStateCallback(const std_msgs::Byte* state){
    globalState=*state;
  }
  void lightSensorCallback(const overkill_msgs::BoolStamped* onOff){
    lightSensor=*onOff;
  }
  void candleCallback(const geometry_msgs::PoseStamped* candle_msg){
    tf2_ros::doTransform(*candle_msg, latestCandleGoalPose, extinguishPoseFromCandle);
    latestCandleGoalPose.header.stamp=candle_msg.header.stamp;
  }
  void babyCallback(const geometry_msgs::PoseStamped* baby_msg){
    tf2_ros::doTransform(*baby_msg, latestBabyGoalPose, goalPoseFromBaby);
    latestBabyGoalPose.header.stamp=baby_msg.header.stamp; //the time

    tf2_ros::doTransform(*baby_msg, latestBabyGoalPose, pickupPoseFromBaby);
    latestBabyGoalPose.header.stamp=baby_msg.header.stamp;
  }
};

inline void wait(const State& state, const ros::Duration time){
  ros::Time start=ros::Time::now()
  while(ros::ok() && state.notStopping && ros::Time::now()-start<time){
    ros::spinOnce();
  }
}
inline void waitForDestination(const State& state, const ros::Duration timeBetweenSpins){
  while(ros::ok() && state.notStopping && !state.mbac.waitForResult(timeBetweenSpins)){
    ros::spinOnce();
  }
}
inline void waitForGlobalState(State& state, GlobalState whatAreWeWaitingFor){
  while(ros::ok() && state.notStopping && state.globalState!=whatAreWeWaitingFor){
    ros::spinOnce();
  }
}

inline void stopMoving(State& state){
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "base_link";
  state.mbac.sendGoal(goal);
}

inline void moveToMovingTarget(State& state, geometry_msgs::PoseStamped* target_pose){
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose=*target_pose;
  state.mbac.sendGoal(goal);
  while(ros::ok() && state.notStopping && !state.mbac.waitForResult(ros::Duration(0.1))){
    AbsPoseDifference diff(goal.target_pose, *target_pose);
    if(diff.absAngle()>0.0872664626 || diff.distSquared()>(0.05*0.05)){
      goal.target_pose=*target_pose;
      state.mbac.sendGoal(goal);
    }
    ros::spinOnce();
  }
  wait(state, ros::Duration(0.5));
  //do it twice just to make sure that it's settled
  while(ros::ok() && state.notStopping && !state.mbac.waitForResult(ros::Duration(0.1))){
    AbsPoseDifference diff(goal.target_pose, *target_pose)
    if(diff.absAngle()>0.0872664626 || diff.distSquared()>(0.05*0.05)){
      goal.target_pose=*target_pose;
      state.mbac.sendGoal(goal);
    }
    ros::spinOnce();
  }
}


inline void moveToAndOverRamp(State& state, uint8 fromMaze){
  move_base_msgs::MoveBaseGoal goal;
  //this goal is sent in the static map stamp. This means that as the tf between static_map and base_link changes due to amcl,
  //the actually position of this goal will change automatically. Amcl will automatically send it to its best guess as to where the goal is
  goal.target_pose.header.frame_id = "static_map";
  if(fromMaze==1){
    goal.target_pose.pose=maze1RampStart;
  } else {
    goal.target_pose.pose=maze2RampStart;
  }

  ros::Time now=ros::Time::now();
  geometry_msgs::TransformStamped lastUsedStaticMapToMap = tfBuffer.lookupTransform("static_map", "map", now);
  goal.target_pose.header.stamp = now;
  state.mbac.sendGoal(goal);

  while(ros::ok() && state.notStopping && !state.mbac.waitForResult(ros::Duration(0.1))){
    now=ros::Time::now();
    geometry_msgs::TransformStamped latestStaticMapToMap=tfBuffer.lookupTransform("static_map", "map", now);
    AbsPoseDifference diff(lastUsedStaticMapToMap, latestStaticMapToMap);
    if(diff.absAngle()>0.034906585 || diff.distSquared()>(0.05*0.05){
      goal.target_pose.header.stamp=now;
      lastUsedStaticMapToMap=latestStaticMapToMap;
      state.mbac.sendGoal(goal);
    }
    ros::spinOnce();
  }


  if(fromMaze==1){
    state.map_to_staticMap1=state.tfBuffer.lookupTransform("map", "static_map", ros::Time(0));
    state.publishStaticMapToMap1=true;
  } else {
    state.map_to_staticMap2=state.tfBuffer.lookupTransform("map", "static_map", ros::Time(0));
    state.publishStaticMapToMap2=true;
  }
  //amcl will now screw up if we continue, so we disable it until we come back
  ros::service::call("disable_amcl");
  //laser_filter deals with disabling the front and back of the laser, allowing us to move through, so gmapping just follows odom for forward back and the nav stack prevents us from crashing into walls.
  //until it gets to the top, where laser_filter will automatically turn the full laser back on, allowing gmapping to get a better picture of what is happening.
  //then laser_filter will disable the front and back of the laser again as it goes down, again trusting odom for distance
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "static_map";
  goal.target_pose.header.stamp = ros::Time::now();
  if(fromMaze==1) {
    goal.target_pose.pose=maze2RampStart;
    goal.target_pose.pose.orientation=facingUp;
  } else {
    goal.target_pose.pose=maze1RampStart;
    goal.target_pose.pose.orientation=facingDown;
  }
  state.mbac.sendGoal(goal);
  waitForDestination(state);

  //now that we're on the other side, we switch what maze we're in and reenable AMCL's localization, giving a global initialization cause we don't know where we are once again.
  //if it's 1, it goes to 2
  //if it's 2, it goes to 1
  state.publishStaticMapToMap1=false;
  state.publishStaticMapToMap2=false;
  state.whichMazeAreWeIn=state.whichMazeAreWeIn%2+1;
  ros::service::call("enable_amcl");
  ros::service::call("global_localization");
  state.maze_publisher.publish(state.whichMazeAreWeIn);
  //give amcl some time to actually have a decent estimate
  wait(state, ros::Duration(1));
}

inline bool sweep(State& state, bool stopOnBaby, bool stopOnCandle){
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "base_link";
  for(int i=0; i<6; ++i){
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.orientation=facingSixty;
    waitForDestination(state);
    wait(0.3); //wait for the camera to figure it out and for us to receive it
    if(stopOnBaby && (ros::Time::now()-state.latestBabyGoalPose.header.stamp).toSec()<0.5){
      return true;
    }
    if(stopOnCandle && (ros::Time::now()-state.latestBabyGoalPose.header.stamp).toSec()<0.5){
      return true;
    }
  }
  return false;
}

inline void searchForBabyAndCandle(State& state){
  //it doesn't matter which order I go in. I'm kinda dumb
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "static_map";
  for(int i=0; i<roomSearchLocations.size(); ++i){
    goal.target_pose.pose=roomSearchLocations[i];
    goal.target_pose.header.stamp = ros::Time::now();
    state.mbac.sendGoal(goal);
    //callbacks should automatically add things to possibleCandleLocation and take care of foundBaby
    while(ros::ok() && state.notStopping && !(ros::Time::now()-state.latestBabyGoalPose.header.stamp).toSec()<0.5 && !state.mbac.waitForResult(ros::Duration(0.05))){
      ros::spinOnce();
    }
    stopMoving(state);
    if((ros::Time::now()-state.latestBabyGoalPose.header.stamp).toSec()<0.5){
      state.lastRoomSearchedBeforeReturn=i;
      return;
    }
    if(sweep(state, true, false)){
      state.lastRoomSearchedBeforeReturn=i;
      return;
    }
  }
}

inline void pickUpBaby(State& state){

  moveToMovingTarget(state, &state.latestBabyGoalPose);

  move_base_msgs::MoveBaseGoal goal;

  //let it settle once again
  wait(state, ros::Duration(0.5));

  goal.target_pose=state.latestBabyPickupPose;

  //tell them we're in position, we no longer want updates about the baby from here on out
  state.baby_move_stat_pubber.publish(1);

  state.mbac.sendGoal(goal);
  while(ros::ok() && state.notStopping && !state.mbac.waitForResult(ros::Duration(0.1))){
    ros::spinOnce();
  }

  //tell them to actually pick it up
  state.baby_move_stat_pubber.publish(2);

  //this should automatically update if we get a pickup_Stat_baby message
  waitForGlobalState(state, PICKED_UP_BABY);
  state.gottenBaby=true;
}

inline void getBaby(State& state){
  moveToAndOverRamp(state, 1);
  searchForBabyAndCandle(state);
  pickUpBaby(state);
}

inline void moveToStart(State& state){
  move_base_msgs::MoveBaseGoal goal;
  //we're going to 0,0 of the map frame
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp=ros::Time:now();
  state.mbac.sendGoal(goal);
  waitForDestination(state);
}

inline void dropBabyOnStartCircle(State& state){
  float32 maxReadingDirection=0;
  float32 maxReading=0;

  for(int i=0; i<state.latest_rawScan.ranges.size(); ++i){
    if(state.latest_rawScan.ranges.reading[i]>state.latest_rawScan.range_max || state.latest_rawScan.ranges.reading[i]<state.latest_rawScan.range_min){
      continue;
    }
    if(state.latest_rawScan.reading[i]>maxReading){
      maxReading=state.latest_rawScan.reading[i];
      maxReadingDirection=i*angle_increment+angle_min;
    }
  }

  move_base_msgs::MoveBaseGoal goal;
  //we face 180 degrees from the max direction, just to make sure we have space to back up
  goal.target_pose.header.frame_id = "base_laser";
  goal.target_pose.header.stamp=ros::Time::now();

  geometry_msgs::Quaternion targetDirection;
  targetDirection.setRPY(0, 0, targetDirection + 3.14);

  goal.target_pose.orientation=targetDirection;

  state.mbac.sendGoal(goal);
  waitForDestination(state);

  goal.target_pose.header.frame_id = "base_link";
  while(lightSensor){
    goal.target_pose.header.stamp=ros::Time::now();
    goal.target_pose.pose.position.x=-0.05;
    state.mbac.sendGoal(goal);
    while(ros::ok() && state.notStopping && !state.mbac.waitForResult(ros::Duration(0.05))){
      ros::spinOnce();
    }
    wait(state, ros::Duration(0.1));
  }
  //this is a long-ish call, which is against guidelines, I'm not about to make a whole ass action
  ros::service::call("drop_baby");
  //we're quite behind, so this could take a while
  ros::spinOnce();
}

inline returnBaby(State& state){
  moveToAndOverRamp(state, 2);
  moveToStart(state);
  dropBabyOnStartCircle(state);
}

inline searchForCandle(State& state, int maze){
  move_base_msgs::MoveBaseGoal goal;
  if(maze==2 && haveSeenCandle){
    goal.target_pose=possibleCandleLocation;
    state.mbac.sendGoal(goal);
    waitForDestination(state);
    if(sweep(state, false, true)){
      return;
    }
  }
  int i=state.lastRoomSearchedBeforeReturn;
  do{
    goal.target_pose.pose=roomSearchLocations[i];
    goal.target_pose.header.stamp = ros::Time::now();
    state.mbac.sendGoal(goal);
    while(ros::ok() && state.notStopping && !(ros::Time::now()-state.latestCandleGoalPose.header.stamp).toSec()<0.5 && !state.mbac.waitForResult(ros::Duration(0.05))){
      ros::spinOnce();
    }
    stopMoving(state);
    if((ros::Time::now()-state.latestCandleGoalPose.header.stamp).toSec()<0.5){
      ros::spinOnce();
      state.lastRoomSearchedBeforeReturn=i;
      return;
    }
    if(sweep(state, false, true)){
      state.lastRoomSearchedBeforeReturn=i;
      return;
    }
    ++i;
  } while (i!=state.lastRoomSearchedBeforeReturn){
}

inline void extinguishCandle(State& state){
  moveToMovingTarget(state, &state.latestCandleGoalPose);
  state.candle_move_stat_pubber.publish(1);
  waitForGlobalState(state, DROPPED_BABY);
}

inline searchAndExtinguishCandles(State& state){
  moveToAndOverRamp(state, 1);
  searchForCandle(state, 2);
  extinguishCandle(state);

  moveToAndOverRamp(state, 2);
  state.lastRoomSearchedBeforeReturn=0;
  searchForCandle(state, 1);
  extinguishCandle(state);
  searchForCandle(state, 1);
  extinguishCandle(state);
}


inline void doEverything(State& state){
  getBaby(state);
  returnBaby(state);
  searchAndExtinguishCandles(state);
}

//if none of the children can go back to it, inline
inline void startup(State& state){
  while(ros::ok() && state.notStopping && !mbac.waitForServer(ros::Duration(0.1))){
    ros::spinOnce();
  }

  ros::service::call("global_localization");

  //wait for the sound
  waitForGlobalState(state, DRIVING_TO_MAZE_2);
  doEverything();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  Nodehandle nh;

  State state(nh);

  ros::Subscriber laserSub = nh.subscribe("raw_laser", 1, &State::rawLaserCallback, &state);
  ros::Subscriber globalStateSub = nh.subscribe("global_state", 5, &State::globalStateCallback, &state);
  ros::Subscriber lightSensorSub = nh.subscribe("light_sensor", 3, &State::lightSensorCallback, &state);
  ros::Subscriber candlePositionSub = nh.subscribe("candle_position", 3, &State::candleCallback, &state);
  ros::Subscriber babyPositionSub = nh.subscribe("baby_position", 3, &State::babyCallback, &state);

  //publish all the static transforms once a second
  nh.createTimer(ros::Duration(1), &State::temporaryMapToStaticMapBroadcaster, &state);
  startup(state)

  move_base_msgs::MoveBaseGoal goal;
  return 0;
}
