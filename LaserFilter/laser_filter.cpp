//this mostly eliminates the ramp from the view of the laser, so that gmapping and amcl don't get all flustered about the weird wall in the way.
//the laser basically gets no view of inside the ramp from the outside, and no view outside the ramp from the inside. Its still gets the walls when inside, to help it avoid those
class LaserFilter{
  Nodehandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  //we also need a tf listerner but im too lazy to set that up right now. to update the robotPositions
  ros::Subscriber subLaser = nh.subscribe("raw_laser", 1000, rawLaserCallback);
  ros::Publisher scanPub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);

  //OVERKILL_TODO create this
  std::unordered_map<std::size_t, bool> isSafe = loadSafeFromDisk();

  void raw_laser_callback(sensor_msgs::LaserScan* rawLaserScan){
    geometry_msgs::TransformStamped robotPose=tfBuffer.lookupTransform("map", "base_laser", ros::Time(0));

    float robotX = robotPose.transform.translation.x;
    float robotY = robotPose.transform.translation.y;

    float robotTheta = 2 * std::cos(robotPose.transform.rotation.w);

    sensor_msgs::LaserScan filteredLaserScan=*rawLaserScan;
    std::size_t cellRobotX=uint8_t(robotX/0.05);
    std::size_t cellRobotY=uint8_t(robotY/0.05);
    float currentTheta=robotTheta;
    for(std::size_t i=0; i<filteredLaserScan->size(); ++i){
      currentTheta+=filteredLaserScan.angle_increment;
      //range 0-50 (6 bits)   //range 0-120 (7 bits)    //range 0-256
      if(!isSafe[cellRobotX | cellRobotY<<6           | (std::size_t(robotTheta/0.0245436926]))<<13]){
        //set it to 0 so that amcl doesn't overthink it
        filteredLaserScan[i]=0;
      }
    }
    scanPub.publish(filteredLaserScan);
  }

};

int main(int argc, char** argv){
  ros::init(argc, argv, "laser_filter");
  LaserFilter laserFilter();
  ros::spin();
}
