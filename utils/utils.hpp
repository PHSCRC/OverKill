void kernelTimeSync(uint32_t& origin_epoch_s, uint32_t& origin_epoch_ns, ros::Time& origin_ros){
  timeval * tv;
  origin_ros=ros::Time::now();-
  gettimeofday(origin_tv);
  origin_epoch_s = tv->tv_sec;
  origin_epoch_ns = tv->tv_usec*1000;
}

ros::Time kernelTimeToTime(timeval * tv){
    static uint32_t origin_epoch_s;
    static uint32_t origin_epoch_ns;
    static ros::Time origin_ros;

    std::call_once(kernelTimeSync, origin_epoch_s, origin_epoch_us, origin_ros);

    return origin_ros-ros::Duration(tv->tv_sec-origin_epoch_s, tv->tv_usec*1000-origin_epoch_ns);
}
