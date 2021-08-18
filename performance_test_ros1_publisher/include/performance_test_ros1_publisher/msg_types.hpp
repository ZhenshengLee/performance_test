#ifndef PERFORMANCE_TEST_ROS1_PUBLISHER_MSG_TYPES_HPP_INCLUDED
#define PERFORMANCE_TEST_ROS1_PUBLISHER_MSG_TYPES_HPP_INCLUDED

#include <chrono>
#include <memory>
#include <tuple>

#include "performance_test_ros1_msgs/Array16k.h"
#include "performance_test_ros1_msgs/Array1k.h"
#include "performance_test_ros1_msgs/Array1m.h"
#include "performance_test_ros1_msgs/Array2m.h"
#include "performance_test_ros1_msgs/Array32k.h"
#include "performance_test_ros1_msgs/Array4k.h"
#include "performance_test_ros1_msgs/Array4m.h"
#include "performance_test_ros1_msgs/Array60k.h"
#include "performance_test_ros1_msgs/Array64k.h"
#include "performance_test_ros1_msgs/Array256k.h"
#include "performance_test_ros1_msgs/NavSatFix.h"
#include "performance_test_ros1_msgs/NavSatStatus.h"
#include "performance_test_ros1_msgs/Point.h"
#include "performance_test_ros1_msgs/Point32.h"
#include "performance_test_ros1_msgs/PointCloud1m.h"
#include "performance_test_ros1_msgs/PointCloud2m.h"
#include "performance_test_ros1_msgs/PointCloud4m.h"
#include "performance_test_ros1_msgs/PointCloud512k.h"
#include "performance_test_ros1_msgs/PointCloud8m.h"
#include "performance_test_ros1_msgs/Polygon.h"
#include "performance_test_ros1_msgs/RadarDetection.h"
#include "performance_test_ros1_msgs/RadarTrack.h"
#include "performance_test_ros1_msgs/Range.h"
#include "performance_test_ros1_msgs/Struct16.h"
#include "performance_test_ros1_msgs/Struct256.h"
#include "performance_test_ros1_msgs/Struct32k.h"
#include "performance_test_ros1_msgs/Struct4k.h"
#include "performance_test_ros1_msgs/Vector3.h"

struct MsgBase
{
  ros::Publisher pub;
  virtual void publish() = 0;
};

template<class T>
struct Msg : public MsgBase
{
  std::shared_ptr<T> data;

  Msg<T>(): data(std::make_shared<T>()) {
    data->id = 0;
  }

  void publish() override
  {
    data->id = ++data->id;
    data->time = std::chrono::steady_clock::now().time_since_epoch().count();
    pub.publish(*data);
  }

  using RosType = T;
};

struct Array16k: public Msg<performance_test_ros1_msgs::Array16k>
{
  static const char* name() { return "Array16k"; }
};

struct Array2m: public Msg<performance_test_ros1_msgs::Array2m>
{
  static const char* name() { return "Array2m"; }
};

struct Struct32k: public Msg<performance_test_ros1_msgs::Struct32k>
{
  static const char* name() { return "Struct32k"; }
};

struct PointCloud1m: public Msg<performance_test_ros1_msgs::PointCloud1m>
{
  static const char* name() { return "PointCloud1m"; }
};

struct PointCloud4m: public Msg<performance_test_ros1_msgs::PointCloud4m>
{
  static const char* name() { return "PointCloud4m"; }
};

using TopicTypeList = std::tuple<Array16k, Array2m, Struct32k, PointCloud1m, PointCloud4m>;

#endif
