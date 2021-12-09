#include "ros/ros.h"

#include "performance_test_ros1_publisher/msg_types.hpp"

#include <tclap/CmdLine.h>

#include <memory>
#include <tuple>
#include <utility>

/// Iterates over std::tuple types
/**
 * \param t tuple to be iterated
 * \param f functor to be applied over each element
 * \returns number of iterated elements
 */
template<std::size_t I = 0, typename FuncT, typename ... Tp>
typename std::enable_if<I == sizeof...(Tp), size_t>::type
for_each(const std::tuple<Tp...> & t, FuncT f)
{
  (void)t;
  (void)f;
  return I;
}

template<std::size_t I = 0, typename FuncT, typename ... Tp>
typename std::enable_if<I != sizeof...(Tp), size_t>::type
for_each(const std::tuple<Tp...> & t, FuncT f)
{
  f(std::get<I>(t));
  return for_each<I + 1, FuncT, Tp...>(t, f);
}

std::shared_ptr<MsgBase> msg_publisher_factory(ros::NodeHandle& nh, std::string topic_name)
{
  std::shared_ptr<MsgBase> data = nullptr;

  for_each(TopicTypeList(),
    [topic_name, &nh, &data](const auto & topic) {
      using T = std::remove_cv_t<std::remove_reference_t<decltype(topic)>>;
      if (std::string(topic.name()) == topic_name) {
        data = std::make_shared<T>();
        data->pub = nh.advertise<typename T::RosType>(topic_name, 100);
      }
    }
  );

  if (data == nullptr)
    throw std::invalid_argument("Topic not found");
  return data;
}

int main(int argc, char**argv)
{
  std::string msg_type;
  uint32_t rate;

  try {
    TCLAP::CmdLine cmd("Apex.AI performance_test_ros1_publisher");
    cmd.ignoreUnmatched(true);

    TCLAP::ValueArg<std::string> msgArg("m", "msg", "The message type.",
      false, "Array16k", "type", cmd);
    
    TCLAP::ValueArg<uint32_t> rateArg("r", "rate",
      "The publishing rate. 0 means publish as fast as possible.", false, 1000, "N", cmd);

    cmd.parse(argc, argv);

    msg_type = msgArg.getValue();
    rate = rateArg.getValue();
  } catch (TCLAP::ArgException & e) {
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
  }

  std::cout << "ROS1 publisher running with rate " << rate << " with message " << msg_type << std::endl;
  ros::init(argc, argv, "point_cloud_publisher");
  ros::NodeHandle n;
  auto msg_publisher = msg_publisher_factory(n, msg_type);

  ros::Rate loop_rate(rate);

  uint64_t id = 0;

  while (ros::ok())
  { 
    msg_publisher->publish();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
