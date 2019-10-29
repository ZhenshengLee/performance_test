#include "ros/ros.h"

#include "performance_test_ros1_publisher/msg_types.hpp"

#include <boost/function.hpp>
#include <boost/mpl/for_each.hpp>
#include <boost/program_options.hpp>

#include <memory>

std::shared_ptr<MsgBase> msg_publisher_factory(ros::NodeHandle& nh, std::string topic_name)
{
  std::shared_ptr<MsgBase> data = nullptr;

  boost::mpl::for_each<TopicTypeList>(
    [topic_name, &nh, &data](auto topic) {
      using T = decltype(topic);
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
  namespace po = boost::program_options; 
  po::options_description desc("Allowed options");
  desc.add_options()
    ("rate,r", po::value<uint32_t>()->default_value(1000), "Publish rate in Hz. Defaults to 1000. 0 = as fast as possible.")
    ("topic,t", po::value<std::string>()->required(), "Topic to use.");

  po::variables_map vm;
  try {
    // Allow unused arguments so we can pass the same command-line used for perf_test, but
    // just ignore the parts that don't affect the publisher
    po::store(
      po::command_line_parser(argc, argv).options(desc)
                                         .allow_unregistered()
                                         .run(),
      vm
    );  // can throw
 
    /** --help option 
     */ 
    if ( vm.count("help")  ) 
    { 
      std::cout << "Basic Command Line Parameter App" << std::endl 
                << desc << std::endl; 
      return 0; 
    } 

    if (!vm.count("topic")) {
      throw std::invalid_argument("--topic is required!");
    }
  }
  catch(po::error& e) 
  { 
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl; 
    std::cerr << desc << std::endl; 
    return 1; 
  }

  auto topic_name = vm["topic"].as<std::string>();
  auto rate = vm["rate"].as<uint32_t>();

  std::cout << "ROS1 publisher running with rate " << rate << " on topic " << topic_name << std::endl;
  ros::init(argc, argv, "point_cloud_publisher");
  ros::NodeHandle n;
  auto msg_publisher = msg_publisher_factory(n, topic_name);

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
