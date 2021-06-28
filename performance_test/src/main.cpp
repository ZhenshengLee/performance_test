// Copyright 2017 Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>

#include "experiment_configuration/experiment_configuration.hpp"
#include "experiment_execution/analyze_runner.hpp"
#include "communication_abstractions/resource_manager.hpp"

int main(int argc, char ** argv)
{
  auto & ec = performance_test::ExperimentConfiguration::get();
  ec.setup(argc, argv);

  if (ec.use_ros2_layers()) {
#ifdef APEX_CERT
    rclcpp::init(argc, argv, rclcpp::InitOptions{}, false);
#else
    rclcpp::init(argc, argv);
#endif
  }

  {
    performance_test::AnalyzeRunner ar;
    ar.run();
  }

  performance_test::ResourceManager::shutdown();
}
