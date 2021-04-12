#!/bin/bash

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]
then
  echo "This script attempts to modify the current environment.  It must be sourced, not run"
  exit 1
fi

# help
if [ "$1" = "--help" ] || [ -z "$1" ] || [[ "$1" == "enable" && -z "$2" ]] || [[ "$1" != "disable" && "$1" != "enable" ]]
then
#  ; then
  echo "Usage: setup_security <enable/disable> <root_directory_to_keys>"
  echo "directory to keys is required when security is enabled"
  echo "Example usage: "
  echo "To enable the security: "
  echo "bash setup_security.sh enable '/ApexOS/apex_ws/demo_keys'"
  echo "To disable the security: "
  echo "bash setup_security.sh disable"
  return 0
fi

# Make soure a ros2 setup.bash has been sourced, otherwise all of the ros2 commands below aren't going to work
if ! ros2 --help &> /dev/null;
then
    echo "ros2 command not found.  Did you forget to source setup.bash before sourcing this script?"
    return 1
fi

if [ "$1" == "enable" ]
then 
  echo 'generating keys'

  max_nodes=5

  # create keys
  if ! ros2 security create_keystore "$2"
  then
    return 1
  fi

  echo "generating keys for /performance_test"
  ros2 security create_key "$2" /performance_test
  ros2 security create_permission "$2" /performance_test policies/policy.xml;  
  echo " "

  #enable security
  echo "Setting environment variables to enable security"
  export ROS_SECURITY_ENABLE=true
  export ROS_SECURITY_STRATEGY=Enforce
  export ROS_SECURITY_KEYSTORE=$2
elif [ "$1" == "disable" ]
then 
  #disable security
  echo "Unsetting environment variables to disable security"
  unset ROS_SECURITY_ENABLE
  unset ROS_SECURITY_STRATEGY
  unset ROS_SECURITY_ROOT_DIRECTORY
else 
  echo 'incorrect parameter '$2
fi
