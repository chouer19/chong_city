/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>

/////////////////////////////////////////////////
// Function is called everytime a message is received.
void cb(ConstContactsPtr &_msg)
{
  // Dump the message contents to stdout.
  //std::cout << _msg->DebugString();
  std::cout << _msg->contact_size() << std::endl;
  for(int i = 0; i < _msg->contact_size(); i++){
      if(_msg->contact(i).has_collision1()){
          std::cout << i << " th collision1 : " << _msg->contact(i).collision1() << std::endl;;
      }
      if(_msg->contact(i).has_collision2()){
          std::cout << i << " th collision2 : " << _msg->contact(i).collision2() << std::endl;
      }
  }
  std::cout << "============================\n";
}

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo world_stats topic
  gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/physics/contacts", cb);
    // Get the message type on the topic.

  // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
