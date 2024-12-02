/**
Software License Agreement (BSD)

\authors   Mike Purvis <mpurvis@clearpathrobotics.com>
\copyright Copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WAR- RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, IN- DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>
#include <socketcan_interface/make_shared.h>
#include <string>

#include "puma_motor_driver/socketcan_gateway.h"

namespace puma_motor_driver {

bool printMsgIfRelevant(Message *msg, int device_number, bool relevant) {
  relevant &= (msg->getApi() & CAN_MSGID_API_M & CAN_API_MC_STATUS) ==
              CAN_API_MC_STATUS; // test if status message
  auto get_field_idx = [](const Message &msg) {
    return ((msg.getApi() & CAN_MSGID_API_ID_M) >> CAN_MSGID_API_S);
  };
  relevant &= get_field_idx(*msg) == get_field_idx(Message(LM_API_STATUS_SPD));
  relevant &= msg->getDeviceNumber() == device_number;
  if (relevant) {
    std::string out_string;
    for (int i = 0; i < int(msg->len); i++) {
      out_string += std::to_string(msg->data[i]);
    }
    std::cout << "Message data: " << std::endl;
    std::cout << out_string << std::endl;
  }
  return relevant;
}

SocketCANGateway::SocketCANGateway(const std::string& canbus_dev):
  canbus_dev_(canbus_dev),
  is_connected_(false),
  can_driver_(ROSCANOPEN_MAKE_SHARED<can::ThreadedSocketCANInterface>())
{
}

SocketCANGateway::~SocketCANGateway()
{
  can_driver_->shutdown();
  can_driver_.reset();
  is_connected_ = false;
}

bool SocketCANGateway::connect()
{
  msg_listener_ =
      can_driver_->createMsgListener(can::CommInterface::FrameDelegate(this, &SocketCANGateway::msgCallback));
  state_listener_ =
      can_driver_->createStateListener(can::StateInterface::StateDelegate(this, &SocketCANGateway::stateCallback));

  std::cout << __PRETTY_FUNCTION__ << ": Trying to connect to " << canbus_dev_ << std::endl;
  if (!can_driver_->init(canbus_dev_, false, can::NoSettings::create()))
  {
    this->stateCallback(can_driver_->getState());
    std::cerr << __PRETTY_FUNCTION__ << ": Failed to connect to " << canbus_dev_ << std::endl;
  }
  else
  {
    std::cout << __PRETTY_FUNCTION__ << ": Connected to " << canbus_dev_ << std::endl;
    is_connected_ = true;
  }
  can_msg_process_thread_ = std::thread([=] { this->process(); });  // NOLINT
  return is_connected_;
}

bool SocketCANGateway::isConnected() const
{
  return is_connected_;
}

bool SocketCANGateway::recv(Message* msg)
{
  std::lock_guard<std::mutex> lock(receive_queue_mutex_);
  static auto last_time = std::chrono::high_resolution_clock::now();
  static double average_diff = 0.0;
  static int counter = 0;
   if (can_receive_queue_.empty())
  {
    return false;
  }
  this->canFrameToMsg(&can_receive_queue_.front(), msg);
  bool relevant = printMsgIfRelevant(msg, 5, true);
  if (relevant){
    const double tmp_diff = double(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now() - last_time).count() * 1e-9);
    average_diff = (tmp_diff + counter * average_diff) / (counter + 1);
    std::cout << "Average time diff: " << average_diff << std::endl;
    counter += 1;
    last_time = std::chrono::high_resolution_clock::now();
    std::cout << "From recv()" << std::endl;
  }
  can_receive_queue_.pop();
  return true;
}

void SocketCANGateway::queue(const Message& msg)
{
  can::Frame frame;
  this->msgToCanFrame(&msg, &frame);
  std::lock_guard<std::mutex> lock(send_queue_mutex_);
  can_send_queue_.push(frame);
}

void SocketCANGateway::canFrameToMsg(const can::Frame* frame, Message* msg)
{
  msg->id = frame->id & CAN_EFF_MASK;
  msg->len = frame->dlc;
  std::memcpy(msg->data, &frame->data, frame->dlc);
}

void SocketCANGateway::msgToCanFrame(const Message* msg, can::Frame* frame)
{
  frame->is_extended = true;
  frame->is_rtr = false;
  frame->is_error = false;
  frame->id = msg->id;
  frame->dlc = msg->len;

  std::memcpy(&frame->data, msg->data, msg->len);
}

void SocketCANGateway::msgCallback(const can::Frame& msg)
{
  std::lock_guard<std::mutex> lock(receive_queue_mutex_);
  can_receive_queue_.push(msg);
  Message readable_msg;
  canFrameToMsg(&msg, &readable_msg);
  bool relevant = printMsgIfRelevant(&readable_msg, 5, true);
  if (relevant){
    std::cout << "From msgCallback()" << std::endl;
  }
}

void SocketCANGateway::stateCallback(const can::State& state)
{
  std::string error;
  can_driver_->translateError(state.internal_error, error);
  std::cerr << __PRETTY_FUNCTION__ << " [CAN device: " << canbus_dev_ << "] State: " << state.driver_state <<
      ", internal_error: " << state.internal_error << " error: " << error << ", error_code:" << state.error_code
      << std::endl;
}

void SocketCANGateway::sendFrame(const Message& msg)
{
  can::Frame frame;
  frame.is_extended = true;
  frame.is_rtr = false;
  frame.is_error = false;
  frame.id = msg.id;
  frame.dlc = msg.len;

  if (!frame.isValid())
  {
    std::cerr << __PRETTY_FUNCTION__ <<  " [CAN device: " << canbus_dev_  << "] CAN frame is not valid, not sending."
        << std::endl;
    return;
  }
  can_driver_->send(frame);
}

void SocketCANGateway::process()
{
  while (is_connected_)
  {
    std::lock_guard<std::mutex> lock(send_queue_mutex_);
    if (!can_send_queue_.empty())
    {
      can_driver_->send(can_send_queue_.front());
      can_send_queue_.pop();
    }
  }
}

}  // namespace puma_motor_driver
