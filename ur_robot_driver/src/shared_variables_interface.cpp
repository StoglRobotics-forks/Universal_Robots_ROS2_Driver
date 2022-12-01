// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2022-11-30
 *
 */
//----------------------------------------------------------------------

#include <ur_robot_driver/shared_variables_interface.h>

namespace urcl
{
namespace control
{
SharedVariablesInterface::SharedVariablesInterface(uint32_t port) : ReverseInterface(port, [](bool foo) { return foo; })
{
}

bool SharedVariablesInterface::writeVariables(const vector6d_t* variables)
{
  if (client_fd_ == -1)
  {
    return false;
  }
  // size of message 6
  uint8_t buffer[sizeof(int32_t) * 6];
  uint8_t* b_pos = buffer;

  if (variables != nullptr)
  {
    for (auto const& pos : *variables)
    {
//      int32_t val = static_cast<int32_t>(pos * MULT_JOINTSTATE);
      int32_t val = static_cast<int32_t>(pos * 1.0);
      val = htobe32(val);
      b_pos += append(b_pos, val);
    }
  }
  else
  {
    b_pos += 6 * sizeof(int32_t);
  }

  size_t written;

  return server_.write(client_fd_, buffer, sizeof(buffer), written);
}

void SharedVariablesInterface::connectionCallback(const int filedescriptor)
{
  if (client_fd_ < 0)
  {
    URCL_LOG_INFO("Robot connected to shared variables interface.");
    client_fd_ = filedescriptor;
  }
  else
  {
    URCL_LOG_ERROR("Connection request to SharedVariablesInterface received while connection already established. Only "
                   "one connection is allowed at a time. Ignoring this request.");
  }
}

void SharedVariablesInterface::disconnectionCallback(const int filedescriptor)
{
  URCL_LOG_INFO("Connection to trajectory interface dropped.", filedescriptor);
  client_fd_ = -1;
}

void SharedVariablesInterface::messageCallback(const int filedescriptor, char* buffer, int nbytesrecv)
{
    URCL_LOG_INFO("Received message of size %d in SharedVariablesInterface", nbytesrecv);

    if (nbytesrecv == 4)
  {
    int32_t* status = reinterpret_cast<int*>(buffer);
    URCL_LOG_INFO("Received message %d on SharedVariablesInterface", be32toh(*status));

    if (handle_message_)
    {
        handle_message_();
    }
    else
    {
      URCL_LOG_DEBUG("Message callback executed, but no external callback was given.");
    }
  }
  else
  {
    URCL_LOG_WARN("Received %d bytes on SharedVariablesInterface. Expecting 4 bytes, so ignoring this message",
                  nbytesrecv);
  }
}
}  // namespace control
}  // namespace urcl
