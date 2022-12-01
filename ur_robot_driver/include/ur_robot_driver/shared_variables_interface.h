// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------

// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2022-11-30
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CLIENT_LIBRARY_SHARED_VARIABLES_INTERFACE_H_INCLUDED
#define UR_CLIENT_LIBRARY_SHARED_VARIABLES_INTERFACE_H_INCLUDED

#include "ur_client_library/control/reverse_interface.h"
#include "ur_client_library/comm/control_mode.h"
#include "ur_client_library/types.h"
#include "ur_client_library/log.h"

namespace urcl
{
namespace control
{
/*!
 * \brief Types for encoding trajectory execution result.
 */
enum class TrajectoryResult : int32_t
{

  TRAJECTORY_RESULT_SUCCESS = 0,   ///< Successful execution
  TRAJECTORY_RESULT_CANCELED = 1,  ///< Canceled by user
  TRAJECTORY_RESULT_FAILURE = 2    ///< Aborted due to error during execution
};

/*!
 * \brief The SharedVariablesInterface class handles trajectory forwarding to the robot. Full
 * trajectories are forwarded to the robot controller and are executed there.
 */
class SharedVariablesInterface : public ReverseInterface
{
public:
  static const int32_t MULT_TIME = 1000;
  static const int32_t JOINT_POINT = 0;
  static const int32_t CARTESIAN_POINT = 1;

  SharedVariablesInterface() = delete;
  /*!
   * \brief Creates a SharedVariablesInterface object including a TCPServer.
   *
   * \param port Port the Server is started on
   */
  SharedVariablesInterface(uint32_t port);

  /*!
   * \brief Disconnects possible clients so the reverse interface object can be safely destroyed.
   */
  virtual ~SharedVariablesInterface() = default;

  /*!
   * \brief Writes needed information to the robot to be read by the URScript program.
   *
   * \param positions A vector of joint or cartesian targets for the robot
   * \param goal_time The goal time to reach the target
   * \param blend_radius The radius to be used for blending between control points
   * \param cartesian True, if the written point is specified in cartesian space, false if in joint space
   *
   * \returns True, if the write was performed successfully, false otherwise.
   */
  bool writeVariables(const vector6d_t* variables);

  void setExternalMessageCallback(std::function<void(void)> callback)
  {
    handle_message_ = callback;
  }

protected:
  virtual void connectionCallback(const int filedescriptor) override;

  virtual void disconnectionCallback(const int filedescriptor) override;

  virtual void messageCallback(const int filedescriptor, char* buffer, int nbytesrecv) override;

private:
  std::function<void(void)> handle_message_;
};

}  // namespace control
}  // namespace urcl

#endif  // UR_CLIENT_LIBRARY_SHARED_VARIABLES_INTERFACE_H_INCLUDED
