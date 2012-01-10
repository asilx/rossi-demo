/*
 * iCub.h
 * Copyright (c) 2011, Kadir Firat Uyanik, Kovan Research Lab, METU
 * kadir@ceng.metu.edu.tr
 *
 * All rights reserved.
 *
 * Software License Agreement (BSD License)
 *
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Kovan Lab nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ICUB_H_
#define ICUB_H_

#include "sstream"
#include "string"
#include "vector"
#include "map"
#include "boost/assign.hpp"

#include "yarp/os/Searchable.h"
#include "yarp/dev/PolyDriver.h"
#include "yarp/dev/ControlBoardInterfaces.h"
#include "yarp/os/Network.h"

namespace al
{

  namespace robot
  {
    const std::string TORSO_FRAME = "torso";
    const std::string HEAD_FRAME = "head";
    const std::string LEFT_ARM_FRAME = "left_arm";
    const std::string RIGHT_ARM_FRAME = "right_arm";
    const std::string LEFT_LEG_FRAME = "left_leg";
    const std::string RIGHT_LEG_FRAME = "right_leg";

    enum Part
    {
      TORSO, HEAD, LEFT_ARM, RIGHT_ARM, LEFT_LEG, RIGHT_LEG
    };

    enum Status
    {
      DRIVER_FAILED = -1, IPOS_FAILED = -2, IPOS_SETREFACC_FAILED = -3, IPOS_SETREFSPEED_FAILED = -4,
      IPOS_POSMOVE_OK = +5, IPOS_POSMOVE_FAILED = -6, IPOS_CHECKMOTIONDONE_FAILED = -7, IPOS_CHECKMOTIONDONE_OK = +8,
      IPOS_CHECKMOTIONDONE_TIMEOUT = -9, IPID_GETERROR_OK = +10, IPID_GETERROR_FAILED = -11, IENC_FAILED = -12,
      IENC_GETPOS_OK = +13, IENC_GETPOS_FAILED = -14, IENC_GETSPEED_OK = +15, IENC_GETSPEED_FAILED = -16,
      IENC_GETACC_OK = +17, IENC_GETACC_FAILED = -18
    };

    class iCub
    {
    public:
      iCub (const std::string& name);

      virtual
      ~iCub ();

      void
      init ();

      yarp::dev::PolyDriver* openDriver(const std::string& part_name);

      Status
      setPos (const std::string& part_name, const int& joint_id, const double& pos = 0.0);

      Status
      getPos (const std::string& part_name, const int& joint_id, double& pos);

      void getAllJointPos(std::map<std::string, double>& map_joint_vals);

    private:
      std::map<std::string, yarp::dev::PolyDriver*> map_drivers_;
      std::map<std::string, yarp::dev::IEncoders*> map_encoders_;
      std::map<std::string, yarp::dev::IPositionControl*> map_pos_ctrls_;
      std::map<std::string, yarp::dev::IVelocityControl*> map_vel_ctrls_;

      std::map<int, std::string> map_frame_names_;
      std::vector<std::string> frame_names_;
      std::string name_;
    };
  }
}

#endif /* ICUB_H_ */
