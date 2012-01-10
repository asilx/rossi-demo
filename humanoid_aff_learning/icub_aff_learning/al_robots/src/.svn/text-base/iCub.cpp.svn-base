/*
 * iCub.cpp
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

#include "al_robots/iCub.h"

namespace al
{

  namespace robot
  {

    iCub::iCub (const std::string& name)
    {
      name_ = name;
    }

    iCub::~iCub ()
    {
      // TODO Auto-generated destructor stub
    }

    yarp::dev::PolyDriver*
    iCub::openDriver (const std::string& part_name)
    {
      yarp::dev::PolyDriver* p_driver = NULL;
      yarp::os::Property opt;
      opt.put ("robot", name_.c_str ());
      opt.put ("device", "remote_controlboard");
      opt.put ("local", ("/" + std::string("al_robot") + "/" + part_name).c_str ());
      opt.put ("remote", ("/" + name_ + "/" + part_name).c_str ());

      p_driver=new yarp::dev::PolyDriver(opt);

      if (!p_driver->isValid ())
      {
        p_driver->close ();
        delete p_driver;
        p_driver = NULL;
      }
      return p_driver;
    }

    void
    iCub::init ()
    {
      yarp::os::Network::init();

      boost::assign::insert (map_frame_names_)
              ((int)TORSO, TORSO_FRAME)
              ((int)HEAD, HEAD_FRAME)
              ((int)LEFT_ARM,LEFT_ARM_FRAME)
              ((int)RIGHT_ARM,RIGHT_ARM_FRAME)
              ((int)LEFT_LEG,LEFT_LEG_FRAME)
              ((int)RIGHT_LEG,RIGHT_LEG_FRAME);

      for (uint i = 0; i < map_frame_names_.size (); i++)
      {
        map_drivers_[map_frame_names_[i]] = openDriver (map_frame_names_[i]);
        if (map_drivers_[map_frame_names_[i]])
        {
          map_drivers_[map_frame_names_[i]]->view (map_encoders_[map_frame_names_[i]]);
          //          map_drivers_[frame_names[i]]->view (map_pos_ctrls__[frame_names[i]]);
          //          map_drivers_[frame_names[i]]->view (map_vel_ctrls_[frame_names[i]]);
        }
      }
    }

    Status
    iCub::setPos (const std::string& part_name, const int& joint_id, const double& pos)
    {
      //TODO: stub for the moment
      return DRIVER_FAILED;
    }

    Status
    iCub::getPos (const std::string& part_name, const int& joint_id, double& pos)
    {
      if (!map_encoders_[part_name])
        return IENC_FAILED;

      if (map_encoders_[part_name]->getEncoder (joint_id, &pos))
        return IENC_GETSPEED_OK;

      return IENC_GETSPEED_FAILED;
    }

    void iCub::getAllJointPos(std::map<std::string, double>& map_joint_vals)
    {
      for(uint i=0;i<map_frame_names_.size();i++)
      {
        int n_jnts=0;
        map_encoders_[map_frame_names_[i]]->getAxes(&n_jnts);
        for(int j=0;j<n_jnts;j++)
        {
          std::string jnt_name = map_frame_names_[i];
          std::stringstream s;
          s<<j;
          jnt_name.append("_" + s.str());

          double val = 0;
          if(map_encoders_[map_frame_names_[i]]->getEncoder(j,&val))
            map_joint_vals[jnt_name]=val;
        }
      }
    }
  }

}
