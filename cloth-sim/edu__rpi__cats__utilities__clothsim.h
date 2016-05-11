//This file is automatically generated. DO NOT EDIT!

#include <RobotRaconteur.h>
#include <boost/signals2.hpp>
#pragma once

namespace edu
{
namespace rpi
{
namespace cats
{
namespace utilities
{
namespace clothsim
{

class Pose;
class ClothState;
class DepthImage;
class ClothSimulator;

class Pose : public RobotRaconteur::RRStructure {
public:
RR_SHARED_PTR<RobotRaconteur::RRArray<double > > R;
RR_SHARED_PTR<RobotRaconteur::RRArray<double > > p;

virtual std::string RRType() {return "edu.rpi.cats.utilities.clothsim.Pose";  }
};

class ClothState : public RobotRaconteur::RRStructure {
public:
double t;
uint16_t numX;
uint16_t numY;
uint32_t n_points;
RR_SHARED_PTR<RobotRaconteur::RRArray<double > > x;
RR_SHARED_PTR<RobotRaconteur::RRArray<double > > y;
RR_SHARED_PTR<RobotRaconteur::RRArray<double > > z;
RR_SHARED_PTR<RobotRaconteur::RRArray<double > > fx;
RR_SHARED_PTR<RobotRaconteur::RRArray<double > > fy;
RR_SHARED_PTR<RobotRaconteur::RRArray<double > > fz;

virtual std::string RRType() {return "edu.rpi.cats.utilities.clothsim.ClothState";  }
};

class DepthImage : public RobotRaconteur::RRStructure {
public:
uint16_t width;
uint16_t height;
RR_SHARED_PTR<RobotRaconteur::RRArray<float > > data;

virtual std::string RRType() {return "edu.rpi.cats.utilities.clothsim.DepthImage";  }
};

class ClothSimulator : public virtual RobotRaconteur::RRObject
{
public:
virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > get_grasped_nodes00()=0;
virtual void set_grasped_nodes00(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value)=0;

virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > get_grasped_nodes10()=0;
virtual void set_grasped_nodes10(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value)=0;

virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > get_grasped_nodes01()=0;
virtual void set_grasped_nodes01(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value)=0;

virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > get_grasped_nodes11()=0;
virtual void set_grasped_nodes11(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value)=0;

virtual void start_recording(std::string record_name)=0;

virtual void stop_recording()=0;

virtual RR_SHARED_PTR<ClothState > stepForwardSim(double tstep, RR_SHARED_PTR<Pose > p00, RR_SHARED_PTR<Pose > p10, RR_SHARED_PTR<Pose > p01, RR_SHARED_PTR<Pose > p11)=0;

virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > getFaceStructure()=0;

virtual void setCameraPose(RR_SHARED_PTR<Pose > pk)=0;

virtual RR_SHARED_PTR<DepthImage > getRenderedImage()=0;

virtual std::string RRType() {return "edu.rpi.cats.utilities.clothsim.ClothSimulator";  }
};

}
}
}
}
}

