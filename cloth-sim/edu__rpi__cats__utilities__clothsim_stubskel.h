//This file is automatically generated. DO NOT EDIT!

#include "edu__rpi__cats__utilities__clothsim.h"
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

class edu__rpi__cats__utilities__clothsimFactory : public virtual RobotRaconteur::ServiceFactory
{
public:
virtual std::string GetServiceName();
virtual std::string DefString();
virtual RR_SHARED_PTR<RobotRaconteur::StructureStub> FindStructureStub(std::string s);
virtual RR_SHARED_PTR<RobotRaconteur::MessageElementStructure> PackStructure(RR_SHARED_PTR<RobotRaconteur::RRStructure> structin);
virtual RR_SHARED_PTR<RobotRaconteur::RRObject> UnpackStructure(RR_SHARED_PTR<RobotRaconteur::MessageElementStructure> mstructin);
virtual RR_SHARED_PTR<RobotRaconteur::ServiceStub> CreateStub(std::string objecttype, std::string path, RR_SHARED_PTR<RobotRaconteur::ClientContext> context);
virtual RR_SHARED_PTR<RobotRaconteur::ServiceSkel> CreateSkel(std::string objecttype, std::string path, RR_SHARED_PTR<RobotRaconteur::RRObject> obj, RR_SHARED_PTR<RobotRaconteur::ServerContext> context);
virtual void DownCastAndThrowException(RobotRaconteur::RobotRaconteurException& exp);
virtual RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> DownCastException(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> exp);
};

class Pose_stub : public virtual RobotRaconteur::StructureStub
{
public:
Pose_stub(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurNode> node) : RobotRaconteur::StructureStub(node) {}
virtual RR_SHARED_PTR<RobotRaconteur::MessageElementStructure> PackStructure(RR_SHARED_PTR<RobotRaconteur::RRObject> s);
virtual RR_SHARED_PTR<RobotRaconteur::RRStructure> UnpackStructure(RR_SHARED_PTR<RobotRaconteur::MessageElementStructure> m);
};

class ClothDefinition_stub : public virtual RobotRaconteur::StructureStub
{
public:
ClothDefinition_stub(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurNode> node) : RobotRaconteur::StructureStub(node) {}
virtual RR_SHARED_PTR<RobotRaconteur::MessageElementStructure> PackStructure(RR_SHARED_PTR<RobotRaconteur::RRObject> s);
virtual RR_SHARED_PTR<RobotRaconteur::RRStructure> UnpackStructure(RR_SHARED_PTR<RobotRaconteur::MessageElementStructure> m);
};

class ClothState_stub : public virtual RobotRaconteur::StructureStub
{
public:
ClothState_stub(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurNode> node) : RobotRaconteur::StructureStub(node) {}
virtual RR_SHARED_PTR<RobotRaconteur::MessageElementStructure> PackStructure(RR_SHARED_PTR<RobotRaconteur::RRObject> s);
virtual RR_SHARED_PTR<RobotRaconteur::RRStructure> UnpackStructure(RR_SHARED_PTR<RobotRaconteur::MessageElementStructure> m);
};

class ClothLinks_stub : public virtual RobotRaconteur::StructureStub
{
public:
ClothLinks_stub(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurNode> node) : RobotRaconteur::StructureStub(node) {}
virtual RR_SHARED_PTR<RobotRaconteur::MessageElementStructure> PackStructure(RR_SHARED_PTR<RobotRaconteur::RRObject> s);
virtual RR_SHARED_PTR<RobotRaconteur::RRStructure> UnpackStructure(RR_SHARED_PTR<RobotRaconteur::MessageElementStructure> m);
};

class DepthImage_stub : public virtual RobotRaconteur::StructureStub
{
public:
DepthImage_stub(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurNode> node) : RobotRaconteur::StructureStub(node) {}
virtual RR_SHARED_PTR<RobotRaconteur::MessageElementStructure> PackStructure(RR_SHARED_PTR<RobotRaconteur::RRObject> s);
virtual RR_SHARED_PTR<RobotRaconteur::RRStructure> UnpackStructure(RR_SHARED_PTR<RobotRaconteur::MessageElementStructure> m);
};

class async_ClothSimulator
{
public:
virtual void async_get_grasped_nodes00(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;
virtual void async_set_grasped_nodes00(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > async_get_grasped_nodes00(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    return RobotRaconteur::detail::async_wrap_for_spawn<RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > >(boost::bind((void (async_ClothSimulator::*)(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) >, int32_t))&async_ClothSimulator::async_get_grasped_nodes00, this, _1, rr_timeout), rr_yield);
}
virtual void async_set_grasped_nodes00(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value, boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_ClothSimulator::*)(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >, boost::function<void(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>, int32_t))&async_ClothSimulator::async_set_grasped_nodes00, this, boost::ref(value), _1, rr_timeout), rr_yield);
}
#endif

virtual void async_get_grasped_nodes10(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;
virtual void async_set_grasped_nodes10(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > async_get_grasped_nodes10(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    return RobotRaconteur::detail::async_wrap_for_spawn<RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > >(boost::bind((void (async_ClothSimulator::*)(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) >, int32_t))&async_ClothSimulator::async_get_grasped_nodes10, this, _1, rr_timeout), rr_yield);
}
virtual void async_set_grasped_nodes10(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value, boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_ClothSimulator::*)(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >, boost::function<void(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>, int32_t))&async_ClothSimulator::async_set_grasped_nodes10, this, boost::ref(value), _1, rr_timeout), rr_yield);
}
#endif

virtual void async_get_grasped_nodes01(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;
virtual void async_set_grasped_nodes01(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > async_get_grasped_nodes01(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    return RobotRaconteur::detail::async_wrap_for_spawn<RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > >(boost::bind((void (async_ClothSimulator::*)(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) >, int32_t))&async_ClothSimulator::async_get_grasped_nodes01, this, _1, rr_timeout), rr_yield);
}
virtual void async_set_grasped_nodes01(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value, boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_ClothSimulator::*)(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >, boost::function<void(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>, int32_t))&async_ClothSimulator::async_set_grasped_nodes01, this, boost::ref(value), _1, rr_timeout), rr_yield);
}
#endif

virtual void async_get_grasped_nodes11(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;
virtual void async_set_grasped_nodes11(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > async_get_grasped_nodes11(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    return RobotRaconteur::detail::async_wrap_for_spawn<RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > >(boost::bind((void (async_ClothSimulator::*)(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) >, int32_t))&async_ClothSimulator::async_get_grasped_nodes11, this, _1, rr_timeout), rr_yield);
}
virtual void async_set_grasped_nodes11(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value, boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_ClothSimulator::*)(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >, boost::function<void(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>, int32_t))&async_ClothSimulator::async_set_grasped_nodes11, this, boost::ref(value), _1, rr_timeout), rr_yield);
}
#endif

virtual void async_getClothDefinition(boost::function<void (RR_SHARED_PTR<ClothDefinition >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
RR_SHARED_PTR<ClothDefinition > async_getClothDefinition(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    return RobotRaconteur::detail::async_wrap_for_spawn<RR_SHARED_PTR<ClothDefinition >>(boost::bind((void (async_ClothSimulator::*)(boost::function<void (RR_SHARED_PTR<ClothDefinition >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_ClothSimulator::async_getClothDefinition, this, _1,rr_timeout), rr_yield);
}
#endif

virtual void async_getClothLinks(boost::function<void (RR_SHARED_PTR<ClothLinks >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
RR_SHARED_PTR<ClothLinks > async_getClothLinks(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    return RobotRaconteur::detail::async_wrap_for_spawn<RR_SHARED_PTR<ClothLinks >>(boost::bind((void (async_ClothSimulator::*)(boost::function<void (RR_SHARED_PTR<ClothLinks >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_ClothSimulator::async_getClothLinks, this, _1,rr_timeout), rr_yield);
}
#endif

virtual void async_setClothStiffness(double stiffness, uint8_t piterations,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual void async_setClothStiffness(double stiffness, uint8_t piterations,boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_ClothSimulator::*)(double,uint8_t,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_ClothSimulator::async_setClothStiffness, this, boost::ref(stiffness),boost::ref(piterations),_1,rr_timeout), rr_yield);
}
#endif

virtual void async_start_recording(std::string record_name,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual void async_start_recording(std::string record_name,boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_ClothSimulator::*)(std::string,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_ClothSimulator::async_start_recording, this, boost::ref(record_name),_1,rr_timeout), rr_yield);
}
#endif

virtual void async_stop_recording(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual void async_stop_recording(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_ClothSimulator::*)(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_ClothSimulator::async_stop_recording, this, _1,rr_timeout), rr_yield);
}
#endif

virtual void async_stepForwardSim(double tstep, RR_SHARED_PTR<Pose > p00, RR_SHARED_PTR<Pose > p10, RR_SHARED_PTR<Pose > p01, RR_SHARED_PTR<Pose > p11,boost::function<void (RR_SHARED_PTR<ClothState >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
RR_SHARED_PTR<ClothState > async_stepForwardSim(double tstep, RR_SHARED_PTR<Pose > p00, RR_SHARED_PTR<Pose > p10, RR_SHARED_PTR<Pose > p01, RR_SHARED_PTR<Pose > p11,boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    return RobotRaconteur::detail::async_wrap_for_spawn<RR_SHARED_PTR<ClothState >>(boost::bind((void (async_ClothSimulator::*)(double,RR_SHARED_PTR<Pose >,RR_SHARED_PTR<Pose >,RR_SHARED_PTR<Pose >,RR_SHARED_PTR<Pose >,boost::function<void (RR_SHARED_PTR<ClothState >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_ClothSimulator::async_stepForwardSim, this, boost::ref(tstep),boost::ref(p00),boost::ref(p10),boost::ref(p01),boost::ref(p11),_1,rr_timeout), rr_yield);
}
#endif

virtual void async_stepSimToConverge(RR_SHARED_PTR<Pose > p00, RR_SHARED_PTR<Pose > p10, RR_SHARED_PTR<Pose > p01, RR_SHARED_PTR<Pose > p11,boost::function<void (RR_SHARED_PTR<ClothState >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
RR_SHARED_PTR<ClothState > async_stepSimToConverge(RR_SHARED_PTR<Pose > p00, RR_SHARED_PTR<Pose > p10, RR_SHARED_PTR<Pose > p01, RR_SHARED_PTR<Pose > p11,boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    return RobotRaconteur::detail::async_wrap_for_spawn<RR_SHARED_PTR<ClothState >>(boost::bind((void (async_ClothSimulator::*)(RR_SHARED_PTR<Pose >,RR_SHARED_PTR<Pose >,RR_SHARED_PTR<Pose >,RR_SHARED_PTR<Pose >,boost::function<void (RR_SHARED_PTR<ClothState >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_ClothSimulator::async_stepSimToConverge, this, boost::ref(p00),boost::ref(p10),boost::ref(p01),boost::ref(p11),_1,rr_timeout), rr_yield);
}
#endif

virtual void async_getFaceStructure(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > async_getFaceStructure(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    return RobotRaconteur::detail::async_wrap_for_spawn<RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >>(boost::bind((void (async_ClothSimulator::*)(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_ClothSimulator::async_getFaceStructure, this, _1,rr_timeout), rr_yield);
}
#endif

virtual void async_setCameraPose(RR_SHARED_PTR<Pose > pk,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual void async_setCameraPose(RR_SHARED_PTR<Pose > pk,boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_ClothSimulator::*)(RR_SHARED_PTR<Pose >,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_ClothSimulator::async_setCameraPose, this, boost::ref(pk),_1,rr_timeout), rr_yield);
}
#endif

virtual void async_getRenderedImage(boost::function<void (RR_SHARED_PTR<DepthImage >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
RR_SHARED_PTR<DepthImage > async_getRenderedImage(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    return RobotRaconteur::detail::async_wrap_for_spawn<RR_SHARED_PTR<DepthImage >>(boost::bind((void (async_ClothSimulator::*)(boost::function<void (RR_SHARED_PTR<DepthImage >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_ClothSimulator::async_getRenderedImage, this, _1,rr_timeout), rr_yield);
}
#endif

};
class ClothSimulator_stub : public virtual ClothSimulator, public virtual async_ClothSimulator, public virtual RobotRaconteur::ServiceStub
{
public:
ClothSimulator_stub(const std::string& path, RR_SHARED_PTR<RobotRaconteur::ClientContext> c);

virtual void RRInitStub();
virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > get_grasped_nodes00();
virtual void set_grasped_nodes00(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value);

virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > get_grasped_nodes10();
virtual void set_grasped_nodes10(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value);

virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > get_grasped_nodes01();
virtual void set_grasped_nodes01(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value);

virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > get_grasped_nodes11();
virtual void set_grasped_nodes11(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value);

virtual RR_SHARED_PTR<ClothDefinition > getClothDefinition();

virtual RR_SHARED_PTR<ClothLinks > getClothLinks();

virtual void setClothStiffness(double stiffness, uint8_t piterations);

virtual void start_recording(std::string record_name);

virtual void stop_recording();

virtual RR_SHARED_PTR<ClothState > stepForwardSim(double tstep, RR_SHARED_PTR<Pose > p00, RR_SHARED_PTR<Pose > p10, RR_SHARED_PTR<Pose > p01, RR_SHARED_PTR<Pose > p11);

virtual RR_SHARED_PTR<ClothState > stepSimToConverge(RR_SHARED_PTR<Pose > p00, RR_SHARED_PTR<Pose > p10, RR_SHARED_PTR<Pose > p01, RR_SHARED_PTR<Pose > p11);

virtual RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > getFaceStructure();

virtual void setCameraPose(RR_SHARED_PTR<Pose > pk);

virtual RR_SHARED_PTR<DepthImage > getRenderedImage();


virtual void DispatchEvent(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);
virtual void DispatchPipeMessage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);
virtual void DispatchWireMessage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);
virtual RR_SHARED_PTR<RobotRaconteur::MessageEntry> CallbackCall(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);
virtual void RRClose();
private:
virtual void async_get_grasped_nodes00(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);
virtual void async_set_grasped_nodes00(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_get_grasped_nodes00(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
virtual void rrend_set_grasped_nodes00(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_get_grasped_nodes10(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);
virtual void async_set_grasped_nodes10(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_get_grasped_nodes10(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
virtual void rrend_set_grasped_nodes10(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_get_grasped_nodes01(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);
virtual void async_set_grasped_nodes01(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_get_grasped_nodes01(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
virtual void rrend_set_grasped_nodes01(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_get_grasped_nodes11(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);
virtual void async_set_grasped_nodes11(RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_get_grasped_nodes11(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
virtual void rrend_set_grasped_nodes11(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_getClothDefinition(boost::function<void (RR_SHARED_PTR<ClothDefinition >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_getClothDefinition(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<ClothDefinition > ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_getClothLinks(boost::function<void (RR_SHARED_PTR<ClothLinks >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_getClothLinks(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<ClothLinks > ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_setClothStiffness(double stiffness, uint8_t piterations,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_setClothStiffness(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_start_recording(std::string record_name,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_start_recording(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_stop_recording(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_stop_recording(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_stepForwardSim(double tstep, RR_SHARED_PTR<Pose > p00, RR_SHARED_PTR<Pose > p10, RR_SHARED_PTR<Pose > p01, RR_SHARED_PTR<Pose > p11,boost::function<void (RR_SHARED_PTR<ClothState >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_stepForwardSim(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<ClothState > ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_stepSimToConverge(RR_SHARED_PTR<Pose > p00, RR_SHARED_PTR<Pose > p10, RR_SHARED_PTR<Pose > p01, RR_SHARED_PTR<Pose > p11,boost::function<void (RR_SHARED_PTR<ClothState >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_stepSimToConverge(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<ClothState > ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_getFaceStructure(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_getFaceStructure(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_setCameraPose(RR_SHARED_PTR<Pose > pk,boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_setCameraPose(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_getRenderedImage(boost::function<void (RR_SHARED_PTR<DepthImage >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_getRenderedImage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<DepthImage > ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual std::string RRType();
};


class ClothSimulator_skel : public virtual RobotRaconteur::ServiceSkel
{
public:
virtual void Init(const std::string& path, RR_SHARED_PTR<RobotRaconteur::RRObject> object, RR_SHARED_PTR<RobotRaconteur::ServerContext> context);
virtual RR_SHARED_PTR<RobotRaconteur::MessageEntry> CallGetProperty(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);

virtual RR_SHARED_PTR<RobotRaconteur::MessageEntry> CallSetProperty(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);

virtual RR_SHARED_PTR<RobotRaconteur::MessageEntry> CallFunction(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);

virtual void ReleaseCastObject();

virtual void RegisterEvents(RR_SHARED_PTR<RobotRaconteur::RRObject> rrobj1);

virtual void UnregisterEvents(RR_SHARED_PTR<RobotRaconteur::RRObject> rrobj1);

virtual RR_SHARED_PTR<RobotRaconteur::RRObject> GetSubObj(const std::string &name, const std::string &ind);

virtual void InitPipeServers(RR_SHARED_PTR<RobotRaconteur::RRObject> rrobj1);

virtual void InitWireServers(RR_SHARED_PTR<RobotRaconteur::RRObject> rrobj1);

virtual void DispatchPipeMessage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, uint32_t e);

virtual void DispatchWireMessage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, uint32_t e);

virtual void InitCallbackServers(RR_SHARED_PTR<RobotRaconteur::RRObject> o);

virtual RR_SHARED_PTR<RobotRaconteur::MessageEntry> CallPipeFunction(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, uint32_t e);

virtual RR_SHARED_PTR<RobotRaconteur::MessageEntry> CallWireFunction(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, uint32_t e);

virtual RR_SHARED_PTR<void> GetCallbackFunction(uint32_t endpoint, const std::string& membername);

virtual RR_SHARED_PTR<RobotRaconteur::MessageEntry> CallMemoryFunction(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::Endpoint> e);

virtual std::string GetObjectType();
virtual RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::ClothSimulator > get_obj();

virtual RR_SHARED_PTR<edu::rpi::cats::utilities::clothsim::async_ClothSimulator > get_asyncobj();

protected:
static void rr_get_grasped_nodes00(RR_WEAK_PTR<edu::rpi::cats::utilities::clothsim::ClothSimulator_skel> skel, RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_get_grasped_nodes10(RR_WEAK_PTR<edu::rpi::cats::utilities::clothsim::ClothSimulator_skel> skel, RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_get_grasped_nodes01(RR_WEAK_PTR<edu::rpi::cats::utilities::clothsim::ClothSimulator_skel> skel, RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_get_grasped_nodes11(RR_WEAK_PTR<edu::rpi::cats::utilities::clothsim::ClothSimulator_skel> skel, RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > value, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_getClothDefinition(RR_WEAK_PTR<edu::rpi::cats::utilities::clothsim::ClothSimulator_skel> skel, RR_SHARED_PTR<ClothDefinition > ret, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_getClothLinks(RR_WEAK_PTR<edu::rpi::cats::utilities::clothsim::ClothSimulator_skel> skel, RR_SHARED_PTR<ClothLinks > ret, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_setClothStiffness(RR_WEAK_PTR<edu::rpi::cats::utilities::clothsim::ClothSimulator_skel> skel, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_start_recording(RR_WEAK_PTR<edu::rpi::cats::utilities::clothsim::ClothSimulator_skel> skel, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_stop_recording(RR_WEAK_PTR<edu::rpi::cats::utilities::clothsim::ClothSimulator_skel> skel, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_stepForwardSim(RR_WEAK_PTR<edu::rpi::cats::utilities::clothsim::ClothSimulator_skel> skel, RR_SHARED_PTR<ClothState > ret, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_stepSimToConverge(RR_WEAK_PTR<edu::rpi::cats::utilities::clothsim::ClothSimulator_skel> skel, RR_SHARED_PTR<ClothState > ret, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_getFaceStructure(RR_WEAK_PTR<edu::rpi::cats::utilities::clothsim::ClothSimulator_skel> skel, RR_SHARED_PTR<RobotRaconteur::RRArray<uint16_t > > ret, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_setCameraPose(RR_WEAK_PTR<edu::rpi::cats::utilities::clothsim::ClothSimulator_skel> skel, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_getRenderedImage(RR_WEAK_PTR<edu::rpi::cats::utilities::clothsim::ClothSimulator_skel> skel, RR_SHARED_PTR<DepthImage > ret, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
 public:
protected:bool rr_InitPipeServersRun;
bool rr_InitWireServersRun;
public: 
private:

};

}
}
}
}
}

