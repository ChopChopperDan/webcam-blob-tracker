//This file is automatically generated. DO NOT EDIT!

#include "edu__rpi__cats__sensors__webcam.h"
#include "edu__rpi__cats__sensors__camera_interface_stubskel.h"
#pragma once

namespace edu
{
namespace rpi
{
namespace cats
{
namespace sensors
{
namespace webcam
{

class edu__rpi__cats__sensors__webcamFactory : public virtual RobotRaconteur::ServiceFactory
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

class async_Webcam : public virtual edu::rpi::cats::sensors::camera_interface::async_Camera
{
public:
virtual void async_getCurrentImage(boost::function<void (RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > async_getCurrentImage(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    return RobotRaconteur::detail::async_wrap_for_spawn<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image >>(boost::bind((void (async_Webcam::*)(boost::function<void (RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_Webcam::async_getCurrentImage, this, _1,rr_timeout), rr_yield);
}
#endif

virtual void async_getImageHeader(boost::function<void (RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > async_getImageHeader(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    return RobotRaconteur::detail::async_wrap_for_spawn<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader >>(boost::bind((void (async_Webcam::*)(boost::function<void (RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader >,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_Webcam::async_getImageHeader, this, _1,rr_timeout), rr_yield);
}
#endif

virtual void async_StartStreaming(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual void async_StartStreaming(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_Webcam::*)(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_Webcam::async_StartStreaming, this, _1,rr_timeout), rr_yield);
}
#endif

virtual void async_StopStreaming(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE) = 0;

#ifdef ROBOTRACONTEUR_USE_ASIO_SPAWN
virtual void async_StopStreaming(boost::asio::yield_context rr_yield, int32_t rr_timeout=RR_TIMEOUT_INFINITE)
{
    RobotRaconteur::detail::async_wrap_for_spawn_void(boost::bind((void (async_Webcam::*)(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>)>,int32_t))&async_Webcam::async_StopStreaming, this, _1,rr_timeout), rr_yield);
}
#endif

};
class Webcam_stub : public virtual Webcam, public virtual async_Webcam, public virtual RobotRaconteur::ServiceStub
{
public:
Webcam_stub(const std::string& path, RR_SHARED_PTR<RobotRaconteur::ClientContext> c);

virtual void RRInitStub();
virtual RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > getCurrentImage();

virtual RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > getImageHeader();

virtual void StartStreaming();

virtual void StopStreaming();

virtual RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > > > get_ImageStream();
virtual void set_ImageStream(RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > > > value);


virtual void DispatchEvent(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);
virtual void DispatchPipeMessage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);
virtual void DispatchWireMessage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);
virtual RR_SHARED_PTR<RobotRaconteur::MessageEntry> CallbackCall(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m);
virtual void RRClose();
private:
RR_SHARED_PTR<RobotRaconteur::PipeClient<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > > > rrvar_ImageStream;
virtual void async_getCurrentImage(boost::function<void (RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_getCurrentImage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_getImageHeader(boost::function<void (RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_getImageHeader(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_StartStreaming(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_StartStreaming(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual void async_StopStreaming(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout=RR_TIMEOUT_INFINITE);

protected:
virtual void rrend_StopStreaming(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler);
public:
virtual std::string RRType();
};


class Webcam_skel : public virtual RobotRaconteur::ServiceSkel
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
virtual RR_SHARED_PTR<edu::rpi::cats::sensors::webcam::Webcam > get_obj();

virtual RR_SHARED_PTR<edu::rpi::cats::sensors::webcam::async_Webcam > get_asyncobj();

protected:
static void rr_getCurrentImage(RR_WEAK_PTR<edu::rpi::cats::sensors::webcam::Webcam_skel> skel, RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > ret, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_getImageHeader(RR_WEAK_PTR<edu::rpi::cats::sensors::webcam::Webcam_skel> skel, RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > ret, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_StartStreaming(RR_WEAK_PTR<edu::rpi::cats::sensors::webcam::Webcam_skel> skel, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
static void rr_StopStreaming(RR_WEAK_PTR<edu::rpi::cats::sensors::webcam::Webcam_skel> skel, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep);
 public:
protected:bool rr_InitPipeServersRun;
bool rr_InitWireServersRun;
RR_SHARED_PTR<RobotRaconteur::PipeServer<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > > > rr_ImageStream_pipe;
public: 
private:

};

}
}
}
}
}

