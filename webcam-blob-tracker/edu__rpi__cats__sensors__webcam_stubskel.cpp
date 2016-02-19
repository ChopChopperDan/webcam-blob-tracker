//This file is automatically generated. DO NOT EDIT!

#ifndef ROBOTRACONTEUR_USE_STDAFX
#include "edu__rpi__cats__sensors__webcam_stubskel.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#else
#include "stdafx.h"
#endif
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
std::string edu__rpi__cats__sensors__webcamFactory::GetServiceName()
{
return "edu.rpi.cats.sensors.webcam";
}
std::string edu__rpi__cats__sensors__webcamFactory::DefString()
{
std::string out(
"service edu.rpi.cats.sensors.webcam\n"
"import edu.rpi.cats.sensors.camera_interface\n"
"\n"
"option version 0.5\n"
"\n"
"object Webcam\n"
"implements edu.rpi.cats.sensors.camera_interface.Camera\n"
"\n"
"function edu.rpi.cats.sensors.camera_interface.Image getCurrentImage()\n"
"\n"
"function edu.rpi.cats.sensors.camera_interface.ImageHeader getImageHeader()\n"
"\n"
"function void StartStreaming()\n"
"function void StopStreaming()\n"
"\n"
"pipe edu.rpi.cats.sensors.camera_interface.Image ImageStream\n"
"end object\n"
);
return out;
}
RR_SHARED_PTR<RobotRaconteur::StructureStub> edu__rpi__cats__sensors__webcamFactory::FindStructureStub(std::string s)
{
boost::tuple<std::string,std::string> res=RobotRaconteur::SplitQualifiedName(s);
std::string servicetype=res.get<0>();
std::string objecttype=res.get<1>();
throw RobotRaconteur::ServiceException("Invalid structure stub type.");
}
RR_SHARED_PTR<RobotRaconteur::MessageElementStructure> edu__rpi__cats__sensors__webcamFactory::PackStructure(RR_SHARED_PTR<RobotRaconteur::RRStructure> structin)
{
std::string type=structin->RRType();boost::tuple<std::string,std::string> res=RobotRaconteur::SplitQualifiedName(type);
std::string servicetype=res.get<0>();
std::string objecttype=res.get<1>();
if (servicetype != "edu.rpi.cats.sensors.webcam") return GetNode()->PackStructure(structin);
RR_SHARED_PTR<RobotRaconteur::StructureStub> stub=FindStructureStub(type);
return stub->PackStructure(structin);
throw RobotRaconteur::ServiceException("Invalid structure stub type.");
}
RR_SHARED_PTR<RobotRaconteur::RRObject> edu__rpi__cats__sensors__webcamFactory::UnpackStructure(RR_SHARED_PTR<RobotRaconteur::MessageElementStructure> mstructin)
{
std::string type=mstructin->GetTypeString();boost::tuple<std::string,std::string> res=RobotRaconteur::SplitQualifiedName(type);
std::string servicetype=res.get<0>();
std::string objecttype=res.get<1>();
if (servicetype != "edu.rpi.cats.sensors.webcam") return GetNode()->UnpackStructure(mstructin);
RR_SHARED_PTR<RobotRaconteur::StructureStub> stub=FindStructureStub(type);
return stub->UnpackStructure(mstructin);
throw RobotRaconteur::ServiceException("Invalid structure stub type.");
}
RR_SHARED_PTR<RobotRaconteur::ServiceStub> edu__rpi__cats__sensors__webcamFactory::CreateStub(std::string type, std::string path, RR_SHARED_PTR<RobotRaconteur::ClientContext> context)
{
boost::tuple<std::string,std::string> res=RobotRaconteur::SplitQualifiedName(type);
std::string servicetype=res.get<0>();
std::string objecttype=res.get<1>();
if (servicetype != "edu.rpi.cats.sensors.webcam") return GetNode()->GetServiceType(servicetype)->CreateStub(type,path,context);
if (objecttype=="Webcam") { RR_SHARED_PTR<Webcam_stub> o=(RR_MAKE_SHARED<Webcam_stub>(path,context)); o->RRInitStub(); return o; }
throw RobotRaconteur::ServiceException("Invalid structure stub type.");
}
RR_SHARED_PTR<RobotRaconteur::ServiceSkel> edu__rpi__cats__sensors__webcamFactory::CreateSkel(std::string type, std::string path, RR_SHARED_PTR<RobotRaconteur::RRObject> obj, RR_SHARED_PTR<RobotRaconteur::ServerContext> context)
{
boost::tuple<std::string,std::string> res=RobotRaconteur::SplitQualifiedName(type);
std::string servicetype=res.get<0>();
std::string objecttype=res.get<1>();
if (servicetype != "edu.rpi.cats.sensors.webcam") return GetNode()->GetServiceType(servicetype)->CreateSkel(type,path,obj,context);
if (objecttype=="Webcam") {RR_SHARED_PTR<Webcam_skel> o=RR_MAKE_SHARED<Webcam_skel>(); o->Init(path,obj,context); return o; }
throw RobotRaconteur::ServiceException("Invalid structure skel type.");
return RR_SHARED_PTR<RobotRaconteur::ServiceSkel>();
}
void edu__rpi__cats__sensors__webcamFactory::DownCastAndThrowException(RobotRaconteur::RobotRaconteurException& rr_exp)
{
std::string rr_type=rr_exp.Error;
if (rr_type.find('.')==std::string::npos)
{
	return;
}
boost::tuple<std::string,std::string> rr_res=RobotRaconteur::SplitQualifiedName(rr_type);
if (rr_res.get<0>() != "edu.rpi.cats.sensors.webcam") GetNode()->DownCastAndThrowException(rr_exp);
return;
}
RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> edu__rpi__cats__sensors__webcamFactory::DownCastException(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> rr_exp){
if (!rr_exp) return rr_exp;
std::string rr_type=rr_exp->Error;
if (rr_type.find('.')==std::string::npos)
{
	return rr_exp;
}
boost::tuple<std::string,std::string> rr_res=RobotRaconteur::SplitQualifiedName(rr_type);
if (rr_res.get<0>() != "edu.rpi.cats.sensors.webcam") return GetNode()->DownCastException(rr_exp);
return rr_exp;
}

Webcam_stub::Webcam_stub(const std::string &path, RR_SHARED_PTR<RobotRaconteur::ClientContext> c) : RobotRaconteur::ServiceStub(path,c){ }
void Webcam_stub::RRInitStub()
{
rrvar_ImageStream=RR_MAKE_SHARED<RobotRaconteur::PipeClient<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > > >("ImageStream",shared_from_this()) ;
}

RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > Webcam_stub::getCurrentImage()
{
RR_SHARED_PTR<RobotRaconteur::MessageEntry> rr_req=RR_MAKE_SHARED<RobotRaconteur::MessageEntry>(RobotRaconteur::MessageEntryType_FunctionCallReq,"getCurrentImage");
RR_SHARED_PTR<RobotRaconteur::MessageEntry> rr_ret=ProcessTransaction(rr_req);
RR_SHARED_PTR<RobotRaconteur::MessageElement> rr_me=rr_ret->FindElement("return");
return RobotRaconteur::rr_cast<edu::rpi::cats::sensors::camera_interface::Image >(RRGetNode()->UnpackStructure(rr_me->CastData<RobotRaconteur::MessageElementStructure>()));
}

RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > Webcam_stub::getImageHeader()
{
RR_SHARED_PTR<RobotRaconteur::MessageEntry> rr_req=RR_MAKE_SHARED<RobotRaconteur::MessageEntry>(RobotRaconteur::MessageEntryType_FunctionCallReq,"getImageHeader");
RR_SHARED_PTR<RobotRaconteur::MessageEntry> rr_ret=ProcessTransaction(rr_req);
RR_SHARED_PTR<RobotRaconteur::MessageElement> rr_me=rr_ret->FindElement("return");
return RobotRaconteur::rr_cast<edu::rpi::cats::sensors::camera_interface::ImageHeader >(RRGetNode()->UnpackStructure(rr_me->CastData<RobotRaconteur::MessageElementStructure>()));
}

void Webcam_stub::StartStreaming()
{
RR_SHARED_PTR<RobotRaconteur::MessageEntry> rr_req=RR_MAKE_SHARED<RobotRaconteur::MessageEntry>(RobotRaconteur::MessageEntryType_FunctionCallReq,"StartStreaming");
RR_SHARED_PTR<RobotRaconteur::MessageEntry> rr_ret=ProcessTransaction(rr_req);
}

void Webcam_stub::StopStreaming()
{
RR_SHARED_PTR<RobotRaconteur::MessageEntry> rr_req=RR_MAKE_SHARED<RobotRaconteur::MessageEntry>(RobotRaconteur::MessageEntryType_FunctionCallReq,"StopStreaming");
RR_SHARED_PTR<RobotRaconteur::MessageEntry> rr_ret=ProcessTransaction(rr_req);
}

RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > > > Webcam_stub::get_ImageStream()
{
RR_SHARED_PTR<RobotRaconteur::PipeClient<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > > > value=rrvar_ImageStream;
if (!value) throw std::runtime_error("Stub has been closed");
return value;
}
void Webcam_stub::set_ImageStream(RR_SHARED_PTR<RobotRaconteur::Pipe<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > > > value)
{
throw std::runtime_error("Not valid for client");
}

void Webcam_stub::DispatchEvent(RR_SHARED_PTR<RobotRaconteur::MessageEntry> rr_m)
{
throw RobotRaconteur::MemberNotFoundException("Member not found");
}
void Webcam_stub::DispatchPipeMessage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m)
{
if (m->MemberName=="ImageStream")
{
rrvar_ImageStream->PipePacketReceived(m);
return;
}
throw RobotRaconteur::MemberNotFoundException("Member not found");
}
void Webcam_stub::DispatchWireMessage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m)
{
throw RobotRaconteur::MemberNotFoundException("Member not found");
}
RR_SHARED_PTR<RobotRaconteur::MessageEntry>Webcam_stub::CallbackCall(RR_SHARED_PTR<RobotRaconteur::MessageEntry> rr_m)
{
std::string ename=rr_m->MemberName;
RR_SHARED_PTR<RobotRaconteur::MessageEntry> rr_mr=RR_MAKE_SHARED<RobotRaconteur::MessageEntry>(RobotRaconteur::MessageEntryType_CallbackCallRet, ename);
rr_mr->ServicePath=rr_m->ServicePath;
rr_mr->TransactionID=rr_m->TransactionID;
throw RobotRaconteur::MemberNotFoundException("Member not found");
}

std::string Webcam_stub::RRType()
{
return "edu.rpi.cats.sensors.webcam.Webcam";
}
void Webcam_stub::RRClose()
{
rrvar_ImageStream->Shutdown();
ServiceStub::RRClose();
}

void Webcam_stub::async_getCurrentImage(boost::function<void (RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout)
{
RR_SHARED_PTR<RobotRaconteur::MessageEntry> rr_req=RR_MAKE_SHARED<RobotRaconteur::MessageEntry>(RobotRaconteur::MessageEntryType_FunctionCallReq,"getCurrentImage");
AsyncProcessTransaction(rr_req,boost::bind(&Webcam_stub::rrend_getCurrentImage, RobotRaconteur::rr_cast<Webcam_stub>(shared_from_this()),_1,_2,rr_handler ),rr_timeout);
}

void Webcam_stub::rrend_getCurrentImage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler)
{
if (err)
{
handler(RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image >(),err);
return;
}
if (m->Error != RobotRaconteur::MessageErrorType_None)
{
handler(RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image >(),RobotRaconteur::RobotRaconteurExceptionUtil::MessageEntryToException(m));
return;
}
RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > rr_ret;
try
{
RR_SHARED_PTR<RobotRaconteur::MessageElement> me=m->FindElement("return");
rr_ret=RobotRaconteur::rr_cast<edu::rpi::cats::sensors::camera_interface::Image >(RRGetNode()->UnpackStructure(me->CastData<RobotRaconteur::MessageElementStructure>()));
}
catch (RobotRaconteur::RobotRaconteurException& err)
{
handler(RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image >(),RobotRaconteur::RobotRaconteurExceptionUtil::DownCastException(err));
return;
}
catch (std::exception& err)
{
handler(RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image >(),RR_MAKE_SHARED<RobotRaconteur::RobotRaconteurRemoteException>(std::string(typeid(err).name()),err.what()));
return;
}
handler(rr_ret, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>());
}
void Webcam_stub::async_getImageHeader(boost::function<void (RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader >, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout)
{
RR_SHARED_PTR<RobotRaconteur::MessageEntry> rr_req=RR_MAKE_SHARED<RobotRaconteur::MessageEntry>(RobotRaconteur::MessageEntryType_FunctionCallReq,"getImageHeader");
AsyncProcessTransaction(rr_req,boost::bind(&Webcam_stub::rrend_getImageHeader, RobotRaconteur::rr_cast<Webcam_stub>(shared_from_this()),_1,_2,rr_handler ),rr_timeout);
}

void Webcam_stub::rrend_getImageHeader(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > ,RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler)
{
if (err)
{
handler(RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader >(),err);
return;
}
if (m->Error != RobotRaconteur::MessageErrorType_None)
{
handler(RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader >(),RobotRaconteur::RobotRaconteurExceptionUtil::MessageEntryToException(m));
return;
}
RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > rr_ret;
try
{
RR_SHARED_PTR<RobotRaconteur::MessageElement> me=m->FindElement("return");
rr_ret=RobotRaconteur::rr_cast<edu::rpi::cats::sensors::camera_interface::ImageHeader >(RRGetNode()->UnpackStructure(me->CastData<RobotRaconteur::MessageElementStructure>()));
}
catch (RobotRaconteur::RobotRaconteurException& err)
{
handler(RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader >(),RobotRaconteur::RobotRaconteurExceptionUtil::DownCastException(err));
return;
}
catch (std::exception& err)
{
handler(RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader >(),RR_MAKE_SHARED<RobotRaconteur::RobotRaconteurRemoteException>(std::string(typeid(err).name()),err.what()));
return;
}
handler(rr_ret, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>());
}
void Webcam_stub::async_StartStreaming(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout)
{
RR_SHARED_PTR<RobotRaconteur::MessageEntry> rr_req=RR_MAKE_SHARED<RobotRaconteur::MessageEntry>(RobotRaconteur::MessageEntryType_FunctionCallReq,"StartStreaming");
AsyncProcessTransaction(rr_req,boost::bind(&Webcam_stub::rrend_StartStreaming, RobotRaconteur::rr_cast<Webcam_stub>(shared_from_this()),_1,_2,rr_handler ),rr_timeout);
}

void Webcam_stub::rrend_StartStreaming(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler)
{
if (err)
{
handler(err);
return;
}
if (m->Error != RobotRaconteur::MessageErrorType_None)
{
handler(RobotRaconteur::RobotRaconteurExceptionUtil::MessageEntryToException(m));
return;
}
handler(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>());
}
void Webcam_stub::async_StopStreaming(boost::function<void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > rr_handler, int32_t rr_timeout)
{
RR_SHARED_PTR<RobotRaconteur::MessageEntry> rr_req=RR_MAKE_SHARED<RobotRaconteur::MessageEntry>(RobotRaconteur::MessageEntryType_FunctionCallReq,"StopStreaming");
AsyncProcessTransaction(rr_req,boost::bind(&Webcam_stub::rrend_StopStreaming, RobotRaconteur::rr_cast<Webcam_stub>(shared_from_this()),_1,_2,rr_handler ),rr_timeout);
}

void Webcam_stub::rrend_StopStreaming(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, boost::function< void (RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>) > handler)
{
if (err)
{
handler(err);
return;
}
if (m->Error != RobotRaconteur::MessageErrorType_None)
{
handler(RobotRaconteur::RobotRaconteurExceptionUtil::MessageEntryToException(m));
return;
}
handler(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException>());
}

void Webcam_skel::Init(const std::string& path, RR_SHARED_PTR<RobotRaconteur::RRObject> object, RR_SHARED_PTR<RobotRaconteur::ServerContext> context)
{
uncastobj=object;
rr_InitPipeServersRun=false;
rr_InitWireServersRun=false;
ServiceSkel::Init(path,object,context);
}
RR_SHARED_PTR<edu::rpi::cats::sensors::webcam::Webcam > Webcam_skel::get_obj()
{
return RobotRaconteur::rr_cast<edu::rpi::cats::sensors::webcam::Webcam >(uncastobj);
}
RR_SHARED_PTR<edu::rpi::cats::sensors::webcam::async_Webcam > Webcam_skel::get_asyncobj()
{
return RR_DYNAMIC_POINTER_CAST<edu::rpi::cats::sensors::webcam::async_Webcam >(uncastobj);
}
void Webcam_skel::ReleaseCastObject() 
{
rr_ImageStream_pipe->Shutdown();
}
std::string Webcam_skel::GetObjectType()
{
return "edu.rpi.cats.sensors.webcam.Webcam";
}
RR_SHARED_PTR<RobotRaconteur::MessageEntry> Webcam_skel::CallGetProperty(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m)
{
RR_SHARED_PTR<RobotRaconteur::MessageEntry> mr=RR_MAKE_SHARED<RobotRaconteur::MessageEntry>(RobotRaconteur::MessageEntryType_PropertyGetRes,m->MemberName);
RR_SHARED_PTR<edu::rpi::cats::sensors::webcam::async_Webcam > async_obj=get_asyncobj();
throw RobotRaconteur::MemberNotFoundException("Member not found");
}

RR_SHARED_PTR<RobotRaconteur::MessageEntry> Webcam_skel::CallSetProperty(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m)
{
RR_SHARED_PTR<RobotRaconteur::MessageEntry> mr=RR_MAKE_SHARED<RobotRaconteur::MessageEntry>(RobotRaconteur::MessageEntryType_PropertySetRes,m->MemberName);
RR_SHARED_PTR<edu::rpi::cats::sensors::webcam::async_Webcam > async_obj=get_asyncobj();
throw RobotRaconteur::MemberNotFoundException("Member not found");
}

RR_SHARED_PTR<RobotRaconteur::MessageEntry> Webcam_skel::CallFunction(RR_SHARED_PTR<RobotRaconteur::MessageEntry> rr_m)
{
RR_SHARED_PTR<RobotRaconteur::MessageEntry> rr_mr=RR_MAKE_SHARED<RobotRaconteur::MessageEntry>(RobotRaconteur::MessageEntryType_FunctionCallRes,rr_m->MemberName);
RR_SHARED_PTR<edu::rpi::cats::sensors::webcam::async_Webcam > async_obj=get_asyncobj();
if (rr_m->MemberName == "getCurrentImage")
{
if (async_obj)
{
RR_WEAK_PTR<edu::rpi::cats::sensors::webcam::Webcam_skel> rr_wp=RobotRaconteur::rr_cast<edu::rpi::cats::sensors::webcam::Webcam_skel>(shared_from_this());
async_obj->async_getCurrentImage(boost::bind(&edu::rpi::cats::sensors::webcam::Webcam_skel::rr_getCurrentImage, rr_wp, _1, _2, rr_m, RobotRaconteur::ServerEndpoint::GetCurrentEndpoint()));
return RR_SHARED_PTR<RobotRaconteur::MessageEntry>();
}
else
{
RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > rr_return=get_obj()->getCurrentImage();
rr_mr->AddElement(RR_MAKE_SHARED<RobotRaconteur::MessageElement>("return",RobotRaconteur::rr_cast<RobotRaconteur::MessageElementData>(RRGetNode()->PackStructure(RobotRaconteur::rr_cast<RobotRaconteur::RRStructure>(rr_return)))));
return rr_mr;
}
}
if (rr_m->MemberName == "getImageHeader")
{
if (async_obj)
{
RR_WEAK_PTR<edu::rpi::cats::sensors::webcam::Webcam_skel> rr_wp=RobotRaconteur::rr_cast<edu::rpi::cats::sensors::webcam::Webcam_skel>(shared_from_this());
async_obj->async_getImageHeader(boost::bind(&edu::rpi::cats::sensors::webcam::Webcam_skel::rr_getImageHeader, rr_wp, _1, _2, rr_m, RobotRaconteur::ServerEndpoint::GetCurrentEndpoint()));
return RR_SHARED_PTR<RobotRaconteur::MessageEntry>();
}
else
{
RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > rr_return=get_obj()->getImageHeader();
rr_mr->AddElement(RR_MAKE_SHARED<RobotRaconteur::MessageElement>("return",RobotRaconteur::rr_cast<RobotRaconteur::MessageElementData>(RRGetNode()->PackStructure(RobotRaconteur::rr_cast<RobotRaconteur::RRStructure>(rr_return)))));
return rr_mr;
}
}
if (rr_m->MemberName == "StartStreaming")
{
if (async_obj)
{
RR_WEAK_PTR<edu::rpi::cats::sensors::webcam::Webcam_skel> rr_wp=RobotRaconteur::rr_cast<edu::rpi::cats::sensors::webcam::Webcam_skel>(shared_from_this());
async_obj->async_StartStreaming(boost::bind(&edu::rpi::cats::sensors::webcam::Webcam_skel::rr_StartStreaming,rr_wp, _1, rr_m, RobotRaconteur::ServerEndpoint::GetCurrentEndpoint()));
return RR_SHARED_PTR<RobotRaconteur::MessageEntry>();
}
else
{
get_obj()->StartStreaming();
rr_mr->AddElement("return",RobotRaconteur::ScalarToRRArray<int32_t>(0));
return rr_mr;
}
}
if (rr_m->MemberName == "StopStreaming")
{
if (async_obj)
{
RR_WEAK_PTR<edu::rpi::cats::sensors::webcam::Webcam_skel> rr_wp=RobotRaconteur::rr_cast<edu::rpi::cats::sensors::webcam::Webcam_skel>(shared_from_this());
async_obj->async_StopStreaming(boost::bind(&edu::rpi::cats::sensors::webcam::Webcam_skel::rr_StopStreaming,rr_wp, _1, rr_m, RobotRaconteur::ServerEndpoint::GetCurrentEndpoint()));
return RR_SHARED_PTR<RobotRaconteur::MessageEntry>();
}
else
{
get_obj()->StopStreaming();
rr_mr->AddElement("return",RobotRaconteur::ScalarToRRArray<int32_t>(0));
return rr_mr;
}
}
throw RobotRaconteur::MemberNotFoundException("Member not found");
}

void Webcam_skel::rr_getCurrentImage(RR_WEAK_PTR<edu::rpi::cats::sensors::webcam::Webcam_skel> skel, RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > ret, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep)
{
if(err)
{
EndAsyncCallFunction(skel,RR_SHARED_PTR<RobotRaconteur::MessageElement>(),err,m, ep);
return;
}
try
{
RR_SHARED_PTR<edu::rpi::cats::sensors::webcam::Webcam_skel> skel1=skel.lock();
if (!skel1) throw std::runtime_error("skel release");
RR_SHARED_PTR<RobotRaconteur::MessageElement> mr=RR_MAKE_SHARED<RobotRaconteur::MessageElement>("return",RobotRaconteur::rr_cast<RobotRaconteur::MessageElementData>(skel1->RRGetNode()->PackStructure(RobotRaconteur::rr_cast<RobotRaconteur::RRStructure>(ret))));
EndAsyncCallFunction(skel, mr, err, m,ep);
}
catch (RobotRaconteur::RobotRaconteurException& err2)
{
EndAsyncCallFunction(skel,RR_SHARED_PTR<RobotRaconteur::MessageElement>(),RobotRaconteur::RobotRaconteurExceptionUtil::DownCastException(err2),m, ep);
}
catch (std::exception& err2)
{
EndAsyncCallFunction(skel,RR_SHARED_PTR<RobotRaconteur::MessageElement>(),RR_MAKE_SHARED<RobotRaconteur::DataTypeException>(err2.what()),m, ep);
}
}
void Webcam_skel::rr_getImageHeader(RR_WEAK_PTR<edu::rpi::cats::sensors::webcam::Webcam_skel> skel, RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::ImageHeader > ret, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep)
{
if(err)
{
EndAsyncCallFunction(skel,RR_SHARED_PTR<RobotRaconteur::MessageElement>(),err,m, ep);
return;
}
try
{
RR_SHARED_PTR<edu::rpi::cats::sensors::webcam::Webcam_skel> skel1=skel.lock();
if (!skel1) throw std::runtime_error("skel release");
RR_SHARED_PTR<RobotRaconteur::MessageElement> mr=RR_MAKE_SHARED<RobotRaconteur::MessageElement>("return",RobotRaconteur::rr_cast<RobotRaconteur::MessageElementData>(skel1->RRGetNode()->PackStructure(RobotRaconteur::rr_cast<RobotRaconteur::RRStructure>(ret))));
EndAsyncCallFunction(skel, mr, err, m,ep);
}
catch (RobotRaconteur::RobotRaconteurException& err2)
{
EndAsyncCallFunction(skel,RR_SHARED_PTR<RobotRaconteur::MessageElement>(),RobotRaconteur::RobotRaconteurExceptionUtil::DownCastException(err2),m, ep);
}
catch (std::exception& err2)
{
EndAsyncCallFunction(skel,RR_SHARED_PTR<RobotRaconteur::MessageElement>(),RR_MAKE_SHARED<RobotRaconteur::DataTypeException>(err2.what()),m, ep);
}
}
void Webcam_skel::rr_StartStreaming(RR_WEAK_PTR<edu::rpi::cats::sensors::webcam::Webcam_skel> skel, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep)
{
if(err)
{
EndAsyncCallFunction(skel,RR_SHARED_PTR<RobotRaconteur::MessageElement>(),err,m, ep);
return;
}
try
{
RR_SHARED_PTR<RobotRaconteur::MessageElement> mr=RR_MAKE_SHARED<RobotRaconteur::MessageElement>("return",RobotRaconteur::ScalarToRRArray<int32_t>(0));
EndAsyncCallFunction(skel, mr, err, m,ep);
}
catch (RobotRaconteur::RobotRaconteurException& err2)
{
EndAsyncCallFunction(skel,RR_SHARED_PTR<RobotRaconteur::MessageElement>(),RobotRaconteur::RobotRaconteurExceptionUtil::DownCastException(err2),m, ep);
}
catch (std::exception& err2)
{
EndAsyncCallFunction(skel,RR_SHARED_PTR<RobotRaconteur::MessageElement>(),RR_MAKE_SHARED<RobotRaconteur::DataTypeException>(err2.what()),m, ep);
}
}
void Webcam_skel::rr_StopStreaming(RR_WEAK_PTR<edu::rpi::cats::sensors::webcam::Webcam_skel> skel, RR_SHARED_PTR<RobotRaconteur::RobotRaconteurException> err, RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::ServerEndpoint> ep)
{
if(err)
{
EndAsyncCallFunction(skel,RR_SHARED_PTR<RobotRaconteur::MessageElement>(),err,m, ep);
return;
}
try
{
RR_SHARED_PTR<RobotRaconteur::MessageElement> mr=RR_MAKE_SHARED<RobotRaconteur::MessageElement>("return",RobotRaconteur::ScalarToRRArray<int32_t>(0));
EndAsyncCallFunction(skel, mr, err, m,ep);
}
catch (RobotRaconteur::RobotRaconteurException& err2)
{
EndAsyncCallFunction(skel,RR_SHARED_PTR<RobotRaconteur::MessageElement>(),RobotRaconteur::RobotRaconteurExceptionUtil::DownCastException(err2),m, ep);
}
catch (std::exception& err2)
{
EndAsyncCallFunction(skel,RR_SHARED_PTR<RobotRaconteur::MessageElement>(),RR_MAKE_SHARED<RobotRaconteur::DataTypeException>(err2.what()),m, ep);
}
}
void Webcam_skel::RegisterEvents(RR_SHARED_PTR<RobotRaconteur::RRObject> rrobj1)
{
RR_SHARED_PTR<edu::rpi::cats::sensors::webcam::Webcam > obj=RobotRaconteur::rr_cast<edu::rpi::cats::sensors::webcam::Webcam >(rrobj1);
RobotRaconteur::ServiceSkel::RegisterEvents(rrobj1);
}

void Webcam_skel::UnregisterEvents(RR_SHARED_PTR<RobotRaconteur::RRObject> rrobj1)
{
RobotRaconteur::ServiceSkel::UnregisterEvents(rrobj1);
}

RR_SHARED_PTR<RobotRaconteur::RRObject> Webcam_skel::GetSubObj(const std::string &name, const std::string &ind)
{
throw RobotRaconteur::MemberNotFoundException("Member not found");
}

void Webcam_skel::InitPipeServers(RR_SHARED_PTR<RobotRaconteur::RRObject> rrobj1)
{
if (rr_InitPipeServersRun) return;
rr_InitPipeServersRun=true;
RR_SHARED_PTR<edu::rpi::cats::sensors::webcam::Webcam > obj=RobotRaconteur::rr_cast<edu::rpi::cats::sensors::webcam::Webcam >(rrobj1);
rr_ImageStream_pipe=RR_MAKE_SHARED<RobotRaconteur::PipeServer<RR_SHARED_PTR<edu::rpi::cats::sensors::camera_interface::Image > > >("ImageStream",shared_from_this());
obj->set_ImageStream(rr_ImageStream_pipe);
}

void Webcam_skel::DispatchPipeMessage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, uint32_t e)
{
if (m->MemberName=="ImageStream")
{
rr_ImageStream_pipe->PipePacketReceived(m,e);
return;
}
throw RobotRaconteur::MemberNotFoundException("Member not found");
}

RR_SHARED_PTR<RobotRaconteur::MessageEntry> Webcam_skel::CallPipeFunction(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, uint32_t e)
{
if (m->MemberName=="ImageStream")
{
return rr_ImageStream_pipe->PipeCommand(m,e);
}
throw RobotRaconteur::MemberNotFoundException("Member not found");
}

void Webcam_skel::InitWireServers(RR_SHARED_PTR<RobotRaconteur::RRObject> rrobj1)
{
if (rr_InitWireServersRun) return;
rr_InitWireServersRun=true;
RR_SHARED_PTR<edu::rpi::cats::sensors::webcam::Webcam > obj=RobotRaconteur::rr_cast<edu::rpi::cats::sensors::webcam::Webcam >(rrobj1);
}

void Webcam_skel::DispatchWireMessage(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, uint32_t e)
{
throw RobotRaconteur::MemberNotFoundException("Member not found");
}

RR_SHARED_PTR<RobotRaconteur::MessageEntry> Webcam_skel::CallWireFunction(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, uint32_t e)
{
throw RobotRaconteur::MemberNotFoundException("Member not found");
}

void Webcam_skel::InitCallbackServers(RR_SHARED_PTR<RobotRaconteur::RRObject> o)
{
RR_SHARED_PTR<edu::rpi::cats::sensors::webcam::Webcam > obj=RobotRaconteur::rr_cast<edu::rpi::cats::sensors::webcam::Webcam >(o);
}
RR_SHARED_PTR<void> Webcam_skel::GetCallbackFunction(uint32_t endpoint, const std::string& membername)
{
throw RobotRaconteur::MemberNotFoundException("Member not found");
}

RR_SHARED_PTR<RobotRaconteur::MessageEntry> Webcam_skel::CallMemoryFunction(RR_SHARED_PTR<RobotRaconteur::MessageEntry> m, RR_SHARED_PTR<RobotRaconteur::Endpoint> e)
{
throw RobotRaconteur::MemberNotFoundException("Member not found");
}
}
}
}
}
}

