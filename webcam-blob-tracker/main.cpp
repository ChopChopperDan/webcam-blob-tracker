
#include "webcam_impl.h"

int main(int argc, char *argv[])
{
	std::string dummy;

	// Register Local Transport
	boost::shared_ptr<RobotRaconteur::LocalTransport> t1 = boost::make_shared<RobotRaconteur::LocalTransport>();
	t1->StartServerAsNodeName("edu.rpi.cats.sensors.webcam");
	RobotRaconteur::RobotRaconteurNode::s()->RegisterTransport(t1);

	// Register TCP Transport on port 78911
	boost::shared_ptr<RobotRaconteur::TcpTransport> t = boost::make_shared<RobotRaconteur::TcpTransport>();
	t->StartServer(78911);
	t->EnableNodeAnnounce(RobotRaconteur::IPNodeDiscoveryFlags_LINK_LOCAL |
		RobotRaconteur::IPNodeDiscoveryFlags_NODE_LOCAL |
		RobotRaconteur::IPNodeDiscoveryFlags_SITE_LOCAL);
	RobotRaconteur::RobotRaconteurNode::s()->RegisterTransport(t);

	// Create the Kinect object
	boost::shared_ptr<webcam_impl> w = boost::make_shared<webcam_impl>();

	// Register the service type with Robot Raconteur
	RobotRaconteur::RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<edu::rpi::cats::sensors::camera_interface::edu__rpi__cats__sensors__camera_interfaceFactory>());
	RobotRaconteur::RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<edu::rpi::cats::sensors::webcam::edu__rpi__cats__sensors__webcamFactory>());

	// Register the Kinect object as a service
	RobotRaconteur::RobotRaconteurNode::s()->RegisterService("Webcam", "edu.rpi.cats.sensors.webcam", w);

	w->StartStreaming();

	std::cout << "Connect to the Webcam Service at: " << std::endl;
	std::cout << "tcp://localhost:78911/edu.rpi.cats.sensors.webcam/Webcam" << std::endl;
	std::cout << "Press enter to finish" << std::endl;
	std::getline(std::cin, dummy);
}