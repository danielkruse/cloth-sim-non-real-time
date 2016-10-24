

#include "cloth_sim_impl.h"

int main(int argc, char *argv[])
{
	// Register Local Transport
	boost::shared_ptr<RobotRaconteur::LocalTransport> t1 = boost::make_shared<RobotRaconteur::LocalTransport>();
	t1->StartServerAsNodeName("edu.rpi.cats.utilities.clothsim");
	RobotRaconteur::RobotRaconteurNode::s()->RegisterTransport(t1);

	// Register TCP Transport on port 8888
	boost::shared_ptr<RobotRaconteur::TcpTransport> t = boost::make_shared<RobotRaconteur::TcpTransport>();
	t->StartServer(10789);
	t->EnableNodeAnnounce(RobotRaconteur::IPNodeDiscoveryFlags_LINK_LOCAL |
							RobotRaconteur::IPNodeDiscoveryFlags_NODE_LOCAL |
							RobotRaconteur::IPNodeDiscoveryFlags_SITE_LOCAL);
	RobotRaconteur::RobotRaconteurNode::s()->RegisterTransport(t);
	
	std::string in_string;
	btScalar width, length;
	std::cout << "How long is the cloth in X (0.6 default): ";
	std::getline(std::cin, in_string);
	if (in_string.length() == 0)
		width = 0.6;
	else
		width = stof(in_string);
	std::cout << "How long is the cloth in Y (0.4 default): ";
	std::getline(std::cin, in_string);
	if (in_string.length() == 0)
		length = 0.4;
	else
		length = stof(in_string);

	std::cout << "w: " << width << " l: " << length << std::endl;
	

	// Create the Kinect object (nx ny w h m k)
	boost::shared_ptr<ClothSimImpl> k = boost::make_shared<ClothSimImpl>(31, 31, width, length, 0.5, 1.0f, 1.0f, 0.1f);

	// Register the service type with Robot Raconteur
	RobotRaconteur::RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<edu::rpi::cats::utilities::clothsim::edu__rpi__cats__utilities__clothsimFactory>());


	// Register the Kinect object as a service
	RobotRaconteur::RobotRaconteurNode::s()->RegisterService("ClothSimulator", "edu.rpi.cats.utilities.clothsim", k);

	std::cout << "Connect to the Cloth Simulator Service at: " << std::endl;
	std::cout << "tcp://localhost:10789/edu.rpi.cats.utilities.clothsim/ClothSimulator" << std::endl;
	std::cout << "Press enter to finish" << std::endl;

	std::getline(std::cin, std::string());
	
	k->shutdown();

	//std::getline(std::cin, std::string());

	

	return 0;
}