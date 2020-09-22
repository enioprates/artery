#ifndef SIMULATION_H
#define SIMULATION_H

#include <string>
#include <thread>
#include <mutex>

#include <ros/ros.h>

using namespace std;
using namespace ros;

class ROSOMNeT {
public:
	ROSOMNeT();
	~ROSOMNeT();

	static ROSOMNeT &getInstance(string stringROS);

	void runROSNode(); //(string stringROS)
	//void runSimulation(string configFileName);
	void stopROS();
	NodeHandle &getROSNode();

	//mutex mutexROS;

private:
	static ROSOMNeT instance;

	std::string configFileName;

	// This has to be unfortunately static as there is no other way how to pass data to OMNeT++ applications
	NodeHandle *rosNode;

	thread* rosThread;
	void rosMain();
};

#endif /* SIMULATION_H */
