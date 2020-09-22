/*
 * Simulation.cpp
 *
 *  Created on: 18. 11. 2015
 *      Author: Vladimir Matena
 */

#include "ROSOMNeT.h"

#include <iostream>
#include <thread>

#include <ros/ros.h>

#include <omnetpp.h>
#include <sectionbasedconfig.h>
#include <inifilereader.h>
#include <fsutils.h>
#include <appreg.h>


using namespace std;
using namespace ros;

//Register_GlobalConfigOption(CFGID_CONFIGURATION_CLASS, "configuration-class", CFG_STRING, "", "Part of the Envir plugin mechanism: selects the class from which all configuration information will be obtained. This option lets you replace omnetpp.ini with some other implementation, e.g. database input. The simulation program still has to bootstrap from an omnetpp.ini (which contains the configuration-class setting). The class should implement the cConfigurationEx interface.");

ROSOMNeT ROSOMNeT::instance;

// Helper macro
#define CREATE_BY_CLASSNAME(var,classname,baseclass,description) \
     baseclass *var ## _tmp = (baseclass *) createOne(classname); \
     var = dynamic_cast<baseclass *>(var ## _tmp); \
     if (!var) \
throw cRuntimeError("Class \"%s\" is not subclassed from " #baseclass, (const char *)classname);

ROSOMNeT& ROSOMNeT::getInstance(string stringROS) {
	cout << stringROS << endl;
	return instance;
}

ROSOMNeT::ROSOMNeT() {}

ROSOMNeT::~ROSOMNeT() {}

NodeHandle & ROSOMNeT::getROSNode() {
	return *rosNode;
}

void ROSOMNeT::stopROS() {
	cout << "Shutting down ROS" << endl;
	shutdown();
	cout << "Joining ROS thread" << endl;
	rosThread->join();

	delete rosNode;
	delete rosThread;
}

void ROSOMNeT::runROSNode(){ //(string stringROS)
	//cout << "Running ROS node" << endl;
	// Initialize ROS
	//init(M_string(), stringROS);
	
	init(M_string(), "ROSOMNeT");

	rosNode = new NodeHandle();

	rosThread = new thread(&ROSOMNeT::rosMain, this);
}

void ROSOMNeT::rosMain() {
	cout << "ROS Main spinning on ROS" << endl;
	
	spin();
	//ros::spin();
	//spinOnce();

	//ros::MultiThreadedSpinner spinner(2); 
    //spinner.spin();

	//ros::MultiThreadedSpinner::spin(); 	
	cout << "No more spinning on ROS" << endl;
}

