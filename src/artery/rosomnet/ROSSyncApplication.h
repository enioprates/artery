/*
 * ROSSyncApplication.h
 *
 *  Created on: 19. 11. 2015
 *      Author: Vladimir Matena
 */

#ifndef SRC_ROSSYNCAPPLICATION_H
#define SRC_ROSSYNCAPPLICATION_H

#include <string>
#include <condition_variable>
#include <mutex>

#include <omnetpp.h>
#include <inet/mobility/contract/IMobility.h>
//#include <omnetpp/csimplemodule.h>

#include <rosgraph_msgs/Clock.h>
#include "ROSOMNeT.h"

using namespace inet;
using namespace std;
using namespace ros;

class ROSSyncApplication: public cSimpleModule {

//friend class cSimpleModule;
public:
		ROSSyncApplication();
		~ROSSyncApplication();


		const double TIME_STEP = 0.001;
		const long CLOCK_QUEUE_LENGTH = 1000;
		int i;
		double tempo;



private:
		const char* ROS_SYNC_MESSAGE = "@ROSSyncMessage@";
		const string CLOCK_TOPIC = "/clock";

		ROSOMNeT& rosomnet;
		Subscriber clockSubscriber;
		cMessage* syncMsg;
		mutex syncMutex;
		condition_variable syncCondition;

		void initialize(int stage);
		void initializeStage0();
		void initializeStage1();
		int numInitStages() const { return 2; }
		void handleMessage(cMessage *msg);
		void clockCallback(const rosgraph_msgs::Clock &msg);


};

#endif /* SRC_ROSSYNCAPPLICATION_H */
