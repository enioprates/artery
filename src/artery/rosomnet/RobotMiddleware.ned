//
// Artery V2X Simulation Framework
// Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
//

import artery.rosomnet.Middleware;
package artery.rosomnet;

// Middleware for ITS G5 services
simple RobotMiddleware extends Middleware
{
	parameters:
		@class(artery::RobotMiddleware);

		string localEnvironmentModule = default("");
		string globalEnvironmentModule = default("");
		string mobilityModule;
}
