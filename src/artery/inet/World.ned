package artery.inet;

import artery.storyboard.Storyboard;
import inet.physicallayer.contract.packetlevel.IRadioMedium;
import traci.Manager;
import artery.rosomnet.Robot;
import artery.rosomnet.ROSSyncNode;

network World
{
    parameters:
        bool withStoryboard = default(false);
        int numRoadSideUnits = default(0);
        int numOfCars = 3;
        traci.mapper.vehicleType = default("artery.inet.Car");
        //traci.mapper.vehicleType = default("artery.rosomnet.Robot");
        traci.nodes.vehicleSinkModule = default(".mobility");
        storyboard.middlewareModule = default(".middleware");

    submodules:
        traci: Manager {
            parameters:
                @display("p=20,30");
        }

        radioMedium: <default("Ieee80211ScalarRadioMedium")> like IRadioMedium {
            parameters:
                @display("p=200,20");
                mediumLimitCache.carrierFrequency = 5.9GHz;
        }

        storyboard: Storyboard if withStoryboard {
            parameters:
                @display("p=100,20");
        }

        rsu[numRoadSideUnits]: RSU {
            parameters:
                mobility.initFromDisplayString = false;
        }

        //car[numOfCars]: Robot{
        //}
        
        //syncNode: ROSSyncNode;
}
