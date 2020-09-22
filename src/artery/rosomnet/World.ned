package artery.rosomnet;

import artery.storyboard.Storyboard;
import inet.physicallayer.contract.packetlevel.IRadioMedium;
import traci.Manager;
import artery.rosomnet.Robot;
import inet.physicallayer.ieee80211.packetlevel.Ieee80211ScalarRadioMedium;
import artery.inet.RSU;


import inet.environment.common.PhysicalEnvironment;



network World
{
    parameters:

        double playgroundSizeX @unit(m) = default(2000m); // x size of the area the nodes are in (in meters)
        double playgroundSizeY @unit(m) = default(2000m); // y size of the area the nodes are in (in meters)
        double playgroundSizeZ @unit(m) = default(2000m); // z size of the area the nodes are in (in meters)
        
        bool visualize = default(true);
        double playgroundLatitude = default(0.0); // geographic position of the playground's north-west corner
        double playgroundLongitude = default(0.0); // geographic position of the playground's north-west corner

        //int numRoadSideUnits = default(0);

        int numRoadSideUnits = 0;

        int numOfCars = 40;

    submodules:

        
        radioMedium: <default("Ieee80211ScalarRadioMedium")> like IRadioMedium {
            parameters:
                @display("p=1500,1500");
                mediumLimitCache.carrierFrequency = 5.9GHz;
        }

        rsu[numRoadSideUnits]: RSU {
            parameters:
                mobility.initFromDisplayString = false;
        }

        syncNode: ROSSyncNode{
            @display("p=1500,1500");
        };

        car[numOfCars]: Robot{
        }

       
}
