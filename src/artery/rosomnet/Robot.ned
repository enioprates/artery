package artery.rosomnet;

import artery.rosomnet.RobotMiddleware;
import artery.nic.IRadioDriver;
import artery.inet.InetRadioDriver;
import inet.node.inet.INetworkNode;
import inet.linklayer.contract.IWirelessNic;
//import artery.networking.Vanetza;
import inet.mobility.contract.IMobility;
import inet.networklayer.common.InterfaceTable;
import artery.inet.PlainVehicle;
import artery.application.VehicleMiddleware;
import inet.mobility.static.StationaryMobility;

//extends PlainRobot
module Robot like INetworkNode
{
    parameters:
        //@display("i=block/wrxtx;is=vs");
        @display("i=car/car_s");
        @networkNode;
        @labels(node,ethernet-node,wireless-node);

        @statistic[posX](source="xCoord(mobilityPos(mobilityStateChanged))"; record=vector?);
        @statistic[posY](source="yCoord(mobilityPos(mobilityStateChanged))"; record=vector?);
        
        int numRadios = default(1);
        *.interfaceTableModule = default(absPath(".interfaceTable"));

    gates:
        input radioIn[numRadios] @directIn;
        inout pppg[] @labels(PPPFrame-conn);
        inout ethg[] @labels(EtherFrame-conn);

    submodules:

       //app: ROSForwarderApplication {}
    
        // Mobility module
        mobility: MobilityROS {
        }

        interfaceTable: InterfaceTable {
            parameters:
                @display("p=50,300");
        }

        wlan[numRadios]: <default("VanetNic")> like IWirelessNic {
            parameters:
                @display("p=250,400,row,100;q=queue");
        }

        radioDriver[numRadios]: InetRadioDriver {
            parameters:
                @display("p=250,300,row,100");
                macModule = ".wlan[" + string(index) + "].mac";
        }
       
        middleware: RobotMiddleware {
            parameters:
                @display("p=250,200");
                radioDriverModule = ".radioDriver[0]";
                mobilityModule = ".mobility1";
        }
        

    connections:
        for i=0..sizeof(radioIn)-1 {
            radioIn[i] --> { @display("m=s"); } --> wlan[i].radioIn;

            wlan[i].upperLayerOut --> radioDriver[i].lowerLayerIn;
            wlan[i].upperLayerIn <-- radioDriver[i].lowerLayerOut;
        }

        middleware.radioDriverOut --> radioDriver[0].middleware$i;
        middleware.radioDriverIn <-- radioDriver[0].middleware$o;
}
