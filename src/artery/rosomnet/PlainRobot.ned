package artery.rosomnet;

import inet.mobility.contract.IMobility;

module PlainRobot
{
    parameters:
        @display("i=blocki/process;is=vs");
        @labels(node);
        mobility.visualRepresentation = "^";

    submodules:
        mobility: <default("artery.inet.Mobility")> like IMobility {
            parameters:
                @display("p=50,200");
        }
}
