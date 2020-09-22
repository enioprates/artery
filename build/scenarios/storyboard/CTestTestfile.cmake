# CMake generated Testfile for 
# Source directory: /home/enio/OneDrive/Cister/ROS/artery/scenarios/storyboard
# Build directory: /home/enio/OneDrive/Cister/ROS/artery/build/scenarios/storyboard
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(storyboard-inet "/home/enio/omnetpp-5.6/bin/opp_run_dbg" "-n" "/home/enio/OneDrive/Cister/ROS/artery/src/artery:/home/enio/OneDrive/Cister/ROS/artery/src/traci:/home/enio/OneDrive/Cister/ROS/artery/extern/veins/examples/veins:/home/enio/OneDrive/Cister/ROS/artery/extern/veins/src/veins:/home/enio/OneDrive/Cister/ROS/artery/extern/inet/src:/home/enio/OneDrive/Cister/ROS/artery/extern/inet/examples:/home/enio/OneDrive/Cister/ROS/artery/extern/inet/tutorials:/home/enio/OneDrive/Cister/ROS/artery/extern/inet/showcases" "-l" "/home/enio/OneDrive/Cister/ROS/artery/build/src/artery/envmod/libartery_envmod.so" "-l" "/home/enio/OneDrive/Cister/ROS/artery/build/scenarios/highway-police/libartery_police.so" "-l" "/home/enio/OneDrive/Cister/ROS/artery/build/src/artery/envmod/libartery_envmod.so" "-l" "/home/enio/OneDrive/Cister/ROS/artery/build/src/artery/storyboard/libartery_storyboard.so" "-l" "/home/enio/OneDrive/Cister/ROS/artery/extern/inet/out/gcc-debug/src/libINET_dbg.so" "-l" "/home/enio/OneDrive/Cister/ROS/artery/extern/veins/out/gcc-debug/src/libveins_dbg.so" "-l" "/home/enio/OneDrive/Cister/ROS/artery/build/src/artery/libartery_core.so" "omnetpp.ini" "-uCmdenv" "--sim-time-limit=60s")
set_tests_properties(storyboard-inet PROPERTIES  WORKING_DIRECTORY "/home/enio/OneDrive/Cister/ROS/artery/scenarios/storyboard")
add_test(storyboard-veins "/home/enio/omnetpp-5.6/bin/opp_run_dbg" "-n" "/home/enio/OneDrive/Cister/ROS/artery/src/artery:/home/enio/OneDrive/Cister/ROS/artery/src/traci:/home/enio/OneDrive/Cister/ROS/artery/extern/veins/examples/veins:/home/enio/OneDrive/Cister/ROS/artery/extern/veins/src/veins:/home/enio/OneDrive/Cister/ROS/artery/extern/inet/src:/home/enio/OneDrive/Cister/ROS/artery/extern/inet/examples:/home/enio/OneDrive/Cister/ROS/artery/extern/inet/tutorials:/home/enio/OneDrive/Cister/ROS/artery/extern/inet/showcases" "-l" "/home/enio/OneDrive/Cister/ROS/artery/build/src/artery/envmod/libartery_envmod.so" "-l" "/home/enio/OneDrive/Cister/ROS/artery/build/scenarios/highway-police/libartery_police.so" "-l" "/home/enio/OneDrive/Cister/ROS/artery/build/src/artery/envmod/libartery_envmod.so" "-l" "/home/enio/OneDrive/Cister/ROS/artery/build/src/artery/storyboard/libartery_storyboard.so" "-l" "/home/enio/OneDrive/Cister/ROS/artery/extern/inet/out/gcc-debug/src/libINET_dbg.so" "-l" "/home/enio/OneDrive/Cister/ROS/artery/extern/veins/out/gcc-debug/src/libveins_dbg.so" "-l" "/home/enio/OneDrive/Cister/ROS/artery/build/src/artery/libartery_core.so" "omnetpp.ini" "-uCmdenv" "-cveins" "--sim-time-limit=60s")
set_tests_properties(storyboard-veins PROPERTIES  WORKING_DIRECTORY "/home/enio/OneDrive/Cister/ROS/artery/scenarios/storyboard")