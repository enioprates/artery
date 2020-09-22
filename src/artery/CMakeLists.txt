find_package(Boost COMPONENTS date_time system REQUIRED)

find_package(roscpp)

#find_package(catkin REQUIRED COMPONENTS message_generation roscpp rostime rosconsole roscpp_serialization ros_its_msgs) 
find_package(ros_its_msgs)
#find_package(rostime)
#find_package(roscpp_serialization)
#find_package(rosconsole)

add_library(artery INTERFACE)
add_library(core SHARED
    #application/CaObject.cc
    #application/CaService.cc
    application/DenmObject.cc
    application/DenService.cc
    application/ExampleService.cc
    #application/Facilities.cc
    #application/ItsG5BaseService.cc
    #application/ItsG5PromiscuousService.cc
    #application/ItsG5Service.cc
    #application/LocalDynamicMap.cc
    #application/Middleware.cc
    #application/StationaryMiddleware.cc
    #application/StoryboardSignal.cc
    #application/Timer.cc
    #application/VehicleDataProvider.cc
    application/VehicleMiddleware.cc
    application/den/ImpactReductionUseCase.cc
    application/den/Memory.cc
    application/den/SuspendableUseCase.cc
    application/den/TrafficJamUseCase.cc
    application/den/UseCase.cc
    netw/GeoNetPacketWrapper.cc
    nic/RadioDriverBase.cc
    traci/MobilityBase.cc
    traci/VehicleController.cc
    traci/VehicleType.cc
    utility/AsioScheduler.cc
    utility/AsioTask.cc
    utility/IdentityRegistry.cc
    utility/FilterRules.cc
    utility/Geometry.cc

    rosomnet/ROSOMNeT.cpp
    #rosomnet/ROSForwarderApplication.cpp
    rosomnet/ROSSyncApplication.cpp
    rosomnet/MobilityROS.cpp
    rosomnet/VehicleDataProvider.cpp
    rosomnet/RobotMiddleware.cpp
    rosomnet/CaService.cc
    #rosomnet/RawService.cc
    rosomnet/ItsG5Service.cc
    rosomnet/Facilities.cc
    rosomnet/CaObject.cc
    rosomnet/RawObject.cc
    rosomnet/LocalDynamicMap.cc
    rosomnet/Middleware.cc
    rosomnet/ItsG5BaseService.cc
    rosomnet/ItsG5PromiscuousService.cc
    rosomnet/StationaryMiddleware.cc
    rosomnet/StoryboardSignal.cc
    rosomnet/Timer.cc

    rosomnet/asn1/RAW.c
    rosomnet/asn1/RAWHeader.c
    rosomnet/asn1/PlatoonPayload.c
    rosomnet/asn1/RawStationID.c
    rosomnet/asn1/CarID.c
    rosomnet/asn1/RawSteeringWheelAngle.c
    rosomnet/asn1/RawHeading.c
    rosomnet/asn1/RawSpeed.c
    rosomnet/asn1/XX.c
    rosomnet/asn1/YY.c
    rosomnet/asn1/ZZ.c
)
target_link_libraries(artery INTERFACE core)
add_library(Artery::Core ALIAS core)

generate_opp_message(messages/AsioData.msg TARGET core)
generate_opp_message(messages/ChannelLoadReport.msg TARGET core)
generate_opp_message(messages/GeoNetPacket.msg TARGET core)

#generate_opp_message(rosomnet/IEEE802154.msg TARGET core)

set_target_properties(core PROPERTIES
    OUTPUT_NAME artery_core
    OMNETPP_LIBRARY ON
    NED_FOLDERS ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(core PUBLIC ${PROJECT_SOURCE_DIR}/src)
target_include_directories(core PUBLIC ${Boost_INCLUDE_DIRS})

#target_include_directories(core PUBLIC /opt/ros/kinetic/include)
#target_include_directories(core PUBLIC /home/enio/OneDrive/Cister/ROS/GazeboTrackPlatoon/CISTER_car_simulator/devel/include)
target_include_directories(core PUBLIC /home/enio/OneDrive/Cister/ROS/Inline/CISTER_car_simulator/devel/include)
include_directories(${catkin_INCLUDE_DIRS})
target_include_directories(core PUBLIC /home/enio/omnetpp-5.6/src/common/)
target_include_directories(core PUBLIC /home/enio/omnetpp-5.6/src/envir/)
target_include_directories(core PUBLIC /home/enio/omnetpp-5.6/include/)
target_include_directories(core PUBLIC /home/enio/omnetpp-5.6/src/)

target_include_directories(core PUBLIC  ${catkin_INCLUDE_DIRS})
target_include_directories(core PUBLIC  ${roscpp_INCLUDE_DIRS})


target_link_libraries(core PUBLIC ${Boost_LIBRARIES})
target_link_libraries(core PUBLIC opp_interface)
target_link_libraries(core PUBLIC traci)
target_link_libraries(core PUBLIC Vanetza::vanetza)

target_link_libraries(core PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${rosconsole_LIBRARIES} ${roscpp_LIBRARIES})

if(TARGET veins)
    message(STATUS "Enable Veins integration")
    set_property(TARGET core APPEND PROPERTY SOURCES
        mac/ChannelLoadMeasurements.cc
        nic/VeinsRadioDriver.cc
        veins/VeinsConnectionManager.cc
        veins/VeinsMobility.cc
        veins/VeinsObstacleControl.cc)
    target_link_libraries(core PUBLIC veins)
endif()

if(TARGET INET)
    message(STATUS "Enable INET integration")
    set_property(TARGET core APPEND PROPERTY SOURCES
        inet/InetRadioDriver.cc
        inet/InetMobility.cc
        inet/VanetRx.cc)
    target_link_libraries(core PUBLIC INET)
endif()

macro(add_artery_feature name)
    add_library(${name} SHARED ${ARGN})
    add_library(Artery::${name} ALIAS ${name})
    set_target_properties(${name} PROPERTIES
        OUTPUT_NAME artery_${name}
        OMNETPP_LIBRARY ON)
    target_link_libraries(${name} PRIVATE core)
    target_link_libraries(artery INTERFACE Artery::${name})
endmacro()

if(WITH_STORYBOARD)
    add_subdirectory(storyboard)
endif()

if(WITH_TRANSFUSION)
    add_subdirectory(transfusion)
endif()

if(WITH_ENVMOD)
    add_subdirectory(envmod)
    set_property(SOURCE application/VehicleMiddleware.cc APPEND PROPERTY COMPILE_DEFINITIONS "WITH_ENVMOD")
endif()

if(WITH_TESTBED)
    add_subdirectory(testbed)
endif()
