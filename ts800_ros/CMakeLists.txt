cmake_minimum_required(VERSION 3.0.2)
project(ts800_ros)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS -rdynamic)
set(CMAKE_BUILD_TYPE Debug)
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g -O0 -Wall -Wextra -Wpedantic")

FILE(GLOB delta_lidar_SDK_SRC   
  "source/application/external_library/delta_2c_lidar/src/*.cpp"
)

include_directories(${Boost_INCLUDE_DIRS})

# Find the current data and time
string(TIMESTAMP BUILD_TIME "%Y-%m-%d %H:%M:%S")
configure_file(source/application/build_time.h.in
              ${CMAKE_SOURCE_DIR}/ts800/application/ros/ts800_ros/source/application/build_time.h
              @ONLY
              IMMEDIATE @ONLY
              NEWLINE_STYLE CRLF
              FORCE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  dynamic_reconfigure
  std_msgs
  #robot_localization
  sensor_msgs
  message_generation
  cv_bridge
)

find_package(Boost REQUIRED COMPONENTS
  thread
  filesystem
)
find_package(yaml-cpp REQUIRED)
find_package( OpenCV REQUIRED)

# carotgrper
#find_package(cartographer REQUIRED)
#find_package(cartographer_ros_msgs REQUIRED)

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/RobotParameter.cfg
)

# add_message_files
add_message_files(
  FILES
  Feedback.msg
  RotationVector.msg
  SubmapEntry.msg
  SubmapList.msg
  SlamControl.msg
)

# generate_messages
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES ts800_ros
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs cv_bridge message_runtime # robot_localization
  # DEPENDS system_lib
)

if(EXISTS "/home/firefly/toolchain")
  message("The toolchain directory exists.")
    include_directories(
      source/application
      source/application/external_library/delta_2c_lidar/include
      source/application/external_library/delta_2c_lidar
      source/application/external_library/jsoncpp
      source/application/external_library/ros
      source/application/external_library
      source/application/robot_framework
      source/application/robot_framework/control
      source/application/subTask
      source/application/service
      source/application/wifi
      source/application/avoiding
      source/application/clean
      source/application/docking
      source/application/pathplan
      source/application/explorer
      source/application/location
      source/application/location/rmcl
      source/application/kidnap
      source/application/map
      source/common
      source/common/file
      source/driver
      source/driver/ui
      ${catkin_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}

      #/opt/ros/melodic/include/cartographer
      #/opt/ros/melodic/include/cartographer_ros_msgs


      /home/firefly/toolchain/system/include
      /home/firefly/toolchain/app/include
    )
else()
  message("The toolchain directory does not exist.")
    include_directories(
      source/application
      source/application/external_library/delta_2c_lidar/include
      source/application/external_library/delta_2c_lidar
      source/application/external_library/jsoncpp
      source/application/external_library/ros
      source/application/external_library
      source/application/robot_framework
      source/application/robot_framework/control
      source/application/subTask
      source/application/service
      source/application/wifi
      source/application/avoiding
      source/application/clean
      source/application/docking
      source/application/pathplan
      source/application/explorer
      source/application/location
      source/application/location/rmcl
      source/application/kidnap
      source/application/map
      source/common
      source/common/file
      source/driver
      source/driver/ui
      ${catkin_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}

      #/opt/ros/melodic/include/cartographer
      #/opt/ros/melodic/include/cartographer_ros_msgs


      ../../../system/Driver
      #../../../system/AWS
    )
endif()

add_executable(ts800_app
  source/application/main.cpp
  
  source/application/application.cpp
  

  source/application/robot_framework/rsaDispatcher.cpp
  source/application/robot_framework/rsfMonitor.cpp
  source/application/robot_framework/debugCtr.cpp
  source/application/robot_framework/rsuContext.cpp
  source/application/robot_framework/MessageHandler.cpp
  source/application/robot_framework/errorCollector.cpp
  source/application/robot_framework/errorHandler.cpp
  source/application/robot_framework/keyHandler.cpp
  source/application/robot_framework/deviceHandler.cpp
  source/application/robot_framework/taskIdle.cpp
  source/application/robot_framework/taskDemoRBTPlus.cpp
  source/application/robot_framework/taskReliability.cpp
  source/application/robot_framework/taskWallClean.cpp
  source/application/robot_framework/taskCleanRoom.cpp
  source/application/robot_framework/taskClean.cpp
  source/application/robot_framework/taskLocalMotion.cpp
  source/application/robot_framework/taskReLocalMotion.cpp
  source/application/robot_framework/taskMovePath.cpp
  source/application/robot_framework/taskAvoidDoorSill.cpp
  source/application/robot_framework/taskDocking.cpp
  source/application/robot_framework/taskUnDocking.cpp
  source/application/robot_framework/taskMoveRoom.cpp
  source/application/robot_framework/taskMoveCleanLine.cpp
  source/application/robot_framework/taskExplorer.cpp
  source/application/robot_framework/taskPathPlan.cpp
  source/application/robot_framework/taskDryMop.cpp
  source/application/robot_framework/taskCharging.cpp
  source/application/robot_framework/taskAvoidCharger.cpp
  source/application/robot_framework/taskWalltrackAvoid.cpp
  source/application/robot_framework/taskWalltrackAvoidAround.cpp
  source/application/robot_framework/taskDockingExplorer.cpp
  source/application/robot_framework/taskError.cpp
  source/application/robot_framework/taskBoot.cpp
  source/application/robot_framework/taskPowerOff.cpp
  source/application/robot_framework/taskFirmwareUpdate.cpp

  source/application/robot_framework/control/control.cpp
  source/application/robot_framework/control/slamControl.cpp
  source/application/robot_framework/control/lidarDriverControl.cpp
  source/application/robot_framework/control/systemControl.cpp
  source/application/robot_framework/control/motionController.cpp
  source/application/robot_framework/control/kinematics.cpp

  source/application/robot_framework/control/motionPlanner/motionPlanner.cpp  
  source/application/robot_framework/control/motionPlanner/wayPoint.cpp
  source/application/robot_framework/control/motionPlanner/wayPointManager.cpp   

  source/application/robot_framework/control/path_controller/pathController.cpp
  
  source/application/robot_framework/externData/externData.cpp
  source/application/robot_framework/externData/rosData/rosData.cpp
  source/application/robot_framework/externData/rosData/rosInterface.cpp

  source/application/robot_framework/coreData/observer.cpp
  source/application/robot_framework/coreData/subject.cpp

  source/application/robot_framework/coreData/serviceData.cpp
  source/application/robot_framework/coreData/serviceData/batteryState.cpp
  source/application/robot_framework/coreData/serviceData/keyState.cpp
  source/application/robot_framework/coreData/serviceData/button.cpp
  source/application/robot_framework/coreData/serviceData/rosKey.cpp
  source/application/robot_framework/coreData/serviceData/appKey.cpp
  source/application/robot_framework/coreData/serviceData/powerState.cpp
  source/application/robot_framework/coreData/serviceData/obstacle.cpp
  source/application/robot_framework/coreData/serviceData/lidarSensor.cpp
  # source/application/robot_framework/coreData/serviceData/rosPublishData.cpp
  source/application/robot_framework/coreData/serviceData/debugData.cpp
  source/application/robot_framework/coreData/serviceData/signals.cpp
  source/application/robot_framework/coreData/serviceData/localization.cpp
  source/application/robot_framework/coreData/serviceData/sensorCliff.cpp
  source/application/robot_framework/coreData/serviceData/tiltingState.cpp
  source/application/robot_framework/coreData/serviceData/robotServerBridge.cpp
  source/application/robot_framework/coreData/serviceData/storeAwsData.cpp
  source/application/robot_framework/coreData/serviceData/error.cpp
  source/application/robot_framework/coreData/serviceData/operationArea.cpp 
  source/application/robot_framework/coreData/serviceData/areaInfo.cpp 
  source/application/robot_framework/coreData/serviceData/forbiddenArea.cpp 
  source/application/robot_framework/coreData/serviceData/doNotDisturb.cpp 
  source/application/robot_framework/coreData/serviceData/cleaningSchedule.cpp 
  source/application/robot_framework/coreData/serviceData/ota.cpp
  source/application/robot_framework/coreData/serviceData/robotData.cpp
  source/application/robot_framework/coreData/serviceData/robotInfo.cpp

  source/application/subTask/subTask.cpp

  source/application/service/serviceMacro.cpp
  source/application/service/service.cpp
  source/application/service/serviceReady.cpp
  source/application/service/serviceManager.cpp
  source/application/service/serviceCharging.cpp
  source/application/service/serviceClean.cpp
  source/application/service/serviceDocking.cpp 
  source/application/service/serviceUnDocking.cpp    
  source/application/service/serviceExplorer.cpp
  source/application/service/serviceIdle.cpp
  source/application/service/serviceWifi.cpp
  source/application/service/robotSlamPose.cpp

  source/application/clean/headingPid.cpp
  source/application/clean/imgProcessor.cpp  
  source/application/clean/linear_wheelcontrol.cpp
  source/application/clean/motionPid.cpp
  source/application/clean/obstaclemap.cpp
  source/application/clean/supplyWater.cpp  
  source/application/clean/walltracking.cpp
  source/application/clean/avoiding.cpp
  source/application/clean/avoidingMotionPlanner.cpp  

  source/application/map/cells.cpp
  source/application/map/cleanArea.cpp
  source/application/map/robotmap.cpp
  source/application/map/cleanRoom.cpp
  source/application/map/gridmap.cpp
  source/application/map/trajectory.cpp
  source/application/map/cleanplan.cpp
  source/application/map/mapStorage.cpp
      
  source/application/docking/signaltracking.cpp
  source/application/docking/findCharger.cpp
  
  source/application/pathplan/dstar.cpp  
  source/application/pathplan/escapeWall.cpp
  source/application/pathplan/pathFinderInterface.cpp

  source/application/position_estimator/lowPassFilter.cpp
  source/application/position_estimator/angleExtrapolation.cpp
  
  source/application/explorer/waveFrontier.cpp

  source/application/location/rmcl/rmcl.cpp
  source/application/location/location.cpp

  source/application/kidnap/kidnap.cpp
  source/application/kidnap/kidnapData.cpp

  source/application/external_library/delta_2c_lidar/lidarInterface.cpp
  ${delta_lidar_SDK_SRC}
  source/application/external_library/jsoncpp/jsoncpp.cpp
  source/application/external_library/ros/rosParameter.cpp
  source/application/external_library/ros/rosPublisher.cpp

  source/application/wifi/wifi.cpp
  source/application/wifi/awsDataManager.cpp

  source/common/clipper.cpp
  source/common/utils.cpp
  source/common/systemTool.cpp
  source/common/ebtime.cpp
  source/common/coordinate.cpp
  source/common/file/fileLog.cpp
  source/common/file/fileJson.cpp

  
  source/driver/systemInterface.cpp
  source/driver/systemInterface_ts800.cpp
  source/driver/systemInterface_q8.cpp
  source/driver/lcdDebug.cpp
  source/driver/appInterface.cpp
  source/driver/userInterface.cpp
  source/driver/ui/led.cpp
  source/driver/ui/sound.cpp
  source/driver/ui/display.cpp
)

if(EXISTS "/home/firefly/toolchain")
  message("The toolchain directory exists.")
    target_link_libraries(ts800_app
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        /home/firefly/toolchain/system/lib/librobotsoundinterface.so
        /home/firefly/toolchain/system/lib/librobotdisplayinterface.so
        /home/firefly/toolchain/system/lib/librobotdatainterface.so
        /home/firefly/toolchain/system/lib/librobotcontrolinterface.so
        /home/firefly/toolchain/system/lib/librobotconfiginterface.so
        /home/firefly/toolchain/system/lib/librobototainterface.so
        
        #AWS lib
        /home/firefly/toolchain/app/lib/libDeviceAWSIoT.so
        /home/firefly/toolchain/app/lib/libDeviceConnection.so
        /home/firefly/toolchain/app/lib/libDeviceDataManage.so
        /home/firefly/toolchain/app/lib/libaws_iot_jobs.so      
        /home/firefly/toolchain/app/lib/libaws_iot_pkcs.so     
        /home/firefly/toolchain/app/lib/libsockets_posix.so   
        /home/firefly/toolchain/app/lib/libaws_iot_json.so      
        /home/firefly/toolchain/app/lib/libaws_iot_shadow.so   
        /home/firefly/toolchain/app/lib/libtinycbor.so
        /home/firefly/toolchain/app/lib/libaws_iot_mqtt.so      
        /home/firefly/toolchain/app/lib/libclock_posix.so      
        /home/firefly/toolchain/app/lib/libtransport_mbedtls_pkcs11_posix.so
        /home/firefly/toolchain/app/lib/libaws_iot_backoff_algorithm.so  
        /home/firefly/toolchain/app/lib/libaws_iot_ota.so       
        /home/firefly/toolchain/app/lib/libmbedtls.so          
        /home/firefly/toolchain/app/lib/libz.so
        /home/firefly/toolchain/app/lib/libaws_iot_defender.so           
        /home/firefly/toolchain/app/lib/libaws_iot_ota_http.so  
        /home/firefly/toolchain/app/lib/libopenssl_posix.so
        /home/firefly/toolchain/app/lib/libaws_iot_http.so               
        /home/firefly/toolchain/app/lib/libaws_iot_ota_mqtt.so  
        /home/firefly/toolchain/app/lib/libplaintext_posix.so
        /home/firefly/toolchain/app/lib/libqrencode.so
        /home/firefly/toolchain/app/lib/libqrencode.so.3.4.4
        /home/firefly/toolchain/app/lib/libssl.so
        /home/firefly/toolchain/app/lib/libssl.so.1.0.0
        /home/firefly/toolchain/app/lib/libssl.so.1.1


        /home/firefly/toolchain/lib/aarch64-linux-gnu/libuuid.so.1
        /home/firefly/toolchain/lib/aarch64-linux-gnu/libdl.so.2
        /home/firefly/toolchain/lib/aarch64-linux-gnu/libcrypt.so.1
        /home/firefly/toolchain/lib/aarch64-linux-gnu/libexpat.so.1

        /home/firefly/toolchain/usr/lib/aarch64-linux-gnu/lapack/liblapack.so.3
        /home/firefly/toolchain/usr/lib/aarch64-linux-gnu/blas/libblas.so.3


        ${Boost_STACKTRACE_LIBRARY} # use libbacktrace backend
        backtrace
        pthread dl
        yaml-cpp
        #cartographer
        #cartographer_ros_msgs
    )
else()
  message("The toolchain directory does not exist.")
    target_link_libraries(ts800_app
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    robotcontrolinterface
    robotdatainterface
    robotdisplayinterface
    robotsoundinterface
    robotconfiginterface
  
    #AWS lib 
    #/home/firefly/toolchain/app/lib/libaws_iot_ota_mqtt.so 
    #../../../system/AWS
    
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libDeviceAWSIoT.so
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libDeviceConnection.so
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libDeviceDataManage.so
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libaws_iot_jobs.so  
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libaws_iot_pkcs.so 
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libsockets_posix.so
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libaws_iot_json.so 
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libaws_iot_shadow.so
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libtinycbor.so
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libaws_iot_mqtt.so
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libclock_posix.so
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libtransport_mbedtls_pkcs11_posix.so
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libaws_iot_backoff_algorithm.so
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libaws_iot_ota.so       
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libmbedtls.so
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libz.so
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libaws_iot_defender.so         
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libaws_iot_ota_http.so
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libopenssl_posix.so
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libaws_iot_http.so           
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libaws_iot_ota_mqtt.so
    /home/firefly/catkin_ws/src/ts800/system/AWS/lib/libplaintext_posix.so

    ${Boost_STACKTRACE_LIBRARY} # use libbacktrace backend
    backtrace
    pthread dl
    yaml-cpp
    #cartographer
    #cartographer_ros_msgs
    )
endif()

#add_executable(ts800_odom_node
#  nodes/odometry/odometry_node.cpp
#  nodes/odometry/odometry.cpp
#)

#if(EXISTS "/home/firefly/toolchain")
#    target_link_libraries(ts800_odom_node
#        ${catkin_LIBRARIES}
#        /home/firefly/toolchain/system/lib/librobotcontrolinterface.so
#        /home/firefly/toolchain/system/lib/librobotdatainterface.so
#        /home/firefly/toolchain/system/lib/librobotconfiginterface.so
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libuuid.so.1
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libdl.so.2
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libcrypt.so.1
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libexpat.so.1
#    )
#else()
#    message("The toolchain directory does not exist.")
#    target_link_libraries(ts800_odom_node
#        ${catkin_LIBRARIES}
#        robotcontrolinterface
#        robotdatainterface
#        robotconfiginterface
#    )
#endif()

#add_executable(ts800_lidar_filter_node
#  nodes/lidarFilter/lidarFilter_node.cpp
#  nodes/lidarFilter/lidarFilter.cpp
#)

#if(EXISTS "/home/firefly/toolchain")
#    message("The toolchain directory exists.")
#    target_link_libraries(ts800_lidar_filter_node
#        ${catkin_LIBRARIES}
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libuuid.so.1
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libdl.so.2
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libcrypt.so.1
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libexpat.so.1
#    )
#else()
#    message("The toolchain directory does not exist.")
#    target_link_libraries(ts800_lidar_filter_node
#        ${catkin_LIBRARIES}
#    )
#endif()

#add_executable(ts800_imu_node
#  nodes/imu/imu_node.cpp
#)
#if(EXISTS "/home/firefly/toolchain")
#    target_link_libraries(ts800_imu_node
#        ${catkin_LIBRARIES}
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libuuid.so.1
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libdl.so.2
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libcrypt.so.1
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libexpat.so.1
#    )
#else()
#    target_link_libraries(ts800_imu_node
#        ${catkin_LIBRARIES}
#    )
#endif()

#add_executable(trajectory_editor_node
#  nodes/trajectory_editor/trajectory_editor_node.cpp
#)
#if(EXISTS "/home/firefly/toolchain")
#    target_link_libraries(trajectory_editor_node
#        ${catkin_LIBRARIES}
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libuuid.so.1
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libdl.so.2
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libcrypt.so.1
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libexpat.so.1
#    )
#else()
#    target_link_libraries(trajectory_editor_node
#        ${catkin_LIBRARIES}
#    )
#endif()

#add_executable(ts800_static_transform_node
#  nodes/static_transform/static_transform.cpp
#)

#if(EXISTS "/home/firefly/toolchain")
#    target_link_libraries(ts800_static_transform_node
#        ${catkin_LIBRARIES}
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libuuid.so.1
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libdl.so.2
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libcrypt.so.1
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libexpat.so.1
#    )
#else()
#    target_link_libraries(ts800_static_transform_node
#        ${catkin_LIBRARIES}
#    )
#endif()

#add_executable(system_monitor_node
#  nodes/system_monitor_ros/system_monitor_node.cpp
#  nodes/system_monitor_ros/system_monitor.cpp
#)

#if(EXISTS "/home/firefly/toolchain")
#    target_link_libraries(system_monitor_node
#        ${catkin_LIBRARIES}
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libuuid.so.1
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libdl.so.2
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libcrypt.so.1
#        /home/firefly/toolchain/lib/aarch64-linux-gnu/libexpat.so.1
#    )
#else()
#    target_link_libraries(system_monitor_node
#        ${catkin_LIBRARIES}
#    )
#endif()
