/*
 *      launch example:
 *      roslaunch system_monitor_ros n_processes:=10
 */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h" 
#include <fstream>
#include <iostream>
#include <thread>
#include <array>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <stdio.h>
#include <unistd.h> // getpid 함수를 사용하기 위해 필요
#include <cstdlib>
#include <dirent.h> 
#include <sys/types.h>
#include <csignal>
#include <cstring>
#include <netinet/in.h>
#include <sys/statvfs.h>
#include <sys/stat.h>
#include <pwd.h>
#include <thread>
#include <sstream>
#include <vector>
#include <algorithm>
#include <fcntl.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#if 0
#include "ts800_ros/SubmapList.h"
#include "ts800_ros/SubmapEntry.h"
#endif
#include "system_monitor.h"

struct stJiffies
{
    int user;
    int nice;
    int system;
    int idle;
};

static std::string pro_name;
int submapListSize = 0;
bool isSlamRun = false;
ros::Time trackedPoseTime;
bool bTrackPoseUpdate;
/**
 * @brief
 * 
  SubmapList
    std_msgs/Header header
    ts800_ros/SubmapEntry[] submap

  SubmapEntry
    int32 trajectory_id
    int32 submap_index
    int32 submap_version
    geometry_msgs/Pose pose
    bool is_frozen
  * 
 * @param msg 
 */
#if 0
void submapListCallback(const ts800_ros::SubmapList::ConstPtr& msg) {
/**
 * @brief
  [ WARN] [1706864155.967735652]: Submap index : 0, Submap trajectory id : 0
  [ WARN] [1706864155.968289527]: Submap index : 1, Submap trajectory id : 0
  [ WARN] [1706864155.968609485]: Submap index : 2, Submap trajectory id : 0
  [ WARN] [1706864155.969137694]: Submap index : 0, Submap trajectory id : 0
  [ WARN] [1706864155.973261569]: Submap index : 1, Submap trajectory id : 0
  [ WARN] [1706864155.975078069]: Submap index : 2, Submap trajectory id : 0
  submap is 3 => Submap index : 0,1,2
 */
  submapListSize =  msg->submap.size();
  //submapListSize =  msg->submap.size();
  //for (const auto& submap_msg : msg->submap) {
  //    ROS_WARN("Submap trajectory id: %d, Submap index : %d, Submnap x: %f, Submnap y: %d", submap_msg.trajectory_id, submap_msg.submap_index, submap_msg.pose.position.x, submap_msg.pose.position.y);
  //}
}
#endif
/**
 * @brief 
 * 
 * @param processName 
 * @return true 
 * @return false 
 */
#if 0
bool isProcessRunning(const std::string& processName)
{
    std::string cmd = "pgrep " + processName + " > /dev/null";
    int result = system(cmd.c_str());

    if (result == 0)
        return true;
    else
        return false;
}
#endif

/**
 * Get last cpu load as a string (not threadsafe)
 */
static char* getCpuLoadString()
{
    /*
     * Easiest seems to be just to read out a line from top, rather than trying to calc it ourselves through /proc stuff
     */
    static bool started = false;
    static char buffer[1024];
    static char lastfullline[256];
    static int bufdx;
    static FILE *fp;

    if(!started) {
        /* Start the top command */

        /* Open the command for reading. */
        fp = popen("top -b  -d1", "r");
        if (fp == NULL) {
          printf("Failed to run top command for profiling\n" );
          exit(1);
        }

        /* Make nonblocking */
        int flags;
        flags = fcntl(fileno(fp), F_GETFL, 0);
        flags |= O_NONBLOCK;
        fcntl(fileno(fp), F_SETFL, flags);

        started = true;
    }


    /*
     * Read out the latest info, and remember the last full line
     */
    while( fgets (buffer + bufdx, sizeof(buffer) - bufdx, fp)!=NULL ) {
        /* New data came in. Iteratively shift everything until a newline into the last full line */
        char *newlinep;
        while((newlinep = strstr(buffer, "\n"))) {
            int shiftsize = (newlinep - buffer + 1);
            *newlinep = 0; /* Nullterminate it for strcpy and strstr */
            /* Check if the line contained "cpu" */
            if(strstr(buffer, "Cpu")) {
                strncpy(lastfullline, buffer, sizeof(lastfullline) - 1); /* Copy full line if it contained cpu info */
            }
            memmove(buffer, buffer + shiftsize, shiftsize); /* Shift left (and discard) the line we just consumed */
        }
    }

    /*
     * Return the last line we processed as valid output
     */
    return lastfullline;
}

/**
 * @brief 
 * 
 * @param str 
 * @return std::vector<std::string> 
 */
std::vector<std::string> split(const std::string& str) {
    std::istringstream iss(str);
    std::vector<std::string> tokens;
    std::string token;
    while (iss >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

/**
 * @brief Get the Proc Id By Name object
 * 
 * @param procName 
 * @return int 
 */
int getProcIdByName(const std::string &procName) {
  int pid = -1;
  auto dp = opendir("/proc");
  if (dp != nullptr) {
      struct dirent *dirp;
      while (pid < 0 && (dirp = readdir(dp))) {
          int id = atoi(dirp->d_name);
          if (id > 0) {
              std::string cmdPath{"/proc/"};
              cmdPath.append(dirp->d_name);
              cmdPath.append("/cmdline");
              std::ifstream cmdFile(cmdPath.c_str());
              std::string cmdLine;
              getline(cmdFile, cmdLine);
              if (!cmdLine.empty()) {
                  size_t pos = cmdLine.find('\0');
                  if (pos != std::string::npos)
                      cmdLine = cmdLine.substr(0, pos);
                  pos = cmdLine.rfind('/');
                  if (pos != std::string::npos)
                      cmdLine = cmdLine.substr(pos + 1);
                  if (strcmp(procName.c_str(), cmdLine.c_str()) == 0) {
                      pid = id;
                  }
              }
          }
      }
  }
  closedir(dp);
  return pid;
}

/**
 * @brief 
 * 
 * @param cmd 
 * @return std::string 
 */
std::string exec(const char* cmd)
{
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != nullptr) {
            result += buffer.data();
        }
    }
    return result;
}

/**
 * @brief 
 * 
 * @param pid 
 * @return double 
 */
double getCPUUsageForProcess(int pid)
{
    //ROS_WARN("getCPUUsageForProcess: %d", pid);

    // top 명령어 실행을 위한 명령어 문자열 생성
    std::ostringstream commandStream;
    commandStream << "top -n 1 -b -p " << pid;
    std::string command = commandStream.str();
    //ROS_WARN("command: %s", command.c_str());

    // top 명령어 실행 및 결과 읽기
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) {
        ROS_WARN("Failed to execute top command");
        return -1.0;
    }
    /*
    top - 10:44:25 up 11 min,  5 users,  load average: 5.50, 2.43, 1.06
    Tasks:   1 total,   0 running,   1 sleeping,   0 stopped,   0 zombie
    %Cpu(s): 16.0 us, 14.4 sy,  0.0 ni, 68.9 id,  0.1 wa,  0.0 hi,  0.5 si,  0.0 st
    KiB Mem :  2032888 total,  1041572 free,   417768 used,   573548 buff/cache
    KiB Swap:        0 total,        0 free,        0 used.  1575180 avail Mem 

      PID USER      PR  NI    VIRT    RES    SHR S  %CPU %MEM     TIME+ COMMAND
    3972 firefly   20   0  854060  32368  19784 S  33.3  1.6   2:17.80 cartographer_no
    */

    // top 명령어의 출력을 읽어서 %CPU 값을 찾음
    char buffer[128];
    double cpuUsage = 0.0;
    bool foundCPULine = false;
    while (!feof(pipe)) {
        if (fgets(buffer, 128, pipe) != NULL) {
            std::string line(buffer);
            // "%CPU"를 포함하는 줄을 찾음
            if (foundCPULine) {
                // 라인을 공백을 기준으로 토큰으로 분리
                std::istringstream iss(line);
                std::string token;
                // %CPU 값을 찾음
                iss >> token; // PID
                iss >> token; // USER
                iss >> token; // PR
                iss >> token; // NI
                iss >> token; // VIRT
                iss >> token; // RES
                iss >> token; // SHR
                iss >> token; // S
                // R: Running : run processor
                // S: Sleeping : sleep
                // D : Diksk Sleep : wait for Disk I/O
                if (token == "R")// || token == "D") 
                { // Running 또는 S 상태인 경우에만 CPU 사용량을 읽음
                    iss >> token; // %CPU
                    cpuUsage = std::stod(token);
                    break;
                }
            }
            if (line.find("%CPU") != std::string::npos) {
                foundCPULine = true;
            }
        }
    }
    pclose(pipe);
    return cpuUsage;
}

void slamTrackedPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (msg != nullptr)
    {
        if ( msg->header.stamp > trackedPoseTime )
        {
            //업데이트 시간 업데이트
            trackedPoseTime = msg->header.stamp;
            bTrackPoseUpdate = true;
        }
        else
        {
            bTrackPoseUpdate = false;
        }
    }
}

/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char* argv[]) {
  ros::init(argc, argv, "System_Monitor_Node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~"); //paramter node handler
  /*
    <node name="system_node" pkg="ts800_ros" type="system_monitor_node" output="screen" >
    <param name ="processor_name" value="cartographer_node"/> # 측정할 프로세서 이름 명시
    </node>
  */
  if ( private_nh.getParam("processor_name", pro_name))
  {
    ROS_WARN("processor_name: %s", pro_name);
  }
  else
  {
    ROS_WARN("getParam is error ");
    pro_name = "cartographer_node";
  }

  //ros::Subscriber submapList_sub_= nh.subscribe("/submap_list", 10, submapListCallback);
  //ros::Publisher cpu_pub_ = nh.advertise<std_msgs::Float64>("/cpu_usage", 1);
  //ros::Publisher submapList_pub_ = nh.advertise<std_msgs::Int32>("/submapList", 1);
  //slam of the pose topic : publish_tracked_pose(6hz)
  ros::Subscriber track_pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/tracked_pose", 5, slamTrackedPoseCallback);  //from of the cartograper pose  

  ros::Rate loop_rate(1); // 1 Hz(1 sec)
  while (ros::ok())
  {
    if (bTrackPoseUpdate)
    {
      int pid = getProcIdByName(pro_name); //"ts800_app" cartographer_node
      //ROS_WARN("cartographer_node PID: %d", pid);
      if (pid != -1) 
      {
        // top -p <PID>
        double cpuUsage = getCPUUsageForProcess(pid);
        if (cpuUsage > 0.0)
        {
          ROS_WARN("CPU Usage for %.2f%%", cpuUsage);
        }
      }
    }
    else
    {
      ROS_WARN("slam is not running.");
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
