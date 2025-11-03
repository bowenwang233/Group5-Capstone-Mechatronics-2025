
//
// File ros2nodeinterface.h
//
// Code generated for Simulink model 'test_module'.
//
// Model version                  : 1.1
// Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
// C/C++ source code generated on : Tue Oct 28 14:49:18 2025
//
#ifndef _ROS2_MATLAB_NODEINTERFACE_
#define _ROS2_MATLAB_NODEINTERFACE_
#ifdef _MSC_VER
#pragma warning(push)
#pragma warning(disable : 4265)
#pragma warning(disable : 4458)
#pragma warning(disable : 4100)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#pragma GCC diagnostic ignored "-Wredundant-decls"
#pragma GCC diagnostic ignored "-Wnon-virtual-dtor"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wshadow"
#endif //_MSC_VER
#include "rclcpp/rclcpp.hpp"
#include "rtwtypes.h"
#include "test_module_types.h"
#include "slros_busmsg_conversion.h"
#include "sensor_msgs/msg/compressed_image.hpp"
extern rclcpp::Node::SharedPtr SLROSNodePtr;
namespace ros2 {
namespace matlab {
  //Semaphore using std::CV and std::mutex
  class Semaphore {
  public:
    std::mutex mMutex;
    std::condition_variable mCV;
    std::atomic_uint mCount;
    //
    Semaphore(int count = 0)
      : mCount(count) {
      }
    //
    inline void notify() {
      std::unique_lock<std::mutex> lock(mMutex);
      mCount++;
      mCV.notify_all();
    }
    //
    inline void wait() {
      std::unique_lock<std::mutex> lock(mMutex);
      while (mCount == 0) {
        mCV.wait(lock);
      }
      if (mCount)
        mCount--;
    }
  };
  //NodeInterface
  class NodeInterface {
    NodeInterface(const NodeInterface& );
    NodeInterface& operator=(const NodeInterface& );
    rclcpp::executors::MultiThreadedExecutor::SharedPtr mExec;
    //
    Semaphore mBaseRateSem;
    std::shared_ptr<std::thread> mBaseRateThread;
    rclcpp::TimerBase::SharedPtr mSchedulerTimer;
    //
    //
    Semaphore mStopSem;
    volatile boolean_T mRunModel;
	// External mode background thread
	std::shared_ptr<std::thread> mExtModeThread;
  public:
    NodeInterface();
    ~NodeInterface();
    //
    void initialize(int argc, char * const argv[]);
    int run();
    void stop(void);
    void terminate(void);
    //
    boolean_T getStopRequestedFlag(void);
    void schedulerThreadCallback(void);
    void baseRateTask(void);
	void extmodeBackgroundTask(void);
    //
    rclcpp::Node::SharedPtr getNode() {
      return SLROSNodePtr;
    }
  }; //class NodeInterface
  //
  std::shared_ptr<ros2::matlab::NodeInterface> getNodeInterface();
}//namespace matlab
}//namespace ros2
#ifdef _MSC_VER
#pragma warning(pop)
#else
#pragma GCC diagnostic pop
#endif //_MSC_VER
#endif //_ROS2_MATLAB_ROS2CGEN_MULTIRATE_
