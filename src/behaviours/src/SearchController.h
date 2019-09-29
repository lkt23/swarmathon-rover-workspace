#ifndef SEARCH_CONTROLLER
#define SEARCH_CONTROLLER

#include <random_numbers/random_numbers.h>
#include "Controller.h"
#include <geometry_msgs/Pose2D.h>

/**
 * This class implements the search control algorithm for the rovers. The code
 * here should be modified and enhanced to improve search performance.
 */
class SearchController : virtual Controller {

public:

  SearchController();

  void Reset() override;

  // performs search pattern
  Result DoWork() override;
  bool ShouldInterrupt() override;
  bool HasWork() override;

  // sets the value of the current location
  //void UpdateData(geometry_msgs::Pose2D currentLocation, geometry_msgs::Pose2D centerLocation);
  void SetCurrentLocation(Point currentLocation);
  void SetCenterLocation(Point centerLocation);
  void SetSuccesfullPickup();
  void SetWaypointTimeout(bool timeout);
  bool SetResumePreviousWaypoint(bool resume);
  void SetAtBoundary(bool atboundary);

protected:

  void ProcessData();

private:

  random_numbers::RandomNumberGenerator* rng;
  Point currentLocation;
  Point centerLocation;
  Point searchLocation;
  int attemptCount = 0;
  //struct for returning data to ROS adapter
  Result result;

  // Search state
  // Flag to allow special behaviour for the first waypoint
  bool first_waypoint = true;
  bool succesfullPickup = false;
  bool set_spiral_waypoints_odom = false;
  int goal_id = 7;
  geometry_msgs::Pose2D trajectLocation[333];
  geometry_msgs::Pose2D trajectLocation_body[333];
  int num_waypoints;
  bool waypoint_timeout = false;
  bool resume_previous_waypoint; // used when rover finished avoiding obstacle/collection zone.
  bool at_boundary = false;

};

#endif /* SEARCH_CONTROLLER */
