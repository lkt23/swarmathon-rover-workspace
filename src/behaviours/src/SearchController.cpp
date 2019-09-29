#include "SearchController.h"
#include <angles/angles.h>
#include <ros/ros.h>

SearchController::SearchController() {
  rng = new random_numbers::RandomNumberGenerator();
  currentLocation.x = 0;
  currentLocation.y = 0;
  currentLocation.theta = 0;

  goal_id = rng->uniformInteger(8,20);

  centerLocation.x = 0;
  centerLocation.y = 0;
  centerLocation.theta = 0;
  result.PIDMode = FAST_PID;

  result.fingerAngle = M_PI/2;
  result.wristAngle = M_PI/4;

  double radiusRate = 0;
  double angleRate = 0;
  double angleincrease = 30;// angle increases by 30 degree
  int wp_per_rov = 12;
  double spiralnumber= 10;
  num_waypoints = wp_per_rov * spiralnumber;

  int id;
  for (int i = 0; i < spiralnumber;i++) {
    for (int j = 0; j < wp_per_rov; j++) {
      id = wp_per_rov*i+j;
      trajectLocation_body[id].x = (i+1.0 + radiusRate) * cos(angleRate) - 1.0; // TODO: why -1.0
      trajectLocation_body[id].y = (i+1.0 + radiusRate) * sin(angleRate);
      angleRate += angleincrease*(M_PI/180);
      radiusRate += 1.0/wp_per_rov;
    }
    radiusRate = 0.0;
    angleRate = 0.0;
  }

}

void SearchController::Reset() {
  result.reset = false;
}

/**
 * This code implements a basic random walk search.
 */
Result SearchController::DoWork() {
//  ROS_INFO_STREAM("RangeController DoWork");
//  ROS_INFO_STREAM_THROTTLE(2,"RangeController DoWork");
  if (!result.wpts.waypoints.empty()) {
    if (hypot(result.wpts.waypoints[0].x-currentLocation.x, result.wpts.waypoints[0].y-currentLocation.y) < 0.10) {
      attemptCount = 0;
//      ROS_INFO_STREAM("Completed 1 waypoint.");
    }
  }

  if (attemptCount > 0 && attemptCount < 1) {
//    ROS_INFO_STREAM("Still working on current waypoint. attempCount =" << attemptCount);
    attemptCount++;
    if (succesfullPickup) {
      succesfullPickup = false;
      attemptCount = 1;
    }
    return result;
  }
  // Set to 1 (don't retry this waypoint), just go to next waypoint.
  // Regular waypoints
  else if (attemptCount >= 1 || attemptCount == 0)
  {
    attemptCount = 1;

//    ROS_INFO_STREAM("attemptCount >= 5 or == 0. Adding new waypoint..");

    result.type = waypoint;
    Point  searchLocation;

    //select new position 50 cm from current location
    if (first_waypoint)
    {
//      ROS_INFO_STREAM("First waypoint aded.");
      first_waypoint = false;
      set_spiral_waypoints_odom = true;
      searchLocation.theta = currentLocation.theta + M_PI;
      searchLocation.x = currentLocation.x + (0.50 * cos(searchLocation.theta));
      searchLocation.y = currentLocation.y + (0.50 * sin(searchLocation.theta));
      searchLocation.id = 3; // 3 strictly > 0 for safety
    }
    else
    {
      // trajectLocation array contains x and y coordinates for all waypoints (120 total) indexed by id

      // Transform waypoints to the rover odom frame
      if (set_spiral_waypoints_odom) {
        set_spiral_waypoints_odom = false;
        double stheta, ctheta;
        stheta = sin(currentLocation.theta);
        ctheta = cos(currentLocation.theta);

        for (int i = 0; i < num_waypoints;i++) {
          trajectLocation[i].x = ctheta*trajectLocation_body[i].x - stheta*trajectLocation_body[i].y;
          trajectLocation[i].y = stheta*trajectLocation_body[i].x + ctheta*trajectLocation_body[i].y;
        }
      }

      // If rover is stuck, return to first waypoint.
      if (waypoint_timeout) {
        goal_id += 6;
        waypoint_timeout = false;
      }

        if (at_boundary) {
            goal_id = 14;
            at_boundary = false;
        }

        // If rover finishes avoiding obstacle/collection zone, resume previous waypoint
      if (resume_previous_waypoint) { // resume from obstacle avoidance
 //       ROS_INFO_STREAM("SearchController:resume from obstacle avoidance.");
        resume_previous_waypoint = false;
        goal_id--; // sets to previous wpt id
      }

 //     ROS_INFO_STREAM("New waypoint added. goal_id=" << goal_id);

      //select new heading from Gaussian distribution around current heading
      searchLocation.x = centerLocation.x + trajectLocation[goal_id].x;
      searchLocation.y = centerLocation.y + trajectLocation[goal_id].y;
      searchLocation.id = goal_id;
//      ROS_INFO_STREAM("Next waypoint is: " << "x=" << searchLocation.x << ", y=" << searchLocation.y);

      goal_id++; // set goal_id to next waypoint
//        ROS_INFO_STREAM("goal_id = " << goal_id);

      if (goal_id == num_waypoints - 1) { // if 120 waypoints are done, go back to waypoint 11.
        goal_id = 11;
      }
    }

//      ROS_INFO_STREAM("justDroppedOff = " << result.justDroppedOff);
    if (result.justDroppedOff){
          searchLocation.x = currentLocation.x + (3.0 * cos(currentLocation.theta + 3.14));
          searchLocation.y = currentLocation.y + (3.0 * sin(currentLocation.theta + 3.14));
          result.justDroppedOff = false;
    }
    result.wpts.waypoints.clear();
    result.wpts.waypoints.insert(result.wpts.waypoints.begin(), searchLocation);
    
    return result;
  }

}

void SearchController::SetCenterLocation(Point centerLocation) {
  
  float diffX = this->centerLocation.x - centerLocation.x;
  float diffY = this->centerLocation.y - centerLocation.y;
  this->centerLocation = centerLocation;
  
  if (!result.wpts.waypoints.empty())
  {
  result.wpts.waypoints.back().x -= diffX;
  result.wpts.waypoints.back().y -= diffY;
  }
  
}

void SearchController::SetCurrentLocation(Point currentLocation) {
  this->currentLocation = currentLocation;
}

void SearchController::ProcessData() {
}

bool SearchController::ShouldInterrupt(){
  ProcessData();

  return false;
}

bool SearchController::HasWork() {
  return true;
}

void SearchController::SetSuccesfullPickup() {
  succesfullPickup = true;
}

void SearchController::SetWaypointTimeout(bool timeout) {
  waypoint_timeout = timeout;
}

bool SearchController::SetResumePreviousWaypoint(bool resume) {
  resume_previous_waypoint = resume;
}

void SearchController::SetAtBoundary(bool atboundary) {
    at_boundary = atboundary;
}
