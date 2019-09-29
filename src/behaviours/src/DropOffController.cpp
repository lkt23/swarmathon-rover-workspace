#include "DropOffController.h"
#include <ros/ros.h>

DropOffController::DropOffController() {

  reachedCollectionPoint = false;

  result.type = behavior;
  result.b = wait;
  result.wristAngle = 0.7;
  result.reset = false;
  interrupt = false;

  circularCenterSearching = false;
  spinner = 0;
  centerApproach = false;
  seenEnoughCenterTags = false;
  prevCount = 0;

  countLeft = 0;
  countRight = 0;

  isPrecisionDriving = false;
  startWaypoint = false;
  dropoffTimeStart = -1;
  timerTimeElapsed = -1;
}

DropOffController::~DropOffController() {

}

// DoWork does following:
// When rover successfully picks up a cube, DropOffController comes into play.
// First DropOffController calculates distance from rover's current location to homebase then set goal location to homebase.
// Rover goes to homebase, if rover doesn't see collection zone, it will do spin search spirally from current location.
// If rover still doesn't see homebase for 1.5min, DropOffController is reset and control returns to SearchController (via LogicControler, of course).
// If rover sees homebase, it dynamically calculates how many center tags on the left and right w.r.t camera.
// If rover see only center tags on 1 side (left or right), then rover rotates towards the side that has tags until it sees tags on both sides.
// When rover sees tags on both sides, it goes straight into collection zone for 3 secs, drops off the cube, and goes back for 4 secs.
// DropOffController then exits and rover then resumes normal search.
// If for some reason, when the 3secs timer for going straight into center hasn't yet started and rover already saw center tag(s)
// but then doesn't see anymore, it will go straight for 10secs until it sees center tags then continues the process.
// If it still doesn't see tags, DropOffController resets and rover resumes normal search.

// Please feel free to tune numerical numbers for better performance.
Result DropOffController::DoWork() {
//  ROS_INFO_STREAM("DropOffController DoWork=" << current_time);
  cout << "8" << endl;

  int count = countLeft + countRight;

  // ======== BEGIN: Logic to make rover give up after 1.5min unsuccessful spin search ============
  if (centerSeen && !lostCenter) { // We've found center. Stop time counting, don't drop off
//    ROS_INFO_STREAM_THROTTLE(3, "We found the center. Stop 2 mins time counting. Set startSpinSearchTimer = false..");
    startSpinSearchTimer = false;
    spinSearchStartTime = current_time;
  }


  if (circularCenterSearching && !centerSeen && !startSpinSearchTimer) { // Start time counting when we start do spin search
//    ROS_INFO_STREAM("Starting Spin Search Time Counting for 2mins...");
    startSpinSearchTimer = true;
    spinSearchStartTime = current_time;
  }


  if (startSpinSearchTimer) { // Check the time elapsed. If >=2mins, just drop off the cube and resume normal search
    long int elapsed = current_time - spinSearchStartTime;
    float spinSearchTimeElapsed =  elapsed/1e3;
    if (spinSearchTimeElapsed >= 90) {
//      ROS_INFO_STREAM("2mins has passed without seeing center from first spin search. Set startSpinSearchTimer = false. Drop off the cube. Resume normal search..");
      // Reset everything
      startSpinSearchTimer = false;

      isPrecisionDriving = false;

      result.type = behavior; // By setting this, rover will resume searching (SearchController becomes active)
      result.b = nextProcess;
      result.reset = true;

      result.fingerAngle = M_PI_2; //open fingers
      result.wristAngle = 0; //raise wrist

      return result;

    }
    // else just do everything in normal
  }
  // ======== END: Logic to make rover give up after 2mins unsuccessful spin search ============

  //if we are in the routine for exiting the circle once we have dropped a block off and reseting all our flags
  //to resart our search.
  if(reachedCollectionPoint)
  {
    cout << "2" << endl;
    isPrecisionDriving = false;

    long int elapsed = current_time - dropoffTimeStart;
    float dropoffTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds


    if (dropoffTimeElapsed >= 5)
    {
      if (finalInterrupt)
      {
//        ROS_INFO_STREAM("Finished going backwards..");

        result.type = behavior;
        result.b = nextProcess;
        result.reset = true;
        return result;
      }
      else
      {
        finalInterrupt = true;
        cout << "1" << endl;
      }
    }
    else if (dropoffTimeElapsed >= 1)
    {
//      ROS_INFO_STREAM("Drop off the cube. Go backwards for 3secs..");
      isPrecisionDriving = true;
      result.type = precisionDriving;

      result.fingerAngle = M_PI_2; //open fingers
      result.wristAngle = 0; //raise wrist

      result.pd.cmdVel = -0.3;
      result.pd.cmdAngularError = 0.0;
    }

    return result;
  }

  double distanceToCenter = hypot(this->centerLocation.x - this->currentLocation.x, this->centerLocation.y - this->currentLocation.y);


  //check to see if we are driving to the center location or if we need to drive in a circle and look.
  if (!timerStarted && distanceToCenter > collectionPointVisualDistance && !circularCenterSearching && count == 0) {
 //   ROS_INFO_STREAM("Moving to center from far distance..., targetHeld=" << targetHeld);
    result.type = waypoint;
    result.wpts.waypoints.clear();
    result.wpts.waypoints.push_back(this->centerLocation);
    result.wpts.waypoints.back().id = -4;
    startWaypoint = false;
    isPrecisionDriving = false;

    return result;
  }

  if (count > 0 || centerSeen) {
 //   ROS_INFO_STREAM("Oh See Centers now... or Used to see the centers");
    // ROS_INFO_STREAM_THROTTLE(3, "DropOffController: countLeft=" << countLeft << ", countRight=" << countRight << ", count=" << count);
    if (!timerStarted && (centerSeen && count == 0)) { // if we doesn't see any count for 10 secs. Drop the cube ~ give up ~ return to search
      if (!lostCenter) {
         ROS_INFO_STREAM("Potentially lost center...Start 10sec timer..");
        timeWithoutSeeingCenterTags = current_time;
        lostCenter = true;
        result.type = precisionDriving;
        result.pd.cmdVel = 0.02;  //0.08; // moving straight
        result.pd.cmdAngularError = 0.0;
        return result;
      }

      long int elapsed = current_time - timeWithoutSeeingCenterTags;
      float timeElapsedWithoutSeeingCenterTags = elapsed/1e3;

      if (timeElapsedWithoutSeeingCenterTags >= 20) {
         ROS_INFO_STREAM("See no center tags for > 20secs. Reset. Resume spin search");
        isPrecisionDriving = false;
        result.type = behavior;
        result.b = nextProcess;
        result.reset = true; // reset everything and return to search
        finalInterrupt = true;
        return result;
      } else { // What to do if < 10 secs:
        result.type = precisionDriving;
        result.pd.cmdVel = 0.02;  //0.08; // moving straight
        result.pd.cmdAngularError = 0.0;
        return result;
      }
    } else {
      lostCenter = false; // put here or create private variable of lostCenter
    }

    centerSeen = true;
    isPrecisionDriving = true;

    if (!timerStarted && ((countLeft == 0 || countRight == 0) && (count > 0 && count <= 10))) { // we arrived at the edge, either turn left or right
      // ROS_INFO_STREAM("We arrived at the edge, either turn left or right.");
//      ROS_INFO_STREAM("Arrived at the edge. Turn left or right..");
      if (countLeft > 0) { // TODO: What is result.type, .b here?
        // turn left
        result.pd.cmdVel = 0.07; // rotate but dont drive. 0.05 is to prevent turning in reverse
        result.pd.cmdAngularError = centeringTurnRate;
      } else {
        // turn right
        result.pd.cmdVel = 0.07; // rotate but dont drive. 0.05 is to prevent turning in reverse
        result.pd.cmdAngularError = -centeringTurnRate;
      }

      result.type = precisionDriving;
      isPrecisionDriving = true;
      return result;
    }

    if (!timerStarted && countRight > 0 && countLeft > 0) { // arrived right at the center or it is time to stop turning (already aimed right at center)
      ROS_INFO_STREAM("Saw tags on both sides. Going straight for 3secs.");
      // ROS_INFO_STREAM("We are heading right at the center. Go straight for 2secs and then drop off. Timer started.");
      // just go straight for 3 secs and then drop off
      // start timer here
      result.pd.cmdVel = searchVelocity;
      result.pd.cmdAngularError = 0;
      result.type = precisionDriving;
      isPrecisionDriving = true;
      timerStarted = true;
      timeStart = current_time;
      return result;
    }

    if (timerStarted) {
      long int elapsed = current_time - timeStart;
      float timeElapsed = elapsed/1e3;
      if (timeElapsed < 2.0) {
//        ROS_INFO_STREAM("Counting to 3secs going straight.");
        result.pd.cmdVel = 0.15;
        result.pd.cmdAngularError = 0;
      } else { // 3secs has passed
//        ROS_INFO_STREAM("3secs has passed.. Start dropOffTimer..");
        dropoffTimeStart = current_time;

        // ROS_INFO_STREAM("3secs has passed. Now drop off the cube, go backwards 3secs. Return to normal search.");
        // stop rover, drop the cube, go backwards 4 secs
        result.pd.cmdVel = 0;
        result.pd.cmdAngularError = 0;
        reachedCollectionPoint = true;
        isPrecisionDriving = false;

        timerStarted = false;
        // just reset to initial value
        centerSeen = false;
        timeStart = current_time;
      }
      return result;
    }

  }

  if (!centerSeen) { //spin search for center
//    ROS_INFO_STREAM("Spinning Search for centers...");

    Point nextSpinPoint;
    //sets a goal that is 60cm from the centerLocation and spinner
    //radians counterclockwise from being purly along the x-axis.
    nextSpinPoint.x = centerLocation.x + (initialSpinSize + spinSizeIncrease) * cos(spinner);
    nextSpinPoint.y = centerLocation.y + (initialSpinSize + spinSizeIncrease) * sin(spinner);
    nextSpinPoint.theta = atan2(nextSpinPoint.y - currentLocation.y, nextSpinPoint.x - currentLocation.x);
    nextSpinPoint.id = rand(); // To avoid 60s timeout timer which can cause rover incorrectly considered as being stuck

    result.type = waypoint; // this will forward control to DriveController
    result.wpts.waypoints.clear();
    result.wpts.waypoints.push_back(nextSpinPoint);

    spinner = spinner + spinDirection * 45.0*(M_PI/180.0); //add 45 degrees in radians to spinner.
    if (spinner > 2*M_PI || spinner < -2*M_PI) {
      spinner -= spinDirection * 2*M_PI;
      spinDirection = -spinDirection;
      spinSizeIncrease += spinSizeIncrement;
    }
    circularCenterSearching = true;
    //safety flag to prevent us trying to drive back to the
    //center since we have a block with us and the above point is
    //greater than collectionPointVisualDistance from the center.
    returnTimer = current_time;
    timerTimeElapsed = 0;
  }

  return result;
}

void DropOffController::Reset() {
//  ROS_INFO_STREAM("DropOffController:: RESETTING...");
  result.type = behavior;
//  result.b = wait; // Commented to ensure rover will go to next process state.
  result.pd.cmdVel = 0;
  result.pd.cmdAngularError = 0;
  result.fingerAngle = -1;
  result.wristAngle = 0.7;
  result.reset = false;
  result.wpts.waypoints.clear();
  spinner = 0;
  spinSizeIncrease = 0;
  prevCount = 0;
  dropoffTimeStart = -1;
  timerTimeElapsed = -1;

  countLeft = 0;
  countRight = 0;


  //reset flags
  reachedCollectionPoint = false;
  seenEnoughCenterTags = false;
  circularCenterSearching = false;
  isPrecisionDriving = false;
  finalInterrupt = false;
  precisionInterrupt = false;
  targetHeld = false;
  startWaypoint = false;
  first_center = true;
  cout << "6" << endl;

  lostCenter = false;
  timerStarted = false;
  startSpinSearchTimer = false;
  spinDirection = 1;
  centerSeen = false;

}

void DropOffController::SetTargetData(vector<Tag> tags) {
  countRight = 0;
  countLeft = 0;

  if(targetHeld) {
    // if a target is detected and we are looking for center tags
    if (tags.size() > 0 && !reachedCollectionPoint) {

      // this loop is to get the number of center tags
      for (int i = 0; i < tags.size(); i++) {
        if (tags[i].getID() == 256) {

          // checks if tag is on the right or left side of the image
          if (tags[i].getPositionX() + cameraOffsetCorrection > 0) {
            countRight++;

          } else {
            countLeft++;
          }
        }
      }
    }
  }

}

void DropOffController::ProcessData() {
  if((countLeft + countRight) > 0) {
    isPrecisionDriving = true;
  } else {
    startWaypoint = true;
  }
}

bool DropOffController::ShouldInterrupt() {
  ProcessData();
  if (startWaypoint && !interrupt) {
    interrupt = true;
    precisionInterrupt = false;
    return true;
  }
  else if (isPrecisionDriving && !precisionInterrupt) {
    precisionInterrupt = true;
    return true;
  }
  if (finalInterrupt) {
    return true;
  }
}

bool DropOffController::HasWork() {

  if(timerTimeElapsed > -1) {
    long int elapsed = current_time - returnTimer;
    timerTimeElapsed = elapsed/1e3; // Convert from milliseconds to seconds
  }

  // Changed from 2 to 5, set time limit for 1 spin search wpt to 5secs.
  if (circularCenterSearching && timerTimeElapsed < 5 && !isPrecisionDriving) {
    return false;
  }

  return ((startWaypoint || isPrecisionDriving));
}


bool DropOffController::IsChangingMode() {
  return isPrecisionDriving;
}

void DropOffController::SetCenterLocation(Point center) {
  centerLocation = center;
}

void DropOffController::SetCurrentLocation(Point current) {
  currentLocation = current;
}

void DropOffController::SetTargetPickedUp() {
  targetHeld = true;
}

void DropOffController::SetBlockBlockingUltrasound(bool blockBlock) {
  targetHeld = targetHeld || blockBlock;
}

void DropOffController::SetCurrentTimeInMilliSecs( long int time )
{
  current_time = time;
}
