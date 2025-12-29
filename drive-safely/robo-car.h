#define LOOP_TIME 50
#define NUMBER_OF_ANGLES_IN_SWEEP 180  //Number of samples to take in a sweep
#define MAX_NUMBER_OF_OBJECTS_IN_SWEEP 20
#define MIN_DISTANCE_TO_MOVE 20  //Dont get any closer than this
#define MIN_DISTANCE_AHEAD 30  //Do point moving ahead if this is the furthest object
#define MIN_DISTANCE_TO_TURN 10  //If we have less than this around then we can't rotate

enum Robot_State {
  INIT,
  INIT_FAILED,
  DRIVE,
  ROTATING,
  SWEEP,
  UTURN_SWEEP,
  BACK_OUT,
  OFF_GROUND,
};

enum Drive_State {
  STOPPED,
  TURNING,
  DRIVE_FORWARD,
  REVERSING
};

enum SWEEP_STATUS {
  CLEAR_TO_DRIVE, //We have a direction to drive
  BLOCKED_AHEAD, //No clear route ahead but have room to turn around
  CANNOT_TURN,  //Need to reverse straight back and re-sweep
};
