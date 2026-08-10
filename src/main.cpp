#include "main.h"
#include "lemlib/api.hpp"
#define ALIGN_DIST 10
#define SCORING_DIST 1.75
#define NORTH 1
#define EAST 2
#define SOUTH 3
#define WEST 4

float Focal_Length = 172; //Constant for AprilTag proccesing
float April_Tag_Size = 1.0f; //This describes what the size in inches the AprilTag is, as it can change in size every year
float AprilTag_Angle_Trim = lemlib::degToRad(3.8);  // adjust for difference in angle between robot and sensor
float AprilTag_X_offset = 0.0f;
float AprilTag_Y_offset =  -2.0f;

//Structure that we will use for moving to each goal using AprilTags
struct Goal_Info
{
    float Goal_x;
    float Goal_y;
    float Goal_Height;
    int Goal_AprilTag_Number;
};

Goal_Info goals[9] = {
    {70.20,70.20,8.77,0},
    {23.11,93.75,5.77,1},
    {23.11,46.66,3.25,2},
    {46.66,23.11,3.25,3},
    {93.75,23.11,5.77,4},
    {117.30,46.66,5.77,1},
    {117.30,93.75,3.25,2},
    {93.75,117.30,3.25,3},
    {46.66,117.30,5.77,4}
};

//Structure we will use later for a bunch of variables needed for AprilTag proccesing. 
struct Tag_Detection
{
   float Distance_To_AprilTag_In;
   float Robot_to_AprilTag_Angle;
   float Pixel_width;
   int Tag_ID;
   bool Tag_Valid; 
};

Tag_Detection AprilTagProccesing (const auto& AprilTag_Object){
    Tag_Detection Outcome;
    Outcome.Tag_ID = AprilTag_Object.id;
    auto& o = AprilTag_Object.object.tag;
    float top       = std::hypot(o.x1-o.x0, o.y1-o.y0);  
    float bottom    = std::hypot(o.x3-o.x2, o.y3-o.y2);
    float left      = std::hypot(o.x0-o.x3, o.y0-o.y3);
    float right     = std::hypot(o.x1-o.x2, o.y1-o.y2);         
    float Average_Edge_Length = (top+bottom+left+right)/4.0f;
    Outcome.Pixel_width = Average_Edge_Length;
    Outcome.Distance_To_AprilTag_In = Focal_Length*April_Tag_Size/((top+bottom)/2.0f);
    float Average_x_cord = (o.x0+o.x1+o.x2+o.x3)/4.0f;
    Outcome.Robot_to_AprilTag_Angle = lemlib::radToDeg(std::atan2(Average_x_cord - 160, Focal_Length)-AprilTag_Angle_Trim);
    Outcome.Tag_Valid = (Average_Edge_Length > 20);
    return Outcome;
}

pros::MotorGroup left_motors({-2,-6}, pros::MotorGearset::blue); // left motors on ports 2 and 6
pros::MotorGroup right_motors({9,5}, pros::MotorGearset::blue); // right motors on ports 5 and 9
pros::Controller controller(pros::E_CONTROLLER_MASTER); 
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              10, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2 (for now)
);

pros::MotorGroup liftMotors({-3,8}, pros::MotorGearset::green);
pros::Motor wristMotor({4}, pros::MotorGearset::green);
pros::Motor Scoring_Rollers({18}, pros::MotorGearset::green);


// create an imu on port 15
pros::Imu imu(15);
// create distance sensor
// pros::Distance scoring_dist_sensor(5); 

// create a v5 rotation sensor on ports 19 & 20
pros::Rotation vert_rotation_sensor(19);
pros::Rotation horv_rotation_sensor(20);

// create a vision sensor
pros::AIVision aivision(10);

// tracking wheels
lemlib::TrackingWheel vertical_tracking_wheel(&vert_rotation_sensor, lemlib::Omniwheel::NEW_2, .2);
lemlib::TrackingWheel horizontal_tracking_wheel(&horv_rotation_sensor, lemlib::Omniwheel::NEW_2, .2);
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);

void screen_task_function() {

    aivision.enable_detection_types(pros::AivisionModeType::tags);
    aivision.set_tag_family(pros::AivisionTagFamily::tag_21H7);
    float width_of_tag;

    while (true) {
        // print robot location to the brain screen
        pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            
        auto objects = aivision.get_all_objects();

        if(objects.empty()) {
            pros::lcd::print(3,"tag\n");
            pros::lcd::print(4,"id %d\n");
            pros::lcd::print(7,"distance:     valid?   RtTA:     \n");
        }
        else {
            for (auto &object : objects) {
                if (pros::AIVision::is_type(object, pros::AivisionDetectType::tag)) {
                    pros::lcd::print(3,"tag\n");
                    pros::lcd::print(4, "id %d\n", object.id);
                    pros::lcd::print(5, "%d %d %d %d %d %d %d %d\n", object.object.tag.x0, object.object.tag.y0, object.object.tag.x1, object.object.tag.y1, object.object.tag.x2, object.object.tag.y2, object.object.tag.x3, object.object.tag.y3);
                    width_of_tag = (sqrt(std::pow(object.object.tag.y1-object.object.tag.y0,2)+std::pow(object.object.tag.x1-object.object.tag.x0,2)) + 
                    sqrt(std::pow(object.object.tag.y3-object.object.tag.y2,2)+std::pow(object.object.tag.x3-object.object.tag.x2,2))) / 2;
                    pros::lcd::print(6, "%f %f %f\n", Focal_Length, April_Tag_Size, width_of_tag);
                    Tag_Detection myTag = AprilTagProccesing(object);
                    pros::lcd::print(7, "distance: %.2f valid? %d RtTA: %.2f\n", Focal_Length * April_Tag_Size / width_of_tag, myTag.Tag_Valid,lemlib::radToDeg(myTag.Robot_to_AprilTag_Angle));
                    // pros::lcd::print(7, "Robot to April Tag Angle: %f", myTag.Robot_to_AprilTag_Angle);
                    
                }
            }
        }
        pros::delay(50);
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void AprilTag_move_to_score(int goal_Number, int direction) {

    float delta_y;
    float delta_x;

    float scoring_x = goals[goal_Number].Goal_x;
    float scoring_y = goals[goal_Number].Goal_y;
    int scoring_angle;
    
    //Sets the x and y to move to the goal that is depicted by the goal number - score on the face from direction
    if(direction==NORTH){
        scoring_y += SCORING_DIST;
        scoring_angle = 0;
    }
    if(direction==SOUTH){
        scoring_y -= SCORING_DIST;
        scoring_angle = 180;
    }
    if(direction==EAST){
        scoring_x += SCORING_DIST;
        scoring_angle = 90;
    }
    if(direction==WEST){
        scoring_x -= SCORING_DIST;
        scoring_angle = -90;
    }

    auto objects = aivision.get_all_objects();
        
    for(int i=0;i<10;i++){
        if(!objects.empty()) {
            break;
        }    

        if(i == 9) {
        pros::lcd::print(5, "We never found the tag");    
        return;
        }
        pros::delay(50);
        objects = aivision.get_all_objects();

    }
 
    Tag_Detection myTag = AprilTagProccesing(objects[0]);

    //This finds the delta x that is needed to move the vision sensor to the target
    delta_x = myTag.Distance_To_AprilTag_In*sin(lemlib::degToRad(chassis.getPose().theta+myTag.Robot_to_AprilTag_Angle)) + 
                //This translates the AI vision sensor point into the robots center using a x and a y offset 
                cos(lemlib::degToRad(chassis.getPose().theta))*AprilTag_X_offset + 
                sin(lemlib::degToRad(chassis.getPose().theta))*AprilTag_Y_offset;
    delta_y = myTag.Distance_To_AprilTag_In*cos(lemlib::degToRad(chassis.getPose().theta+myTag.Robot_to_AprilTag_Angle)) + 
                cos(lemlib::degToRad(chassis.getPose().theta))*AprilTag_Y_offset - 
                sin(lemlib::degToRad(chassis.getPose().theta))*AprilTag_X_offset;

    pros::lcd::print(5, "a_x:%.2f  a_y:%.2f  start_dist:%.2f\n", roundf(100*chassis.getPose().x)/100, roundf(100*chassis.getPose().y)/100, roundf(100*myTag.Distance_To_AprilTag_In)/100);
    pros::lcd::print(6, "dx:%.2f  dy:%.2f  d_angle:%.2f\n", roundf(100*delta_x)/100, roundf(100*delta_y)/100, roundf(100*myTag.Robot_to_AprilTag_Angle)/100);

    chassis.setPose(scoring_x+delta_x,scoring_y+delta_y, chassis.getPose().theta);
    chassis.moveToPose(scoring_x,scoring_y, scoring_angle, 3000, {.forwards = false});
    
}

void Score_In_Goal(int goal_Number, int direction) {

    float align_x = goals[goal_Number].Goal_x;
    float align_y = goals[goal_Number].Goal_y;
    int align_angle;
    
    //Sets the x and y to move to the goal that is depicted by the goal number - score on the face from direction
    if(direction==NORTH){
        align_y += ALIGN_DIST;
        align_angle = 0;
    }
    if(direction==SOUTH){
        align_y -= ALIGN_DIST;
        align_angle = 180;
    }
    if(direction==EAST){
        align_x += ALIGN_DIST;
        align_angle = 90;
    }
    if(direction==WEST){
        align_x -= ALIGN_DIST;
        align_angle = -90;
    }

    chassis.moveToPose(align_x,align_y,align_angle,5000, {.forwards = false});
    chassis.waitUntilDone();

    /*wristMotor.move(127); // Move full speed up
    pros::delay(500);  
    wristMotor.move(0);
    liftMotors.move(127);
    while (scoring_dist_sensor.get_distance()>20) {

        pros::delay(10);
    }

    pros::delay(1000);
    liftMotors.move(0);
    */

    AprilTag_move_to_score(goal_Number, direction);

    /*Scoring_Rollers.move(-127);
    pros::delay(1000);
    Scoring_Rollers.move(0);
*/
}

extern const char* auto_names[] = {"auto_1", "auto_2"};
extern const int auton_count = 2;
extern int selected_auto = 0;

void auto_1() {

    //chassis.moveToPose(0, 40, 0, 3000);
    //chassis.waitUntilDone();
    chassis.setPose(61,-7,180);
    Score_In_Goal(3,EAST);
}

void auto_2() {

    chassis.turnToHeading(90, 3000);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 3000);
    chassis.waitUntilDone();
}

void auton_selector() {

    while (true)
    {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT))
        {
            selected_auto = (selected_auto - 1 + auton_count) % auton_count;
            pros::delay(100);  
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT))
        {
            selected_auto = (selected_auto + 1) % auton_count;
            pros::delay(100);
        }
        
        //controller.clear();
        //pros::delay(50);
        controller.set_text(0,0, auto_names[selected_auto]);
        
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X) && 
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
            pros::delay(50);
            controller.set_text(0,0,"The chosen:");
            pros::delay(50);
            controller.set_text(1,0, auto_names[selected_auto]);
            return;
            }
    
        pros::delay(50);  
    }
}

void initialize() {
    pros::lcd::initialize();
    //pros::lcd::set_text(1, "Hello PROS User!");

    controller.clear();
    pros::delay(50);                       
    //controller.set_text(0,0,"Hi Andrew");
    chassis.calibrate(); // calibrate sensors
    chassis.setPose (0,0,0);
    
    // print position to brain screen
    static pros::Task screen_task = pros::Task(screen_task_function);

    auton_selector();
    
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    
    switch(selected_auto) {
        case 0: 
            auto_1();
            break;
        case 1:
            auto_2();
            break;
        default:
            controller.set_text(0,0,"Err: Auto Sel");

    }
}  

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

//controller.set_text(2,0,"Hi Andrew");
//pros::delay(50);        
//controller.set_text(1,0,auto_names[selected_auto]);

    while (true) {

        Scoring_Rollers.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        // controller.set_text(0,0,"The chosen: ");
        //controller.set_text(2,0,"Hi Andrew");

        //controller.set_text(1,0,auto_names[selected_auto]);       
        
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B) && 
            controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
                autonomous();
        }
        
        
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.arcade(leftY, rightX);
/*
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            Scoring_Rollers.move(127); // Move full speed up
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            Scoring_Rollers.move(-127); // Move full speed down
        } else {
            Scoring_Rollers.move(0); // Stop moving motors
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            liftMotors.move(127); // Move full speed up
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            liftMotors.move(-127); // Move full speed down
        } else {
            liftMotors.move(0); // Stop moving motors
        }

         if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            wristMotor.move(127); // Move full speed up
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            wristMotor.move(-127); // Move full speed down
        } else {
            wristMotor.move(0); // Stop moving motors
        }
  */     
        // delay to save resources
        pros::delay(25);                            // Run for 20 ms then update
    }
}

