package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer.RobotStatus;
import frc.robot.util.Range;
import frc.robot.util.lights.animations.AdvancedAnimation;
import frc.robot.util.lights.animations.LightState;

public final class Constants {
    //Current limit for stopping motors from exceeding max power draw
    public static final SupplyCurrentLimitConfiguration CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 20, 20, 0.1); //enable these limits, current limit, trigger threshold, trigger threshold time

    //Robot mode (for odometry toggling)
    public static RobotStatus robotStatus = RobotStatus.AUTO;

    public static Translation2d goalPos = new Translation2d(27, 13.5);

    public static Color allianceColor = Color.Red;

    public static class Drivetrain {
        //Hardware ID's
        public static final int RIGHT_DRIVETRAIN_LEADER = 01;
        public static final int RIGHT_DRIVETRAIN_FOLLOWER = 02;
        public static final int LEFT_DRIVETRAIN_LEADER = 04;
        public static final int LEFT_DRIVETRAIN_FOLLOWER = 03;

        public static final int PIGEON_GYRO = 05;
        
        public static final int COMPRESSOR_ID = 06;
        public static final int PDH_ID = 07;

        //Values for converting the motor counts to distance traveled
        public static final double COUNTS_PER_REV = 2048; //Drivetrain motors (falcon 500)
        public static final double GEAR_RATIO = 
            ((12.0) / (40.0)) *
            (14.0 / 40.0);
        public static final double WHEEL_SIZE = 6; //In inches

        public static double LEFT_P = 3.5;
        public static double LEFT_I = 0;
        public static double LEFT_D = 0.;

        public static double LEFT_KS = .75;
        public static double LEFT_KV = 1.9;

        public static double RIGHT_P = 3.5;
        public static double RIGHT_I = 0;
        public static double RIGHT_D = 0.;

        public static double RIGHT_KS = .75;
        public static double RIGHT_KV = 1.9;

        //The multiplier used by default when the robot is in normal mode (instead of turbo)
        public static double NORMAL_SPEED_MULT = 0.6;
        public static double SLOW_SPEED_MULT = 0.1;

        //Max velocity (in meters per second because that's what pathweaver does)
        public static final double MAX_LINEAR_VELOCITY = 3;
        public static final double MAX_LINEAR_ACCELERATION = 2;
        //Max angular velocity (in degrees per second (because radians are cringe))
        public static final double MAX_ANGULAR_VELOCITY = Math.toDegrees(Math.PI);

        public static final double MAX_VOLTAGE = 11.5;
        
        public static final double TRACK_WIDTH = 0.6798; //meters
        

        //Tested to be good values by WPILib
        public static final double K_RAMSETE_B = 12.5;
        public static final double K_RAMSETE_ZETA = 0.7;

        //TODO: Set this to the actual achievable error
        public static final double PREFERRED_DISTANCE_ERROR = Units.inchesToMeters(3);
        public static final double PREFERRED_ANGLE_ERROR = Units.degreesToRadians(5);
    }

    public static class Intake {
        //Hardware
        public static final int ROLLER_1_ID = 11;
        
        public static final int SOLENOID_1_PORT_1 = 0;
        public static final int SOLENOID_1_PORT_2 = 2;

        //Control variables
        public static final double INTAKE_SPEED = 0.8;
    }

    public static class Conveyor {
        //Hardware
        public static final int CONVEYOR_STAGE1_ID = 21;
        public static final int CONVEYOR_STAGE2_ID = 22;
        public static final int PROX_STAGE1_PORT = 0; //DIO port
        public static final int PROX_STAGE2_PORT = 1; //DIO port
        public static final int PROX_STAGE3_PORT = 2; //DIO port

        public static final int COLOR_SENSOR_PORT1 = 3; //DIO port
        public static final int COLOR_SENSOR_PORT2 = 4; //DIO port

        //Control variables
        public static final double DEFAULT_CONVEYOR_SPEED = 0.25;
        public static final double EJECTION_DELAY = .5;
    }

    
    public static final class Turret{ 
        //Hardware devices
        public static final int BOTTOM_FLYWHEEL = 31;
        public static final int TURRET_SPINNER = 33;
        public static final int HALL_EFFECT_CENTER = 5; //DIO port
        public static final int HALL_EFFECT_CLOCKWISE = 6; //DIO port
        public static final int HALL_EFFECT_COUNTERCLOCKWISE = 7; //DIO port

        //TOOD: Get the actual values for this
        public static final double CENTRAL_SENSOR_ANGLE = -5;
        public static final double CLOCKWISE_SENSOR_ANGLE = -175;
        public static final double COUNTERCLOCKWISE_SENSOR_ANGLE = 175;


        //Flywheel control
        public static final double BOTTOM_FLYWHEEL_S = 0.75;
        public static final double BOTTOM_FLYWHEEL_V = 0.00175;
        public static final double BOTTOM_FLYWHEEL_P = 0.001;
        public static final double BOTTOM_FLYWHEEL_I = 0;
        public static final double BOTTOM_FLYWHEEL_D = 0.0005;

        public static final double TOP_FLYWHEEL_S = 0.25;
        public static final double TOP_FLYWHEEL_V = 0.0017;
        public static final double TOP_FLYWHEEL_P = 0.01;
        public static final double TOP_FLYWHEEL_I = 0;
        public static final double TOP_FLYWHEEL_D = 0;

        //TODO: Tune this value to attainable error
        public static final double FLYWHEEL_ALLOWED_ERROR = 50; //The allowed error for the flywheel setpoint (in RPM)
        public static final double FLYWHEEL_TARGET_RPM = 3800; //The normal target rpm the turret will target
        public static final double FLYWHEEL_LOW_RPM = 1800; //The normal target rpm the turret will target

        //Spinner control
        public static final double COUNTS_SPINNER_ENCODER = 2048;
        public static final double TURRET_GEAR_RATIO = (1.0/3.0) * (10.0 / 140.0);

        public static final double SPINNER_CLOCKWISE_LIMIT = -165; //Clockwise turns are negative
        public static final double SPINNER_COUNTERCLOCKWISE_LIMIT = 165; //Counter-clockwise turns are psotiive
        public static final double ACCEPTABLE_ERROR = 4; //How close the turret has to get to it's setpoint before isBusy() returns false
        public static final Range INVALID_SHOOTING_RANGE = new Range(200, 300);
        public static final double SEARCH_VEL = 90; //In deg/sec: The speed the spinner will search at when it doesn't have a target
        public static final double ZERO_VEL = 45; //In deg/sec: The speed the spinner use to zero out at the start of a match
        public static final double MANUAL_SPEED = 0.1; //How fast the spinner can move manually (in the event of a crash)

        public static final double SPINNER_P = 0.017;
        public static final double SPINNER_I = 0.0;
        public static final double SPINNER_D = 0.01;

        public static final double SPINNER_MAX_VEL = 360; //Deg/sec
        public static final double SPINNER_MAX_ACCEL = 480; //Deg/sec/sec

        public static final double SPINNER_S = 0.8;
        public static final double SPINNER_V = 0.009;
    
        public static final SupplyCurrentLimitConfiguration FLYWHEEL_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(true, 20, 20, 0.1); //enable these limits, current limit, trigger threshold, trigger threshold time

    }

    public static class Climber {
        //Hardware
        public static final int LEFT_CLIMBER_ID = 41;
        public static final int RIGHT_CLIMBER_ID = 42;

        public static final int BRAKE = 4; //Pneumatics port

        public static final double LEFT_P = 0.0001;
        public static final double LEFT_I = 0;
        public static final double LEFT_D = 0;
        public static final double LEFT_KS = 1;
        public static final double LEFT_KV = .000035;
        // public static final double LEFT_KV = .0000;

        public static final double RIGHT_P = 0.0001;
        public static final double RIGHT_I = 0;
        public static final double RIGHT_D = 0;
        public static final double RIGHT_KS = 1;
        public static final double RIGHT_KV = .000035;
        // public static final double RIGHT_KV = .0000;

        public static final double MAX_VEL = 200 * 2048; //Encoder counts per second
        public static final double MAX_ACCEL = 250 * 2048; //Encoder counts per second per second

        public static final double MID_HEIGHT = 28; //The height (in inches) that the climber will move to for the mid level climb
        public static final double LOW_HEIGHT = 12; //Same as above (in inches) but for low goal
        public static final double LOWERED_HEIGHT = -40; //The fully lowered position of the climber (also in inches)
    }

    public static class Lights {
        public static final int CONTROLLER_ID = 51;

        public static final AdvancedAnimation DEFAULT_ANIMAION = new AdvancedAnimation(new LightState(0, 0, 255, .66, .33), new LightState(255, 255, 255, .66, .33));
        
    }

    public static class Controller {
        //Driver Controller
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int DRIVER_LEFT_JOYSTICK_Y_AXIS = 1; //Left stick y
        public static final int DRIVER_LEFT_JOYSTICK_X_AXIS = 0; //Left stick x
        public static final int DRIVER_RIGHT_JOYSTICK_Y_AXIS = 5; //Right stick y

        //Operator Controller
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static enum Color {
        Red, Blue, SensorNotDetected, NoBallDetected
    }
    
}