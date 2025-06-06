package frc.team10505.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Constants {
    public final class HardwareConstants {
        // THE CURRENT LIMITS AND CAN IDS FOR THE DRIVETRAIN CAN BE FOUND IN
        // TunerConstants.java

        /* Robo Rio ports/channels */
        public final static int DRIVE_LEDS_CHANNEL = 0;

        /* CAN IDS */
        public final static int CORAL_LEFT_MOTOR_ID = 2;
        public final static int CORAL_RIGHT_MOTOR_ID = 3;

        public final static int ALGAE_INTAKE_MOTOR_ID = 7;
        public final static int ALGAE_PIVOT_MOTOR_ID = 8;

        public final static int ELEVATOR_MOTOR_ID = 10;
        public final static int ELEVATOR_FOLLOWER_MOTOR_ID = 11;

        /* 40-51 can ids are used in the drivetrain */

        public final static int DRIVE_RIGHT_LASER_ID = 52;
        public final static int DRIVE_LEFT_LASER_ID = 53;

        public final static int CORAL_IN_LASER_ID = 60;
        public final static int CORAL_OUT_LASER_ID = 61;

        /* Current Limits */
        public final static int ALGAE_PIVOT_MOTOR_CURRENT_LIMIT = 15;
        public final static int ALGAE_INTAKE_MOTOR_CURRENT_LIMIT = 25;

        public final static int CORAL_MOTOR_CURRENT_LIMIT = 15;

        public final static int ELEVATOR_MOTOR_CURRENT_LIMIT = 40;

        /* Encoder Offsets */
        public final static double ALGAE_PIVOT_ENCODER_OFFSET = 0;
        public final static double ALGAE_PIVOT_ENCODER_SCALE = 360;

        /* Gearstacks */
        public final static double ELEVATOR_GEARSTACK = 12;
        public final static double ALGAE_PIVOT_GEARSTACK = 80;
    }

    public class ElevatorConstants {
        public static final double ELEV_DOWN = 0;
        public static final double ELEV_L2 = 8;
        public static final double ELEV_L3 = 23.5;
        public static final double ELEV_L4 = 48;
        public static final double ELEV_L4_BUMP = 55;

        public static final double ELEV_BARGE = 55.5;

    }

    public class IntakeConstants {
        public static final double CORAL_SLOW_SPEED = 0.3;
        public static final double CORAL_INTAKE_SPEED = 0.37;
        public static final double CORAL_SCORE_SPEED = 0.25;
        public static final double CORAL_SCORE_L4_SPEED = 0.05;


        public final static double CORAL_TROUGH_LEFT_SPEED = 0.5;
    public final static double CORAL_TROUGH_RIGHT_SPEED = 0.225;
    }

    public class AlgaeConstants {
        public static final double ALGAE_PIVOT_DOWN = -90;
        public static final double ALGAE_PIVOT_UP = 90;
        public static final double ALGAE_PIVOT_OUT = 0;
        public static final double ALGAE_PIVOT_HOLD_ABOVE = 13;//TODO made up number
        public static final double ALGAE_PIVOT_HOLD_BELOW = -13;


        public static final double ALGAE_INTAKE_SPEED = 0.6;// TODO made up random number
        public static final double ALGAE_HOLD_SPEED = 0.05;// TODO made up random number
        public static final double ALGAE_SLOW_SPEED = 0.3;
    }

    public class VisionConstants{    
        public final static String FRONT_CAM_NAME = "Front Cam";
        public final static int FRONT_CAM_WIDTH_RES = 4656;
        public final static int FRONT_CAM_HEIGHT_RES = 3496;
        public final static double FRONT_CAM_FOV_DEG = 90;
        public final static Transform3d FRONT_CAM_TO_ROBOT = new Transform3d(
                new Translation3d(-1.0, 1.0, 0.0),
                new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0)));
        
        public final static String BACK_CAM_NAME = "Back Cam";
        public final static int BACK_CAM_WIDTH_RES = 4656;
        public final static int BACK_CAM_HEIGHT_RES = 3496;
        public final static double BACK_CAM_FOV_DEG = 90;
        public final static Transform3d BACK_CAM_TO_ROBOT= new Transform3d(
          new Translation3d(0.38, -0.35, 0.178),
          new Rotation3d(0.0, Units.degreesToRadians(15.0), Units.degreesToRadians(0.0)));
        
      }  
}
