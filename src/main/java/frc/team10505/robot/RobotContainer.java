package frc.team10505.robot;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.subsystems.ElevatorSubystem;
import frc.team10505.robot.subsystems.CoralSubsystem;
import frc.team10505.robot.subsystems.DrivetrainSubsystem;
import frc.team10505.robot.Poses;

import frc.team10505.robot.generated.TunerConstants;
import frc.team10505.robot.subsystems.AlgaeSubsystem;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static frc.team10505.robot.Constants.ElevatorConstants.*;
import static frc.team10505.robot.Constants.IntakeConstants.*;

import java.util.List;

public class RobotContainer {
    private CommandXboxController xboxController = new CommandXboxController(0);
    private CommandJoystick joystick = new CommandJoystick(0);
    private CommandJoystick joystick2 = new CommandJoystick(1);

    private final ElevatorSubystem elevSubsys = new ElevatorSubystem();
    private final CoralSubsystem coralSubsys;
    private final AlgaeSubsystem algaeSubsys = new AlgaeSubsystem();
    private final DrivetrainSubsystem driveSubsys = TunerConstants.createDrivetrain();

    private Tasks task = Tasks.TRAVEL;
    private boolean leftSide = false;

    private SendableChooser<Double> polarityChooser = new SendableChooser<>();
    private SendableChooser<List<Waypoint>> nextReefSpotChooser = new SendableChooser<>();
    private SendableChooser<Double> nextLevelChooser = new SendableChooser<>();

    public RobotContainer() {
        if (Utils.isSimulation()) {
            coralSubsys = new CoralSubsystem(joystick);
            simConfigButtonBindings();
        } else {
            coralSubsys = new CoralSubsystem();
            configButtonBindings();
        }

        SmartDashboard.putData("Polarity", polarityChooser);
        polarityChooser.setDefaultOption("Positive", 1.0);
        polarityChooser.addOption("negative", -1.0);

        SmartDashboard.putData("Next Level", nextLevelChooser);
        nextLevelChooser.setDefaultOption("L1", 1.0);
        nextLevelChooser.addOption("L2", 2.0);
        nextLevelChooser.addOption("L3", 3.0);
        nextLevelChooser.addOption("L4", 4.0);

        SmartDashboard.putData("Next Reef Spot", nextReefSpotChooser);
        nextReefSpotChooser.setDefaultOption("A (Left Side, Front Center)", Poses.reefAPose);
        nextReefSpotChooser.addOption("B (Right Side, Front Center)", Poses.reefBPose);
        nextReefSpotChooser.addOption("C (Left Side, Front Right)", Poses.reefCPose);
        nextReefSpotChooser.addOption("D (Right Side, Front Right)", Poses.reefDPose);
        nextReefSpotChooser.addOption("E (Left Side, Back Right)", Poses.reefEPose);
        nextReefSpotChooser.addOption("F (Right Side, Back Right)", Poses.reefFPose);
        nextReefSpotChooser.addOption("G (Left Side, Back Center)", Poses.reefGPose);
        nextReefSpotChooser.addOption("H (Right Side, Back Center)", Poses.reefHPose);
        nextReefSpotChooser.addOption("I (Left Side, Back Left)", Poses.reefIPose);
        nextReefSpotChooser.addOption("J (Right Side, Back Left)", Poses.reefJPose);
        nextReefSpotChooser.addOption("K (Left Side, Front Left)", Poses.reefKPose);
        nextReefSpotChooser.addOption("L (Right Side, Front Left)", Poses.reefLPose);

        configDefaultCommands();
    }

    private void configDefaultCommands() {

    }

    private void configButtonBindings() {

    }

    private void simConfigButtonBindings() {
        // joystick.button(1).onTrue(elevSubsys.setHeight(ELEV_DOWN));
        // joystick.button(2).onTrue(elevSubsys.setHeight(ELEV_L2));
        // joystick.button(3).onTrue(elevSubsys.setHeight(ELEV_L3));
        // joystick.button(4).onTrue(elevSubsys.setHeight(ELEV_L4));

        joystick.button(1).onTrue(algaeSubsys.setAngle(-70));
        joystick.button(2).onTrue(algaeSubsys.setAngle(-15));
        joystick.button(3).onTrue(algaeSubsys.setAngle(0));
        joystick.button(4).onTrue(algaeSubsys.setAngle(50));
        // joystick.button(3).whileTrue(algaeSubsys.runIntake(INTAKE_FAST));
        // joystick.button(4).whileTrue(algaeSubsys.runcoral(coral_SLOW));

        // joystick2.button(1).onTrue(changeLevel(1));
        // joystick2.button(2).onTrue(changeLevel(2));
        // joystick2.button(3).onTrue(changeLevel(3));
        // joystick2.button(4).onTrue(changeLevel(4));

        // joystick2.button(1).onTrue(coralSubsys.setIntake(INTAKE_SLOW));
        // joystick2.button(2).onTrue(coralSubsys.setIntake(INTAKE_FAST));
        // joystick2.button(3).onTrue(coralSubsys.runIntake(INTAKE_SLOW).until(() ->
        // coralSubsys.seesFirstSensor()));
        // joystick2.button(4).onTrue(coralSubsys.runIntake(INTAKE_FAST).until(() ->
        // coralSubsys.seesFirstSensor()&& coralSubsys.seesSecondSensor()));

    }

    // private Command changeLevel(double newLevel){
    // return runOnce(() -> {
    // reefLevel = newLevel;
    // });
    // }

    private void run() {
        switch (task) {
            case SEEK_CORAL:
                    SmartDashboard.putString("Case Message", "Case is Seek coral");
                    elevSubsys.setHeight(ELEV_DOWN);
                    coralSubsys.seekingCoral();

                    if (coralSubsys.seesFirstSensor() && coralSubsys.seesSecondSensor()) {
                        SmartDashboard.putString("Case Message", "Has coral, moving on!");
                        task = Tasks.SEEK_REEF;
                    }
                break;
            case SEEK_REEF:
                    SmartDashboard.putString("Case Message", "Case is Seek reef");
                    driveSubsys.goToReef(nextReefSpotChooser.getSelected());
                break;
            case SCORE_CORAL:
                    SmartDashboard.putString("Case Message", "Case is Score Coral");

                break;
            case TRAVEL:
                    SmartDashboard.putString("Case Message", "Case is travel");
                    elevSubsys.setHeight(ELEV_DOWN);

                break;
            default:
                    SmartDashboard.putString("Case Message", "Case is default");

                break;

        }
    }
}
