package frc.team10505.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.subsystems.ElevatorSubystem;
import frc.team10505.robot.subsystems.drive.DrivetrainSubsystem;
import frc.team10505.robot.subsystems.drive.Lasers;
import frc.team10505.robot.subsystems.drive.Pathplanning;
import frc.team10505.robot.subsystems.drive.Poses.ReefSpot;
import frc.team10505.robot.subsystems.drive.generated.TunerConstants;
import frc.team10505.robot.subsystems.CoralSubsystem;
import frc.team10505.robot.simulation.Simulation;
import frc.team10505.robot.subsystems.AlgaeSubsystem;

import static frc.team10505.robot.Constants.ElevatorConstants.*;
import static frc.team10505.robot.Constants.IntakeConstants.CORAL_FAST_SPEED;
import static frc.team10505.robot.Constants.IntakeConstants.CORAL_INTAKE_SPEED;
import static frc.team10505.robot.Constants.IntakeConstants.CORAL_SLOW_SPEED;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.team10505.robot.Constants.AlgaeConstants.*;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private SwerveRequest.RobotCentric robotDrive = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private CommandXboxController xboxController = new CommandXboxController(0);
    private CommandJoystick joystick = new CommandJoystick(0);
    private CommandJoystick joystick2 = new CommandJoystick(1);

    private final ElevatorSubystem elevSubsys = new ElevatorSubystem();
    private final CoralSubsystem coralSubsys;
    private final AlgaeSubsystem algaeSubsys = new AlgaeSubsystem();
    private final DrivetrainSubsystem driveSubsys = TunerConstants.createDrivetrain();
    private final Pathplanning flypath = new Pathplanning();
    private final Lasers lasers;
    private Simulation simulation;
    private final Superstructure superstructure;

    private Tasks task = Tasks.TRAVEL;
    private boolean leftSide = false;

    private SendableChooser<Double> polarityChooser = new SendableChooser<>();
    private SendableChooser<ReefSpot> nextReefSpotChooser = new SendableChooser<>();
    private SendableChooser<Double> nextLevelChooser = new SendableChooser<>();
    public SendableChooser<Command> autoChooser;

    public RobotContainer() {
        if (Utils.isSimulation()) {
            coralSubsys = new CoralSubsystem(joystick);
            lasers = new Lasers(joystick);
            simulation = new Simulation(algaeSubsys, coralSubsys, elevSubsys, lasers);
            simConfigButtonBindings();
        } else {
            coralSubsys = new CoralSubsystem();
            lasers = new Lasers();
            configButtonBindings();
        }

        superstructure = new Superstructure(algaeSubsys, coralSubsys, elevSubsys, driveSubsys);


        //calling everying config cause its fun lol
        configNamedCommands();
        driveSubsys.configAutoBuilder(autoChooser);
        //configSendableChoosers();
        configDefaultCommands();
    }

    private void configNamedCommands(){
        NamedCommands.registerCommand("Test", print("Named Commands Test"));

        NamedCommands.registerCommand("Elev Down", elevSubsys.setHeight(ELEV_DOWN));
        NamedCommands.registerCommand("Elev L2", elevSubsys.setHeight(ELEV_L2));
        NamedCommands.registerCommand("Elev L3", elevSubsys.setHeight(ELEV_L3));
        NamedCommands.registerCommand("Elev L4", elevSubsys.setHeight(ELEV_L4));

        
    }

    private void configDefaultCommands() {

    }

    private void configButtonBindings() {

    }

    private void simConfigButtonBindings() {
        joystick.button(1).onTrue(elevSubsys.setHeight(ELEV_DOWN));
        joystick.button(2).onTrue(elevSubsys.setHeight(ELEV_L2));
        joystick.button(3).onTrue(elevSubsys.setHeight(ELEV_L3));
        joystick.button(4).onTrue(elevSubsys.setHeight(ELEV_L4));

        // joystick.button(1).onTrue(algaeSubsys.setAngle(-70));
        // joystick.button(2).onTrue(algaeSubsys.setAngle(-15));
        // joystick.button(3).onTrue(algaeSubsys.setAngle(0));
        // joystick.button(4).onTrue(algaeSubsys.setAngle(50));
        // joystick.button(3).whileTrue(algaeSubsys.runIntake(INTAKE_FAST));
        // joystick.button(4).whileTrue(algaeSubsys.runcoral(coral_SLOW));


        joystick2.button(1).onTrue(coralSubsys.setIntake(CORAL_SLOW_SPEED));
        joystick2.button(2).onTrue(coralSubsys.setIntake(CORAL_FAST_SPEED));
        joystick2.button(3).onTrue(coralSubsys.runIntake(CORAL_INTAKE_SPEED).until(() ->
        coralSubsys.inSensor()));
        joystick2.button(4).onTrue(coralSubsys.runIntake(CORAL_SLOW_SPEED).until(() ->
        coralSubsys.inSensor()&& coralSubsys.outSensor()));

    }

    private void configSendableChoosers(){
        SmartDashboard.putData("Polarity Chooser", polarityChooser);
        polarityChooser.setDefaultOption("Positive", 1.0);
        polarityChooser.addOption("negative", -1.0);

        SmartDashboard.putData("Next Level Chooser", nextLevelChooser);
        nextLevelChooser.setDefaultOption("L1", 1.0);
        nextLevelChooser.addOption("L2", 2.0);
        nextLevelChooser.addOption("L3", 3.0);
        nextLevelChooser.addOption("L4", 4.0);

        SmartDashboard.putData("Next Reef Spot Chooser", nextReefSpotChooser);
        nextReefSpotChooser.setDefaultOption("A (Left Side, Front Center)", ReefSpot.A);
        nextReefSpotChooser.addOption("B (Right Side, Front Center)", ReefSpot.B);
        nextReefSpotChooser.addOption("C (Left Side, Front Right)", ReefSpot.C);
        nextReefSpotChooser.addOption("D (Right Side, Front Right)", ReefSpot.D);
        nextReefSpotChooser.addOption("E (Left Side, Back Right)", ReefSpot.E);
        nextReefSpotChooser.addOption("F (Right Side, Back Right)", ReefSpot.F);
        nextReefSpotChooser.addOption("G (Left Side, Back Center)", ReefSpot.G);
        nextReefSpotChooser.addOption("H (Right Side, Back Center)", ReefSpot.H);
        nextReefSpotChooser.addOption("I (Left Side, Back Left)", ReefSpot.I);
        nextReefSpotChooser.addOption("J (Right Side, Back Left)", ReefSpot.J);
        nextReefSpotChooser.addOption("K (Left Side, Front Left)", ReefSpot.K);
        nextReefSpotChooser.addOption("L (Right Side, Front Left)", ReefSpot.L);

        SmartDashboard.putData("Auto Chooser", autoChooser);
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

                    if (coralSubsys.inSensor() && coralSubsys.outSensor()) {
                        SmartDashboard.putString("Case Message", "Has coral, moving on!");
                        task = Tasks.SEEK_REEF;
                    }
                break;
            case SEEK_REEF:
                    SmartDashboard.putString("Case Message", "Case is Seek reef");
                    //driveSubsys.goToReef(nextReefSpotChooser.getSelected());
                break;
            case SCORE_CORAL:
                    SmartDashboard.putString("Case Message", "Case is Score Coral");

                break;
            case SEEK_BARGE:
                    SmartDashboard.putString("Case Message", "Cse is Seek Barge");

                    algaeSubsys.setIntake(ALGAE_HOLD_SPEED);
                    driveSubsys.applyRequest(() -> robotDrive.withVelocityX(-1)).withTimeout(0.3);
                    elevSubsys.setHeight(ELEV_DOWN);
                    flypath.goToBarge(driveSubsys.getState().Pose);
                    task = Tasks.BARGE_SHOT;
                break;
            case BARGE_SHOT:
                    SmartDashboard.putString("Case Message", "Case is barge shot");

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
