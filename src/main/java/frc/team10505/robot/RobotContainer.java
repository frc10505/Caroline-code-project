package frc.team10505.robot;


import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.subsystems.ElevatorSubystem;
import frc.team10505.robot.subsystems.IntakeSubsystem;
import frc.team10505.robot.subsystems.PivotSubsystem;

import static frc.team10505.robot.Constants.ElevatorConstants.*;
import static frc.team10505.robot.Constants.IntakeConstants.*;



public class RobotContainer {
    private CommandXboxController xboxController = new CommandXboxController(0);
    private CommandJoystick joystick = new CommandJoystick(0);
    private CommandJoystick joystick2 = new CommandJoystick(1);

    private final ElevatorSubystem elevSubsys = new ElevatorSubystem();
    private final IntakeSubsystem intakeSubsys;
    private final PivotSubsystem pivotSubsys = new PivotSubsystem();

    private Tasks taskNumber = Tasks.TRAVEL;

    public RobotContainer(){
        if(Utils.isSimulation()){
            intakeSubsys = new IntakeSubsystem(joystick);
            simConfigButtonBindings();
        }else{
            intakeSubsys = new IntakeSubsystem();
            configButtonBindings();
        }
    }

    private void configButtonBindings(){
        
    }

    private void simConfigButtonBindings(){
        // joystick.button(1).onTrue(elevSubsys.setHeight(ELEV_DOWN));
        // joystick.button(2).onTrue(elevSubsys.setHeight(ELEV_L2));
        // joystick.button(3).onTrue(elevSubsys.setHeight(ELEV_L3));
        // joystick.button(4).onTrue(elevSubsys.setHeight(ELEV_L4));

        joystick.button(1).onTrue(pivotSubsys.setAngle(-70));
        joystick.button(2).onTrue(pivotSubsys.setAngle(-15));
        joystick.button(3).onTrue(pivotSubsys.setAngle(0));
        joystick.button(4).onTrue(pivotSubsys.setAngle(50));
        // joystick.button(3).whileTrue(pivotSubsys.runIntake(INTAKE_FAST));
        // joystick.button(4).whileTrue(pivotSubsys.runIntake(INTAKE_SLOW));



        joystick2.button(1).onTrue(intakeSubsys.setIntake(INTAKE_SLOW));
        joystick2.button(2).onTrue(intakeSubsys.setIntake(INTAKE_FAST));
        joystick2.button(3).onTrue(intakeSubsys.runIntake(INTAKE_SLOW).until(() -> intakeSubsys.seesFirstSensor()));
        joystick2.button(4).onTrue(intakeSubsys.runIntake(INTAKE_FAST).until(() -> intakeSubsys.seesFirstSensor()&& intakeSubsys.seesSecondSensor()));

    }

    private void run(){
        switch (taskNumber) {
            case TRAVEL:
                SmartDashboard.putString("Case Message", "Case is travel");
                elevSubsys.setHeight(ELEV_DOWN);

                break;
            case SEEK_CORAL:
                SmartDashboard.putString("Case Message", "Case is Seek coral");
                elevSubsys.setHeight(ELEV_DOWN);
                //superstructure.intakeCoral();

                break;
            case SCORE_CORAL:
                SmartDashboard.putString("Case Message", "Case is Score Coral");
            default:
                break;
        }
    }
}
