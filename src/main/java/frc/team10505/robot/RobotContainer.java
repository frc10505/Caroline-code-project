package frc.team10505.robot;


import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.subsystems.ElevatorSubystem;
import frc.team10505.robot.subsystems.IntakeSubsystem;

import static frc.team10505.robot.Constants.ElevatorConstants.*;
import static frc.team10505.robot.Constants.IntakeConstants.*;


public class RobotContainer {
    private CommandXboxController xboxController = new CommandXboxController(0);
    private CommandJoystick joystick = new CommandJoystick(0);
    private CommandJoystick joystick2 = new CommandJoystick(1);

    private final ElevatorSubystem elevSubsys = new ElevatorSubystem();
    private final IntakeSubsystem intakeSubsys;

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
        joystick.button(1).onTrue(elevSubsys.setHeight(ELEV_DOWN));
        joystick.button(2).onTrue(elevSubsys.setHeight(ELEV_L2));
        joystick.button(3).onTrue(elevSubsys.setHeight(ELEV_L3));
        joystick.button(4).onTrue(elevSubsys.setHeight(ELEV_L4));

        joystick2.button(1).onTrue(intakeSubsys.setIntake(INTAKE_SLOW));
        joystick2.button(2).onTrue(intakeSubsys.setIntake(INTAKE_FAST));
        joystick2.button(3).onTrue(intakeSubsys.runIntake(INTAKE_SLOW).until(() -> intakeSubsys.seesFirstSensor()));
        joystick2.button(4).onTrue(intakeSubsys.runIntake(INTAKE_FAST).until(() -> intakeSubsys.seesFirstSensor()&& intakeSubsys.seesSecondSensor()));

    }
}
