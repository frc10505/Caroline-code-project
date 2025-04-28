package frc.team10505.robot;


import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.team10505.robot.subsystems.ElevatorSubystem;
import static frc.team10505.robot.Constants.ElevatorConstants.*;

public class RobotContainer {
    private CommandXboxController xboxController = new CommandXboxController(0);
    private CommandJoystick joystick = new CommandJoystick(0);
    private final ElevatorSubystem elevSubsys = new ElevatorSubystem();

    public RobotContainer(){
        if(Utils.isSimulation()){
            simConfigButtonBindings();
        }else{
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

    }
}
