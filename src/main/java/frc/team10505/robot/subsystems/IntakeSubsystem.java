package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class IntakeSubsystem extends SubsystemBase {
    /*Variables */
    private CommandJoystick joystick;

    /*Constructor */
    public IntakeSubsystem(){

    }
    public IntakeSubsystem(CommandJoystick joystick){
        this.joystick = joystick;
    }


    /*Calculations */
    //booleans that return if a sensor is triggered(Reads a distance less than whatever we tell it)
    public boolean seesFirstSensor(){
        if(Utils.isSimulation()){
            return joystick.button(1).getAsBoolean();
        } else{
            try{
                return false;//TODO change to whatever we had in comp code
            } catch (Exception e){
                Commands.print("FIRST LASER FAILED");
                return false;
            }
        }
    }

    public boolean seesSecondSensor(){
        if(Utils.isSimulation()){
            return joystick.button(2).getAsBoolean();
        } else{
            try{
                return false;//TODO change to whatever we had in comp code
            } catch (Exception e){
                Commands.print("SECOND LASER FAILED");
                return false;
            }
        }
    }

    @Override
    public void periodic(){

    }
}