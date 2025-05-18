package frc.team10505.robot;

import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.CoralSubsystem;
import frc.team10505.robot.subsystems.DrivetrainSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubystem;

public class Superstructure {
    private AlgaeSubsystem algaeSubsys;
    private CoralSubsystem coralSubsys;
    private ElevatorSubystem elevSubsys;
    private DrivetrainSubsystem driveSubsys;

    public Superstructure(AlgaeSubsystem algaeSubsys, CoralSubsystem coralSubsys, ElevatorSubystem elevSubsys, DrivetrainSubsystem driveSubsys){
        this.algaeSubsys = algaeSubsys;
        this.coralSubsys = coralSubsys;
        this.elevSubsys = elevSubsys;
        this.driveSubsys = driveSubsys;
    }

}
