package frc.team10505.robot;

import static frc.team10505.robot.Constants.AlgaeConstants.*;
import static frc.team10505.robot.Constants.ElevatorConstants.*;
import static frc.team10505.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import frc.team10505.robot.subsystems.AlgaeSubsystem;
import frc.team10505.robot.subsystems.CoralSubsystem;
import frc.team10505.robot.subsystems.ElevatorSubystem;
import frc.team10505.robot.subsystems.drive.DrivetrainSubsystem;
import frc.team10505.robot.subsystems.drive.Lasers;

public class Superstructure {
    private AlgaeSubsystem algaeSubsys;
    private CoralSubsystem coralSubsys;
    private ElevatorSubystem elevSubsys;
    private DrivetrainSubsystem driveSubsys;
    private Lasers lasers;

    private final SwerveRequest.ApplyRobotSpeeds autoDrive = new SwerveRequest.ApplyRobotSpeeds();

    public Superstructure(AlgaeSubsystem algaeSubsys, CoralSubsystem coralSubsys, ElevatorSubystem elevSubsys,
            DrivetrainSubsystem driveSubsys, Lasers lasers) {
        this.algaeSubsys = algaeSubsys;
        this.coralSubsys = coralSubsys;
        this.elevSubsys = elevSubsys;
        this.driveSubsys = driveSubsys;
        this.lasers = lasers;
    }

    /**
     * run end command (until in sensor, then runs slowly) until in sensor &&
     * outsensor, compatable with auton
     */
    public Command intakeCoral() {
        return coralSubsys.slowEndIntake(CORAL_INTAKE_SPEED)
                .until(() -> coralSubsys.outSensor() && coralSubsys.inSensor());
    }

    /**
     * Sequence of run end(intaking coral) until !out sensor, then setting the elev
     * down, compatable with auton
     */
    public Command scoreCoral() {
        return sequence(
            coralSubsys.runIntake(CORAL_SCORE_SPEED).until(() -> !coralSubsys.outSensor()),
            elevSubsys.setHeight(ELEV_DOWN)
        );
    }

    /** run end command until !out sensor, compatable with auton */
    public Command scoreCoralL1() {
        return coralSubsys.trough().until(() -> !coralSubsys.outSensor());
    }

    /** Command sequence, NOT compatable with auton */
    public Command scoreCoralL4() {
        return sequence(
            coralSubsys.runIntake(CORAL_SCORE_L4_SPEED),
            waitSeconds(0.45),
            elevSubsys.setHeight(ELEV_L4_BUMP),
            waitUntil(() -> elevSubsys.isAbove(52)),
            waitSeconds(0.2),
            coralSubsys.setIntake(0),
            elevSubsys.setHeight(ELEV_DOWN)
        );
    }

    /**Command sequence, NOT compatable with auton */
    public Command manualL4Bump(){
        return sequence(
            elevSubsys.setHeight(ELEV_L4_BUMP),
            race(
                waitUntil(()-> elevSubsys.isAbove(52)),
                waitSeconds(0.5)//TODO question if this race group is good
            ),
            waitSeconds(0.2),
            coralSubsys.setIntake(0),
            elevSubsys.setHeight(0)
        );
    }

    /**Command sequence, 1/2 to be the auton compatable version of scoreCoralL4()*/
    public Command autoScoreCoralL4(){
        return sequence(elevSubsys.setHeight(ELEV_L4),
                waitUntil(() -> elevSubsys.isNearGoal()),
                coralSubsys.setIntake(CORAL_SCORE_L4_SPEED), // TODO test- in comps, we used intake speed here
                race(
                        waitUntil(() -> !coralSubsys.outSensor()),
                        waitSeconds(1.2))
            );
    }

    /**Command sequence, 2/2 to be the auton compatable version of scoreCoralL4()*/
    public Command autoL4Bump() {
        return sequence(
                elevSubsys.setHeight(ELEV_L4_BUMP), 
                waitUntil(() -> (elevSubsys.isAbove(52.0))),
                waitSeconds(0.2),
                coralSubsys.setIntake(0));
    }

    /**Command sequence that automatically runs through the barge shot series, NOT compatable with auton */
    public Command bargeShot(){
        return sequence(
            bombsAway(),
            detonate(),
            regurgitateAlgae(),
            takeCover()
        );
    }

            /** command sequence, first(/4) of the barge shot series, compatable with auton */
            public Command bombsAway() {
                return sequence(
                        elevSubsys.setHeight(ELEV_BARGE),
                        waitUntil(() -> elevSubsys.isAbove(42.5)));
            }

            /**command sequence, second(/4) of the barge shot series, NOT compatable with auton */
            public Command detonate() {
                return sequence(
                    autoDetonate1(),
                    autoDetonate2(),
                    autoDetonate3(),
                    autoDetonate4()
                );
            }

                    /** command 1/4 to be the auton compatatble version of dentonate() */
                    public Command autoDetonate1(){
                        return algaeSubsys.setVoltage(-1.5).withTimeout(0.05);
                    }

                    /** command 2/4 to be the auton compatatble version of dentonate() */
                    public Command autoDetonate2(){
                        return algaeSubsys.setVoltage(5).until(() -> algaeSubsys.getPivotEncoder() > 50);
                    }

                    /** command 3/4 to be the auton compatatble version of dentonate() */
                    public Command autoDetonate3(){
                        return algaeSubsys.setIntake(0.3);
                    }

                    /** command 4/4 to be the auton compatatble version of dentonate() */
                    public Command autoDetonate4(){
                        return algaeSubsys.setAngle(ALGAE_PIVOT_UP);
                    }

            /**Command, 3/4 */
            public Command regurgitateAlgae(){
                return algaeSubsys.runIntake(ALGAE_SLOW_SPEED).withTimeout(0.6);
            }

            /**Command sequence, third(of 3) of the barge shot series, compatable with auton */
            public Command takeCover(){
                return sequence(
                    elevSubsys.setHeight(ELEV_DOWN),
                    algaeSubsys.setToTravel()
                );
            }

    public Command holdAlgaeBelow(){
        return sequence(
            algaeSubsys.stopIntake(),
            //algaeSubsys.setIntake(ALGAE_HOLD_SPEED),//TODO test and question if this would be good
            algaeSubsys.setAngle(ALGAE_PIVOT_HOLD_BELOW)
        );
    }

    public Command holdAlgaeAbove(){
        return sequence(
            algaeSubsys.stopIntake(),
            //algaeSubsys.setIntake(-ALGAE_HOLD_SPEED),//TODO test and question if this would be good
            algaeSubsys.setAngle(ALGAE_PIVOT_HOLD_ABOVE)
        );
    }

    /**!!ONLY FOR AUTON!! command sequence */
    public Command autoAlignRight(){
        return driveSubsys.applyRequest(()-> autoDrive.withSpeeds(
                new ChassisSpeeds(0.0, -0.75, 0.0)))
                .until(()-> !lasers.seesRightSensor());
    }

    /**!!ONLY FOR AUTON!! command sequence */
    public Command autoAlignLeft(){
        return driveSubsys.applyRequest(()-> autoDrive.withSpeeds(
                new ChassisSpeeds(0.0, 0.6, 0.0)))
                .until(()-> !lasers.seesLeftSensor());
    }

    /**!!ONLY FOR AUTON!! command sequence *NOTE- is not a super helpful command, avoid using when possible* */
    public Command autoDriveForward(){
        return driveSubsys.applyRequest(()-> autoDrive.withSpeeds(
                new ChassisSpeeds(0.0, 0.6, 0.0)))
                .until(()-> lasers.seesLeftSensor() | lasers.seesRightSensor());
    }

}
