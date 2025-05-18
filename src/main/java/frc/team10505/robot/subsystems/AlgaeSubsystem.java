package frc.team10505.robot.subsystems;

import static frc.team10505.robot.subsystems.HardwareConstants.*;

import com.ctre.phoenix6.Utils;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.team10505.robot.subsystems.HardwareConstants.*;

public class AlgaeSubsystem extends SubsystemBase {
    /*Variables */
    private final SparkMax pivotMotor = new SparkMax(PIVOT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax intakeMotor = new SparkMax(PIVOT_ALGAE_INTAKE_MOTOR_ID, MotorType.kBrushless);

    private final PIDController controller;
    private final ArmFeedforward ffeController;

    private double setPoint;
    private double speed;

    /*Simulation variables */
    private double pivotStartingAngle = 50;

    private final Mechanism2d mech = new Mechanism2d(2, 2);
    private final MechanismRoot2d pivotRoot = mech.getRoot("pivot Root", 0.3, 0.5);
    private final MechanismLigament2d pivotViz = pivotRoot.append(new MechanismLigament2d("Pivot Ligament", 0.4, pivotStartingAngle, 20, new Color8Bit(Color.kOrange)));
    private final MechanismRoot2d intakeRoot = mech.getRoot("intake Root", Math.cos(Units.degreesToRadians(pivotStartingAngle)) * 0.4, Math.sin(Units.degreesToRadians(pivotStartingAngle)) * 0.4);
    private final MechanismLigament2d intakeViz = intakeRoot.append(new MechanismLigament2d("algae Intake Ligament", 0.1, pivotStartingAngle, 10, new Color8Bit(Color.kBisque)));

    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), PIVOT_GEARING, SingleJointedArmSim.estimateMOI(0.3, 4), 0.3, Units.degreesToRadians(-110), Units.degreesToRadians(110), true, Units.degreesToRadians(pivotStartingAngle));
    private final FlywheelSim intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.01, 5), DCMotor.getNEO(1));

    /*Constructor */
    public AlgaeSubsystem(){
        if(Utils.isSimulation()){
            controller = new PIDController(0.3, 0, 0);
            ffeController = new ArmFeedforward(0, 27.2, 0.1, 0.1);
        } else {
            controller = new PIDController(0, 0, 0);
            ffeController = new ArmFeedforward(0, 1, 0.1, 0.1);
        }

        SmartDashboard.putData("Pivot Subsys sim", mech);
    }

    /*Commands to reference */
    public Command setAngle(double newAngle){
        return runOnce(() -> {
            setPoint = newAngle;
        });
    }

    /**run end command that stops once an end condition is met */
    public Command runIntake(double newSpeed){
        return runEnd(() ->{
            intakeMotor.set(newSpeed);
            speed = newSpeed;
        }, () ->{
            intakeMotor.set(0);
            speed = 0;
        });
    }

    /**runOnce command. Does not wait for an end condition and does not stop itself */
    public Command setIntakeSpeed(double newSpeed){
        return runOnce(() -> {
            intakeMotor.set(newSpeed);
            speed = newSpeed;
        });
    }

    /*calculations */
    private double getEncoder(){
        if(Utils.isSimulation()){
            return pivotViz.getAngle();
        } else{
            return 0;//TODO change irl
        }
    }

    private double getEffort(){
        return controller.calculate(getEncoder(), setPoint) + ffeController.calculate(Units.degreesToRadians(getEncoder()), 0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("pivot encoder", getEncoder());
        SmartDashboard.putNumber("Pivot effort", getEffort());
        SmartDashboard.putNumber("Pivot_intake speed", speed);
        SmartDashboard.putNumber("Pivot angle", setPoint);
        pivotMotor.setVoltage(getEffort());
        if(Utils.isSimulation()){
            pivotSim.setInput(getEffort());
            pivotSim.update(0.001);
            pivotViz.setAngle(Units.radiansToDegrees(pivotSim.getAngleRads()));

            intakeSim.setInput(speed);
            intakeSim.update(0.005);
            intakeViz.setAngle(intakeViz.getAngle() + (speed * 0.05));

            intakeRoot.setPosition(Math.cos(Units.degreesToRadians(pivotViz.getAngle())) * 0.4 + 0.3, Math.sin(Units.degreesToRadians(pivotViz.getAngle()) * 0.4 + 0.5));
        }
    }
}
