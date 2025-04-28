package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.team10505.robot.subsystems.HardwareConstants.*;
public class ElevatorSubystem extends SubsystemBase {
    /*Variables */
    //NOTE- many of our varibles' values are assigned in the constructor
        //this is so that we can assign different values for the same variable based off if we are using a simulation or not
    //motors
    private final TalonFX elevatorMotor;
    private final TalonFX elevatorMotorFollower;

    //pid and feed forward controllers
    private final PIDController controller;
    private final ElevatorFeedforward ffeController;

    //encoder
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(ELEV_ENCODER_CHANNEL);
    private final double encoderOffset = 0;//TODO change irl

    //var for where the elevator height will try to get to. Until changed(w a method, typically bound to a button), it'll be 0
    private double setPoint = 0;
    private double currentPos;

    private double simStartingHeight = 0.1;

    /*simulation variables */
    private final Mechanism2d elevMech = new Mechanism2d(2, 4, new Color8Bit(Color.kMediumVioletRed));
    private final MechanismRoot2d elevRoot = elevMech.getRoot("elevRoot", 1, 0.2);
    private final MechanismLigament2d elevViz = elevRoot.append(new MechanismLigament2d("elevViz", simStartingHeight+2, 0, 50, new Color8Bit(Color.kPeru)));
    private final MechanismRoot2d elevLeftRoot = elevMech.getRoot("elevRoot", 0.75, 0.2);
    private final MechanismLigament2d elevLeftViz = elevRoot.append(new MechanismLigament2d("elevLeftViz", 2.1, 0, 10, new Color8Bit(Color.kSteelBlue)));
    private final MechanismRoot2d elevRightRoot = elevMech.getRoot("elevRoot", 1.25, 0.2);
    private final MechanismLigament2d elevRightViz = elevRoot.append(new MechanismLigament2d("elevRightViz", 2.1, 0, 10, new Color8Bit(Color.kSteelBlue)));

    private final ElevatorSim elevSim = new ElevatorSim(DCMotor.getKrakenX60(2), ELEV_GEARING, 10, 0.1, 0, 5, true, simStartingHeight);

    /*constructor */
    public ElevatorSubystem(){
        SmartDashboard.putData("Elevator Sim", elevMech);
        if(Utils.isSimulation()){
            elevatorMotor = new TalonFX(ELEV_MOTOR_ID);
            elevatorMotorFollower = new TalonFX(ELEV_MOTOR_FOLLOWER_ID);

            controller = new PIDController(1, 0, 0);
            ffeController = new ElevatorFeedforward(0, 1, 0);
        }else{
            elevatorMotor = new TalonFX(ELEV_MOTOR_ID, canbusName);
            elevatorMotorFollower = new TalonFX(ELEV_MOTOR_FOLLOWER_ID, canbusName);

            controller = new PIDController(1, 0, 0);//TODO change irl
            ffeController = new ElevatorFeedforward(0, 1, 0);//TODO change irl

        }
    }
    
    /*Commands to referense */    
    public Command setHeight(double newHeight){
        return runOnce(() -> {
            setPoint = newHeight;
        });
    }

    /*calulations */
    private double getEncoder(){
        if(Utils.isSimulation()){
            return elevViz.getLength();
        }else{
            return (encoder.get() * 360) - encoderOffset;
        }
    }
    
    //method to calculate what voltage to set the motors to
    private double getEffort(){
        return controller.calculate(currentPos, setPoint) + ffeController.calculate(0);
    }

    /*periodic method */
    @Override
    public void periodic(){
        elevatorMotor.setVoltage(getEffort());
        if(Utils.isSimulation()){
            elevSim.setInput(elevatorMotor.getMotorVoltage().getValueAsDouble());
            elevSim.update(0.01);
            elevRoot.setPosition(1, elevSim.getPositionMeters());
        }

        elevViz.setColor(new Color8Bit(new Color(Math.random(), Math.random(), Math.random())));

        SmartDashboard.putNumber("Elev setpoint", setPoint);
        SmartDashboard.putNumber("Elev encoder", getEncoder());
        SmartDashboard.putNumber("Elev Motor Output", elevatorMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Elev Motor Follower Output", elevatorMotorFollower.getMotorVoltage().getValueAsDouble());

    }
}
