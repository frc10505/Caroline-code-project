package frc.team10505.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import static frc.team10505.robot.subsystems.HardwareConstants.*;

public class IntakeSubsystem extends SubsystemBase {
    /*Variables */
    private final double laserDistance = 50.0;//in millimeters. is used to compare to what a laser can currently reads
    private CommandJoystick joystick;

    //motors
    private final SparkMax leftMotor = new SparkMax(INTAKE_LEFT_MOTOR_ID, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(INTAKE_RIGHT_MOTOR_ID, MotorType.kBrushless);

    private final SparkMaxConfig leftMotorConfig = new SparkMaxConfig();
    private final SparkMaxConfig rightMotorConfig = new SparkMaxConfig();

    private SparkSim leftMotorSim;
    private SparkSim rightMotorSim;

    //lasers
    private LaserCan firstLaser;
    private LaserCan secondLaser;

    private double speed;//used only in sim and logging

    /*Simulation variables */
    private final Color8Bit red = new Color8Bit(Color.kFirstRed);
    private final Color8Bit green = new Color8Bit(Color.kGreen);
    private final Color8Bit grey = new Color8Bit(Color.kSlateGray);
    private final Color8Bit whiteish = new Color8Bit(Color.kAntiqueWhite);

    private final Mechanism2d mech = new Mechanism2d(2, 2);

    private final MechanismRoot2d leftFirstRoot = mech.getRoot("left first root", 0.5, 1.5);
    private final MechanismRoot2d leftSecondRoot = mech.getRoot("left second root", 0.5, 0.5);
    private final MechanismRoot2d rightFirstRoot = mech.getRoot("right first root", 1.5, 1.5);
    private final MechanismRoot2d rightSecondRoot = mech.getRoot("right second root", 1.5, 0.5);

    private final MechanismRoot2d firstLaserRoot = mech.getRoot("first laser root", 1, 1.7);
    private final MechanismRoot2d secondLaserRoot = mech.getRoot("second laser root", 1, 0.3);
    private final MechanismRoot2d firstDorkyCoralRoot = mech.getRoot("first dorky coral root", 0.75, 1.7);
    private final MechanismRoot2d secondDorkyCoralRoot = mech.getRoot("second dorky coral root", 0.75, 0.3);


    private final MechanismLigament2d leftFirstViz = leftFirstRoot.append(new MechanismLigament2d("left First lig", 0.45, 180, 10, grey));
    private final MechanismLigament2d leftSecondViz = leftSecondRoot.append(new MechanismLigament2d("left Second lig", 0.4, 180, 10, grey));
    private final MechanismLigament2d rightFirstViz = rightFirstRoot.append(new MechanismLigament2d("right First lig", 0.45, 0, 10, grey));
    private final MechanismLigament2d rightSecondViz = rightSecondRoot.append(new MechanismLigament2d("right Second lig", 0.4, 0, 10, grey));
   
    private final MechanismLigament2d firstLaserViz = firstLaserRoot.append(new MechanismLigament2d("first laser lig", 0.3, 90, 30, red));
    private final MechanismLigament2d secondLaserViz = secondLaserRoot.append(new MechanismLigament2d("second laser lig", 0.3, 90, 30, red));
    private final MechanismLigament2d firstDorkyCoralViz = firstDorkyCoralRoot.append(new MechanismLigament2d("first dorky coral lig", 0.5, 0, 0, whiteish));
    private final MechanismLigament2d secondDorkyCoralViz = secondDorkyCoralRoot.append(new MechanismLigament2d("second dorky coral lig", 0.5, 0, 0, whiteish));
    
    private final FlywheelSim leftSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 0.0005, INTAKE_GEARING), DCMotor.getNeo550(1));
    private final FlywheelSim rightSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeo550(1), 0.0005, INTAKE_GEARING), DCMotor.getNeo550(1));

    /*Constructors */

    /**Intended for use in real life. 
     * Gives a value to our laserCans
     */
    public IntakeSubsystem(){
        firstLaser = new LaserCan(INTAKE_FIRST_LASER_ID);
        secondLaser = new LaserCan(INTAKE_SECOND_LASER_ID);
    }

    /**Intended for use in simulations.
     * Gives a value to a joystick
     */
    public IntakeSubsystem(CommandJoystick joystick){
        this.joystick = joystick;
        leftMotorSim = new SparkSim(leftMotor, DCMotor.getNeo550(1));
        rightMotorSim = new SparkSim(rightMotor, DCMotor.getNeo550(1));
        SmartDashboard.putData("Intake Mech", mech);
    }


    /*Calculations */
    //booleans that return if a sensor is triggered(aka reads a distance less than whatever we tell it)
    public boolean seesFirstSensor(){
        if(Utils.isSimulation()){
            return joystick.button(1).getAsBoolean();
        } else{
            try{
                LaserCan.Measurement meas = firstLaser.getMeasurement();
                return (meas.distance_mm < laserDistance && meas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
            } catch (Exception e){
                print("FIRST LASER FAILED");
                return false;
            }
        }
    }

    public boolean seesSecondSensor(){
        if(Utils.isSimulation()){
            return joystick.button(2).getAsBoolean();
        } else{
            try{
                LaserCan.Measurement meas = secondLaser.getMeasurement();
                return (meas.distance_mm < laserDistance && meas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);            
            } catch (Exception e){
                print("SECOND LASER FAILED");
                return false;
            }
        }
    }

    public Command setIntake(double newSpeed){
        return runOnce(() ->{
            leftMotor.set(newSpeed);
            rightMotor.set(newSpeed);
            speed = newSpeed;
        });
    }

    /**This is a runEnd command. The end condition is setting the motor speeds to zero */
    public Command runIntake(double newSpeed){
        return runEnd(() ->{
            leftMotor.set(newSpeed);
            rightMotor.set(newSpeed);
            speed = newSpeed;
        }, () ->{
            leftMotor.set(0);
            rightMotor.set(0);
            speed = 0;
        });
    }

    @Override
    public void periodic(){
        if(Utils.isSimulation()){
            if(seesFirstSensor()){
                firstLaserViz.setColor(green);
                firstDorkyCoralViz.setLineWeight(125);
            } else{
                firstLaserViz.setColor(red);
                firstDorkyCoralViz.setLineWeight(0);
            }

            if(seesSecondSensor()){
                secondLaserViz.setColor(green);
                secondDorkyCoralViz.setLineWeight(125);
            } else{
                secondLaserViz.setColor(red);
                secondDorkyCoralViz.setLineWeight(0);
            }

            leftSim.setInput(-speed*12);
            leftSim.update(0.001);
            rightSim.setInput(speed*12);
            rightSim.update(0.001);

            leftFirstViz.setAngle(leftFirstViz.getAngle() + (leftSim.getAngularVelocityRPM() * 0.025));
            leftSecondViz.setAngle(leftFirstViz.getAngle());
            rightFirstViz.setAngle(rightFirstViz.getAngle() + (rightSim.getAngularVelocityRPM() * 0.025));
            rightSecondViz.setAngle(rightFirstViz.getAngle());
    

            //TODO mess with
           // leftMotorSim.iterate(ELEV_MOTOR_FOLLOWER_ID, ELEV_GEARING, ELEV_ENCODER_CHANNEL);
            SmartDashboard.putNumber("Intake/left motor/bus voltage", leftMotor.getBusVoltage());
            SmartDashboard.putNumber("Intake/left motor/applied output", leftMotor.getAppliedOutput());
            SmartDashboard.putNumber("Intake/left motor sim/bus voltage", leftMotorSim.getBusVoltage());
            SmartDashboard.putNumber("Intake/left motor sim/applied output", leftMotorSim.getAppliedOutput());
    
        }
       

    }
}