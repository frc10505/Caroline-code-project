package frc.team10505.robot.subsystems.drive;

import static frc.team10505.robot.Constants.HardwareConstants.*;

import com.ctre.phoenix6.Utils;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class Lasers extends SubsystemBase {
    private LaserCan rightLaser;
    private LaserCan leftLaser;

    private Spark leds;

    public enum LEDState {
        BOTH,
        NEITHER,
        RIGHT,
        LEFT
    }

    public LEDState currentLedState = LEDState.NEITHER;

    private CommandJoystick joystick;

    /**NOT for use in simulation */
    public Lasers() {
        rightLaser = new LaserCan(DRIVE_RIGHT_LASER_ID);
        leftLaser = new LaserCan(DRIVE_LEFT_LASER_ID);
        leds = new Spark(DRIVE_LEDS_CHANNEL);
    }

    /**
     * Constructor - NOT FOR USE IN REAL LIFE!!
     * ONLY FOR SIMULATIONS
     */
    public Lasers(CommandJoystick joystick) {
        if (Utils.isSimulation()) {
            this.joystick = joystick;
        } else {
            rightLaser = new LaserCan(DRIVE_RIGHT_LASER_ID);
            leftLaser = new LaserCan(DRIVE_LEFT_LASER_ID);
            leds = new Spark(DRIVE_LEDS_CHANNEL);
        }
    }

    public boolean seesLeftSensor() {
        if (Utils.isSimulation() || Utils.isReplay()) {
            return joystick.button(3).getAsBoolean();
        } else {
            try {
                LaserCan.Measurement leftMeas = leftLaser.getMeasurement();
                return (leftMeas.distance_mm < 290
                        && leftMeas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);
            } catch (NullPointerException l) {
                SmartDashboard.putString("Errors", "drivetrain left laser meas failed");
                return false;
            }
        }
    }

    public boolean seesRightSensor() {
        if (Utils.isSimulation() || Utils.isReplay()) {
            return joystick.button(4).getAsBoolean();

        } else {
            try {
                LaserCan.Measurement RightMeas = rightLaser.getMeasurement();
                return (RightMeas.distance_mm < 290 && RightMeas.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT);

            } catch (NullPointerException r) {
                SmartDashboard.putString("Errors", "drivetrain right laser meas failed");
                return false;
            }
        }
    }

    public void updateLedState() {
        if (seesRightSensor() && seesLeftSensor()) {
            currentLedState = LEDState.BOTH;
        } else if (seesRightSensor() && !seesLeftSensor()) {
            currentLedState = LEDState.RIGHT;
        } else if (!seesRightSensor() && seesLeftSensor()) {
            currentLedState = LEDState.LEFT;
        } else {
            currentLedState = LEDState.NEITHER;
        }
    }

    @Override
    public void periodic() {
        updateLedState();
        SmartDashboard.putBoolean("Drive Right Laser", seesRightSensor());
        SmartDashboard.putBoolean("Drive Left Laser", seesLeftSensor());
        SmartDashboard.putString("Drive Current Led State", currentLedState.toString());

        if (!Utils.isSimulation()) {
            if (currentLedState == LEDState.BOTH) {
                leds.set(0.35);// flashy color 2 (Blue)
            } else if (currentLedState == LEDState.RIGHT || currentLedState == LEDState.LEFT) {
                leds.set(0.77);// solid greed
            } else {
                leds.set(0.61);// solid red
            }
        }

    }
}
