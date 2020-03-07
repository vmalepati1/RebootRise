package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Index extends SubsystemBase {
    // ramp release logic
    private boolean release = false;
    private double releaseStart = 0;

    // loader logic
    private enum LoadState {
        DOWN, LOAD, RETRACTING
    }
    private boolean loaderClear = true;
    private LoadState loadState = LoadState.DOWN;
    private Timer loadUp = new Timer();
    private double holdTime = 0.750;

    // ** Components **
    // sensors
    private final DigitalInput ballSensor, loaderSensor, intakeSensor, shootSensor;

    // motors
    private final CANSparkMax indexMotor;
    private final int stallLimit = 40, freeLimit = 20;

    // solenoids
    private final Solenoid rampRelease;
    private final Solenoid loader;

    public Index() {
        // sensors
        ballSensor = new DigitalInput(Constants.BALL_SENSOR); // ball at hopper
        loaderSensor = new DigitalInput(Constants.LOADER_SENSOR); // loader retract sensor
        intakeSensor = new DigitalInput(Constants.INTAKE_SENSOR);
        shootSensor = new DigitalInput(Constants.SHOOT_SENSOR);

        // index motor
        indexMotor = new CANSparkMax(Constants.INDEXER_CAN, MotorType.kBrushed);
        indexMotor.set(0);
        indexMotor.setInverted(false);
        indexMotor.setIdleMode(IdleMode.kBrake);
        indexMotor.setSmartCurrentLimit(stallLimit, freeLimit);

        // reamp release cylinder
        rampRelease = new Solenoid(Constants.RAMP_RELEASE);

        // shooter loader cylinder
        loader = new Solenoid(Constants.LOADER);
        loader.set(false);
        rampRelease.set(false);
        loadUp.start();
    }
    
    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        // ramp release timing
        if (release) {
            if (Timer.getFPGATimestamp() - releaseStart > 3.0) {
                rampRelease.set(false);
                release = false;
            }
        }
        // loading cycle
        switch (loadState) {
            case DOWN:
                loaderClear = true;
                break;
            case RETRACTING:
                if (loaderSensor.get()) {
                    loadState = LoadState.DOWN;
                }
                break;
            case LOAD:
                if (loadUp.get() > holdTime) {
                    retractLoader();
                }
                break;
            default:
                break;
        }
    }

    public void retractLoader() {
        loader.set(false);
        loadState = LoadState.RETRACTING;
    }

    public void load(double holdTime) {
        loaderClear = false;
        loader.set(true);
        loadUp.reset();
        loadState = LoadState.LOAD;
        this.holdTime = holdTime;
    }

    public void load() {
        load(holdTime);
    }
    
    public void releaseRamp() {
        rampRelease.set(true);
        release = true;
        releaseStart = Timer.getFPGATimestamp();
    }

    public boolean atHopper() {
        return !intakeSensor.get();
    }

    public boolean atShoot() {
        return !shootSensor.get();
    }

    /**
     * weather or not the indexer actualy started to spin
     * @return boolean
     */
    public boolean spinCW() {
        if (loaderClear) {
            this.indexMotor.set(1);
        }
        return loaderClear;
    }

    /**
     * can the indexer rotate
     * @return boolean
     */
    public boolean canRotate() {
        return loaderClear;
    }

    public void stop() {
        this.indexMotor.set(0);
    }

    public boolean ballPresent() {
        // light on (true) when no ball so use !
        // only when in load position
        return !ballSensor.get() && atHopper() && (indexMotor.get() == 0);
    }

}