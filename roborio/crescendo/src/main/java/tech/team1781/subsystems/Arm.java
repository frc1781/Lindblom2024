package tech.team1781.subsystems;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import tech.team1781.ConfigMap;

import com.revrobotics.SparkLimitSwitch;

public class Arm extends Subsystem {
    private CANSparkMax mRightMotor;
    private CANSparkMax mLeftMotor;
    private RelativeEncoder mLeftEncoder;
    private ArmState mDesiredArmState;
    private ProfiledPIDController mPositionPID = new ProfiledPIDController(0.1, 0, 0, new TrapezoidProfile.Constraints(200, 50));
    private HashMap<ArmState, Double> mPositions = new HashMap<>();

    private double mDesiredPosition = 0;
    private boolean mIsManual = false;

    public Arm() {
        super("Arm", ArmState.START);
        mRightMotor = new CANSparkMax(
            ConfigMap.ARM_PIVOT_RIGHT_MOTOR,
            CANSparkMax.MotorType.kBrushless
        );

        mLeftMotor = new CANSparkMax(
            ConfigMap.ARM_PIVOT_LEFT_MOTOR,
            CANSparkMax.MotorType.kBrushless
        );

        mLeftEncoder = mLeftMotor.getEncoder();
        mLeftEncoder.setPositionConversionFactor(ConfigMap.ARM_GEAR_RATIO * 360.0 * 1.2); //will tell us angle in degrees
        mDesiredArmState = ArmState.START;

        mPositions.put(ArmState.START, 71.9);
        mPositions.put(ArmState.SAFE, 63.0);
        mPositions.put(ArmState.PODIUM, 43.8);
        mPositions.put(ArmState.SUBWOOFER, 25.0);
        mPositions.put(ArmState.COLLECT, 0.0);

    }

    public enum ArmState implements Subsystem.SubsystemState {
        // Will probably add set angles
        START,
        SAFE,
        PODIUM,
        SUBWOOFER,
        COLLECT
    }

    @Override
    public void genericPeriodic() {
    }

    @Override
    public void init() {
        if (currentMode == OperatingMode.DISABLED) {
            mRightMotor.follow(mLeftMotor, true);
            mLeftMotor.setIdleMode(IdleMode.kBrake);
            mRightMotor.setIdleMode(IdleMode.kBrake);
            System.out.println("initialized arm moters for following...");
            System.out.println("ENSURE ARM IN ZERO POSITION!!!!! Just set encoder to zero");
            mLeftEncoder.setPosition(0);
        }
    }

    @Override
    public void getToState() {
        var desiredPosition = mDesiredPosition; // mPositions.get(mDesiredPosition);
        var armDutyCycle = mPositionPID.calculate(mLeftEncoder.getPosition(), desiredPosition);

        if(mIsManual) {
            mLeftMotor.set(armDutyCycle);
            System.out.println("manual");
        } else {
            System.out.println("not manual");
            mDesiredPosition = mPositions.get(getState());
        }

        switch ((ArmState) getState()) {
            case START:
            break;

            case SAFE:
            break;

            case PODIUM:
            break;

            case SUBWOOFER:
            break;

            case COLLECT:
            break;
        }
    }

    @Override
    public boolean matchesDesiredState() {
        switch ((ArmState) getState()) {
            case START:
            break;

            case SAFE:
            break;

            case PODIUM:
            break;

            case SUBWOOFER:
            break;

            case COLLECT:
            break;
        }
        return false;
    }

    @Override
    public void autonomousPeriodic() {

    }

    public void driveManual(double armDutyCycle) {
        armDutyCycle *= 0.5;

        if(Math.abs(armDutyCycle) >=  0.1) {
            mIsManual = true;
            mLeftMotor.set(armDutyCycle);
        }
        mLeftMotor.set(armDutyCycle);
    }

    @Override
    public void teleopPeriodic() {

        // System.out.printf("arm left encoder %.3f\n", getAngle());
        //mLeftMotor.set(0);  //temp
    }

    @Override
    public void setDesiredState(SubsystemState state) {
        super.setDesiredState(state);

        mIsManual = false;
    }

    public double getAngle() {
        return mLeftEncoder.getPosition();
    }
}
