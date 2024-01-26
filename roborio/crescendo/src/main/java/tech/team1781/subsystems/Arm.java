package tech.team1781.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import tech.team1781.ConfigMap;

import com.revrobotics.SparkLimitSwitch;

public class Arm extends Subsystem {
    private CANSparkMax mRightMotor;
    private CANSparkMax mLeftMotor;
    private RelativeEncoder mLeftEncoder;
    private ArmState mDesiredArmState;

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
        mLeftEncoder.setPositionConversionFactor(ConfigMap.ARM_GEAR_RATIO * 360); //will tell us angle in degrees
        mDesiredArmState = ArmState.START;
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
        mLeftMotor.set(armDutyCycle);
        System.out.printf("arm duty cycle: %.2f\n", armDutyCycle);
    }

    @Override
    public void teleopPeriodic() {

        System.out.printf("arm left encoder %.3f\n", getAngle());
        //mLeftMotor.set(0);  //temp
    }

    public double getAngle() {
        return mLeftEncoder.getPosition();
    }
}
