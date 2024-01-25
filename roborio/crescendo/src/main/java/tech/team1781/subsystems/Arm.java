package tech.team1781.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import tech.team1781.ConfigMap;

import com.revrobotics.SparkLimitSwitch;

public class Arm extends Subsystem {

    // Right Motor
    private CANSparkMax mLeaderMotor = new CANSparkMax(ConfigMap.ARM_PIVOT_RIGHT_MOTOR,
            CANSparkMax.MotorType.kBrushless);
    // left Motor
    private CANSparkMax mFollowerMotor = new CANSparkMax(ConfigMap.ARM_PIVOT_LEFT_MOTOR,
            CANSparkMax.MotorType.kBrushless);

    private ArmState mDesiredArmState = ArmState.IDLE;

    public Arm() {
        super("Arm", ArmState.IDLE);
    }

    public enum ArmState implements Subsystem.SubsystemState {
        // Will probably add set angles
        IDLE, EXTEND, RETRACT
    }

    @Override
    public void genericPeriodic() {
        mFollowerMotor.follow(mLeaderMotor, true);

    }

    @Override
    public void init() {
        mFollowerMotor.follow(mLeaderMotor, true);

    }

    @Override
    public void getToState() {
        switch ((ArmState) getState()) {
            case IDLE:
            break;

            case EXTEND:
            break;

            case RETRACT:
            break;
        }
    }

    @Override
    public boolean matchesDesiredState() {
        switch ((ArmState) getState()) {
            case IDLE:
                return mLeaderMotor.get() == 0;
            case EXTEND:
                return mLeaderMotor.get() == .5;
            case RETRACT:
                return mLeaderMotor.get() == -.5;
        }
        return false;
    }

    @Override
    public void autonomousPeriodic() {
        mFollowerMotor.follow(mLeaderMotor, true);
    }

    @Override
    public void teleopPeriodic() {
        mFollowerMotor.follow(mLeaderMotor, true);

    }

    public double angle(double r) {
        double angle = ConfigMap.ARM_ANGLE_CONVERSION * r * 360;
        return angle;
    }

    public double getAngleRot(double angle) {
        double r = angle/(ConfigMap.ARM_ANGLE_CONVERSION*360);
        return r;
    }

}
