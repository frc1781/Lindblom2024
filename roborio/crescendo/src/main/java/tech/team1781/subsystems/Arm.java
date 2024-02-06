package tech.team1781.subsystems;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import tech.team1781.ConfigMap;

import com.revrobotics.SparkLimitSwitch;

public class Arm extends Subsystem {
    private CANSparkMax mRightMotor;
    private CANSparkMax mLeftMotor;
    private RelativeEncoder mLeftEncoder;
    private ArmState mDesiredArmState;
    private ProfiledPIDController mPositionPID = new ProfiledPIDController(0.1, 0, 0,
            new TrapezoidProfile.Constraints(200, 100));
    private HashMap<ArmState, Double> mPositions = new HashMap<>();

    private GenericEntry mArmPositionEntry = ConfigMap.SHUFFLEBOARD_TAB.add("Arm Position", -1).getEntry();
    private GenericEntry mSpeakerDistanceEntry = ConfigMap.SHUFFLEBOARD_TAB.add("Distance", 1).getEntry();


    private double mDesiredPosition = 0;
    private boolean mHasAutoAngled = false;
    private double mAngleFromDistance = 0;

    public Arm() {
        super("Arm", ArmState.START);
        mRightMotor = new CANSparkMax(
                ConfigMap.ARM_PIVOT_RIGHT_MOTOR,
                CANSparkMax.MotorType.kBrushless);

        mLeftMotor = new CANSparkMax(
                ConfigMap.ARM_PIVOT_LEFT_MOTOR,
                CANSparkMax.MotorType.kBrushless);

        mLeftEncoder = mLeftMotor.getEncoder();
        mLeftEncoder.setPositionConversionFactor(ConfigMap.ARM_GEAR_RATIO * 360.0 * 1.2); // will tell us angle in
                                                                                          // degrees
        mDesiredArmState = ArmState.START;

        mRightMotor.follow(mLeftMotor, true);
        mLeftMotor.setIdleMode(IdleMode.kBrake);
        mRightMotor.setIdleMode(IdleMode.kBrake);
        System.out.println("initialized arm moters for following...");
        System.out.println("ENSURE ARM IN ZERO POSITION!!!!! Just set encoder to zero");
        mLeftEncoder.setPosition(0);

        mPositions.put(ArmState.START, 71.9);
        mPositions.put(ArmState.SAFE, 63.0);
        mPositions.put(ArmState.PODIUM, 43.8);
        mPositions.put(ArmState.SUBWOOFER, 40.0);
        mPositions.put(ArmState.COLLECT, 0.0);

    }

    public enum ArmState implements Subsystem.SubsystemState {
        // Will probably add set angles
        START,
        SAFE,
        PODIUM,
        SUBWOOFER,
        COLLECT,
        AUTO_ANGLE,
        MANUAL_ADJUST
    }

    @Override
    public void genericPeriodic() {
        // System.out.println(getAngle());
    }

    @Override
    public void init() {
        mAngleFromDistance = 0;
        if (currentMode == OperatingMode.DISABLED) {
        }
    }

    @Override
    public void getToState() {
        var desiredPosition = mDesiredPosition; 
        var armDutyCycle = mPositionPID.calculate(mLeftEncoder.getPosition(), desiredPosition);
        System.out.println("arm dc: " + armDutyCycle);
        mArmPositionEntry.setDouble(mLeftEncoder.getPosition());
        mLeftMotor.set(armDutyCycle);

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
            case AUTO_ANGLE:
                calculateAngleFromDistance(mSpeakerDistanceEntry.getDouble(-1));
                mDesiredPosition = mAngleFromDistance;
                break;
            
            default:
                break;
        }
    }

    @Override
    public boolean matchesDesiredState() {
        switch ((ArmState) getState()) {
            case START:
                return true;
            default:
                return matchesPosition();

        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    // public void driveManual(double armDutyCycle) {
    //     armDutyCycle *= 0.5;

    //     if (Math.abs(armDutyCycle) >= 0.1) {
    //         mIsManual = true;
    //         setDesiredState(ArmState.MANUAL);
    //         mLeftMotor.set(armDutyCycle);
    //         mDesiredPosition = mLeftEncoder.getPosition();
    //     } else {
    //         mIsManual = false;
    //     }
    // }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void setDesiredState(SubsystemState state) {
        super.setDesiredState(state);

        if (state != ArmState.AUTO_ANGLE) {
            mDesiredPosition = mPositions.get(state);
        } else if (state == ArmState.AUTO_ANGLE) {
            //will add code to set angle by distance to april tag
        }
    }

    public double getAngle() {
        return mLeftEncoder.getPosition();
    }

    public void calculateAngleFromDistance(double dist) {
        final double start = 26.2;
        final double coefficient = 16.8;

        dist = Math.log(dist);
        mAngleFromDistance = start + (dist * coefficient);
    }

    private boolean matchesPosition() {
        var diff = mDesiredPosition - mLeftEncoder.getPosition();
        return Math.abs(diff) <= 1;
    }
}
