package tech.team1781.subsystems;
import java.util.HashMap;
import java.util.concurrent.TimeUnit;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import tech.team1781.ConfigMap;
import com.revrobotics.SparkLimitSwitch;

public class Arm extends Subsystem {
    private CANSparkMax mRightMotor;
    private CANSparkMax mLeftMotor;
    private RelativeEncoder mLeftEncoder;
    private ProfiledPIDController mPositionPID = new ProfiledPIDController(0.05, 0, 0,
            new TrapezoidProfile.Constraints(80, 450));
    private HashMap<ArmState, Double> mPositions = new HashMap<>();
    private GenericEntry mArmPositionEntry = ConfigMap.SHUFFLEBOARD_TAB.add("Arm Position", -1).getEntry();
    private GenericEntry mSpeakerDistanceEntry = ConfigMap.SHUFFLEBOARD_TAB.add("Distance", 1).getEntry();
    private double mDesiredPosition = 0;
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
        mLeftEncoder.setVelocityConversionFactor((ConfigMap.ARM_GEAR_RATIO * 360.0 * 1.2)/60);
        mLeftEncoder.setPositionConversionFactor(ConfigMap.ARM_GEAR_RATIO * 360.0 * 1.2); // will tell us angle in degrees
        mRightMotor.follow(mLeftMotor, true);
        mLeftMotor.setIdleMode(IdleMode.kBrake);
        mRightMotor.setIdleMode(IdleMode.kBrake);
        System.out.println("initialized arm moters for following...");
        System.out.println("ENSURE ARM IN ZERO POSITION!!!!! Just set encoder to zero");
        mLeftEncoder.setPosition(0);
        mPositions.put(ArmState.START, 0.0); // Temporary, used to be 71.9
        mPositions.put(ArmState.SAFE, 63.0);
        mPositions.put(ArmState.PODIUM, 43.8);
        mPositions.put(ArmState.SUBWOOFER, 40.0);
        mPositions.put(ArmState.COLLECT, 0.0);
    }

    public enum ArmState implements Subsystem.SubsystemState {
        START,
        SAFE,
        PODIUM,
        SUBWOOFER,
        COLLECT,
        MANUAL,
        AUTO_ANGLE
    }

    @Override
    public void genericPeriodic() {
        // System.out.println(getAngle());
    }

    @Override
    public void init() {
        mAngleFromDistance = 0;
        mLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        mLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        mLeftMotor.setSmartCurrentLimit(30);
        mLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 90);
        mLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        mLeftMotor.burnFlash();
        mDesiredPosition = mLeftEncoder.getPosition();
        setDesiredState(ArmState.START);
        if (currentMode == OperatingMode.DISABLED) {
        }
    }

    @Override
    public void getToState() {
        var desiredPosition = mDesiredPosition; // mPositions.get(mDesiredPosition);
        var armDutyCycle = mPositionPID.calculate(mLeftEncoder.getPosition(), desiredPosition);
        // System.out.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
        //   (currentTime-startTime)/1000000000.0,
        //   mDesiredPosition,
        //   armDutyCycle,
        //   getAngle(),
        //   mLeftEncoder.getVelocity(),
        //   mLeftMotor.getAppliedOutput()
        // );
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

    @Override
    public void teleopPeriodic() {
        // System.out.printf("arm left encoder %.3f\n", getAngle());
        // mLeftMotor.set(0); //temp
    }

    @Override
    public void setDesiredState(SubsystemState state) {
        super.setDesiredState(state);

        if (state != ArmState.MANUAL && state != ArmState.AUTO_ANGLE) {
            mDesiredPosition = mPositions.get(state);
        } else if (state == ArmState.AUTO_ANGLE) {

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

    public void manualControlAngle(double d) {
        setDesiredState(ArmState.MANUAL);
        mDesiredPosition += d;
    }

    private boolean matchesPosition() {
        var diff = mDesiredPosition - mLeftEncoder.getPosition();
        return Math.abs(diff) <= 1;
    }
}
