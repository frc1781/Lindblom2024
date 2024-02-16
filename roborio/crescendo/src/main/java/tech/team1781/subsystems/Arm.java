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
import tech.team1781.utils.LimelightHelper;

import com.revrobotics.SparkLimitSwitch;

public class Arm extends Subsystem {
    private CANSparkMax mRightMotor;
    private CANSparkMax mLeftMotor;
    private RelativeEncoder mLeftEncoder;
    private ProfiledPIDController mPositionPID = new ProfiledPIDController(0.05, 0, 0,
            new TrapezoidProfile.Constraints(80, 450));
    private HashMap<ArmState, Double> mPositions = new HashMap<>();
    private GenericEntry mArmPositionEntry = ConfigMap.SHUFFLEBOARD_TAB.add("Arm Position", -1).getEntry();
    //private GenericEntry mSpeakerDistanceEntry = ConfigMap.SHUFFLEBOARD_TAB.add("Distance", 1).getEntry();
    private double mDesiredPosition = 0;
    private double mSpeakerDistance = 0;

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
        mLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        mLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        mLeftMotor.setSmartCurrentLimit(30);
        mLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 90);
        mLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        mLeftMotor.burnFlash();
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

        mDesiredPosition = mLeftEncoder.getPosition();
        setDesiredState(ArmState.START);
        if (currentMode == OperatingMode.DISABLED) {
        }
    }

    @Override
    public void getToState() {
        if (getState() == ArmState.AUTO_ANGLE) {
            mDesiredPosition = calculateAngleFromDistance();
        }
        var armDutyCycle = mPositionPID.calculate(mLeftEncoder.getPosition(), mDesiredPosition);
        mArmPositionEntry.setDouble(mLeftEncoder.getPosition());
        mLeftMotor.set(armDutyCycle);
        // System.out.printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
        //   (currentTime-startTime)/1000000000.0,
        //   mDesiredPosition,
        //   armDutyCycle,
        //   getAngle(),
        //   mLeftEncoder.getVelocity(),
        //   mLeftMotor.getAppliedOutput()
        // );
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
        } else if (state == ArmState.MANUAL) {
            // desired position has already been adjusted and should remain the same
        } else if (state == ArmState.AUTO_ANGLE) {
            mDesiredPosition = calculateAngleFromDistance();
        }
    }

    public double getAngle() {
        return mLeftEncoder.getPosition();
    }

    private double calculateAngleFromDistance() {
        // final double start = 32;
        // final double coefficient = 18.3;
        // // double dist = LimelightHelper.getDistanceOfApriltag(4);
        // double dist = mSpeakerDistance - ConfigMap.DRIVETRAIN_TRACKWIDTH/2;
        // double angle = 32.0;
        // if (dist < 0.5) {//can not see april tag
        //     angle = 32.0;
        // } else {
        //     angle = Math.log(dist) * coefficient + start;
        // }
        
        // System.out.printf("dist %.2f, angle %.2f\n", dist, angle); 
        // if (angle > 51) {
        //     angle = 51;
        // }
        // return angle;
        return 32.0;
    }

    public void manualAdjustAngle(double d) {
        setDesiredState(ArmState.MANUAL);

        mDesiredPosition += d;
        if(mDesiredPosition > ConfigMap.MAX_THRESHOLD_ARM) {
            mDesiredPosition = ConfigMap.MAX_THRESHOLD_ARM;
        }

        if(mDesiredPosition < ConfigMap.MIN_THRESHOLD_ARM) {
            mDesiredPosition = ConfigMap.MIN_THRESHOLD_ARM;
        }
        
    }

    public void setSpeakerDistance(double d) {
        mSpeakerDistance = d;
    }

    private boolean matchesPosition() {
        var diff = mDesiredPosition - mLeftEncoder.getPosition();
        return Math.abs(diff) <= 1;
    }
}
