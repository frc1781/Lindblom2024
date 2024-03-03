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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import tech.team1781.ConfigMap;
import tech.team1781.ShuffleboardStyle;
import tech.team1781.control.ControlSystem;
import tech.team1781.utils.EVector;

import com.revrobotics.SparkLimitSwitch;

public class Arm extends Subsystem {
    private final double KICKSTAND_OFF_POSITION;

    private CANSparkMax mRightMotor;
    private CANSparkMax mLeftMotor;
    private RelativeEncoder mLeftEncoder;
    private ProfiledPIDController mPositionPID = new ProfiledPIDController(0.05, 0, 0,
            new TrapezoidProfile.Constraints(80, 450));
    private HashMap<ArmState, Double> mPositions = new HashMap<>();
    private GenericEntry mArmPositionEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Arm Angle", -1,
            ShuffleboardStyle.ARM_ANGLE);
    private GenericEntry mArmAimSpotEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Arm Aim Spot", "N/A", ShuffleboardStyle.ARM_AIM_SPOT);

    private double mDesiredPosition = 0;
    private double mSpeakerDistance = 0;
    private Pose2d mRobotPose;
    private CURRENT_AIM_SPOT mCurrentAimSpot = CURRENT_AIM_SPOT.UNDEFEINED;
    private boolean mKickstandOff = false;

    public Arm() {
        super("Arm", ArmState.KICKSTAND);
        mRightMotor = new CANSparkMax(
                ConfigMap.ARM_PIVOT_RIGHT_MOTOR,
                CANSparkMax.MotorType.kBrushless);

        mLeftMotor = new CANSparkMax(
                ConfigMap.ARM_PIVOT_LEFT_MOTOR,
                CANSparkMax.MotorType.kBrushless);

        mLeftEncoder = mLeftMotor.getEncoder();
        mLeftEncoder.setVelocityConversionFactor((ConfigMap.ARM_GEAR_RATIO * 360.0 * 1.2) / 60);
        mLeftEncoder.setPositionConversionFactor(ConfigMap.ARM_GEAR_RATIO * 360.0 * 1.2); // will tell us angle in
                                                                                          // degrees
        mRightMotor.follow(mLeftMotor, true);
        mLeftMotor.setIdleMode(IdleMode.kBrake);
        mRightMotor.setIdleMode(IdleMode.kBrake);
        // System.out.println("initialized arm moters for following...");
        // System.out.println("ENSURE ARM IN ZERO POSITION!!!!! Just set encoder to zero");
        // mLeftEncoder.setPosition(0);
        final double KICKSTAND_POSITION = 68.8;
        KICKSTAND_OFF_POSITION = KICKSTAND_POSITION + 2;
        mLeftEncoder.setPosition(KICKSTAND_POSITION);
        this.currentState = ArmState.KICKSTAND;
        mKickstandOff = false;


        mLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        mLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        mLeftMotor.setSmartCurrentLimit(30);
        mLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 90);
        mLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        mLeftMotor.burnFlash();
        mPositions.put(ArmState.START, 0.0); // Temporary, used to be 71.9
        mPositions.put(ArmState.SAFE, 63.0);
        mPositions.put(ArmState.PODIUM, CURRENT_AIM_SPOT.PODIUM.getPosition());
        mPositions.put(ArmState.SUBWOOFER, 40.0);
        mPositions.put(ArmState.AMP, 43.0);
        mPositions.put(ArmState.COLLECT, 0.0);
        mPositions.put(ArmState.COLLECT_HIGH, 56.0);
        mPositions.put(ArmState.SKIP, 59.0);
    }

    public enum ArmState implements Subsystem.SubsystemState {
        KICKSTAND,
        START,
        SAFE,
        PODIUM,
        SUBWOOFER,
        COLLECT,
        COLLECT_HIGH,
        MANUAL,
        AUTO_ANGLE,
        AMP,
        SKIP
    }

    @Override
    public void genericPeriodic() {
        // System.out.println(getAngle());
        mArmAimSpotEntry.setString(mCurrentAimSpot.toString());

        if(mLeftMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed()) {
            mLeftEncoder.setPosition(0);
        }
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
        switch((ArmState) getState()) {
            case KICKSTAND:
                mDesiredPosition = KICKSTAND_OFF_POSITION + 0.5;
                if(mLeftEncoder.getPosition() >= KICKSTAND_OFF_POSITION) {
                    mKickstandOff = true;
                }
            case AUTO_ANGLE:
                mDesiredPosition = calculateAngleFromDistance();
                break;
            default:
                mDesiredPosition = mPositions.get(getState());
                break;
        }

        var armDutyCycle = mPositionPID.calculate(mLeftEncoder.getPosition(), mDesiredPosition);
        mArmPositionEntry.setDouble(mLeftEncoder.getPosition());
        mLeftMotor.set(armDutyCycle);
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
        if(!mKickstandOff && state == ArmState.KICKSTAND) {
            return;
        }

        super.setDesiredState(state);

        if (state != ArmState.MANUAL && state != ArmState.AUTO_ANGLE) {
            mDesiredPosition = mPositions.get(state);
        } else if (state == ArmState.AUTO_ANGLE) {
            mDesiredPosition = calculateAngleFromDistance();
        }
    }

    public double getAngle() {
        return mLeftEncoder.getPosition();
    }

    public void updateAimSpots(Pose2d robotPose) {
        boolean foundAimSpot = false;

        for(CURRENT_AIM_SPOT aimSpot : CURRENT_AIM_SPOT.values()) {
            if(aimSpot == CURRENT_AIM_SPOT.UNDEFEINED) continue;
            if(aimSpot.atPosition(robotPose)) {
                mCurrentAimSpot = aimSpot;
                foundAimSpot = true;
                break;
            }
        }

        if(!foundAimSpot) {
            mCurrentAimSpot = CURRENT_AIM_SPOT.UNDEFEINED;
        }
    }

    private double calculateAngleFromDistance() {

        if(mCurrentAimSpot != CURRENT_AIM_SPOT.UNDEFEINED) {
            return mCurrentAimSpot.getPosition();
        }

        return CURRENT_AIM_SPOT.SUBWOOFER.getPosition();

        // final double start = 32;
        // final double coefficient = 18.3;
        // // double dist = LimelightHelper.getDistanceOfApriltag(4);
        // // double dist = mSpeakerDistance - ConfigMap.DRIVETRAIN_TRACKWIDTH/2;
        // double dist = mSpeakerDistance;
        // dist = Math.abs(dist);
        // double angle = 32.0;
        // if (dist < 0.5) {// can not see april tag
        //     angle = 32.0;
        // } else {
        //     angle = Math.log(dist) * coefficient + start;
        // }

        // System.out.printf("dist %.2f, angle %.2f\n", dist, angle);
        // if (angle > 51) {
        //     angle = 51;
        // }

        // return angle;
        // return 32.0;
    }

    public void manualAdjustAngle(double d) {
        setDesiredState(ArmState.MANUAL);

        mDesiredPosition += d;
        if (mDesiredPosition > ConfigMap.MAX_THRESHOLD_ARM) {
            mDesiredPosition = ConfigMap.MAX_THRESHOLD_ARM;
        }

        if (mDesiredPosition < ConfigMap.MIN_THRESHOLD_ARM) {
            mDesiredPosition = ConfigMap.MIN_THRESHOLD_ARM;
        }

    }

    public void setSpeakerDistance(double d) {
        mSpeakerDistance = d;
    }

    private boolean matchesPosition() {
        return Math.abs(mLeftEncoder.getPosition() - mDesiredPosition) < 1;
    }


    //44.4 for n3
    private enum CURRENT_AIM_SPOT {
        UNDEFEINED(0.0, EVector.newVector(), EVector.newVector(), 0.0),
        SUBWOOFER(35, ConfigMap.RED_SPEAKER_LOCATION, ConfigMap.BLUE_SPEAKER_LOCATION, 2.5),
        PODIUM(48, ConfigMap.RED_PODIUM, ConfigMap.BLUE_PODIUM, 1),
        NOTE_3(44.4, EVector.newVector(14.5, 4.27) ,EVector.newVector(2.48, 4.27), 0.5),
        NOTE_2(44, EVector.newVector(14.13, 5.53) ,EVector.newVector(2.48, 5.53), 0.5),
        NOTE_1(48, EVector.newVector(14.06, 6.74),EVector.newVector(2.48, 6.74), 0.5);

        private double position;
        private EVector redPosition;
        private EVector bluePosition;
        private double distanceTolerance;
        private CURRENT_AIM_SPOT(double _position, EVector _redPosition, EVector _bluePosition, double _distanceTolerance) {
            position = _position;
            redPosition = _redPosition;
            bluePosition = _bluePosition;
            distanceTolerance = _distanceTolerance;
        }

        public boolean atPosition(Pose2d currentPose2d) {
            EVector target = ControlSystem.isRed() ? redPosition : bluePosition;
            EVector currentPose = EVector.fromPose(currentPose2d);
            currentPose.z = 0;
            double dist = target.dist(currentPose);

            return dist <= distanceTolerance;
        }

        public double getPosition() {
            return position;
        }
    }

}
