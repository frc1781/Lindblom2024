package tech.team1781.subsystems;

import java.util.HashMap;
import java.util.concurrent.TimeUnit;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
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
    private CANSparkMax mRightMotor;
    private CANSparkMax mLeftMotor;
    
    private RelativeEncoder mLeftEncoder; 
    private ProfiledPIDController mPositionPID = new ProfiledPIDController(0.05, 0, 0,
            new TrapezoidProfile.Constraints(80, 450));
    private HashMap<ArmState, Double> mPositions = new HashMap<>();
    private GenericEntry mArmPositionEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Arm Angle", -1,
            ShuffleboardStyle.ARM_ANGLE);
    private GenericEntry mSparkErrorEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Spark Max Errored", false, ShuffleboardStyle.ARM_ERROR);
        private GenericEntry mSparkDataNotSentEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "encoder", false, ShuffleboardStyle.ARM_ERROR);
    private GenericEntry mArmAimSpotEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Arm Aim Spot", "N/A", ShuffleboardStyle.ARM_AIM_SPOT);
    private boolean mResetPosition = false;
    private double mDesiredPosition = 0;
    private double mSpeakerDistance = 0;
    private Pose2d mRobotPose;
    private CURRENT_AIM_SPOT mCurrentAimSpot = CURRENT_AIM_SPOT.UNDEFEINED;
    private double KICKSTAND_POSITION = 62.0;
    private double FORWARD_LIMIT_POSITION = 69.0;
    public Arm() {
        super("Arm", ArmState.SAFE);
        mRightMotor = new CANSparkMax(
                ConfigMap.ARM_PIVOT_RIGHT_MOTOR,
                CANSparkMax.MotorType.kBrushless);

        mLeftMotor = new CANSparkMax(
                ConfigMap.ARM_PIVOT_LEFT_MOTOR,
                CANSparkMax.MotorType.kBrushless);
        mLeftMotor.setSmartCurrentLimit(40);
        mRightMotor.setSmartCurrentLimit(40);
        mLeftEncoder = mLeftMotor.getEncoder();
        mLeftEncoder.setVelocityConversionFactor((ConfigMap.ARM_GEAR_RATIO * 360.0 * 1.2) / 60);
        mLeftEncoder.setPositionConversionFactor(ConfigMap.ARM_GEAR_RATIO * 360.0 * 1.2); // will tell us angle in
                                                                                          // degrees
        mRightMotor.follow(mLeftMotor, true);
        mLeftMotor.setIdleMode(IdleMode.kBrake);
        mRightMotor.setIdleMode(IdleMode.kBrake);
        mLeftMotor.burnFlash();
        mRightMotor.burnFlash();
        mSparkDataNotSentEntry.setBoolean(mLeftEncoder.setPosition(KICKSTAND_POSITION) != REVLibError.kOk);
        mPositionPID.reset(KICKSTAND_POSITION);
        System.out.println("-------------------------------------------------");
        System.out.println("   ARM SET TO KICKSTAND ENCODER POSITION         ");
        System.out.println("         ensure that kick stand is on            ");
        System.out.println("-------------------------------------------------");

        mLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        mLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        mLeftMotor.setSmartCurrentLimit(30);
        mLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 70);
        mLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        mLeftMotor.burnFlash();
        mPositions.put(ArmState.SAFE, 66.0);
        mPositions.put(ArmState.PODIUM, CURRENT_AIM_SPOT.PODIUM.getPosition());
        mPositions.put(ArmState.SUBWOOFER, CURRENT_AIM_SPOT.SUBWOOFER.getPosition()); //was 36
        mPositions.put(ArmState.AMP, 46.0);
        mPositions.put(ArmState.COLLECT, 0.0);
        mPositions.put(ArmState.COLLECT_HIGH, 55.7);
        mPositions.put(ArmState.SKIP, 55.0);
        mPositions.put(ArmState.KICKSTAND, 69.0);
        mPositions.put(ArmState.LOB, CURRENT_AIM_SPOT.SUBWOOFER.getPosition());
        mPositions.put(ArmState.NOTE_ONE, CURRENT_AIM_SPOT.NOTE_1.getPosition());
        mPositions.put(ArmState.NOTE_THREE, CURRENT_AIM_SPOT.NOTE_3.getPosition());
    } //58 

    public enum ArmState implements Subsystem.SubsystemState {
        KICKSTAND,
        SAFE,
        PODIUM,
        SUBWOOFER,
        COLLECT,
        COLLECT_HIGH,
        MANUAL,
        AUTO_ANGLE,
        AMP,
        SKIP,
        NOTE_ONE,
        NOTE_THREE,
        LOB
    }

    @Override
    public void genericPeriodic() {
        if (mLeftEncoder.getPosition() < 6) {
          mLeftMotor.setIdleMode(IdleMode.kCoast);
          mRightMotor.setIdleMode(IdleMode.kCoast);
        }
        else{
          mLeftMotor.setIdleMode(IdleMode.kBrake);
          mRightMotor.setIdleMode(IdleMode.kBrake);
        }

        mArmAimSpotEntry.setString(mCurrentAimSpot.toString());
        if (mLeftEncoder.getPosition() > 10.0 && mLeftMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed()) {
            mLeftEncoder.setPosition(FORWARD_LIMIT_POSITION); 
            System.out.println("***************************reset after hitting forward limit*******************");
        }
    }

    @Override
    public void init() {
        setDesiredState(ArmState.SAFE);
        mDesiredPosition = mLeftEncoder.getPosition();
        if (currentMode == OperatingMode.AUTONOMOUS) {
            mSparkDataNotSentEntry.setBoolean(mLeftEncoder.setPosition(KICKSTAND_POSITION) != REVLibError.kOk);
        }
    }

    @Override
    public void getToState() {
        switch((ArmState) getState()) {
            case AUTO_ANGLE:
                mDesiredPosition = calculateAngleFromDistance();
                break;
            case MANUAL:
            break;
            default:
                mDesiredPosition = mPositions.get(getState());
                break;
        }

        double currentArmAngle = mLeftEncoder.getPosition();
        mArmPositionEntry.setDouble(currentArmAngle);
        if (currentArmAngle != 0.0) {

            var armDutyCycle = mPositionPID.calculate(currentArmAngle, mDesiredPosition);
            if(mSparkErrorEntry.getBoolean(false))
                mSparkErrorEntry.setBoolean(false);
        
            if (getState() == ArmState.COLLECT && getAngle() < 10.0) {
                armDutyCycle = 0.0;
                mLeftEncoder.setPosition(0.01);
            } 
            mLeftMotor.set(armDutyCycle);  
        } else {
            mSparkErrorEntry.setBoolean(true);
        }
    }

    @Override
    public boolean matchesDesiredState() {
        switch ((ArmState) getState()) {
            case COLLECT:
                return mLeftEncoder.getPosition() <= 0.01;
            default:
                return matchesPosition();
        }
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void setDesiredState(SubsystemState state) {
        if(state == ArmState.SAFE && state == ArmState.MANUAL) {
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
        mRobotPose = robotPose;
    }

    private double calculateAngleFromDistance() {
        boolean foundAimSpot = false;

        for(CURRENT_AIM_SPOT aimSpot : CURRENT_AIM_SPOT.values()) {
            if(aimSpot == CURRENT_AIM_SPOT.UNDEFEINED) continue;
            if(aimSpot.atPosition(mRobotPose)) {
                mCurrentAimSpot = aimSpot;
                foundAimSpot = true;
                break;
            }
        }

        if(!foundAimSpot) {
            mCurrentAimSpot = CURRENT_AIM_SPOT.UNDEFEINED;
        }

        if(mCurrentAimSpot != CURRENT_AIM_SPOT.UNDEFEINED) {
            return mCurrentAimSpot.getPosition();
        }

        return CURRENT_AIM_SPOT.SUBWOOFER.getPosition();
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
        //System.out.println("diff: " + Math.abs(mLeftEncoder.getPosition() - mDesiredPosition));
        return Math.abs(mLeftEncoder.getPosition() - mDesiredPosition) < 0.8;
    }

    private enum CURRENT_AIM_SPOT {
        UNDEFEINED(0.0, EVector.newVector(), EVector.newVector(), 0.0),
        SUBWOOFER(32.5, ConfigMap.RED_SPEAKER_LOCATION, ConfigMap.BLUE_SPEAKER_LOCATION, 2.5),
        PODIUM(45, ConfigMap.RED_PODIUM, ConfigMap.BLUE_PODIUM, 1),
        NOTE_3(42.4, EVector.newVector(14.5, 4.27) ,EVector.newVector(2.48, 4.27), 1),
        NOTE_2(50, EVector.newVector(14.13, 5.53) ,EVector.newVector(2.48, 5.53), 0.5),
        NOTE_1(50, EVector.newVector(14.06, 6.74),EVector.newVector(2.48, 6.74), 0.5);

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