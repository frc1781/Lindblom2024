package tech.team1781.subsystems;

import java.util.HashMap;
import java.util.concurrent.TimeUnit;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import tech.team1781.ConfigMap;
import tech.team1781.ShuffleboardStyle;
import tech.team1781.control.ControlSystem;
import tech.team1781.utils.EVector;
import tech.team1781.utils.NetworkLogger;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;

import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxAlternateEncoder;

public class Arm extends Subsystem {
    private CANSparkMax mRightMotor;
    private CANSparkMax mLeftMotor;

    private RelativeEncoder mLeftEncoder;
    private AbsoluteEncoder mArmAbsoluteEncoder;
    private ProfiledPIDController mPositionPID = new ProfiledPIDController(0.04, 0, 0,
            new TrapezoidProfile.Constraints(90, 450));
    private HashMap<ArmState, Double> mPositions = new HashMap<>();

    private GenericEntry mArmPositionEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Arm Angle", -1,
            ShuffleboardStyle.ARM_ANGLE);
    private GenericEntry mSparkErrorEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Spark Max Errored",
            false, ShuffleboardStyle.ARM_ERROR);
    private GenericEntry mSparkDataNotSentEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "encoder",
            false, ShuffleboardStyle.ARM_ERROR);
    private GenericEntry mArmAimSpotEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Arm Aim Spot",
            "N/A", ShuffleboardStyle.ARM_AIM_SPOT);

    private double mDesiredPosition = 0;
    private Pose2d mRobotPose;
    private CURRENT_AIM_SPOT mCurrentAimSpot = CURRENT_AIM_SPOT.UNDEFEINED;
    private double KICKSTAND_POSITION = 70.0; // was 73 Was 62.0
    private double mPrevAbsoluteAngle = KICKSTAND_POSITION;
    private double mPrevRecordedAngle = 0.0;
    private IdleMode mIdleMode;

    public Arm() {
        super("Arm", ArmState.KICKSTAND);
        mRightMotor = new CANSparkMax(
                ConfigMap.ARM_PIVOT_RIGHT_MOTOR,
                CANSparkMax.MotorType.kBrushless);
        mRightMotor.restoreFactoryDefaults();
        mLeftMotor = new CANSparkMax(
                ConfigMap.ARM_PIVOT_LEFT_MOTOR,
                CANSparkMax.MotorType.kBrushless);
        mLeftMotor.restoreFactoryDefaults();
        mLeftMotor.setSmartCurrentLimit(40);
        mRightMotor.setSmartCurrentLimit(40);
        mLeftEncoder = mLeftMotor.getEncoder();
        //mLeftEncoder.setVelocityConversionFactor((ConfigMap.ARM_GEAR_RATIO * 360.0 * 1.27) / 60); // why 1.4? not sure,
        mLeftEncoder.setPositionConversionFactor(ConfigMap.ARM_CONVERSION_REL_TO_ANGLE);
        //mLeftEncoder.setPositionConversionFactor(1.0);
        mLeftEncoder.setVelocityConversionFactor(ConfigMap.ARM_CONVERSION_REL_TO_ANGLE/60.0);
        mArmAbsoluteEncoder = mRightMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        //mArmAbsoluteEncoder.setAverageDepth(1);
        mArmAbsoluteEncoder.setInverted(true);
        mRightMotor.follow(mLeftMotor, true);

        mIdleMode = IdleMode.kBrake;
        mRightMotor.setIdleMode(mIdleMode);
        mLeftMotor.setIdleMode(mIdleMode);
      
        System.out.println("conversion factor: " + mLeftEncoder.getPositionConversionFactor());
        mLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        mLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
        mLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 80);
        mLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -10);
        mLeftMotor.burnFlash();
        mRightMotor.burnFlash();

        mPositions.put(ArmState.SAFE, 70.0); 
        mPositions.put(ArmState.PODIUM, CURRENT_AIM_SPOT.PODIUM.getPosition());
        mPositions.put(ArmState.SUBWOOFER, CURRENT_AIM_SPOT.SUBWOOFER.getPosition()); // was 36
        mPositions.put(ArmState.AMP, 50.0); // Was 46.0
        mPositions.put(ArmState.COLLECT, 0.0);
        mPositions.put(ArmState.COLLECT_HIGH, 60.0); // Was 55.7
        mPositions.put(ArmState.SKIP, 55.0);
        mPositions.put(ArmState.KICKSTAND, 80.0);
        mPositions.put(ArmState.LOB, CURRENT_AIM_SPOT.SUBWOOFER.getPosition());
        mPositions.put(ArmState.NOTE_ONE, CURRENT_AIM_SPOT.NOTE_1.getPosition());
        mPositions.put(ArmState.NOTE_TWO, CURRENT_AIM_SPOT.NOTE_2.getPosition());
        mPositions.put(ArmState.NOTE_THREE, CURRENT_AIM_SPOT.NOTE_3.getPosition());
        mPositions.put(ArmState.FAR_SHOT, 58.0);



        NetworkLogger.initLog("Arm Matches State", true);
        NetworkLogger.initLog("Raw Absolute Arm", 0.0);
    }

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
        NOTE_TWO,
        NOTE_THREE,
        LOB,
        FAR_SHOT
    }

    @Override
    public void genericPeriodic() {
        NetworkLogger.logData("Arm Matches State", matchesDesiredState());
        NetworkLogger.logData("Raw Absolute Arm", getAngleAbsolute());

        // testEntry.setDouble(getAngleAbsolute());
        if (mLeftEncoder.getPosition() < 10) {
            setIdleMode(IdleMode.kCoast);
        } else {
            setIdleMode(IdleMode.kBrake);
        }


        mArmAimSpotEntry.setString(mCurrentAimSpot.toString());

        //dropped to ground, reset relative encoder only when going down.
        if (mArmAbsoluteEncoder.getPosition() < 0.24 && mLeftEncoder.getVelocity() < 0.0) {
            mLeftEncoder.setPosition(0.01);
        }
        if (mLeftEncoder.getPosition() < 0.0) {
            mLeftEncoder.setPosition(0.01);
        }
        syncArm();
        // if (Math.abs(mLeftEncoder.getVelocity()) < 0) {
            // mLeftEncoder.setPosition(getAngleAbsolute());
        // }
    }

    @Override
    public void init() {
        //mSparkDataNotSentEntry.setBoolean(mLeftEncoder.setPosition(KICKSTAND_POSITION) != REVLibError.kOk);
        setDesiredState(ArmState.KICKSTAND);
        syncArm();
        mPositionPID.reset(getAngle());
        mSparkDataNotSentEntry.setBoolean(mLeftEncoder.setPosition(getAngleAbsolute()) != REVLibError.kOk);
    }

    @Override
    public void getToState() {
        switch ((ArmState) getState()) {
            case AUTO_ANGLE:
                mDesiredPosition = calculateAngleFromDistance();
                break;
            case MANUAL:
                break;
            default:
                mDesiredPosition = mPositions.get(getState());
                break;
        }

        double currentArmAngle = getAngle();
        mArmPositionEntry.setDouble(currentArmAngle);
        if (currentArmAngle != 0.0) {
            var armDutyCycle = mPositionPID.calculate(currentArmAngle, mDesiredPosition);
            if (mSparkErrorEntry.getBoolean(false))
                mSparkErrorEntry.setBoolean(false);

            if (getState() == ArmState.COLLECT && getAngle() < 10.0) { // drop into position on ground
                armDutyCycle = 0.0;
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
                return getAngle() < 4.0; // should fall to position of zero
            case KICKSTAND:
                return mLeftMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).isPressed() || matchesPosition();
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
        if (state == ArmState.SAFE && state == ArmState.MANUAL) {
            return;
        }

        super.setDesiredState(state);

        if (state != ArmState.MANUAL && state != ArmState.AUTO_ANGLE) {
            mDesiredPosition = mPositions.get(state);
        } else if (state == ArmState.AUTO_ANGLE) {
            mDesiredPosition = calculateAngleFromDistance();
        }
    }

    private void setIdleMode(IdleMode mode) {
        if (mode == mIdleMode) {
            return;
        }

        mLeftMotor.setIdleMode(mode);
        mRightMotor.setIdleMode(mode);
        System.out.println(mode);
        mIdleMode = mode;
        
    }

    private void syncArm() {
        double abs = getAngleAbsolute();
        if(abs != mPrevRecordedAngle) {
            mLeftEncoder.setPosition(abs);
            mPrevRecordedAngle = abs;
        }

    }

    private double getAngle() {
        return mLeftEncoder.getPosition();
    }

    private double getAngleAbsolute() {
        double reportedPosition = mArmAbsoluteEncoder.getPosition();
        if (reportedPosition > 0.1) {
            mPrevAbsoluteAngle = 360.0 * (mArmAbsoluteEncoder.getPosition() - ConfigMap.ARM_OFFSET); // the absolute encoder reads
        }
        return mPrevAbsoluteAngle;
    }

    public void updateAimSpots(Pose2d robotPose) {
        mRobotPose = robotPose;
    }

    private double calculateAngleFromDistance() {
        boolean foundAimSpot = false;

        for (CURRENT_AIM_SPOT aimSpot : CURRENT_AIM_SPOT.values()) {
            if (aimSpot == CURRENT_AIM_SPOT.UNDEFEINED)
                continue;
            if (aimSpot.atPosition(mRobotPose)) {
                mCurrentAimSpot = aimSpot;
                foundAimSpot = true;
                break;
            }
        }

        if (!foundAimSpot) {
            mCurrentAimSpot = CURRENT_AIM_SPOT.UNDEFEINED;
        }

        if (mCurrentAimSpot != CURRENT_AIM_SPOT.UNDEFEINED) {
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

    private boolean matchesPosition() {
        return Math.abs(mLeftEncoder.getPosition() - mDesiredPosition) < 1.5;
    }

    private enum CURRENT_AIM_SPOT {
        UNDEFEINED(0.0, EVector.newVector(), EVector.newVector(), 0.0),
        SUBWOOFER(35, ConfigMap.RED_SPEAKER_LOCATION, ConfigMap.BLUE_SPEAKER_LOCATION, 2.5), // Was 32.5
        PODIUM(50.0, ConfigMap.RED_PODIUM, ConfigMap.BLUE_PODIUM, 1), // Pos used to be 45
        NOTE_3(53, EVector.newVector(14.5, 4.27), EVector.newVector(2.48, 4.27), 1), // was 42.4
        NOTE_2(52, EVector.newVector(14.13, 5.53), EVector.newVector(2.48, 5.53), 0.5), // Was 50
        NOTE_1(51, EVector.newVector(14.06, 6.74), EVector.newVector(2.48, 6.74), 0.5); // Was 50

        private double position;
        private EVector redPosition;
        private EVector bluePosition;
        private double distanceTolerance;

        private CURRENT_AIM_SPOT(double _position, EVector _redPosition, EVector _bluePosition,
                double _distanceTolerance) {
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