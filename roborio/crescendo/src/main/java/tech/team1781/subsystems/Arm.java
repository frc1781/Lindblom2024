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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkMaxAlternateEncoder;

public class Arm extends Subsystem {
    private CANSparkMax mRightMotor;
    private CANSparkMax mLeftMotor;
        
    private RelativeEncoder mLeftEncoder; 
    private AbsoluteEncoder mArmAbsoluteEncoder;
    private DutyCycleEncoder mArmAbsoluteDCEncoder;
    private ProfiledPIDController mPositionPID = new ProfiledPIDController(0.05, 0, 0,
            new TrapezoidProfile.Constraints(80, 450));
    private HashMap<ArmState, Double> mPositions = new HashMap<>();

    private GenericEntry mArmPositionEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Arm Angle", -1, ShuffleboardStyle.ARM_ANGLE);
    private GenericEntry mSparkErrorEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Spark Max Errored", false, ShuffleboardStyle.ARM_ERROR);
    private GenericEntry mSparkDataNotSentEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "encoder", false, ShuffleboardStyle.ARM_ERROR);
    private GenericEntry mArmAimSpotEntry = ShuffleboardStyle.getEntry(ConfigMap.SHUFFLEBOARD_TAB, "Arm Aim Spot", "N/A", ShuffleboardStyle.ARM_AIM_SPOT);

    private double mDesiredPosition = 0;
    private Pose2d mRobotPose;
    private CURRENT_AIM_SPOT mCurrentAimSpot = CURRENT_AIM_SPOT.UNDEFEINED;
    private double KICKSTAND_POSITION = 73.0;  // Was 62.0
    private double mPrevAbsoluteAngle = KICKSTAND_POSITION;
    
    private GenericEntry testEntry = Shuffleboard.getTab("test").add("test", 0.0).getEntry();

    public Arm() {
        super("Arm", ArmState.SAFE);
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
        mLeftEncoder.setVelocityConversionFactor((ConfigMap.ARM_GEAR_RATIO * 360.0 * 1.27) / 60);  //why 1.4?  not sure, the gear ratio is off?  it is the closest to the abs encoder
                                                                                                  // ^ It was the gear ratio, incorrect tooth count values for sprockets used, after fix angles are closer
        mLeftEncoder.setPositionConversionFactor(ConfigMap.ARM_GEAR_RATIO * 360.0 * 1.27); // will tell us angle in
        mArmAbsoluteEncoder = mRightMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        mArmAbsoluteEncoder.setAverageDepth(1);
        mArmAbsoluteEncoder.setInverted(true);
        mArmAbsoluteDCEncoder = new DutyCycleEncoder(new DigitalInput(4));
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
        mLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 80);
        mLeftMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, -10);
        mLeftMotor.setSmartCurrentLimit(30);
        mLeftMotor.burnFlash();

        mPositions.put(ArmState.SAFE, 75.5);        // Was 66.0
        mPositions.put(ArmState.PODIUM, CURRENT_AIM_SPOT.PODIUM.getPosition());
        mPositions.put(ArmState.SUBWOOFER, CURRENT_AIM_SPOT.SUBWOOFER.getPosition()); //was 36
        mPositions.put(ArmState.AMP, 50.5);             // Was 46.0
        mPositions.put(ArmState.COLLECT, 0.0);
        mPositions.put(ArmState.COLLECT_HIGH, 63.2);    // Was 55.7
        mPositions.put(ArmState.SKIP, 55.0);
        mPositions.put(ArmState.KICKSTAND, 84.0);
        mPositions.put(ArmState.LOB, CURRENT_AIM_SPOT.SUBWOOFER.getPosition());
        mPositions.put(ArmState.NOTE_ONE, CURRENT_AIM_SPOT.NOTE_1.getPosition());
        mPositions.put(ArmState.NOTE_THREE, CURRENT_AIM_SPOT.NOTE_3.getPosition());
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
        NOTE_THREE,
        LOB
    }

    @Override
    public void genericPeriodic() {
        testEntry.setDouble(getAngleAbsolute());
        if (mLeftEncoder.getPosition() < 10) {
          mLeftMotor.setIdleMode(IdleMode.kCoast);
          mRightMotor.setIdleMode(IdleMode.kCoast);
        }
        else{
         mLeftMotor.setIdleMode(IdleMode.kBrake);
         mRightMotor.setIdleMode(IdleMode.kBrake);
        }

        mArmAimSpotEntry.setString(mCurrentAimSpot.toString());
        if (mLeftEncoder.getPosition() > 10.0 && mLeftMotor.getForwardLimitSwitch(Type.kNormallyOpen).isPressed()) {
            mLeftEncoder.setPosition(getAngleAbsolute()); 
            System.out.println("***************************reset after hitting forward limit*******************");
        }
        // System.out.printf("arm abs: %.3f arm rel %.3f\n", getAngleAbsolute(), getAngle());
    }

    @Override
    public void init() {
        setDesiredState(ArmState.SAFE);
        syncArmEncoder();
        mDesiredPosition = mLeftEncoder.getPosition();
        // if (currentMode == OperatingMode.AUTONOMOUS) {
        //     mSparkDataNotSentEntry.setBoolean(mLeftEncoder.setPosition(KICKSTAND_POSITION) != REVLibError.kOk);
        // }
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

        double currentArmAngle = getAngle(); 
        mArmPositionEntry.setDouble(currentArmAngle);
        if (currentArmAngle != 0.0) {
            var armDutyCycle = mPositionPID.calculate(currentArmAngle, mDesiredPosition);
            if(mSparkErrorEntry.getBoolean(false))
                mSparkErrorEntry.setBoolean(false);
        
            if (getState() == ArmState.COLLECT && getAngle() < 10.0) { //drop into position on ground
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
                return getAngle() < 4.0; //should fall to position of zero
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

    private double getAngle() {
        double relAngle = mLeftEncoder.getPosition();
        if (Math.abs(relAngle - getAngleAbsolute()) > 0.5 && (getAngleAbsolute() <= 2.0)) {
            syncArmEncoder();
            System.out.println("reset rel encoder");
        }
        return mLeftEncoder.getPosition();
    }

    private void syncArmEncoder() {
        System.out.println("synced relative encoder to: " + getAngleAbsolute() + " from: " + mLeftEncoder.getPosition());
        mLeftEncoder.setPosition(getAngleAbsolute());
    }

    private double getAbsoluteAngle() {
        double reportedPosition = mArmAbsoluteDCEncoder.getAbsolutePosition();
        if (reportedPosition < 0.5) {
           //error condition, not a possible real value
           return mPrevAbsoluteAngle;
        }
        
        mPrevAbsoluteAngle = 360.0 * reportedPosition - 0.722;
        return mPrevAbsoluteAngle;
    }

    private double getAngleAbsolute() {
        double reportedPosition = mArmAbsoluteEncoder.getPosition();
        //double reportedPosition = mArmAbsoluteDCEncoder.getAbsolutePosition();
        if (reportedPosition > 0.5)
        {
            mPrevAbsoluteAngle = 360.0 * (mArmAbsoluteEncoder.getPosition() - 0.722); //the absolute encoder reads 0.722 when arm is on the floor
            //mPrevAbsoluteAngle = 360.0 * (mArmAbsoluteEncoder.getPosition() - 0.722); //the absolute encoder reads 0.722 when arm is on the floor
        }
        return mPrevAbsoluteAngle;   
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

    private boolean matchesPosition() {
        return Math.abs(mLeftEncoder.getPosition() - mDesiredPosition) < 1.0;
    }

    private enum CURRENT_AIM_SPOT {
        UNDEFEINED(0.0, EVector.newVector(), EVector.newVector(), 0.0),
        SUBWOOFER(33.2, ConfigMap.RED_SPEAKER_LOCATION, ConfigMap.BLUE_SPEAKER_LOCATION, 2.5),      // Was 32.5
        PODIUM(47.0, ConfigMap.RED_PODIUM, ConfigMap.BLUE_PODIUM, 1),                                           // Pos used to be 45
        NOTE_3(47, EVector.newVector(14.5, 4.27) ,EVector.newVector(2.48, 4.27), 1), // was 42.4
        NOTE_2(44, EVector.newVector(14.13, 5.53) ,EVector.newVector(2.48, 5.53), 0.5), // Was 50
        NOTE_1(58, EVector.newVector(14.06, 6.74),EVector.newVector(2.48, 6.74), 0.5); // Was 50

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