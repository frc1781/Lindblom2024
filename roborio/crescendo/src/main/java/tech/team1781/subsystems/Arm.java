package tech.team1781.subsystems;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import tech.team1781.ConfigMap;

import com.revrobotics.SparkLimitSwitch;

public class Arm extends Subsystem {
    private CANSparkMax mRightMotor;
    private CANSparkMax mLeftMotor;
    private RelativeEncoder mLeftEncoder;
    private SparkLimitSwitch mDownLimitSwitch;
    private SparkLimitSwitch mUpLimitSwitch;
    private ProfiledPIDController mPositionPID = new ProfiledPIDController(0.1, 0, 0,
            new TrapezoidProfile.Constraints(200, 50));
    private HashMap<ArmState, Double> mPositions = new HashMap<>();

    private GenericEntry mArmPositionEntry = ConfigMap.SHUFFLEBOARD_TAB.add("Arm Position", -1).getEntry();

    private double mDesiredPosition = 0;

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

        mDownLimitSwitch = mLeftMotor.getReverseLimitSwitch(Type.kNormallyOpen);
        mUpLimitSwitch = mLeftMotor.getForwardLimitSwitch(Type.kNormallyOpen);

        currentState = ArmState.START;

        mRightMotor.follow(mLeftMotor, true);
        mLeftMotor.setIdleMode(IdleMode.kBrake);
        mRightMotor.setIdleMode(IdleMode.kBrake);
        System.out.println("initialized arm moters for following...");
        System.out.println("ENSURE ARM IN ZERO POSITION!!!!! Just set encoder to zero");
        mLeftEncoder.setPosition(0);

        mPositions.put(ArmState.START, 0.0);
        mPositions.put(ArmState.SAFE, 63.0);
        mPositions.put(ArmState.PODIUM, 43.8);
        mPositions.put(ArmState.SUBWOOFER, 25.0);
        mPositions.put(ArmState.COLLECT, 0.0);

    }

    public enum ArmState implements Subsystem.SubsystemState {
        // Will probably add set angles
        START("Start"),
        SAFE("Safe"),
        PODIUM("Podium"),
        SUBWOOFER("Subwoofer"),
        COLLECT("Collect"),
        AUTO_AIM("Auto Aim"),
        MANUAL("Manual");

        private final String name;

        private ArmState(String _name) {
             name = _name;
        }

        public String toString() {
            return name;
        } 
    }

    @Override
    public void genericPeriodic() {
    }

    @Override
    public void init() {
        mLeftMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
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
        mDesiredPosition = MathUtil.clamp(mDesiredPosition,0.0,90.0);
        var armDutyCycle = mPositionPID.calculate(mLeftEncoder.getPosition(), mDesiredPosition);
        mArmPositionEntry.setDouble(mLeftEncoder.getPosition());
        System.out.println(getAngle());
        mLeftMotor.set(armDutyCycle);
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
                return atOptimalAngle();

            case COLLECT:
                return atOptimalAngle();
        }
        return false;
    }

    @Override
    public void autonomousPeriodic() {

    }

    public void driveManual(double joyStickInput) {
        if(currentState != ArmState.MANUAL) {
            return;
        }
        mDesiredPosition = MathUtil.clamp(mDesiredPosition + joyStickInput * .5, 0, 82);
        System.out.println(mDesiredPosition);
    }

    private boolean overExtended() {
        if (mUpLimitSwitch.isPressed()) {
            System.out.println("Arm limit switch hit");
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void setDesiredState(SubsystemState state) {
        super.setDesiredState(state);
        if (state != ArmState.AUTO_AIM && state != ArmState.MANUAL) {
            mDesiredPosition = mPositions.get(state);
        }
    }

    public void setOptimalAngle(double distance, double viewAngle) {
        // Calculate desired angle
        // Find desired angle based on distance & orientation to April Tag
        // Dist in cm
        // Create threshold for mDesiredPosition to actually change
        mDesiredPosition = 90 - (60-distance);
        // mDesiredPosition = 25.0;
    }

    public boolean atOptimalAngle() {
        return Math.abs(getAngle() - mDesiredPosition) < ConfigMap.ARM_POSITION_TOLERANCE;
    }

    public double getAngle() {
        return mLeftEncoder.getPosition();
    }
}
