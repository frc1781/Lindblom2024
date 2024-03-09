// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import tech.team1781.ConfigMap;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import tech.team1781.DriverInput;
import tech.team1781.autonomous.AutonomousHandler;
import tech.team1781.autonomous.RoutineOverException;
import tech.team1781.autonomous.routines.DriverCustomAuto;
import tech.team1781.autonomous.routines.ExampleRoutine;
import tech.team1781.autonomous.routines.FourNoteRoutine;
import tech.team1781.autonomous.routines.Match45Routine;
import tech.team1781.autonomous.routines.RedP1Routine;
import tech.team1781.autonomous.routines.RedP1SeekRoutine;
import tech.team1781.autonomous.routines.RedP3Routine;
import tech.team1781.autonomous.routines.RedP3SeekRoutine;
import tech.team1781.autonomous.routines.SYSIDRoutine;
import tech.team1781.autonomous.routines.ShootAndOut;
import tech.team1781.control.ControlSystem;
import tech.team1781.subsystems.Arm.ArmState;
import tech.team1781.subsystems.Subsystem.OperatingMode;
import tech.team1781.utils.PreferenceHandler;
import tech.team1781.utils.PreferenceHandler.ValueHolder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  // control and autonomous
  private Compressor mCompressor;
  private ControlSystem mControlSystem;
  private AutonomousHandler mAutonomousHandler;
  private DriverInput mDriverInput;
  private GenericEntry mSaveConfigButton = ConfigMap.CONFIG_TAB.add("Save Config", false).withWidget(BuiltInWidgets.kToggleButton).getEntry();

  private boolean ranTeleop = false;
  private boolean ranAuto = false;

  @Override
  public void robotInit() {
    mCompressor = new Compressor(ConfigMap.FIRST_PCM_ID,
        PneumaticsModuleType.REVPH);
    mCompressor.enableDigital();
    
    mControlSystem = new ControlSystem();
    mAutonomousHandler = new AutonomousHandler(mControlSystem, 
      new DriverCustomAuto(),
      new RedP1Routine(),
      new RedP3Routine(),
      new Match45Routine(),
      new ShootAndOut(),
      new ExampleRoutine());
    mDriverInput = new DriverInput();
    mControlSystem.init(OperatingMode.DISABLED);
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    // PreferenceHandler.addValue("frontLeftOffset", ConfigMap.FRONT_LEFT_MODULE_STEER_OFFSET);
    // PreferenceHandler.addValue("frontRightOffset", ConfigMap.FRONT_RIGHT_MODULE_STEER_OFFSET);
    // PreferenceHandler.addValue("backLeftOffset", ConfigMap.BACK_LEFT_MODULE_STEER_OFFSET);
    // PreferenceHandler.addValue("backRightOffset", ConfigMap.BACK_RIGHT_MODULE_STEER_OFFSET);

    mDriverInput.addClickListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.RESET_NAVX, (isPressed)->{
      if(isPressed) {
        mControlSystem.zeroNavX();
      }
    });

    mDriverInput.addHoldListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.KEEP_DOWN, (isPressed) -> {
      mControlSystem.keepArmDown(isPressed);
    });

    mDriverInput.addHoldListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.COLLECT, (isPressed) -> {
      mControlSystem.setCollecting(isPressed);
    });

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.SPIT, (isPressed) -> {
      mControlSystem.setSpit(isPressed);
    });

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.SHOOT, (isPressed) -> {
      mControlSystem.setShooting(isPressed);
    });


    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.PREPARE_TO_SHOOT, (isPressed) -> {
      mControlSystem.setPrepareToShoot(isPressed);
    });

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.ANGLE_UP, (isPressed) -> {
      if(isPressed) {
        mControlSystem.manualAdjustAngle(1);
      }
    });

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.ANGLE_DOWN, (isPressed) -> {
      if(isPressed) {
        mControlSystem.manualAdjustAngle(-1);
      }
    });

    mDriverInput.addHoldListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.NOTE_COLLECTION, (isHeld) -> {
        mControlSystem.setAutoCollectionButton(isHeld);
    });

    mDriverInput.addClickListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.CALIBRATE_POSITION, (isPressed)-> {
      if(isPressed){
        mControlSystem.calibratePosition();
      }
    });

    mDriverInput.addHoldListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.COLLECT_HIGH, (isPressed)-> {
      mControlSystem.setCollectHigh(isPressed);
    });

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.SCORE_AMP, (isPressed)->{
      mControlSystem.setAmp(isPressed);
    });

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.SCORE_PODIUM, (isPressed)->{
      mControlSystem.shootPodium(isPressed);
    });


    mDriverInput.addHoldListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.AUTO_AIM, (isHeld) -> {
      mControlSystem.setCenteringOnAprilTag(isHeld);
    });

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.SKIP, (isHeld) -> {
      mControlSystem.skipNote(isHeld);
    });


  }

  @Override
  public void robotPeriodic() {
    if(mSaveConfigButton.getBoolean(false)) {
      // PreferenceHandler.updateValues();
      mSaveConfigButton.setBoolean(false);
    }
  }

  @Override
  public void autonomousInit() {
    mControlSystem.init(OperatingMode.AUTONOMOUS);
    mAutonomousHandler.init();
    ranAuto = true;
  }

  @Override
  public void autonomousPeriodic() {
    try {
      mAutonomousHandler.run();
    } catch (RoutineOverException e) {
      e.printStackTrace();
    }

    mControlSystem.run(null);
  }

  @Override
  public void teleopInit() {
    mControlSystem.init(OperatingMode.TELEOP);
    ranTeleop = true;
  }

  @Override
  public void teleopPeriodic() {
    mControlSystem.run(mDriverInput.run()); // add copilot input
  }

  @Override
  public void disabledInit() {
    if(ranAuto && ranTeleop) {
      DataLogManager.stop();
    }
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
