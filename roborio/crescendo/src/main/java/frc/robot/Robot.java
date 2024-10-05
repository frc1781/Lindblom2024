// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import tech.team1781.ConfigMap;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import tech.team1781.DriverInput;
import tech.team1781.autonomous.AutonomousHandler;
import tech.team1781.autonomous.RoutineOverException;
import tech.team1781.autonomous.routines.*;
import tech.team1781.control.ControlSystem;
import tech.team1781.subsystems.Subsystem.OperatingMode;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  private Compressor mCompressor;
  private ControlSystem mControlSystem;
  private AutonomousHandler mAutonomousHandler;
  private DriverInput mDriverInput;
  private PowerDistribution mPowerDistributionHub = new PowerDistribution(ConfigMap.PDH_ID, ModuleType.kRev);
  private final int PDH_CHANNELS = 24;

  private boolean mRanTeleop = false;
  private boolean mRanAuto = false;
  private boolean mAutoRoutineOver = false;
  private DriverStation.Alliance currentAlliance;

  @Override
  public void robotInit() {
    Logger.recordMetadata("RobotName", "GLaDOS");
    Logger.recordMetadata("Team", "1781");

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter());
      Logger.addDataReceiver(new NT4Publisher());

      new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    } else {
      //setUseTiming(false);
      Logger.addDataReceiver(new NT4Publisher());
    }

    Logger.start();

    mCompressor = new Compressor(ConfigMap.FIRST_PCM_ID,
        PneumaticsModuleType.REVPH);
    mCompressor.enableDigital();

    mAutonomousHandler = new AutonomousHandler(
        new P1N1(),
        new P2N2(),
        new P3N3(),
        new P1N1N2(),
        new P1N1C1Close(),
        new P2N2C2Subwoofer(),
        new P3N3C5Close(),
        new P3Leave(),
        new P1WaitLeave(),
        new P2N2N3Subwoofer(),
        new P2N2N1Subwoofer(),
        new Hockey(),
        new P1N1C1Far(),
        new P2FourNote(),
        new P3C5N3(),
        new P3C5C4(),
        new P3C4C3(),
        new P3C5C4Score(),
        new BlueP3C5C4Score(),
        new P3C4C3Score(),
        new BlueP3C4C3Score(),
        new P1C1(),
        new BlueP1C1(),
        new TestRoutine(),
        new TestP2N2());

    mControlSystem = new ControlSystem(mAutonomousHandler);
    mAutonomousHandler.setControlSystem(mControlSystem);
    
    mDriverInput = new DriverInput();
    mControlSystem.init(OperatingMode.DISABLED);
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    //Driver controls
    mDriverInput.addHoldListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.COLLECT, (isPressed) -> mControlSystem.setCollecting(isPressed));

    mDriverInput.addHoldListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.KEEP_DOWN, (isPressed) -> mControlSystem.keepArmDown(isPressed));

    mDriverInput.addHoldListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.CENTER_AMP, (isPressed) -> mControlSystem.setCenteringOnAmp(isPressed));

    mDriverInput.addHoldListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.DRIVER_REJECT, (isPressed) -> mControlSystem.setSpit(isPressed));

    mDriverInput.addHoldListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.COLLECT_HIGH, (isPressed) -> mControlSystem.setCollectHigh(isPressed));

    mDriverInput.addHoldListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.AUTO_AIM, (isHeld) -> mControlSystem.setCenteringOnAprilTag(isHeld));

    mDriverInput.addClickListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.RESET_NAVX, (isPressed) -> {
      if (isPressed) {
        mControlSystem.zeroNavX();
      }
    });

    // Co-pilot Controls
    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.LOB, (isHeld) -> mControlSystem.lobNote(isHeld));

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.SCORE_AMP, (isPressed) -> mControlSystem.setAmp(isPressed));

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.SCORE_PODIUM, (isPressed) -> mControlSystem.shootPodium(isPressed));

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.SHOOT, (isPressed) -> mControlSystem.setShooting(isPressed));

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.PREPARE_TO_SHOOT, (isPressed) -> mControlSystem.setPrepareToShoot(isPressed));

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.SPIT, (isPressed) -> mControlSystem.setSpit(isPressed));

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.ANGLE_UP, (isPressed) -> {
      if (isPressed) {
        mControlSystem.manualAdjustAngle(1);
      }
    });

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.ANGLE_DOWN, (isPressed) -> {
      if (isPressed) {
        mControlSystem.manualAdjustAngle(-1);
      }
    });

    // mDriverInput.addHoldListener(ConfigMap.DRIVER_CONTROLLER_PORT,
    // ConfigMap.NOTE_COLLECTION, (isHeld) -> {
    // mControlSystem.setAutoCollectionButton(isHeld);
    // });

    // mDriverInput.addClickListener(ConfigMap.DRIVER_CONTROLLER_PORT,
    // ConfigMap.CALIBRATE_POSITION, (isPressed) -> {
    // if (isPressed) {
    // mControlSystem.calibratePosition();
    // }
    // });

    mDriverInput.addHoldListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.COLLECT_HIGH, (isPressed) -> {
      mControlSystem.setCollectHigh(isPressed);
    });

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.SCORE_AMP, (isPressed) -> {
      mControlSystem.setAmp(isPressed);
    });

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.SCORE_PODIUM, (isPressed) -> {
      mControlSystem.shootPodium(isPressed);
    });

    mDriverInput.addHoldListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.AUTO_AIM, (isHeld) -> {
      mControlSystem.setCenteringOnAprilTag(isHeld);
    });

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.LOB, (isHeld) -> {
      mControlSystem.lobNote(isHeld);
    });}

    

  @Override
  public void robotPeriodic() {
    // if (mSaveConfigButton.getBoolean(false)) {
    // PreferenceHandler.updateValues();
    // mSaveConfigButton.setBoolean(false);
    // }

   

  }

  @Override
  public void autonomousInit() {
    mAutonomousHandler.init();
    mControlSystem.init(OperatingMode.AUTONOMOUS);
    mRanAuto = true;
    mAutoRoutineOver = false;
  }

  @Override
  public void autonomousPeriodic() {
    if (!mAutoRoutineOver) {
      try {
        mAutonomousHandler.run();
      } catch (RoutineOverException e) {
        mAutoRoutineOver = true;
      }
    }
    mControlSystem.run(null);
  }

  @Override
  public void teleopInit() {
    mControlSystem.init(OperatingMode.TELEOP);
    mRanTeleop = true;
  }

  @Override
  public void teleopPeriodic() {
    mControlSystem.run(mDriverInput.run()); // add copilot input
  }

  @Override
  public void disabledInit() {
    if (mRanAuto && mRanTeleop) {
      DataLogManager.stop();
    }

    mAutonomousHandler.checkSelectedRoutine();
    mControlSystem.disabledLighting();
  }

  @Override
  public void disabledPeriodic() {
    mControlSystem.disabledPeriodic();
    mAutonomousHandler.checkSelectedRoutine();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}
