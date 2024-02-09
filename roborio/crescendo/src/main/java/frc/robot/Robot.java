// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import tech.team1781.ConfigMap;
import tech.team1781.DriverInput;
import tech.team1781.autonomous.AutonomousHandler;
import tech.team1781.autonomous.RoutineOverException;
import tech.team1781.autonomous.routines.FourNoteRoutine;
import tech.team1781.control.ControlSystem;
import tech.team1781.subsystems.Arm.ArmState;
import tech.team1781.subsystems.Subsystem.OperatingMode;

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
  private ControlSystem mControlSystem;
  private AutonomousHandler mAutonomousHandler;
  private DriverInput mDriverInput;

  @Override
  public void robotInit() {
    mControlSystem = new ControlSystem();
    mAutonomousHandler = new AutonomousHandler(mControlSystem, new FourNoteRoutine());
    mDriverInput = new DriverInput();
    mControlSystem.init(OperatingMode.DISABLED);

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

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, "E", (isPressed) -> {
      if(isPressed) {
        mControlSystem.moveArm(3);
      }
    });

    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, "W", (isPressed) -> {
      if(isPressed) {
        mControlSystem.moveArm(-3);
      }
    });

    // mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.CENTER_TO_APRIL_TAG, (isHeld) -> {
    //     mControlSystem.centerOnAprilTag(isHeld);
    // });
    mDriverInput.addHoldListener(ConfigMap.CO_PILOT_PORT, ConfigMap.CENTER_TO_APRIL_TAG, (isHeld) -> {
         mControlSystem.centerOnAprilTag(isHeld);
    });

    // // mDriverInput.addClickListener(0, "B", (isPressed) -> {
    // //   if (isPressed) {
    // //     mControlSystem.setArmState(ArmState.AUTO_ANGLE);
    // //   }
    // // });

    // // mDriverInput.addClickListener(0, "A", (isPressed) -> {
    // //   if (isPressed) {
    // //     mControlSystem.setArmState(ArmState.COLLECT);
    // //   }
    // // });

    // mDriverInput.addHoldListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.COLLECT, (isPressed) -> {
    //   if(isPressed) {
    //     mControlSystem.setCollecting();
    //   }
    // });

    // mDriverInput.addHoldListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.SHOOT, (isPressed) -> {
    //   if(isPressed) {
    //     mControlSystem.setShooting();
    //   }
    // });

    // mDriverInput.addHoldListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.SPIT, (isPressed) -> {
    //   if(isPressed)
    //     mControlSystem.setSpit();
    // });

  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    mControlSystem.init(OperatingMode.AUTONOMOUS);
    mAutonomousHandler.init();
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
  }

  @Override
  public void teleopPeriodic() {
    mControlSystem.run(mDriverInput.run()); // add copilot input
  }

  @Override
  public void disabledInit() {
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
