// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.TimedRobot;
import tech.team1781.ConfigMap;
import tech.team1781.DriverInput;
import tech.team1781.DriverInput.ControllerSide;
import tech.team1781.autonomous.AutonomousHandler;
import tech.team1781.autonomous.RoutineOverException;
import tech.team1781.autonomous.routines.ExampleRoutine;
import tech.team1781.autonomous.routines.PIDTuningRoutine;
import tech.team1781.control.ControlSystem;
import tech.team1781.subsystems.Subsystem.OperatingMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  //control and autonomous
  private ControlSystem mControlSystem;
  private AutonomousHandler mAutonomousHandler;
  private DriverInput mDriverInput;

  @Override
  public void robotInit() {
    mControlSystem = new ControlSystem();
    mAutonomousHandler = new AutonomousHandler(mControlSystem, new ExampleRoutine(), new PIDTuningRoutine());
    mDriverInput = new DriverInput();
    mControlSystem.init(OperatingMode.DISABLED);

    mDriverInput.addClickListener(ConfigMap.DRIVER_CONTROLLER_PORT, ConfigMap.RESET_NAVX, (isPressed)->{
      if(isPressed) {
        mControlSystem.zeroNavX();
      }
    });
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
    mControlSystem.run(mDriverInput.run());  //add copilot input
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
