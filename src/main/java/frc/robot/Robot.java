// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.time.LocalDateTime;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating t
 import java.time.LocalDateTime;

 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

    private Spark m_leftMotor = new Spark(0);
    private Spark m_rightMotor = new Spark(1);

    private Joystick driverJoystick = new Joystick(0);

    private boolean toggledvalue = false;
    private LocalDateTime lastToggleDateTime =java.time.LocalDateTime.now(); 

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {

    // Definicion de giro y velocidad
    double turn = driverJoystick.getRawAxis(2);
    double speed = -driverJoystick.getRawAxis(1);

    // Asignacion del movimiento del robot
    double left = speed + turn;
    double right = turn - speed;

    double leftMotor = left;
    double rightMotor = right;

    // Seteo del movimiento de los motores
    m_leftMotor.set(leftMotor);
    m_rightMotor.set(rightMotor);
  }

  @Override
  public void disabledInit() {}

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
