// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.List;
import com.revrobotics.*;
import com.revrobotics.CANSparkLowLevel.MotorType;
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
    private final PWMSparkMax m_leftDrive = new PWMSparkMax(0);
    private final PWMSparkMax m_rightDrive = new PWMSparkMax(1);
    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);
    private final XboxController m_Controller = new XboxController(0);
    private Spark m_leftMotor = new Spark(0);
    private Spark m_rightMotor = new Spark(1);
    private Joystick driverJoystick = new Joystick(0);

    /* Definición de controladores para el shooter */
    private CANSparkMax m_shooter1 = new CANSparkMax(3, MotorType.kBrushed);
    private CANSparkMax m_shooter2 = new CANSparkMax(4, MotorType.kBrushed);

    /* Variable que guarda el tiempo en el que inicia el match */
    private double tiempoInicio;
    private double tiempo;

  @Override
  public void robotInit() {
    // posicion = PathPlannerAuto.getStaringPoseFromAutoFile(nombreAutonomo);
    // camino = PathPlannerAuto.getPathGroupFromAutoFile(nombreAutonomo);
    
    // System.out.println(posicion);
    // System.out.println(camino);

    // posicion.getTranslation();
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    tiempoInicio = Timer.getFPGATimestamp();
    autonomo();
  }

  @Override
  public void autonomousPeriodic() {
    tiempo = Timer.getFPGATimestamp();    
    if(tiempo - tiempoInicio < 1.5){
      //System.out.println(tiempo);
    }else {
      m_leftMotor.set(0);
      m_rightMotor.set(0);
    }
  }

  @Override
  public void teleopInit() {
    //autonomo.cancel();
  }

  @Override
  public void teleopPeriodic() {

    m_robotDrive.arcadeDrive(-m_Controller.getRightY(), m_Controller.getRightX());

    // Definicion de giro y velocidad
    double turn = m_Controller.getRightX();
    double speed = -m_Controller.getRightY();

    // Asignacion del movimiento del robot
    double left = speed + turn;
    double right = turn - speed;

    double leftMotor = left;
    double rightMotor = right;

    // Seteo del movimiento de los motores
    m_leftMotor.set(leftMotor);
    m_rightMotor.set(rightMotor);

    // Código para el funcionamiento del shooter
    if(m_Controller.getRightTriggerAxis() > 0.5){
      agarrar();
    }else if(m_Controller.getLeftTriggerAxis() > 0.5){
      lanzar();
    }else{
      // Detener motores
      stop();
    }

    if (m_Controller.getAButton()) {
      lanzar50();
    }

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

  public void agarrar(){
    m_shooter1.set(0.7D);
    m_shooter2.set(0.7D);
  }

  public void lanzar(){
    m_shooter1.set(-0.7D);
    m_shooter2.set(-0.7D);
  }

  public void lanzar50(){
     m_shooter1.set(-0.45D);
    m_shooter2.set(-0.45D);
    //:)
  }

  public void stop(){
    m_shooter1.set(0.0D);
    m_shooter2.set(0.0D);
  }

  public void autonomo(){
    System.out.println("Autonomo");
    m_leftMotor.set(0.5D);
    m_rightMotor.set(-0.5D);
  }
}
