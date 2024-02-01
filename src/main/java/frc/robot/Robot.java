// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// To import these library, I had to go to "Manage Vendor Libraries" and then install Rev Robotics from the website https://software-metadata.revrobotics.com/REVLib-2024.json
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;


/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and motor controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  private static final int kMotorPort = 0;
  private static final int kMotorPort2 = 1;
  private static final int kDriveControllerJoystickPort = 0;
  private static final int kSystemControllerJoystickPort = 1;
  private static final int kEncoderPortA = 0;
  private static final int kEncoderPortB = 1;
  private static final int kArmCANId = 1;

  private PWMSparkMax m_motor;
  private PWMSparkMax m_motor2;
  private Joystick m_driveController;
  private Joystick m_systemController;
  private Encoder m_encoder;
  private CANSparkMax m_arm;

  @Override
  public void robotInit() {
    m_motor = new PWMSparkMax(kMotorPort);
    m_motor2 = new PWMSparkMax(kMotorPort2);
    m_arm = new CANSparkMax(kArmCANId, MotorType.kBrushless);
    m_driveController = new Joystick(kDriveControllerJoystickPort);
    m_systemController = new Joystick(kSystemControllerJoystickPort);
  
    m_encoder = new Encoder(kEncoderPortA, kEncoderPortB);
    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPR encoder.
    m_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Encoder", m_encoder.getDistance());
    SmartDashboard.putNumber("Drive Controller X (wheel 1):", m_driveController.getX());
    SmartDashboard.putNumber("Drive Controller Y (wheel 2):", m_driveController.getY());
    SmartDashboard.putNumber("System Controller Y (arm):", m_driveController.getY());
  }

  @Override
  public void teleopPeriodic() {
    m_motor.set(m_driveController.getY());
    m_motor2.set(m_driveController.getX());

    m_arm.set(m_systemController.getY());

  }
}
