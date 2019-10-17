/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/**
 * Add your docs here.
 */
public class Robot_Elevador extends Subsystem {
    
    private CANPIDController m_pidController;
    private CANEncoder m_encoder;
    CANDigitalInput LS_DN;
	  CANDigitalInput LS_UP;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, Power_E;
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public Robot_Elevador (){

    m_pidController = RobotMap.motor_pid.getPIDController();

    // Encoder object created to display position values
    m_encoder = RobotMap.motor_pid.getEncoder();
    LS_UP= RobotMap.motor_pid.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    LS_DN =RobotMap.motor_pid.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
    LS_DN.enableLimitSwitch(true);
    LS_UP.enableLimitSwitch(false);
    // PID coefficients
    kP = 1; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);





  }

  public void Mover_A(double vueltas){
    m_pidController.setReference(vueltas, ControlType.kPosition);
      System.out.println(m_encoder.getPosition());
  }

  public void Main_move(){

    Power_E=-(Robot.m_oi.stk_sub.getRawAxis(1));

    RobotMap.motor_pid.set(Power_E);

    
    
  }
}
