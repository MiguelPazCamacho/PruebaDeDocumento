/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Robot_Chasis extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
TalonSRX motor_r,motor_l;
public Robot_Chasis (){
  motor_l=RobotMap.mn_chasis_fl;
  motor_r=RobotMap.mn_chasis_fr;
}
  @Override
  public void initDefaultCommand() {
   
        // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public static void Robot_maindrive(){

    /*Robot.m_oi.stick.getRawAxis(5);
    RobotMap.mn_chasis_fr.set(ControlMode.PercentOutput,.25);
    RobotMap.mn_chasis_fl.set(ControlMode.PercentOutput,.25);
*/
  }

  public static void Robot_maindrive_stop(){
    
  }
  public  void maindrive_move(){
    System.out.println("aidaaaaa");
    motor_l.set(ControlMode.PercentOutput,.5);
    motor_r.set(ControlMode.PercentOutput,.5);
  }

  
}
