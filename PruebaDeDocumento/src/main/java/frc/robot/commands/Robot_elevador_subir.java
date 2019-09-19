/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Robot_Elevador;

public class Robot_elevador_subir extends Command {
  public Robot_elevador_subir() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.robot_Elevador);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   Robot.robot_Elevador.subir();


  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //timers,sensores
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.robot_Elevador.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  end();
  }
}
