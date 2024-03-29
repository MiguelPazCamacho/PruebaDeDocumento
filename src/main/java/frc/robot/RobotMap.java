/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;



/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  private static final int id_mn_chasis_fl= 1;
  private static final int id_mn_chasis_bl= 2;
  
  private static final int id_mn_chasis_fr= 3;
  private static final int id_mn_chasis_br= 4;

  private static final int id_motor_pid = 6;

  private static final int puerto_encoder_A= 0 ;
  private static final int puerto_encoder_B= 1 ;

  
  
  public static TalonSRX mn_chasis_fl =new TalonSRX(id_mn_chasis_fl);
  public static TalonSRX mn_chasis_bl = new TalonSRX(id_mn_chasis_bl);
  public static TalonSRX mn_chasis_fr = new TalonSRX(id_mn_chasis_fr);
  public static TalonSRX mn_chasis_br = new TalonSRX(id_mn_chasis_br);

  public static CANSparkMax motor_pid= new CANSparkMax(id_motor_pid, MotorType.kBrushless) ;

  public static Encoder Enc_chasis = new Encoder(puerto_encoder_A, puerto_encoder_B);

  public static void init(){

/*
    //Elevador
    TalonSRX mn_elevador_l= new TalonSRX (id_mn_elevador_l);
    mn_elevador_l.setNeutralMode(NeutralMode.Brake);
    mn_elevador_l.set(ControlMode.PercentOutput, 0);
    mn_elevador_l.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    mn_elevador_l.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    
    TalonSRX mn_elevador_r= new TalonSRX(id_mn_elevador_r);
    mn_elevador_r.setNeutralMode(NeutralMode.Brake);
    mn_elevador_r.set(ControlMode.PercentOutput, 0);
    mn_elevador_r.setInverted(true);
    mn_elevador_r.follow(mn_elevador_l);
*/

    motor_pid.setIdleMode(IdleMode.kCoast);

    //Chasis
    //chasis lado izquierdo
    
    mn_chasis_fl.setNeutralMode(NeutralMode.Brake);
    mn_chasis_fl.set(ControlMode.PercentOutput,0);

    
    mn_chasis_bl.setNeutralMode(NeutralMode.Brake);
    mn_chasis_bl.set(ControlMode.PercentOutput,0);
    mn_chasis_bl.follow(mn_chasis_fl);
    
    //chasis lado derecho
    mn_chasis_fr.setNeutralMode(NeutralMode.Brake);
    mn_chasis_fr.set(ControlMode.PercentOutput,0);
    mn_chasis_fr.setInverted(true);

    
    mn_chasis_br.setNeutralMode(NeutralMode.Brake);
    mn_chasis_br.set(ControlMode.PercentOutput,0);
    mn_chasis_br.follow(mn_chasis_fr);
    mn_chasis_br.setInverted(true);

  }


  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
