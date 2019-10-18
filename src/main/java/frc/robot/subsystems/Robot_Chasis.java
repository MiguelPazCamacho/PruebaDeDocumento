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

  //Estos dos motores son los motores master de cada lado
  TalonSRX motor_r,motor_l;
  //Las primeras tres variables de poder son las que recibimos del Joystick y
  //las últimas variables las usamos para modificar los valores de acuerdo a nuestras
  //condiciones.
  double Power_f,Power_b, Power_s, Power_t, Tolerance_Triggers,Tolerance_Stick, Aceleracion,Giro;
  //Estas variables son las que va a recibir el motor al final, esto depende de la velocidad y
  //la rotación que hayamos implicado.
  double Power_Chasis_L, Power_Chasis_R;

  private double power;
	private double distance;
	/////para el pid/////////////
	private double integral_val=0;
	private double pre_input = 0;
	private final double MAX_INTEGRAL_VAL = 0.3;
	private final double MIN_INTEGRAL_VAL = -0.3;
	private final double MAX_PID_VAL = 0.5;
  private final double MIN_PID_VAL = -0.5;
  private final double kp=.0025, ki=0, kd=0;
	/////////////////////////////
	

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
  }
  public static void Avanzar(){
    
  }
  public double PID_fun(double setpoint,double actual_point,double kp,double ki,double kd){
    double output_val = 0;  //la salida
      double dt = 0.1;  //tiempo que tarda entre medidas
      double epsilon = 2;  //tolerancia
      
    //para tener un pid variable a cierto rango
    double kp2 = kp*3;
    double kd2 = kd*3;
      double range_tol = 10;
      
      //obtiene el error
      double error = setpoint - actual_point;
      if(Math.abs(error) <= epsilon){
        error = 0;
        integral_val=0;  //resetea
        pre_input=0; //resetea
        return(0);
      }else{
        ///calcula la integral
        //considerando rectangulos pequenos donde dt es lo ancho y el error lo largo
        integral_val=integral_val + (error*dt);  //el area al graficar la variable contra tiempo(y se suma con lo que ya hay)
        ////para limitar la integral
        if(integral_val> (MAX_INTEGRAL_VAL)){
          integral_val = (MAX_INTEGRAL_VAL);}
        if(integral_val< (MIN_INTEGRAL_VAL)){
          integral_val = (MIN_INTEGRAL_VAL);}
      }
               
      //calcula la derivada
      double derivative_val = (actual_point - pre_input)/dt;  //la funcion dx/dt donde dx es la diferencia entre el ultimo error y el nuevo

      if( Math.abs(error)>range_tol){   //si esta muy lejos
        //calculates the output
        output_val = (kp2*error) + (ki*integral_val) - (kd2*derivative_val);
      }else{  //si esta muy cerca
        //calculates the output
        output_val = (kp*error) + (ki*integral_val) - (kd*derivative_val);
      }
     
      //Saturation filter, para asegurar que no pase los valores maximos ni minimos
      //sirve como una rampa tambien para evitar cambios bruscos
      if(output_val > MAX_PID_VAL){output_val=MAX_PID_VAL;}
      if(output_val < MIN_PID_VAL){output_val=MIN_PID_VAL;}

      //update error, guardando el nuevo error que sera el viejo
      pre_input =actual_point;

      return(output_val); ///regresa el rewsultado del pid
  }

  protected void DriveMove(double Z3,double translationX){
		double SENSIBILITY = 0.5,
		SENSIBILITY2 = 0.7,
		Left_Speed=0,
		Right_Speed=0;
		Z3 = (Z3 * -1);
		
		///ecuacion del drive
    	if((Z3 > 0) || (Z3 < 0)){
    		if(translationX>SENSIBILITY) {
    			translationX=SENSIBILITY;
    		}
    		if(translationX<-SENSIBILITY) {
    			translationX=-SENSIBILITY;
    		}
    		Left_Speed = (Z3 + (translationX));
    		Right_Speed =(Z3 + (translationX));
    	}else{ //giros rapidos
    		if(translationX>SENSIBILITY2) {
    			translationX=SENSIBILITY2;
    		}
    		if(translationX<-SENSIBILITY2) {
    			translationX=-SENSIBILITY2;
    		}
    		Left_Speed = translationX;
        	Right_Speed = -translationX;
    	}
    	
    	///por si se pasa (seguridad)
    	if(Left_Speed >1){Left_Speed=1;}
    	if(Right_Speed >1){Right_Speed=1;} 
    	if(Left_Speed <-1){Left_Speed=-1;}
    	if(Right_Speed <-1){Right_Speed=-1;} 
		
		RobotMap.mn_chasis_fl.set(ControlMode.PercentOutput,Left_Speed);
		RobotMap.mn_chasis_fr.set(ControlMode.PercentOutput,Right_Speed);
		RobotMap.mn_chasis_bl.set(ControlMode.PercentOutput,Left_Speed);
		RobotMap.mn_chasis_br.set(ControlMode.PercentOutput,Right_Speed);
		
	//	System.out.println(Left_Speed +"---"+Right_Speed);
	//	System.out.println(RobotMap.left_enc.getDistance() + RobotMap.right_enc.getDistance());
	
		System.out.println(Left_Speed +"---"+Right_Speed);
		//System.out.println("angulo="+ Robot_Heading.ahrs.getAngle());

		
	}

  public void Mover_Chasis_Distancia(double setpoint){
    double actual_point= RobotMap.Enc_chasis.getDistance();
    PID_fun(setpoint, actual_point , kp, ki, kd);
  }
  public void Robot_maindrive_stop(){
    motor_l.set(ControlMode.PercentOutput,0);
    motor_r.set(ControlMode.PercentOutput,0);
  }
  public  void maindrive_move(){
    
    
    //Esta es nuestra tolerancia en tanto triggers y stick
    Tolerance_Stick = .20;
    Tolerance_Triggers= .10;
    
    //Estas son nuestras variables que guardamos de los sticks
    Power_f = Robot.m_oi.stick.getRawAxis(3);
    Power_b = -Robot.m_oi.stick.getRawAxis(2);
    Power_s = Robot.m_oi.stick.getRawAxis(0);
    
    //Estas condicionales sirven para dar la aceleracion despues de nuestra tolerancia
    //pero tambien si tenemos los dos triggers presionados se le dara preferencia a la instruccion
    // de avanzar y si ninguno esta presionado se mantendra la aceleración en 0.

    if (Power_f >= Tolerance_Triggers){
    Aceleracion=Power_f;
    }else if (Power_b <= -Tolerance_Triggers){
    Aceleracion=Power_b;
    } else{
    Aceleracion=0;
    } 
    //En esta condicional se compara que nuestro poder en el stick sea mayor que nuestra tolerancia
    //si no cumple con nuestra tolerancia el poder del stick será 0.
    if (Power_s >= Tolerance_Stick || Power_s <=-Tolerance_Stick){
    Giro= Power_s;
      //La variable Power_s pasa directamente porque cumple con nuestra tolerancia.
    }else{
    Giro=0;
    }

    //Aquí en estas dos variables se imprime la velocidad de cada lado
    //En este caso estamos sumando la aceleración del trigger más el valor de rotación del stick
    //La última parte donde estamos multiplicando ese valor de .3 es para reducir la fuerza del giro.

    Power_Chasis_L = (Aceleracion*.75)+(Giro*.4);
    Power_Chasis_R = (Aceleracion*.75)-(Giro*.4);

    

    //Al momento de sumar la aceleración y el poder de la rotación nos pueden dar valores 
    //mayores a 1 o -1, por eso estamos condicionando que nuestros dos valores del Chasis 
    //no pueden ser mayor que 1 o -1.
    if (Power_Chasis_L > 1){
    Power_Chasis_L=1;
    }else if (Power_Chasis_L <- 1){
      Power_Chasis_L=-1;
    }

    if (Power_Chasis_R > 1){
      Power_Chasis_R=1;
      }
    else if (Power_Chasis_R<-1){
    Power_Chasis_R=-1;
    }

    System.out.println(Power_Chasis_L);

    //Al final de todo nuestro proceso de condicionales y tolrerancias aplicamos 
    //nuestros valores correctos al motor.

    motor_l.set(ControlMode.PercentOutput, Power_Chasis_L);
    motor_r.set(ControlMode.PercentOutput, Power_Chasis_R);
  

  
  
    
  /* código para mover chasis tanque convencional  
  power_l = -Robot.m_oi.stick.getRawAxis(1);
    power_r = -Robot.m_oi.stick.getRawAxis(5);
    

    if (power_l <= 0.25 && power_l >= -0.25){
    power_l=0;
  
    }
   
    if (power_r<= 0.25 && power_r>= -0.25){
     power_r=0;
    }

    System.out.println("R="+power_r);
    System.out.println("L="+power_l);
    
    RobotMap.mn_chasis_fl.set(ControlMode.PercentOutput,power_l);
    RobotMap.mn_chasis_fr.set(ControlMode.PercentOutput,power_r);
    */
  }

  
}
