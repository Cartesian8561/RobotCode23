// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimitConstants;;

public class ArmSubsystem extends SubsystemBase {
 
  private static final CANSparkMax angleMotor = new CANSparkMax(10, MotorType.kBrushed);
  private static final WPI_TalonFX lengthMotor = new WPI_TalonFX(6);
  private static final DigitalInput limitSwitch = new DigitalInput(DriveConstants.limitSwitchPort);
  
  private static Thread thread;
  private static final AbsoluteEncoder angleEncoder = angleMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private static final CartesianEncoder distanceTranslator = new CartesianEncoder(1.0, angleEncoder.getPosition());
  //sallamasyon şeyleri kontrol et


  public ArmSubsystem() {
    try{
      thread = new Thread(new Runnable() {
  
        @Override
        public void run() {
          updateEncoder();
        }
      });
    }catch(Exception e){System.out.println("ananiskm");}
    thread.start();
    lengthMotor.set(ControlMode.PercentOutput, 0);
    lengthMotor.configFactoryDefault();
    lengthMotor.setNeutralMode(NeutralMode.Brake);
    lengthMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0,0);
    lengthMotor.getSensorCollection().setIntegratedSensorPosition(0.0, 0);
  }


/* 
  public double getAngle(){
    return getArmAngle() * ArmConstants.rotDis;
  }
*/

  public double getArmDistance(){
    return lengthMotor.getSelectedSensorPosition();
  }

  public double getArmEncoderDistance(){
    return getArmDistance() / ((double) 2048);
  }

  public double getArmLinearVelocity(){
    return lengthMotor.getSensorCollection().getIntegratedSensorVelocity();
  }

  public static double getArmAngle(){
    return distanceTranslator.getDistance(angleEncoder.getPosition());
  }

  public double getArmAngularVelocity(){
    return angleEncoder.getVelocity();
  }

  public double getArmDegrees(){
    return 2.7/90 * getArmAngle(); 

  }

  public void resetEncoder(){
    distanceTranslator.resetRotation(angleEncoder.getPosition());
  }

  public void setAngleMotorVolts(double volts){
    if (volts < 0 && getArmAngle() < LimitConstants.positiveAnglePoint){
      angleMotor.setVoltage(volts);
      return;
    }else if(volts > 0 && !limitSwitch.get() && getArmAngle() > LimitConstants.negativeAnglePoint){
      angleMotor.setVoltage(volts);
      return;
    }else{
      angleMotor.setVoltage(0.0);
    }
  }

  public void setEncoderValue(int rotation, double position){
    distanceTranslator.setValues(rotation, position, angleEncoder.getPosition());
  }
  public void setLengthMotorVolts(double volts){
    if(volts > 0 && getArmEncoderDistance() > LimitConstants.positiveDistancePoint){
      lengthMotor.setVoltage(0.0);
      return;
    }else if(volts < 0 && getArmEncoderDistance() < LimitConstants.negativeDistancePoint){
      lengthMotor.setVoltage(0.0);
      return;
    }else{
      lengthMotor.setVoltage(volts);
      return;
    }
  }


  public static double getScaler(){
    double scaler = (2.7 - getArmAngle())/2.7;
    if(scaler > 0.6){
      return 0.6;
    }
    return scaler;
  }
  
  /* 
  public void setAngleMotor(double speed){
    angleMotor.set(speed);
  }
  */

  private void setArmAngle(double speed){
    if(speed > 0 && !limitSwitch.get() && Math.abs(speed) > 0.12){
      angleMotor.set(speed);
    }else if (speed < 0 && Math.abs(speed) > 0.12){
      angleMotor.set(speed);
    }else{
      angleMotor.set(0);
    }
  }

  private void setArmLength(double speed){
    if(Math.abs(speed) > 0.12){
      if(speed > 0 //&& getArmEncoderDistance() < LimitConstants.positiveDistancePoint){
      ){
        lengthMotor.set(ControlMode.PercentOutput, speed * 1.1);
      }else if(speed < 0 //&& getArmEncoderDistance() > LimitConstants.negativeDistancePoint){
        
      ){lengthMotor.set(ControlMode.PercentOutput, speed / 1.2);
      }else if(speed == 0){
        lengthMotor.set(ControlMode.PercentOutput, speed);
      }
      //lengthMotor.setVoltage(speed);
    }else{
      lengthMotor.setVoltage(0.0);
    }
  }

  public void setArm(double armAngleSpeed, double armLengthSpeed){
    updateEncoder();
    setArmAngle(armAngleSpeed);
    setArmLength(armLengthSpeed);
  }

  public void zeroAllOutputs(){
    //cubeMotor.setVoltage(0);
    angleMotor.setVoltage(0);
    angleMotor.set(0);
    //coneMotor.setVoltage(0);
  }

  public double getArmDistanceScaler(){
    double scaler = -(getArmDistance() - 13000) /80000;
    if(scaler < 0.3){
      return 0.3;
    }
    return scaler;
  }

  /* 
  public void setCubeMotorVolts(double volts){
    cubeMotor.setVoltage(volts);
  }

  public void setConeMotorVolts(double volts){
    coneMotor.setVoltage(volts);
  }
  

  public void setCubeMotor(double speed){
    cubeMotor.set(speed);
  }

  public void setConeMotor(double speed){
    coneMotor.set(speed);

  }

*/


  public void updateEncoder(){
    distanceTranslator.updateDistance(angleEncoder.getPosition());
  }

  @Override
  public void periodic() {
    //distanceTranslator.updateNegativeDistance(angleEncoder.getPosition(), angleMotor.getAppliedOutput());

    if(limitSwitch.get()){
      resetEncoder();
    }

    SmartDashboard.putNumber("arm angle", getArmAngle());
    SmartDashboard.putNumber("arm length", getArmEncoderDistance());
    SmartDashboard.putNumber("voltage", angleMotor.getAppliedOutput());
    SmartDashboard.putNumber("angle scaler", getScaler());
    SmartDashboard.putNumber("distance scaler", getArmDistanceScaler());
    SmartDashboard.putBoolean("limit switch",  limitSwitch.get());
    SmartDashboard.putNumber("arm angle motor", angleMotor.getAppliedOutput());
  }
}
