// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX leaderMotor;
  private TalonFX followerMotor;

  private DigitalInput topLS;
  private DigitalInput hallEffect;

  public Climber(int leaderMotorID, int followerMotorID, int topLSID, int hallEffectID) {
    leaderMotor = new TalonFX(leaderMotorID);
    leaderMotor.getConfigurator().apply(ClimberConstants.leaderConfig);
    leaderMotor.getConfigurator().refresh(ClimberConstants.leaderConfig);
    
    followerMotor = new TalonFX(followerMotorID);
    followerMotor.getConfigurator().apply(ClimberConstants.followerConfig);
    followerMotor.getConfigurator().refresh(ClimberConstants.followerConfig);

    followerMotor.setControl(new Follower(leaderMotorID, MotorAlignmentValue.Aligned));

    topLS = new DigitalInput(topLSID);
    hallEffect = new DigitalInput(hallEffectID);
  }

  public void goToHeight(double position){
    leaderMotor.setControl(new MotionMagicVoltage(position));
  }

  public void stopMotors(){
    leaderMotor.stopMotor();
    followerMotor.stopMotor();
  }
  
  public double deadzone(double speed){
    if(Math.abs(speed) < 0.1){
      return 0;
    }
    else if(speed >= ClimberConstants.MAX_SPEED){
      return ClimberConstants.MAX_SPEED;
    }
    else if(speed <= -ClimberConstants.MAX_SPEED){
      return -ClimberConstants.MAX_SPEED;
    }
    else{
      return speed;
    }
  }

  public void setSpeed(DoubleSupplier speed){
    leaderMotor.set(speed.getAsDouble());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(topLS.get()){
      leaderMotor.setPosition(0);
      followerMotor.setPosition(0);
    }

    if(topLS.get() && (leaderMotor.get() > 0)){
      leaderMotor.set(0);
    }
    else if(hallEffect.get() && (leaderMotor.get() < 0)){
      leaderMotor.set(0);
    }
  }
}
