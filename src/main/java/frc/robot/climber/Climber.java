// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import static edu.wpi.first.units.Units.Amps;

import java.util.function.DoubleSupplier;
import java.util.spi.CurrencyNameProvider;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX leaderMotor;
  private TalonFX followerMotor;

  private DigitalInput topLS;
  private DigitalInput hallEffect;

   //Climber hook distances (position) for each rung of the tower
  public final double AUTO_L1 = 0;
  public final double L1RUNG = 0;
  public final double L2L3RUNG = 0;
  public final double RATCHET = 0;
  private final double MAX_SPEED = 0.5;

  final TalonFXConfiguration leaderConfig = new TalonFXConfiguration()
    .withMotorOutput(
      new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.CounterClockwise_Positive))
    .withCurrentLimits(
      new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(false)
        .withStatorCurrentLimit(Amps.of(0))
        .withSupplyCurrentLimitEnable(false)
        .withSupplyCurrentLimit(Amps.of(0))
        )
    .withSoftwareLimitSwitch(
      new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(50)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(0)
        )
    .withSlot0(
      new Slot0Configs()
        .withKS(0)
        .withKG(0)
        .withKV(0)
        .withKA(0)
        .withKP(0)
        .withKI(0)
        .withKD(0)
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
    )
    .withMotionMagic(
      new MotionMagicConfigs()
        .withMotionMagicExpo_kV(0)
        .withMotionMagicExpo_kA(0)
        .withMotionMagicCruiseVelocity(0)
        .withMotionMagicAcceleration(0)
        );

  final TalonFXConfiguration followerConfig = new TalonFXConfiguration()
    .withMotorOutput(
      new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
      )
    .withCurrentLimits(
      new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(false)
        .withStatorCurrentLimit(Amps.of(0))
        .withSupplyCurrentLimitEnable(false)
        .withSupplyCurrentLimit(Amps.of(0))
      )
    .withSoftwareLimitSwitch(
      new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(50)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(0)
    );

  public Climber(int leaderMotorID, int followerMotorID, int topLSID, int hallEffectID) {
    leaderMotor = new TalonFX(leaderMotorID);
    leaderMotor.getConfigurator().apply(leaderConfig);
    leaderMotor.getConfigurator().refresh(leaderConfig);
    
    followerMotor = new TalonFX(followerMotorID);
    followerMotor.getConfigurator().apply(followerConfig);
    followerMotor.getConfigurator().refresh(followerConfig);

    followerMotor.setControl(new Follower(leaderMotorID, MotorAlignmentValue.Opposed));

    topLS = new DigitalInput(topLSID);
    hallEffect = new DigitalInput(hallEffectID);
  }

  public double getLeaderPosition(){
    return leaderMotor.getPosition().getValueAsDouble();
  }

  public double getFollowerPosition(){
    return followerMotor.getPosition().getValueAsDouble();
  }

  public void goToHeight(double position){
    leaderMotor.setControl(new MotionMagicVoltage(position));
  }

  public void stopMotors(){
    leaderMotor.stopMotor();
    followerMotor.stopMotor();
  }
  
  //Deadzone of the controller to set the speed of the climber
  public double deadzone(double speed){
    if(Math.abs(speed) < 0.1){
      return 0;
    }
    else if(speed >= MAX_SPEED){
      return MAX_SPEED;
    }
    else if(speed <= MAX_SPEED){
      return -MAX_SPEED;
    }
    else{
      return speed;
    }
  }

  public void setSpeed(DoubleSupplier speed){
    leaderMotor.set(deadzone(speed.getAsDouble()));
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

    SmartDashboard.putNumber("[C] Leader Position", getLeaderPosition());
    SmartDashboard.putNumber("[C] Follower Position", getFollowerPosition());

  }
}
