// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import static edu.wpi.first.units.Units.Amps;

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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj2.command.Command;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX leaderMotor;
  private TalonFX followerMotor;
  TalonFX PivotMotor;
 
  private final PIDController pivotPID;
  private final DigitalInput LimitSwitch;

  private DigitalInput topLS;
  private DigitalInput hallEffect;

  private double setpoint;

   //Climber hook distances (position) for each rung of the tower
  public final double AUTO_L1 = 0;
  public final double L1RUNG = 0;
  public final double L2L3RUNG = 0;
  public final double RATCHET = 0;
  private final double MAX_SPEED = 0.5;

  final TalonFXConfiguration pivotConfig = new TalonFXConfiguration()
    .withMotorOutput(
      new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)).withCurrentLimits(
      new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(false)
        .withStatorCurrentLimit(Amps.of(0))
        .withSupplyCurrentLimitEnable(false)
        .withSupplyCurrentLimit(Amps.of(0))
        ).withSoftwareLimitSwitch(
        new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(50)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(0)
      );
        

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

  public Climber(int leaderMotorID, int followerMotorID, int topLSID, int hallEffectID, int pivotID, int LimitSwitchID) {
    PivotMotor = new TalonFX(pivotID); 
    pivotPID = new PIDController(0.2, 0, 0);
    LimitSwitch = new DigitalInput(LimitSwitchID);
    pivotPID.setTolerance(0);
    
    
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

  public boolean LimitSwitchPressed() {
    return !LimitSwitch.get();
  }

  public void goToAngle(double targetAngle) {
    double output = pivotPID.calculate(getPivotEncoder(), targetAngle);
    PivotMotor.set(output);
  }

  public double getPivotEncoder() {
    return PivotMotor.getRotorPosition().getValueAsDouble();
  }

  public boolean pidAtSetpoint() {
    return pivotPID.atSetpoint();
  }

  public void stopMotor() {
    PivotMotor.stopMotor();
  }

  public double getLeaderPosition(){
    return leaderMotor.getPosition().getValueAsDouble();
  }

  public double getFollowerPosition(){
    return followerMotor.getPosition().getValueAsDouble();
  }

  public boolean getHallEffectValue(){
    return !hallEffect.get();
  }
  
  public boolean isAtSetpoint(){
    return Math.abs(getLeaderPosition() - setpoint) >= 0;
  }

  public void goToHeight(double position){
    setpoint = position;
    leaderMotor.setControl(new MotionMagicVoltage(position));
  }

  public void stopMotors(){
    leaderMotor.stopMotor();
  }

  public Command clockwise() {
    return this.run(() -> PivotMotor.set(0.1));
  }

  public Command counterClockWise() {
    return this.run(() -> PivotMotor.set(-0.1));
  }

  public Command stopPivot() {
    return this.run(() -> stopMotor());
  }

  public Command up(){
    return this.run(() -> leaderMotor.set(MAX_SPEED));
  }

  public Command down(){
    return this.run(() -> leaderMotor.set(-MAX_SPEED));
  }

  public Command stop(){
    return this.run(() -> stopMotors());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(LimitSwitchPressed()) {
      PivotMotor.setPosition(0);
    }
    
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
    SmartDashboard.putBoolean("[C] Hall Effect", getHallEffectValue());
    SmartDashboard.putNumber("[C] Setpoint", setpoint);

    SmartDashboard.putBoolean("Limit Switch", LimitSwitchPressed());
    SmartDashboard.putNumber("Pivot Position", getPivotEncoder());
  }
}
