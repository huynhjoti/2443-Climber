// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.climber;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusSignal;
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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  private TalonFX leaderMotor;
  private TalonFX followerMotor;
  private TalonFX PivotMotor;
 
  private DigitalInput pivotLS;
  private DigitalInput topLS;
  private DigitalInput hallEffect;

   //Climber hook distances (position) for each rung of the tower
  public final double AUTO_L1 = 0;
  public final double L1RUNG = 0;
  public final double L2L3RUNG = 0;
  public final double RATCHET = 0;
  private final double MAX_SPEED = 0.5;
  
  //Pivot Position to make the climber vertical
  public final double VERTICAL_ANGLE = 0;

  private final double CMARGIN = 0;
  private final double PMARGIN = 0;

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
        .withForwardSoftLimitThreshold(49)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(0)
         ).withMotionMagic(new MotionMagicConfigs()
         .withMotionMagicCruiseVelocity(0)
         .withMotionMagicAcceleration(0)
         .withMotionMagicExpo_kV(0)
         .withMotionMagicExpo_kA(0)
         ).withSlot0(new Slot0Configs()
         .withKS(0)
         .withKG(0)
         .withKV(0)
         .withKA(0)
         .withKP(0)
         .withKI(0)
         .withKD(0)
         .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
         .withGravityType(GravityTypeValue.Arm_Cosine)
         );   

  final TalonFXConfiguration leaderConfig = new TalonFXConfiguration()
    .withMotorOutput(
      new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
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
        .withForwardSoftLimitThreshold(49)
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
        .withNeutralMode(NeutralModeValue.Brake)
      )
    .withCurrentLimits(
      new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(false)
        .withStatorCurrentLimit(Amps.of(0))
        .withSupplyCurrentLimitEnable(false)
        .withSupplyCurrentLimit(Amps.of(0))
      );

  private MotionMagicVoltage profileReq = new MotionMagicVoltage(0);

  private MotionMagicVoltage pivotMagic = new MotionMagicVoltage(0);

  private double climbSetpoint = 0.0;
  private double pivotSetpoint = 0.0;

  public Climber(int leaderMotorID, int followerMotorID, int topLSID, int hallEffectID, int pivotID, int pivotLSID) {
    PivotMotor = new TalonFX(pivotID); 
    pivotLS = new DigitalInput(pivotLSID);
    
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

  public boolean pivotLSPressed() {
    return !pivotLS.get();
  }

  public boolean atPosition(){
    return MathUtil.isNear(pivotSetpoint, getPivotEncoder(), PMARGIN);
  }

  public void setClimberSetpoint(double newSetpoint){
    climbSetpoint = newSetpoint;
  }

  public void setPivotSetpoint(double newSetpoint){
    pivotSetpoint = newSetpoint;
  }

  public double getPivotEncoder() {
    return PivotMotor.getRotorPosition().getValueAsDouble();
  }

  public void stopMotor() {
    PivotMotor.stopMotor();
  }

  public double getLeaderPosition(){
    return leaderMotor.getPosition().getValueAsDouble();
  }

  public boolean getHallEffectValue(){
    return !hallEffect.get();
  }

  public void stopMotors(){
    leaderMotor.stopMotor();
  }

  //Pivot Manual Commands
  public Command clockwise() {
    return this.run(() -> PivotMotor.set(0.1));
  }

  public Command counterClockWise() {
    return this.run(() -> PivotMotor.set(-0.1));
  }

  public Command stopPivot() {
    return this.run(() -> stopMotor());
  }

  //Elevator Manual Commands
  public Command up(){
    return this.run(() -> leaderMotor.set(MAX_SPEED));
  }

  public Command down(){
    return this.run(() -> leaderMotor.set(-MAX_SPEED));
  }

  public Command stop(){
    return this.run(() -> stopMotors());
  }

  public Trigger topLSTrigger(){
    return new Trigger(() -> topLS.get());
  }

  public Trigger hallEffectTrigger(){
    return new Trigger(() -> hallEffect.get());
  }

  public Trigger atPositionTrigger(){
    return new Trigger(() -> MathUtil.isNear(climbSetpoint, getLeaderPosition(), CMARGIN));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(pivotLSPressed()) {
      PivotMotor.setPosition(0);
    }

    if(pivotLSPressed() && PivotMotor.get() < 0){
      stopMotor();
    }

    if(topLS.get()){
      leaderMotor.setPosition(0);
    }

    if(topLS.get() && (leaderMotor.get() > 0)){
      stopMotors();
    }
    else if(hallEffect.get() && (leaderMotor.get() < 0)){
      stopMotors();
    }

    leaderMotor.setControl(profileReq.withPosition(climbSetpoint));
    PivotMotor.setControl(pivotMagic.withPosition(pivotSetpoint));

    SmartDashboard.putNumber("[C] Leader Position", getLeaderPosition());
    SmartDashboard.putBoolean("[C] Hall Effect", getHallEffectValue());

    SmartDashboard.putBoolean("Limit Switch", pivotLSPressed());
    SmartDashboard.putNumber("Pivot Position", getPivotEncoder());
  }
}
