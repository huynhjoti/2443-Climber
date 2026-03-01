// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//CLIMBER TUNING ORDER
//TODO: 1. Determine the max and min encoder values for the climber
//TODO: 2. Determine the kS, kG values for Motion Magic of the climber
//TODO: 3. Determine the Cruise Velocity and Cruise Acceleration values for Motion Magic of the climber
//TODO: 4. Determine the kP and/or kI/kD values for Motion Magic of the climber
//TODO: 5. Determine the distances (position) for each rung of the tower for the climber
//TODO: 6. Test level1 Command and tune as necessary

//PIVOT TUNING ORDER
//TODO: 1. Determine the max and min encoder values for the pivot
//TODO: 2. Determine the kS value for Motion Magic of the pivot
//TODO: 3. Determine the Cruise Velocity and Cruise Acceleration values for Motion Magic of the pivot
//TODO: 4. Determine the kP and/or kI/kD value for Motion Magic of the pivot
//TODO: 5. Determine the angle for the pivot to be vertical
//TODO: 6. Test verticalPivot Command and tune as necessary

//WHEN ALL POSITIONS AND COMMANDS ARE TUNED
//TODO: 1. Integrate the climber and pivot commands to create a sequential command sequence for auto and teleop.
//TODO: 2. Test the sequential command sequence and tune as necessary.
//TODO: 3. Tune the SUPPLY CURRENT LIMIT for the climber and pivot
//TODO: 4. Determine if STATOR CURRENT LIMIT is necessary for the climber and pivot and tune as necessary.

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
import com.ctre.phoenix6.controls.VoltageOut;
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
  private TalonFX leaderMotor;
  private TalonFX followerMotor;
  private TalonFX PivotMotor;
 
  private DigitalInput pivotLS;
  private DigitalInput topLS;
  private DigitalInput hallEffect;
  
  //Max and min encoder values for the pivot and climber
  //This is used to set the setpoint limits for the climber and pivot. 
  //These values should be determined through testing.
  private double minimumHeight = 0;
  private double maximumHeight = 0;
  private double minimumAngle = 0;
  private double maximumAngle = 0;

   //Climber hook distances (position) for each rung of the tower
  public final double AUTO_L1 = 0;
  public final double L1RUNG = 0;
  public final double L2L3RUNG = 0;
  public final double RATCHET = 0;
  private final double MANUAL_SPEED = 0.5;
  
  //Pivot Position to make the climber vertical
  public final double VERTICAL_ANGLE = 0;

  //Margins for the triggers to determine if the climber/pivot is at the setpoint. 
  private final double CMARGIN = 0;
  private final double PMARGIN = 0;

  final TalonFXConfiguration pivotConfig = new TalonFXConfiguration()
    .withMotorOutput(
      new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        ).withCurrentLimits(
      new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(false)
        .withStatorCurrentLimit(Amps.of(0))
        .withSupplyCurrentLimitEnable(false)
        .withSupplyCurrentLimit(Amps.of(0))
        ).withSoftwareLimitSwitch(
        new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(0)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(0)
         ).withMotionMagic(new MotionMagicConfigs()
         .withMotionMagicCruiseVelocity(0)
         .withMotionMagicAcceleration(0)
         .withMotionMagicExpo_kV(0) // Use 0.12 when testing
         .withMotionMagicExpo_kA(0) // Use o.1 when testing
         ).withSlot0(new Slot0Configs()
         .withKS(0)
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
        .withForwardSoftLimitThreshold(0)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(0)
        )
    .withSlot0(
      new Slot0Configs()
        .withKS(0)
        .withKG(0)
        .withKP(0)
        .withKI(0)
        .withKD(0)
        .withGravityType(GravityTypeValue.Elevator_Static)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
    )
    .withMotionMagic(
      new MotionMagicConfigs()
        .withMotionMagicExpo_kV(0) // Use 0.12 when testing
        .withMotionMagicExpo_kA(0) // Use 0.1 when testing
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

  //Motion Magic Control Modes for the climber and pivot.
  private MotionMagicVoltage profileReq = new MotionMagicVoltage(0);
  private MotionMagicVoltage pivotMagic = new MotionMagicVoltage(0);

  //VoltageOut Control Modes for the climber and pivot.
  private VoltageOut climbVoltageReq = new VoltageOut(0);
  private VoltageOut pivotVoltageReq = new VoltageOut(0);

  //Motion Magic Setpoints for the climber and pivot.
  private double climbSetpoint = 0.0;
  private double pivotSetpoint = 0.0;

  //VoltageOut Setpoints for tuning kS and/or kG for the climber and pivot.
  private double climbVoltage = 0.0;
  private double pivotVoltage = 0.0;

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

  public boolean topLSPressed() {
    return !topLS.get();
  }

  public boolean hallEffectPressed(){
    return !hallEffect.get();
  }

  public double getPivotEncoder() {
    return PivotMotor.getRotorPosition().getValueAsDouble();
  }

  public double getLeaderPosition(){
    return leaderMotor.getPosition().getValueAsDouble();
  }

  private void setClimberSetpoint(double newSetpoint){
    double newHeight = MathUtil.clamp(newSetpoint, minimumHeight, maximumHeight);
    climbSetpoint = newHeight;
  }

  private void setPivotSetpoint(double newSetpoint){
    double newAngle = MathUtil.clamp(newSetpoint, minimumAngle, maximumAngle);
    pivotSetpoint = newAngle;
  }

  private void stopMotor() {
    PivotMotor.stopMotor();
  }

  private void setClimbVoltage(double voltage){
    double newVoltage = MathUtil.clamp(voltage, -12, 12);
    climbVoltage = newVoltage;
  }

  private void setPivotVoltage(double voltage){
    double newVoltage = MathUtil.clamp(voltage, -12, 12);
    pivotVoltage = newVoltage;
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
    return this.run(() -> leaderMotor.set(MANUAL_SPEED));
  }

  public Command down(){
    return this.run(() -> leaderMotor.set(-MANUAL_SPEED));
  }

  public Command stop(){
    return this.run(() -> stopMotors());
  }

  public Trigger topLSTrigger(){
    return new Trigger(() -> topLSPressed());
  }

  public Trigger hallEffectTrigger(){
    return new Trigger(() -> hallEffectPressed());
  }

  public Trigger pivotLSTrigger(){
    return new Trigger(() -> pivotLSPressed());
  }

  public Trigger climbAtPositionTrigger(){
    return new Trigger(() -> MathUtil.isNear(climbSetpoint, getLeaderPosition(), CMARGIN));
  }

  public Trigger pivotAtPositionTrigger(){
    return new Trigger(() ->MathUtil.isNear(pivotSetpoint, getPivotEncoder(), PMARGIN));
  }

  //Use this command to tune kS and kG for Motion Magic.
  public Command climbVoltage(){
    return this.run(() -> setClimbVoltage(0.01));
  }

  //Used to stop the climber when using VoltageOut control mode for tuning
  public Command zeroVoltage(){
    return this.run(() -> setClimbVoltage(0));
  }

  //Use this command to tune kS for Motion Magic of the pivot.
  public Command pivotVoltage(){
    return this.run(() -> setPivotVoltage(0.01));
  }

  //Used to stop the pivot when using VoltageOut control mode for tuning
  public Command zeroPivotVoltage(){
    return this.run(() -> setPivotVoltage(0));
  }

  public Command verticalPivot(){
    return this.run(() -> setClimberSetpoint(VERTICAL_ANGLE)).until(pivotAtPositionTrigger());
  }

  public Command level1(){
    return this.run(() -> setClimberSetpoint(AUTO_L1)).until(climbAtPositionTrigger());
  }

  //This method display important info to the tester
  private void troubleshoot(){
    SmartDashboard.putNumber("[C] Leader Position", getLeaderPosition());
    SmartDashboard.putNumber("[C] Elevator Setpoint", climbSetpoint);
    SmartDashboard.putBoolean("[C] Hall Effect", hallEffectPressed());
    SmartDashboard.putBoolean("[C] Climb At Setpoint", climbAtPositionTrigger().getAsBoolean());

    SmartDashboard.putBoolean("[C] Pivot Limit Switch", pivotLSPressed());
    SmartDashboard.putNumber("[C] Pivot Position", getPivotEncoder());
    SmartDashboard.putNumber("[C] Pivot Setpoint", pivotSetpoint);
    SmartDashboard.putBoolean("[C] Pivot At Setpoint", pivotAtPositionTrigger().getAsBoolean());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(pivotLSPressed()) {
      PivotMotor.setPosition(0);
      // setPivotSetpoint(0);
    }

     if(topLS.get()){
      leaderMotor.setPosition(0);
      // setClimberSetpoint(0);
    }

    //Safety for manual/voltage control of the pivot. Comment when not using manual/voltage control.
    // if(pivotLSPressed() && PivotMotor.get() < 0){
    //   stopMotor();
    // }

    if (pivotLSPressed() && PivotMotor.getMotorVoltage().getValueAsDouble()< 0){
      stopMotor();
    }

    //Safety for manual/voltage control of the climber. Comment when not using manual/voltage control.
    // if(topLSPressed() && (leaderMotor.get() > 0)){
    //   stopMotors();
    // }
    // else if(hallEffectPressed() && (leaderMotor.get() < 0)){
    //   stopMotors();
    // }

    if (topLSPressed() && leaderMotor.getMotorVoltage().getValueAsDouble() > 0){
      stopMotors();
    }
    else if(hallEffectPressed() && leaderMotor.getMotorVoltage().getValueAsDouble() < 0){
      stopMotors();
    }

    //Motion Magic Control Modes for climber and pivot.
    // leaderMotor.setControl(profileReq.withPosition(climbSetpoint));
    // PivotMotor.setControl(pivotMagic.withPosition(pivotSetpoint));

    //Only for tuning kS and kG for Motion Magic. Comment when done.
    leaderMotor.setControl(climbVoltageReq.withOutput(climbVoltage));
    PivotMotor.setControl(pivotVoltageReq.withOutput(pivotVoltage));

    troubleshoot();
  }
}
