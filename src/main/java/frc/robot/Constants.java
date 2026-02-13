// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

/** Add your docs here. */
public final class Constants {
    public static class ClimberConstants{
        
        //Climber hook position for each rung of the tower
        public static final double L1RUNG = 0;
        public static final double L2L3RUNG = 0;
        public static final double ACTIVATE = 0;
        public static final double MAX_SPEED = 0;

        //Motor Configurations for Leader and Follower Motor
        public static final TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        public static final MotionMagicConfigs climberMMConfig = new MotionMagicConfigs();
        static{
            leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            
            //Current Limit Config for Leader Motor
            leaderConfig.CurrentLimits.StatorCurrentLimitEnable = false;
            leaderConfig.CurrentLimits.StatorCurrentLimit = 0;
            leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
            leaderConfig.CurrentLimits.SupplyCurrentLimit = 0;
            
            //Software Limit Switch Config for Leader Motor
            leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            leaderConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
            leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
            leaderConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
            
            followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            followerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

            //Current Limit Config for Follower Motor
            followerConfig.CurrentLimits.StatorCurrentLimitEnable = false;
            followerConfig.CurrentLimits.StatorCurrentLimit = 0;
            followerConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
            followerConfig.CurrentLimits.SupplyCurrentLimit = 0;

            //Software Limit Switch Config for Follower Motor
            followerConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
            followerConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0;
            followerConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
            followerConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
            
            //Slot 0 Config for the Leader Motor
            leaderConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;
            leaderConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;
            leaderConfig.Slot0.kS = 0;
            leaderConfig.Slot0.kV = 0;
            leaderConfig.Slot0.kG = 0;
            leaderConfig.Slot0.kA = 0;
            leaderConfig.Slot0.kP = 0;
            leaderConfig.Slot0.kI = 0;
            leaderConfig.Slot0.kD = 0;

            //Motion Magic Config for the Leader Motor
            climberMMConfig.MotionMagicCruiseVelocity = 0;
            climberMMConfig.MotionMagicAcceleration = 0;
            climberMMConfig.MotionMagicExpo_kV = 0;
            climberMMConfig.MotionMagicExpo_kA = 0;
        }
    }
}
