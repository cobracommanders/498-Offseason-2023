package org.team498.C2023.subsystems.intakerollers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team498.C2023.Ports.IntakeRollers.*;

import org.littletonrobotics.junction.Logger;

public class IntakeRollerIOFalcon500 extends SubsystemBase implements IntakeRollerIO {
    private final TalonFX bottom;
    private final TalonFX top;
    private final TalonFX third;

    public IntakeRollerIOFalcon500() {
        bottom = new TalonFX(BOTTOM_ROLLER);
        top = new TalonFX(TOP_ROLLER);
        third = new TalonFX(THIRD_ROLLER);

        bottom.setNeutralMode(NeutralMode.Coast);
        top.setNeutralMode(NeutralMode.Coast);
        third.setNeutralMode(NeutralMode.Coast);

        bottom.setInverted(false);
        top.setInverted(false);
        third.setInverted(false);

        bottom.config_kP(0, 0.2);
        bottom.config_kD(0, 0);
        bottom.config_kF(0, 0.1);
    }

    @Override
    public void updateInputs(IntakeRollerIOInputs inputs) {
        // inputs.topCurrentAmps = top.getStatorCurrent();
        // inputs.topTemp = (top.getTemperature() * 1.8) + 32;

        // inputs.bottomCurrentAmps = top.getStatorCurrent();
        // inputs.bottomTemp = (top.getTemperature() * 1.8) + 32;

        // inputs.thirdCurrentAmps = third.getOutputCurrent();
        // inputs.thirdTemp = (third.getMotorTemperature() * 1.8) + 32;

        SmartDashboard.putNumber("Intake Velocity", bottom.getSelectedSensorVelocity());
    }

    @Override
    public void setSpeed(double bottomSpeed, double topSpeed, double thirdSpeed) {
        // bottom.set(ControlMode.Velocity, bottomSpeed * 1000);
        bottom.set(ControlMode.PercentOutput, bottomSpeed);
        top.set(ControlMode.PercentOutput, topSpeed);
        third.set(ControlMode.PercentOutput, thirdSpeed);
        SmartDashboard.putNumber("Intake Setpoint", bottomSpeed * 1000);

    }
}
