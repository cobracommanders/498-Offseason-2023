package org.team498.C2023.subsystems.manipulator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.team498.C2023.Ports.Manipulator.*;

public class ManipulatorIONEO extends SubsystemBase implements ManipulatorIO {
    private final CANSparkMax motor;

    public ManipulatorIONEO() {
        motor = new CANSparkMax(ROLLERS, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.setIdleMode(IdleMode.kCoast);
    }

    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
        inputs.motorCurrentAmps = motor.getOutputCurrent();
        inputs.motorTemp = (motor.getMotorTemperature() * 1.8) + 32;
        inputs.velocityRotationsPerSecond = motor.getEncoder().getVelocity();
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(-speed);
    }
}
