package org.team498.C2023.subsystems.elevatorwrist;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalInput;

import static org.team498.C2023.Ports.ElevatorWrist.*;

import org.team498.lib.drivers.LazySparkMax;

import static org.team498.C2023.Constants.ElevatorWristConstants.*;


public class ElevatorWristIONEO extends SubsystemBase implements ElevatorWristIO {
    private final LazySparkMax motor;
    private final DutyCycle encoder;

    private final PIDController PID;

    private ControlMode controlMode = ControlMode.PID;

    private double speed = 0;

    public ElevatorWristIONEO() {
        motor = new LazySparkMax(WRIST, MotorType.kBrushless);

        configMotor(motor);

        encoder = new DutyCycle(new DigitalInput(ENCODER_PORT));

        PID = new PIDController(P, I, D);
        PID.setTolerance(0);
    }

    @Override
    public void setPosition(double position) {
        this.controlMode = ControlMode.PID;
        PID.setSetpoint(position);
    }

    @Override
    public void setManual(double speed) {
        this.controlMode = ControlMode.MANUAL;
        this.speed = speed;
    }

    @Override
    public void periodic() {
        double percentOutput = switch (controlMode) {
            case PID: 
                yield PID.calculate(getAngle());
            case MANUAL:
                yield this.speed;
        };
    
        motor.set(-percentOutput);
    }

    @Override
    public void updateInputs(ElevatorWristIOInputs inputs) {
        inputs.angle = getAngle();
        inputs.targetAngle = PID.getSetpoint();

        inputs.rawAbsoluteEncoder = encoder.getOutput();

        // inputs.motorAppliedVolts = motor.getBusVoltage() * motor.get();
        // inputs.motorCurrentAmps = motor.getOutputCurrent();
        // inputs.motorTemp = (motor.getMotorTemperature() * 1.8) + 32;
    }

    @Override
    public void setBrakeMode(boolean enable) {
        motor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    private void configMotor(CANSparkMax motor) {
        motor.restoreFactoryDefaults();
        motor.setInverted(false);
        motor.burnFlash(); 
    }

    private double getAngle() {
        double angle = encoder.getOutput() + 0.5;

        if (angle < 1)
            angle += 1;

        return angle - 1.370912;
    }


    public enum ControlMode {
        PID,
        MANUAL
    }
}
