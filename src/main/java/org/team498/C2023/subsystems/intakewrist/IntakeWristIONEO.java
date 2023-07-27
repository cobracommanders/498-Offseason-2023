package org.team498.C2023.subsystems.intakewrist;

import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;

import static org.team498.C2023.Constants.IntakeWristConstants.*;
import static org.team498.C2023.Ports.IntakeWrist.*;

import org.team498.C2023.State;
import org.team498.lib.drivers.LazySparkMax;

public class IntakeWristIONEO extends SubsystemBase implements IntakeWristIO {
    private final LazySparkMax left;
    private final LazySparkMax right;
    private final DutyCycle encoder;

    private final ProfiledPIDController PID;

    public IntakeWristIONEO() {
        left = new LazySparkMax(L_WRIST, MotorType.kBrushless);
        right = new LazySparkMax(R_WRIST, MotorType.kBrushless);

        left.restoreFactoryDefaults();
        right.restoreFactoryDefaults();

        PID = new ProfiledPIDController(2.5, 0, 0.075, new TrapezoidProfile.Constraints(2.5, 6));
        PID.reset(State.IntakeWrist.IDLE_IN.position);
        PID.setTolerance(0);

        encoder = new DutyCycle(new DigitalInput(ENCODER_PORT));
    }

    @Override
    public void setPosition(double position) {
        PID.setGoal(position);
    }

    @Override
    public void periodic() {
        double speed = PID.calculate(getAngle());
       left.set(speed);
        right.set(-speed);
    }

    @Override
    public void updateInputs(IntakeWristIOInputs inputs) {
        inputs.angle = getAngle();
        inputs.targetAngle = PID.getGoal().position;

        inputs.rawAbsoluteEncoder = encoder.getOutput();

        // inputs.leftAppliedVolts = left.getBusVoltage() * left.get();
        // inputs.leftCurrentAmps = left.getOutputCurrent();
        // inputs.leftTemp = (left.getMotorTemperature() * 1.8) + 32;

        // inputs.rightAppliedVolts = right.getBusVoltage() * right.get();
        // inputs.rightCurrentAmps = right.getOutputCurrent();
        // inputs.rightTemp = (right.getMotorTemperature() * 1.8) + 32;
    }

    @Override
    public void setBrakeMode(boolean enable) {
        left.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
        right.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);       
    }

    private double getAngle() {
        double angle = encoder.getOutput() + 0.75;
        if (angle < 1)
            angle += 1;

        return angle - 0.75 - 0.685961;
    }
}
