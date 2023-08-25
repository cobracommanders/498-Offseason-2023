package org.team498.C2023.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;

import static org.team498.C2023.Constants.ElevatorConstants.*;
import static org.team498.C2023.Ports.Elevator.*;
import static org.team498.C2023.Ports.Accessories.*;

public class ElevatorIOFalcon500 extends SubsystemBase implements ElevatorIO {
    private final TalonFX front;
    private final TalonFX back;
    private final DutyCycle encoder;

    private final ProfiledPIDController PID;

    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(S, G, V);

    private ControlMode controlMode = ControlMode.PID;

    private double speed = 0;

    public ElevatorIOFalcon500() {
       // front = new TalonFX(F_ELEVATOR_ID);
       // back = new TalonFX(B_ELEVATOR_ID);
       front = new TalonFX(F_ELEVATOR_ID, DriveBus);
       back = new TalonFX(B_ELEVATOR_ID, DriveBus);




        configMotor(front);
        configMotor(back);

        encoder = new DutyCycle(new DigitalInput(ENCODER_PORT));

        PID = new ProfiledPIDController(P, I, D, new TrapezoidProfile.Constraints(3, 5));
        PID.reset(0);
        PID.setTolerance(0);
    }

    @Override
    public void setPosition(double position) {
        this.controlMode = ControlMode.PID;
        PID.setGoal(position);
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
                double speed = PID.calculate(getPositionMeters());
                yield speed + feedforward.calculate(speed);
            case MANUAL:
                yield this.speed + feedforward.calculate(this.speed);

            // TODO change ControlMode to ElevatorMode and add a disabled state
        };
    
        front.set(TalonFXControlMode.PercentOutput, -percentOutput);
        back.set(TalonFXControlMode.PercentOutput, -percentOutput);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = getPositionMeters();
        inputs.targetPositionMeters = PID.getGoal().position;

        inputs.rawAbsoluteEncoder = encoder.getOutput();

        inputs.frontAppliedVolts = front.getMotorOutputVoltage();
        inputs.frontCurrentAmps = front.getStatorCurrent();
        inputs.frontTemp = (front.getTemperature() * 1.8) + 32;
        inputs.frontRawEncoder = front.getSelectedSensorPosition();

        inputs.backAppliedVolts = back.getMotorOutputVoltage();
        inputs.backCurrentAmps = back.getStatorCurrent();
        inputs.backTemp = (back.getTemperature() * 1.8) + 32;
        inputs.backRawEncoder = back.getSelectedSensorPosition();
    }

    @Override
    public void updateInitialPosition(boolean inAutoPose) {
        if (inAutoPose) {
            double angle = encoder.getOutput() + 0.5;
            if (angle < 1)
                angle += 1;
            
            front.setSelectedSensorPosition((angle - 0.0) * 2048);
        } else {
            double angle = encoder.getOutput() + 0.75;
            if (angle < 1)
                angle += 1;
            front.setSelectedSensorPosition(((angle - 0.744160 - 0.75) * -5) * 2048);
        }
    }

    @Override
    public void setBrakeMode(boolean enable) {
        front.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
        back.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);        
    }

    private void configMotor(TalonFX motor) {
        motor.configFactoryDefault();
        motor.setNeutralMode(NeutralMode.Brake);
        motor.setInverted(false);
    }

    private double getPositionMeters() {return (-front.getSelectedSensorPosition() / 2048) / MOTOR_ROTATION_TO_METERS;}


    public enum ControlMode {
        PID,
        MANUAL
    }
}
