package org.team498.C2023.subsystems.elevator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.Robot;

import static org.team498.C2023.Constants.ElevatorConstants.*;

public class ElevatorIOSim extends SubsystemBase implements ElevatorIO {
    private final ElevatorSim elevatorSim = new ElevatorSim(DCMotor.getFalcon500(2),
                                                            5,
                                                            5.36781211,
                                                            Units.inchesToMeters(1.273),
                                                            0,
                                                            1.5,
                                                            true,
                                                            VecBuilder.fill(0.00)
    );

    double simPosition = 0;

    private final ProfiledPIDController PID;

    private final ElevatorFeedforward feedforward = new ElevatorFeedforward(S, G, V);

    private ControlMode controlMode = ControlMode.PID;

    private double speed = 0;

    public ElevatorIOSim() {
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
                double speed = PID.calculate(simPosition);
                yield speed + feedforward.calculate(speed);
            case MANUAL:
                yield this.speed + feedforward.calculate(this.speed);
        };

        elevatorSim.setInput(percentOutput * RobotController.getBatteryVoltage());
        elevatorSim.update(Robot.DEFAULT_PERIOD);
        simPosition = elevatorSim.getPositionMeters();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.positionMeters = simPosition;
        inputs.targetPositionMeters = PID.getGoal().position;
    }

    public enum ControlMode {
        PID,
        MANUAL
    }
}
