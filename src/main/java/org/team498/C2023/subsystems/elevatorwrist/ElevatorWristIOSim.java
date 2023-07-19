package org.team498.C2023.subsystems.elevatorwrist;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.team498.C2023.Robot;

import static org.team498.C2023.Constants.ElevatorWristConstants.*;


public class ElevatorWristIOSim extends SubsystemBase implements ElevatorWristIO {
    private final SingleJointedArmSim elevatorWristSim = new SingleJointedArmSim(DCMotor.getNEO(1),
                                                                                 5,
                                                                                 SingleJointedArmSim.estimateMOI(4, 0.2),
                                                                                 4,
                                                                                 Math.toRadians(-99999),
                                                                                 Math.toRadians(99999),
                                                                                 false,
                                                                                 VecBuilder.fill(0.00)
    );

    double simAngle = 0;

    private final PIDController PID;

    private ControlMode controlMode = ControlMode.PID;

    private double speed = 0;

    public ElevatorWristIOSim() {
        PID = new PIDController(10, I, 2.5);
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
                yield PID.calculate(simAngle);
            case MANUAL:
                yield this.speed;
        };


        elevatorWristSim.setInput(percentOutput * RobotController.getBatteryVoltage());
        elevatorWristSim.update(Robot.DEFAULT_PERIOD);
        simAngle = Math.toDegrees(elevatorWristSim.getAngleRads()) / 360;
    }

    @Override
    public void updateInputs(ElevatorWristIOInputs inputs) {
        inputs.angle = -simAngle;
        inputs.targetAngle = -PID.getSetpoint();
    }

    public enum ControlMode {
        PID,
        MANUAL
    }
}
