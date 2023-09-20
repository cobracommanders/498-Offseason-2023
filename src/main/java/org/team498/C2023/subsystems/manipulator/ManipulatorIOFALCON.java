package org.team498.C2023.subsystems.manipulator;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static org.team498.C2023.Ports.Accessories.RioBus;
import static org.team498.C2023.Ports.Manipulator.*;

public class ManipulatorIOFALCON extends SubsystemBase implements ManipulatorIO {
    private double currentSpeed = 0;
    private double currentSpeedRight = 0;
    private final TalonFX lMotor;
    private final TalonFX rMotor;
    private PIDController  PIDController = new PIDController(1.0 / 2000.0, 0.000, 0);

    SlewRateLimiter limiter = new SlewRateLimiter(6.5, -1000, 0);

    //new PIDController(1.0 / 2000.0, 0.000, 0);

    public ManipulatorIOFALCON() {
        lMotor = new TalonFX(L_ROLLERS, RioBus);
        rMotor = new TalonFX(R_ROLLERS,RioBus);


        lMotor.setInverted(false);
        lMotor.configFactoryDefault();
        lMotor.setNeutralMode(NeutralMode.Brake); 

        rMotor.configFactoryDefault();
        rMotor.setInverted(true); //setting the motor inputs to follow the left motor but inverted
        
        rMotor.setNeutralMode(NeutralMode.Brake);
        //rMotor.follow(lMotor); 
    }

    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
        inputs.motorCurrentAmps = lMotor.getSupplyCurrent();
        inputs.motorTemp = (lMotor.getTemperature() * 1.8) + 32;
        inputs.velocityRotationsPerSecond = lMotor.getSelectedSensorVelocity(); //TODO check this too?
    }

    @Override
    public void periodic() {
        PIDController.setSetpoint(limiter.calculate(currentSpeed));
        currentSpeed = PIDController.calculate(lMotor.getSelectedSensorVelocity());
        lMotor.set(TalonFXControlMode.PercentOutput, currentSpeed);
        rMotor.set(TalonFXControlMode.PercentOutput, currentSpeedRight);
    }

    @Override
    public void setSpeed(double speed) {
        double setpoint = Math.abs(speed) * 11;
        if(setpoint == 0)setpoint = 9999;
        limiter = new SlewRateLimiter(setpoint, -9999, currentSpeed);
        if(speed > 0){currentSpeed = 1; currentSpeedRight = 1;}
        else if(speed < 0){currentSpeed = -1; currentSpeedRight = -1;}
        else if(speed == 0){currentSpeed = 0; currentSpeedRight = 0;}

    }
}
