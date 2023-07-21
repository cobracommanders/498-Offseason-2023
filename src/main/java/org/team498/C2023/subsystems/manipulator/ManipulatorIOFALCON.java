package org.team498.C2023.subsystems.manipulator;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static org.team498.C2023.Ports.Accessories.RioBus;
import static org.team498.C2023.Ports.Manipulator.*;

public class ManipulatorIOFALCON extends SubsystemBase implements ManipulatorIO {
    private final TalonFX lMotor;
    private final TalonFX rMotor;

    public ManipulatorIOFALCON() {
        lMotor = new TalonFX(L_ROLLERS, RioBus);
        rMotor = new TalonFX(R_ROLLERS,RioBus);


        lMotor.setInverted(false);
        lMotor.configFactoryDefault();
        lMotor.setNeutralMode(NeutralMode.Coast); 


        rMotor.setInverted(true); //setting the motor inputs to follow the left motor but inverted
        rMotor.configFactoryDefault();
        rMotor.follow(lMotor); 
    }

    @Override
    public void updateInputs(ManipulatorIOInputs inputs) {
        inputs.motorCurrentAmps = lMotor.getSupplyCurrent();
        inputs.motorTemp = (lMotor.getTemperature() * 1.8) + 32;
        inputs.velocityRotationsPerSecond = lMotor.getSelectedSensorVelocity(); //TODO check this too?
    }

    @Override
    public void setSpeed(double speed) {
        lMotor.set(TalonFXControlMode.PercentOutput, .5); //TODO update this eventually
    }
}
