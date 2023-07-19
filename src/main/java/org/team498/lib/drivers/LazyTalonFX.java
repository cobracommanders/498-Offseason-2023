package org.team498.lib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

// Credit to team 254 for inspiring this class
public class LazyTalonFX extends TalonFX {
    private ControlMode currentControlMode = null;
    private double currentSetpoint = Double.NaN;
    private DemandType currentDemandType = null;
    private double currentDemand1 = Double.NaN;
    private NeutralMode currentNeutralMode = null;

    public LazyTalonFX(int deviceNumber) {
        super(deviceNumber);
    }

    @Override
    public void set(ControlMode mode, double setpoint) {
        if (setpoint != currentSetpoint || mode != currentControlMode) {
            this.currentControlMode = mode;
            this.currentSetpoint = setpoint;
            super.set(mode, setpoint);
        }
    }

    @Override
    public void set(TalonFXControlMode mode, double value) {
        set(mode.toControlMode(), value);
    }

    @Override
    public void set(ControlMode mode, double demand0, DemandType demand1Type, double demand1) {
        if (currentSetpoint != demand0 || mode != currentControlMode || currentDemandType != demand1Type || currentDemand1 != demand1) {
            this.currentControlMode = mode;
            this.currentSetpoint = demand0;
            this.currentDemandType = demand1Type;
            this.currentDemand1 = demand1;
            super.set(mode, demand0, demand1Type, demand1);
        }
    }

    @Override
    public void set(TalonFXControlMode mode, double demand0, DemandType demand1Type, double demand1) {
        set(mode.toControlMode(), demand0, demand1Type, demand1);
    }

    @Override
    public void setNeutralMode(NeutralMode neutralMode) {
        if (currentNeutralMode != neutralMode) {
            currentNeutralMode = neutralMode;
            super.setNeutralMode(neutralMode);
        }
    }
}
