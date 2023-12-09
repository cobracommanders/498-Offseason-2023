package org.team498.lib.drivers;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj.RobotBase;
import org.team498.lib.util.RotationUtil;

import static org.team498.C2023.Ports.Drivetrain.GYRO;

public class Gyro extends com.ctre.phoenix6.hardware.Pigeon2 {

    /** @return yaw angle in degrees (CCW positive), ranging from -180 to 180 degrees */
    
    public synchronized double yaw() {
        return RotationUtil.toSignedDegrees(super.getYaw().getValueAsDouble());             
    }
    
    public double pitch() {
        return super.getPitch().getValueAsDouble();
    }

    public void setSimAngle(double angle) {
    }

    private Gyro(int CANId) {
        super(CANId);
    }

    private static Gyro instance;

    public static Gyro getInstance() {
        if (instance == null) {
            instance = new Gyro(GYRO);
        }
        return instance;
    }

}
