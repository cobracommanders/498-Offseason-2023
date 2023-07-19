package org.team498.lib.drivers;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.RobotBase;
import org.team498.lib.util.RotationUtil;

import static org.team498.C2023.Ports.Drivetrain.GYRO;

public class Gyro extends Pigeon2 {
    private double simAngle = 0;

    /** @return yaw angle in degrees (CCW positive), ranging from -180 to 180 degrees */
    @Override
    public synchronized double getYaw() {
        return RobotBase.isReal()
               ? RotationUtil.toSignedDegrees(super.getYaw())
               : -RotationUtil.toSignedDegrees(simAngle);
    }
    
    public double getPitch() {
        return super.getPitch();
    }

    public void setSimAngle(double angle) {
        simAngle = angle;
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
