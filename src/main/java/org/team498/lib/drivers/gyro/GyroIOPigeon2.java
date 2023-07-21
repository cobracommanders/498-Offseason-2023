package org.team498.lib.drivers.gyro;

import org.team498.lib.util.RotationUtil;
import com.ctre.phoenix.sensors.Pigeon2;
import static org.team498.C2023.Ports.Accessories.DriveBus;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;
    public GyroIOPigeon2(int GYRO) {
        pigeon = new Pigeon2(GYRO, DriveBus);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.pitch = pigeon.getPitch();
        inputs.roll = pigeon.getRoll();
        inputs.yaw = RotationUtil.toSignedDegrees(pigeon.getYaw());
    }

    @Override
    public void setYaw(double yaw) {
        pigeon.setYaw(yaw);
    }
}
