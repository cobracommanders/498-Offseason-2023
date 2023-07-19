package org.team498.lib.drivers.gyro;

import org.team498.lib.util.RotationUtil;

import com.ctre.phoenix.sensors.Pigeon2;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;
    String canivoreName = "bus";
    public GyroIOPigeon2(int GYRO) {
        pigeon = new Pigeon2(GYRO, canivoreName);
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
