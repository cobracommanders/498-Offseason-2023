package org.team498.lib.drivers.swervemodule;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

import org.team498.lib.util.Falcon500Conversions;

import static org.team498.C2023.Constants.DrivetrainConstants.*;
import static org.team498.C2023.Ports.Drivetrain.*;
import static org.team498.C2023.Ports.Accessories.DriveBus;

public class ModuleIOFalcon500 implements ModuleIO {
    public enum Module {
        FL(FL_DRIVE, FL_STEER, FL_CANCODER, FL_MODULE_OFFSET),
        FR(FR_DRIVE, FR_STEER, FR_CANCODER, FR_MODULE_OFFSET),
        BL(BL_DRIVE, BL_STEER, BL_CANCODER, BL_MODULE_OFFSET),
        BR(BR_DRIVE, BR_STEER, BR_CANCODER, BR_MODULE_OFFSET);

        private final int driveID, steerID, encoderID;
        private final String busID;
        private final double angleOffset;

        Module(int driveID, int steerID, int encoderID, double angleOffset) {
            String busID = DriveBus;
            this.driveID = driveID;
            this.steerID = steerID;
            this.encoderID = encoderID;
            this.angleOffset = angleOffset;
            this.busID = busID;
        }
    }

    private final Module module;
    private final WPI_TalonFX drive;
    private final WPI_TalonFX steer;
    private final WPI_CANCoder encoder;
    private final double angleOffset;

    private SwerveModuleState currentTarget = new SwerveModuleState();
    
    public ModuleIOFalcon500(Module module) {
        this.module = module;  
        drive = new WPI_TalonFX(module.driveID, module.busID);
        steer = new WPI_TalonFX(module.steerID, module.busID);
        encoder = new WPI_CANCoder(module.encoderID, module.busID);
        angleOffset = module.angleOffset;

        configDriveMotor(drive);
        configSteerMotor(steer);
        configCANCoder(encoder);
    }

    @Override
    public void setState(SwerveModuleState state) {
        currentTarget = optimize(state, Falcon500Conversions.falconToDegrees(steer.getSelectedSensorPosition(), MK4I_STEER_REDUCTION_L3));

        drive.set(ControlMode.Velocity, Falcon500Conversions.MPSToFalcon(currentTarget.speedMetersPerSecond, Units.inchesToMeters(DRIVE_WHEEL_DIAMETER), MK4I_DRIVE_REDUCTION_L3));
        steer.set(ControlMode.Position, Falcon500Conversions.degreesToFalcon(currentTarget.angle.getDegrees(), MK4I_STEER_REDUCTION_L3));
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.positionMeters = (Falcon500Conversions.falconToDegrees(drive.getSelectedSensorPosition(), MK4I_DRIVE_REDUCTION_L3) / 360) * Units.inchesToMeters(DRIVE_WHEEL_CIRCUMFERENCE);
        inputs.speedMetersPerSecond = Falcon500Conversions.falconToMPS(drive.getSelectedSensorVelocity(), Units.inchesToMeters(DRIVE_WHEEL_CIRCUMFERENCE), MK4I_DRIVE_REDUCTION_L3);
        inputs.angle = Falcon500Conversions.falconToDegrees(steer.getSelectedSensorPosition(), MK4I_STEER_REDUCTION_L3);

        // inputs.targetSpeedMetersPerSecond = currentTarget.speedMetersPerSecond;
        // inputs.targetAngle = currentTarget.angle.getDegrees();

        // inputs.driveAppliedVolts = drive.getMotorOutputVoltage();
        inputs.driveCurrentAmps = drive.getStatorCurrent();
        //inputs.driveTemp = (drive.getTemperature() * 1.8) + 32;
        // inputs.driveRawEncoder = drive.getSelectedSensorPosition();

        // inputs.steerAppliedVolts = steer.getMotorOutputVoltage();
        inputs.steerCurrentAmps = steer.getStatorCurrent();
        //inputs.steerTemp = (steer.getTemperature() * 1.8) + 32;
        // inputs.steerRawEncoder = steer.getSelectedSensorPosition();
    }

    @Override
    public void updateIntegratedEncoder() {
        steer.setSelectedSensorPosition(Falcon500Conversions.degreesToFalcon(encoder.getAbsolutePosition() - angleOffset, MK4I_STEER_REDUCTION_L3));
    }

    @Override
    public void setBrakeMode(boolean enable) {
        steer.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);
        drive.setNeutralMode(enable ? NeutralMode.Brake : NeutralMode.Coast);        
    }

    // Custom optimize method by team 364
    private SwerveModuleState optimize(SwerveModuleState desiredState, double currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle, desiredState.angle.getDegrees());

        double targetSpeed = desiredState.speedMetersPerSecond;

        double delta = targetAngle - currentAngle;

        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            if (delta > 90) {targetAngle -= 180;} else {targetAngle += 180;}
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    private double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }

    private void configDriveMotor(TalonFX motor) {
        motor.configFactoryDefault();

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.supplyCurrLimit.currentLimit = 35;
        driveConfig.supplyCurrLimit.enable = true;
        motor.configAllSettings(driveConfig);

        motor.configOpenloopRamp(1);
        motor.setInverted(false);

        motor.config_kP(0, 0.025);
        motor.config_kI(0, 0.0);
        motor.config_kD(0, 0.5);
    }

    private void configSteerMotor(TalonFX motor) {
        motor.configFactoryDefault();

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.supplyCurrLimit.currentLimit = 20;
        steerConfig.supplyCurrLimit.enable = true;
        motor.configAllSettings(steerConfig);

        motor.configOpenloopRamp(1);
        motor.setSensorPhase(true);
        motor.setInverted(true);

        motor.config_kP(0, 0.2);
        motor.config_kI(0, 0.0);
        motor.config_kD(0, 0.1);
    }

    private void configCANCoder(CANCoder CANCoder) {
        CANCoder.configFactoryDefault();
        CANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        CANCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
    }

    @Override
    public String getName() {
        return module.name();
    }
}
