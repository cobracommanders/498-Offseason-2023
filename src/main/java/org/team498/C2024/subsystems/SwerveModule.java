package org.team498.C2024.subsystems;

import static org.team498.C2024.Constants.DrivetrainConstants.DRIVE_WHEEL_CIRCUMFERENCE;
import static org.team498.C2024.Constants.DrivetrainConstants.DRIVE_WHEEL_DIAMETER;
import static org.team498.C2024.Constants.DrivetrainConstants.MK4I_DRIVE_REDUCTION_L3;
import static org.team498.C2024.Constants.DrivetrainConstants.MK4I_STEER_REDUCTION_L3;

import org.team498.lib.drivers.LazyTalonFX;
import org.team498.lib.util.Falcon500Conversions;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;

public class SwerveModule {
    private final LazyTalonFX drive;
    private final LazyTalonFX steer;
    private final CANcoder encoder;

    private SwerveModuleState currentTarget = new SwerveModuleState();
    double angleOffset;
    public SwerveModule(int driveID, int steerID, int encoderID, String canBus, double angleOffset){
        drive = new LazyTalonFX(driveID,canBus);
        steer = new LazyTalonFX(steerID,canBus);
        encoder = new CANcoder(encoderID,canBus);
        this.angleOffset = angleOffset;
        configDriveMotor(drive);
        configSteerMotor(steer);
        configCANCoder(encoder);
    }
    public void setState(SwerveModuleState state) {
        currentTarget = optimize(state, Falcon500Conversions.falconToDegrees(steer.getPosition().getValueAsDouble(), MK4I_STEER_REDUCTION_L3));

        drive.set(Falcon500Conversions.MPSToFalcon(currentTarget.speedMetersPerSecond, Units.inchesToMeters(DRIVE_WHEEL_DIAMETER), MK4I_DRIVE_REDUCTION_L3));
        steer.setPosition(Falcon500Conversions.degreesToFalcon(currentTarget.angle.getDegrees(), MK4I_STEER_REDUCTION_L3));
    }
    public void updateIntegratedEncoder() {
        steer.setPosition(Falcon500Conversions.degreesToFalcon(encoder.getAbsolutePosition().getValueAsDouble() - angleOffset, MK4I_STEER_REDUCTION_L3));
    }
    public double getSpeed(){
        return currentTarget.speedMetersPerSecond;
    }
    public double getPosition(){
        return Falcon500Conversions.falconToDegrees(drive.getSelectedSensorPosition(), MK4I_DRIVE_REDUCTION_L3) / 360 * Units.inchesToMeters(DRIVE_WHEEL_CIRCUMFERENCE);
   }

   public double getAngle(){
    return encoder.getAbsolutePosition().getValueAsDouble() * 360;
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
       // motor.configFactoryDefault();

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimit = 35;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.Slot0.kP = 0.025;
        driveConfig.Slot0.kI = 0.0;
        driveConfig.Slot0.kD = 0.5;
        //TODO: ADD FEET FOWARD
        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1;
        motor.getConfigurator().apply(driveConfig);
        motor.setInverted(false);
    }

    private void configSteerMotor(TalonFX motor) {
       // motor.configFactoryDefault();

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.CurrentLimits.SupplyCurrentLimit = 20;
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.Slot0.kP = 0.2;
        steerConfig.Slot0.kI = 0.0;
        steerConfig.Slot0.kD = 0.1;
        steerConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1;
        motor.getConfigurator().apply(steerConfig);
        motor.setInverted(true);    
    }

    private void configCANCoder(CANcoder CANCoder) {
        //CANCoder.configFactoryDefault();
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        CANCoder.getConfigurator().apply(encoderConfig);
        //CANCoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    }

}

