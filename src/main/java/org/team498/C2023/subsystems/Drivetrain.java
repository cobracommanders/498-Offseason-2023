package org.team498.C2023.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.team498.C2023.Constants;
import org.team498.C2023.Ports;
import org.team498.C2023.Robot;
import org.team498.C2023.RobotPosition;
import org.team498.C2023.Constants.Mode;
import org.team498.C2023.subsystems.vision.Vision;
import org.team498.lib.drivers.gyro.GyroIO;
import org.team498.lib.drivers.gyro.GyroIOInputsAutoLogged;
import org.team498.lib.drivers.gyro.GyroIOPigeon2;
import org.team498.lib.drivers.gyro.GyroIOSim;
import org.team498.lib.drivers.swervemodule.ModuleIO;
import org.team498.lib.drivers.swervemodule.ModuleIOFalcon500;
import org.team498.lib.drivers.swervemodule.ModuleIOInputsAutoLogged;
import org.team498.lib.drivers.swervemodule.ModuleIOSim;
import org.team498.lib.logger.LoggerUtil;
import org.team498.lib.util.PoseUtil;
import org.team498.lib.wpilib.ChassisSpeeds;

import static org.team498.C2023.Constants.DrivetrainConstants.*;

import javax.xml.crypto.Data;

public class Drivetrain extends SubsystemBase {
    private final ModuleIO[] modules;
    private final ModuleIOInputsAutoLogged[] moduleInputs = new ModuleIOInputsAutoLogged[] {new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged(), new ModuleIOInputsAutoLogged()};
    private final ProfiledPIDController angleController = new ProfiledPIDController(AngleConstants.P, AngleConstants.I, AngleConstants.D, AngleConstants.CONTROLLER_CONSTRAINTS);
    private final PIDController xController = new PIDController(PoseConstants.P, PoseConstants.I, PoseConstants.D);
    private final PIDController yController = new PIDController(PoseConstants.P, PoseConstants.I, PoseConstants.D);
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;
    private SwerveModuleState[] stateSetpoints;
    
    private final Field2d field2d = new Field2d();
    private final GyroIO gyro;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private Drivetrain() { 
        modules = switch (Constants.mode) {
            case REAL, REPLAY ->
                new ModuleIO[] {
                    new ModuleIOFalcon500(ModuleIOFalcon500.Module.FL),
                    new ModuleIOFalcon500(ModuleIOFalcon500.Module.FR),
                    new ModuleIOFalcon500(ModuleIOFalcon500.Module.BL),
                    new ModuleIOFalcon500(ModuleIOFalcon500.Module.BR),
                    };
            case SIM -> new ModuleIO[] {
                    new ModuleIOSim("FL"),
                    new ModuleIOSim("FR"),
                    new ModuleIOSim("BL"),
                    new ModuleIOSim("BR")
            };
        };

        gyro = switch (Constants.mode) {
            case REAL, REPLAY -> new GyroIOPigeon2(Ports.Drivetrain.GYRO);
            case SIM -> new GyroIOSim();
        };

        angleController.enableContinuousInput(-180, 180);
        angleController.setTolerance(0);
        xController.setTolerance(0);
        yController.setTolerance(0);
        xLimiter.reset(0);
        yLimiter.reset(0);

        kinematics = new SwerveDriveKinematics(getModuleTranslations());

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(getYaw()), getModulePositions(), new Pose2d(), 
            VecBuilder.fill(Units.inchesToMeters(3), Units.inchesToMeters(3), Math.toDegrees(4)), // Odometry standard deviation
            VecBuilder.fill(Units.inchesToMeters(10), Units.inchesToMeters(10), Math.toDegrees(15))); // Vision standard deviation

        stateSetpoints = getModuleStates();

        for (ModuleIO m : modules) m.setBrakeMode(true);
    }

    @Override
    public void periodic() {
        field2d.setRobotPose(getPose().getX(), getPose().getY(), getPose().getRotation());
        SmartDashboard.putData(field2d);
        SmartDashboard.putNumber("Pitch", gyroInputs.pitch);
        for (int i = 0; i < modules.length; i++) {
            modules[i].updateInputs(moduleInputs[i]);
            //Logger.getInstance().processInputs("Drive/" + modules[i].getName() + "_Module", moduleInputs[i]);

            // modules[i].setBrakeMode(RobotState.isEnabled());

            if (RobotState.isDisabled()) {
                modules[i].updateIntegratedEncoder();
            }
        }
        gyro.updateInputs(gyroInputs);
        //Logger.getInstance().processInputs("Gyro", gyroInputs);

        
        poseEstimator.update(Rotation2d.fromDegrees(getYaw()), getModulePositions());
        var visionPose = Vision.getInstance().rightEstimatedPose();
        visionPose.ifPresent(pose -> {
            boolean temp = true;
            Alliance alliance = Robot.alliance;
            if (Math.abs(getAllianceAdjustedYaw()) > 135) {
            
            //if (RobotPosition.inCommunity(pose.estimatedPose.toPose2d())) {
                for (PhotonTrackedTarget target : visionPose.get().targetsUsed) {
                    if (alliance == Alliance.Red && (target.getFiducialId() == 5 || target.getFiducialId() == 6 || target.getFiducialId() == 7 || target.getFiducialId() == 8)) {
                        temp = false;
                    }
                    else if (alliance == Alliance.Blue && (target.getFiducialId() == 1 || target.getFiducialId() == 2 || target.getFiducialId() == 3 || target.getFiducialId() == 4)) {
                        //poseEstimator.addVisionMeasurement(PoseUtil.toPose2d(pose.estimatedPose), pose.timestampSeconds);
                        temp = false;
  
                    }
                }
                if (temp){
                    poseEstimator.addVisionMeasurement(PoseUtil.toPose2d(pose.estimatedPose), pose.timestampSeconds);
                }
            }
        }
            
        );
        //Logger.getInstance().recordOutput("Odometry", getPose());
        
        //LoggerUtil.recordOutput("Drive/RealStates", getModuleStates());

        if (Constants.mode == Mode.REPLAY) {
            var targetStates = new SwerveModuleState[4];
            for (int i = 0; i < modules.length; i++) {
                // targetStates[i] = new SwerveModuleState(moduleInputs[i].targetSpeedMetersPerSecond, Rotation2d.fromDegrees(moduleInputs[i].targetAngle));
            }
            //LoggerUtil.recordOutput("Drive/TargetStates", targetStates);
        }

        if (Constants.mode == Mode.SIM) {
            gyro.setYaw(gyroInputs.yaw + Math.toDegrees(kinematics.toChassisSpeeds(stateSetpoints).omegaRadiansPerSecond) * Robot.DEFAULT_PERIOD);
        }
    }

    public double getPitch(){
        return gyroInputs.pitch;
    }

    public void drive(double vx, double vy, double degreesPerSecond, boolean fieldOriented) {
        ChassisSpeeds speeds = fieldOriented
                               ? ChassisSpeeds.fromFieldRelativeSpeeds(-vx, -vy, Math.toRadians(degreesPerSecond), getYaw())
                               : new ChassisSpeeds(-vx, -vy, Math.toRadians(degreesPerSecond));

        speeds.vxMetersPerSecond = xLimiter.calculate(speeds.vxMetersPerSecond);
        speeds.vyMetersPerSecond = yLimiter.calculate(speeds.vyMetersPerSecond);
        double dt = Robot.defaultPeriodSecs * 2;
        Pose2d newPose = new Pose2d(speeds.vxMetersPerSecond * dt, speeds.vyMetersPerSecond * dt, Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt));
        Twist2d twist = new Pose2d().log(newPose);
        ChassisSpeeds updatedSpeeds = new ChassisSpeeds(twist.dx / dt, twist.dy / dt, twist.dtheta / dt);
        
        stateSetpoints = kinematics.toSwerveModuleStates(updatedSpeeds);

        setModuleStates(stateSetpoints);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        for (int i = 0; i < modules.length; i++) modules[i].setState(states[i]);
    }

    public SwerveModulePosition[] getModulePositions() {
        var positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) positions[i] = new SwerveModulePosition(moduleInputs[i].positionMeters, Rotation2d.fromDegrees(moduleInputs[i].angle));
        return positions;
    }

    public SwerveModuleState[] getModuleStates() {
        var states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) states[i] = new SwerveModuleState(moduleInputs[i].speedMetersPerSecond, Rotation2d.fromDegrees(moduleInputs[i].angle));
        return states;
    }

public void setPositionGoal(Pose2d pose) {xController.setSetpoint(pose.getX()); yController.setSetpoint(pose.getY()); setAngleGoal(pose.getRotation().getDegrees()); /*Logger.getInstance().recordOutput("TargetPose", pose);*/}
    public ChassisSpeeds calculatePositionSpeed() {return new ChassisSpeeds(xController.calculate(getPose().getX()), yController.calculate(getPose().getY()), calculateAngleSpeed());}
    public boolean atPositionGoal() {return (Math.abs(xController.getPositionError()) < PoseConstants.EPSILON) && (Math.abs(yController.getPositionError()) < PoseConstants.EPSILON) && atAngleGoal();}

    public void setXGoal(double pose) {xController.setSetpoint(pose);}
    public double calculateXSpeed() {return xController.calculate(getPose().getX());}
    public boolean atXGoal() {return Math.abs(xController.getPositionError()) < PoseConstants.EPSILON;}
    
    public void setYGoal(double pose) {yController.setSetpoint(pose);}
    public double calculateYSpeed() {return yController.calculate(getPose().getY());}
    public boolean atYGoal() {return Math.abs(yController.getPositionError()) < PoseConstants.EPSILON;}

    public void setAngleGoal(double angle) {angleController.setGoal(angle);}
    public double calculateAngleSpeed() {return angleController.calculate(getYaw());}
    public boolean atAngleGoal() {return Math.abs(angleController.getPositionError()) < AngleConstants.EPSILON;}

    public Pose2d getPose() {return poseEstimator.getEstimatedPosition();}
    public void setPose(Pose2d pose) {poseEstimator.resetPosition(Rotation2d.fromDegrees(getYaw()), getModulePositions(), pose);}
    public double getYaw() {return gyroInputs.yaw;}
    public double getAllianceAdjustedYaw() {
        double result = getYaw();
        return Robot.alliance == Alliance.Red ? ((result + 180 + 180) % 360) - 180 : result;
    }
    public void setYaw(double angle) {gyro.setYaw(angle); angleController.reset(angle);}
    /** Return a double array with a value for yaw pitch and roll in that order */
    public double[] getGyro() {return new double[] {gyroInputs.yaw, gyroInputs.pitch, gyroInputs.roll};}

    public void stop() {drive(0, 0, 0, false);}
    public void X() {setModuleStates(new SwerveModuleState[] {new SwerveModuleState(0, Rotation2d.fromDegrees(45)), new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), new SwerveModuleState(0, Rotation2d.fromDegrees(45))});}

    public ChassisSpeeds getCurrentSpeeds() {return ChassisSpeeds.fromWPIChassisSpeeds(kinematics.toChassisSpeeds(getModuleStates()));}

    private Translation2d[] getModuleTranslations() {
        double moduleDistance = Units.inchesToMeters(SWERVE_MODULE_DISTANCE_FROM_CENTER);
        Translation2d FL_ModulePosition = new Translation2d(moduleDistance, moduleDistance);
        Translation2d FR_ModulePosition = new Translation2d(moduleDistance, -moduleDistance);
        Translation2d BL_ModulePosition = new Translation2d(-moduleDistance, moduleDistance);
        Translation2d BR_ModulePosition = new Translation2d(-moduleDistance, -moduleDistance);
        return new Translation2d[] {FL_ModulePosition, FR_ModulePosition, BL_ModulePosition, BR_ModulePosition};
    }

    private static Drivetrain instance;

    public static Drivetrain getInstance() {
        if (instance == null) {
            instance = new Drivetrain();
        }
        return instance;
    }
}
