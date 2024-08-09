package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveModule[] mSwerveMods;
    public Field2d m_Field = new Field2d();//Creates a field object to visualize the robot pose in smartdashboard. 
    public Pigeon2 gyro;

    private final SwerveDrivePoseEstimator m_PoseEstimator;
    
    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        m_PoseEstimator = 
            new SwerveDrivePoseEstimator(
                Constants.Swerve.swerveKinematics, 
                getGyroYaw(), 
                getModulePositions(), 
                new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
                    VecBuilder.fill(0.05, 0.05, Math.toRadians(5)),
                    VecBuilder.fill(0.5, 0.5, Math.toRadians(30)));
    }

    /*Drive Function */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /*Get Info Functions */
    public Pose2d getPose() {return m_PoseEstimator.getEstimatedPosition();}
    public Rotation2d getGyroYaw() {return Rotation2d.fromDegrees(gyro.getYaw().getValue());}
    public Rotation2d getHeading(){return getPose().getRotation();}
    
    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;}

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;}

    /*Setter Funtions */
    public void setPose(Pose2d pose) {m_PoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);}
    public void setHeading(Rotation2d heading){
        m_PoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading)); 
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) { //Used in example auto
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);   
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    /*Zero/Reset Functions */
    public void zeroHeading(){
        m_PoseEstimator.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }
    
    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        updateVisionLocalization();
        m_Field.setRobotPose(m_PoseEstimator.getEstimatedPosition());
        SmartDashboard.putData("Feild", m_Field);

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    /*Vision Functions */
    public void updateVisionLocalization(){
        m_PoseEstimator.update(getGyroYaw(), getModulePositions());

        boolean useMegaTag2 = Constants.useMegaTag2; //This is a work around because otherwise I get dead code warnings and it looks bad. 
        boolean doRejectUpdate = false;
        if(useMegaTag2 == false){
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.limelightName);
            
            if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1){
                if(mt1.rawFiducials[0].ambiguity > .7){doRejectUpdate = true;}
                if(mt1.rawFiducials[0].distToCamera > 3){doRejectUpdate = true;}
            }

            if(mt1.tagCount == 0){doRejectUpdate = true;}

            if(!doRejectUpdate){
                m_PoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
                m_PoseEstimator.addVisionMeasurement(
                    mt1.pose,
                    mt1.timestampSeconds);
            }
        }
        else if (useMegaTag2 == true){
            LimelightHelpers.SetRobotOrientation(Constants.limelightName, m_PoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.limelightName);
            if(Math.abs(gyro.getRate()) > 720) {doRejectUpdate = true;}// If the angular velocity is greater than 720 degrees per second, ignore vision updates
            if(mt2.tagCount == 0){doRejectUpdate = true;}
            if(!doRejectUpdate){
                m_PoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
                m_PoseEstimator.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
            }
        }
    }
}