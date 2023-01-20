package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;
  public AHRS gyro;

  //testing for now, bad practice boo :(
  public Rotation2d[] angleOffsets;

  public Swerve() {
    gyro = new AHRS(Constants.Swerve.pigeonID);
    zeroGyro();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

      /*angleOffsets = new Rotation2d[] {
        Rotation2d.fromDegrees(90.0),
        Rotation2d.fromDegrees(270.0),
        Rotation2d.fromDegrees(140.0),
        Rotation2d.fromDegrees(125.0)
      };*/
      angleOffsets = new Rotation2d[] {
        Rotation2d.fromDegrees(0.0),
        Rotation2d.fromDegrees(0.0),
        Rotation2d.fromDegrees(0.0),
        Rotation2d.fromDegrees(8.0)
      };

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(),
    new SwerveModulePosition[] {
      mSwerveMods[0].getPosition(),
      mSwerveMods[1].getPosition(),
      mSwerveMods[2].getPosition(),
      mSwerveMods[3].getPosition()
    });

  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
    //String text = "";
    for (SwerveModule mod : mSwerveMods) {
      Rotation2d currentAngle = swerveModuleStates[mod.moduleNumber].angle;
      swerveModuleStates[mod.moduleNumber].angle = currentAngle.plus(angleOffsets[mod.moduleNumber]);
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
      //text += Integer.toString(mod.moduleNumber);
      //text += currentAngle.plus(angleOffsets[mod.moduleNumber]);
    }
    //System.out.println("swerve states: "+text);
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), new SwerveModulePosition[] {
      mSwerveMods[0].getPosition(),
      mSwerveMods[1].getPosition(),
      mSwerveMods[2].getPosition(),
      mSwerveMods[3].getPosition()
    }, pose);
  }

  public void resetModuleZeros() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void zeroGyro() {
    gyro.zeroYaw();
  }

  public Rotation2d getYaw() {
    return (Constants.Swerve.invertGyro)
        ? Rotation2d.fromDegrees(360 - (gyro.getYaw() + 180))
        : Rotation2d.fromDegrees(gyro.getYaw() + 180);
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), new SwerveModulePosition[] {
      mSwerveMods[0].getPosition(),
      mSwerveMods[1].getPosition(),
      mSwerveMods[2].getPosition(),
      mSwerveMods[3].getPosition()
    });

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
  }
}
