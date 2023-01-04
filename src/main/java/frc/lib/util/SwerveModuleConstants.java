package frc.lib.util;

public class SwerveModuleConstants {
  public final int driveMotorID;
  public final int angleMotorID;
  public final int pwmID;
  public final double angleOffset;

  /**
   * Swerve Module Constants to be used when creating swerve modules.
   *
   * @param driveMotorID
   * @param angleMotorID
   * @param pwmID
   * @param angleOffset
   */
  public SwerveModuleConstants(
      int driveMotorID, int angleMotorID, int pwmID, double angleOffset) {
    this.driveMotorID = driveMotorID;
    this.angleMotorID = angleMotorID;
    this.pwmID = pwmID;
    this.angleOffset = angleOffset;
  }
}
