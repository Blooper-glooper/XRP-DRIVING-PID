package frc.robot.Subsystems.Drivetrain;

public class DrivetrainConstants {
    public static final int leftMotorID = 0;
    public static final int rightMotorID = 1;

    public static final boolean isRightInverted = true;

    public static final int leftEncoderA = 4;
    public static final int leftEncoderB = 5;

    public static final int rightEncoderA = 6;
    public static final int rightEncoderB = 7;

    public static final double drive_kP = 20;
    public static final double drive_kI = 0;
    public static final double drive_kD = 0;

    public static final double turn_kP = 1.75;
    public static final double turn_kI = 0;
    public static final double turn_kD = 0;

    public static final double kS = 6.4;
    public static final double kV = 0;

    public static final double distancePerPulse = (Math.PI * 0.06) / 360;

    public static final double driveTolerance = 0.01;
}
