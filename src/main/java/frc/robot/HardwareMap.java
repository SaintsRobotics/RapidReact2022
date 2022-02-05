package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveModule;

/** File for storing the hardware of the robot. */
public class HardwareMap {
  /** File for storing the hardware of the drivetrain. */
  public class SwerveDrivetrainHardware {
    public SwerveModule frontLeft = new SwerveModule(
        swerveModuleHardware.frontLeftDriveMotor,
        swerveModuleHardware.frontLeftTurningMotor,
        swerveModuleHardware.frontLeftAbsoluteEncoder);
    public SwerveModule rearLeft = new SwerveModule(
        swerveModuleHardware.rearLeftDriveMotor,
        swerveModuleHardware.rearLeftTurningMotor,
        swerveModuleHardware.rearLeftAbsoluteEncoder);
    public SwerveModule frontRight = new SwerveModule(
        swerveModuleHardware.frontRightDriveMotor,
        swerveModuleHardware.frontRightTurningMotor,
        swerveModuleHardware.frontRightAbsoluteEncoder);
    public SwerveModule rearRight = new SwerveModule(
        swerveModuleHardware.rearRightDriveMotor,
        swerveModuleHardware.rearRightTurningMotor,
        swerveModuleHardware.rearRightAbsoluteEncoder);

    public AHRS gyro = new AHRS();
  }

  /** File for storing the hardware of the swerve module. */
  public class SwerveModuleHardware {
    public CANSparkMax frontLeftTurningMotor = new CANSparkMax(
        SwerveConstants.kFrontLeftTurningMotorPort, MotorType.kBrushless);
    public CANSparkMax frontLeftDriveMotor = new CANSparkMax(
        SwerveConstants.kFrontLeftDriveMotorPort, MotorType.kBrushless);
    public AbsoluteEncoder frontLeftAbsoluteEncoder = new AbsoluteEncoder(
        SwerveConstants.kFrontLeftTurningEncoderPort,
        SwerveConstants.kFrontLeftTurningEncoderReversed,
        SwerveConstants.kFrontLeftTurningEncoderOffset);

    public CANSparkMax rearLeftTurningMotor = new CANSparkMax(
        SwerveConstants.kRearLeftTurningMotorPort, MotorType.kBrushless);
    public CANSparkMax rearLeftDriveMotor = new CANSparkMax(
        SwerveConstants.kRearLeftDriveMotorPort, MotorType.kBrushless);
    public AbsoluteEncoder rearLeftAbsoluteEncoder = new AbsoluteEncoder(
        SwerveConstants.kRearLeftTurningEncoderPort,
        SwerveConstants.kRearLeftTurningEncoderReversed,
        SwerveConstants.kRearLeftTurningEncoderOffset);

    public CANSparkMax frontRightTurningMotor = new CANSparkMax(
        SwerveConstants.kFrontRightTurningMotorPort, MotorType.kBrushless);
    public CANSparkMax frontRightDriveMotor = new CANSparkMax(
        SwerveConstants.kFrontRightDriveMotorPort, MotorType.kBrushless);
    public AbsoluteEncoder frontRightAbsoluteEncoder = new AbsoluteEncoder(
        SwerveConstants.kFrontRightTurningEncoderPort,
        SwerveConstants.kFrontRightTurningEncoderReversed,
        SwerveConstants.kFrontRightTurningEncoderOffset);

    public CANSparkMax rearRightTurningMotor = new CANSparkMax(
        SwerveConstants.kRearRightTurningMotorPort, MotorType.kBrushless);
    public CANSparkMax rearRightDriveMotor = new CANSparkMax(
        SwerveConstants.kRearRightDriveMotorPort, MotorType.kBrushless);
    public AbsoluteEncoder rearRightAbsoluteEncoder = new AbsoluteEncoder(
        SwerveConstants.kRearRightTurningEncoderPort,
        SwerveConstants.kRearRightTurningEncoderReversed,
        SwerveConstants.kRearRightTurningEncoderOffset);
  }

  public class IntakeHardware {
    public CANSparkMax intakeController;
    public CANSparkMax armController;
    public CANSparkMax feederController;

    public IntakeHardware() {
            intakeController = new CANSparkMax(25, null);
            armController = new CANSparkMax(24, null);
            feederController = new CANSparkMax(23, null);
            armController.setIdleMode(IdleMode.kBrake);
    }
}

  public SwerveModuleHardware swerveModuleHardware;
  public SwerveDrivetrainHardware swerveDrivetrainHardware;
  public IntakeHardware intakeHardware;

  /** Creates a new {@link HardwareMap}. */
  public HardwareMap() {
    swerveModuleHardware = new SwerveModuleHardware();
    swerveDrivetrainHardware = new SwerveDrivetrainHardware();
    intakeHardware = new IntakeHardware();
  }
}
