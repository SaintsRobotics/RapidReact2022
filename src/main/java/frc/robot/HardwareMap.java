package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveModule;

/** File for storing the hardware of the robot. */
public class HardwareMap {
    /** File for storing the hardware of the drivetrain. */
    public class SwerveDrivetrainHardware {
        public SwerveModule frontLeft = new SwerveModule(swerveModuleHardware,
                swerveModuleHardware.frontLeftDriveMotor, swerveModuleHardware.frontLeftTurningMotor,
                swerveModuleHardware.frontLeftEncoder);
        public SwerveModule rearLeft = new SwerveModule(swerveModuleHardware,
                swerveModuleHardware.rearLeftDriveMotor, swerveModuleHardware.rearLeftTurningMotor,
                swerveModuleHardware.rearLeftEncoder);
        public SwerveModule frontRight = new SwerveModule(swerveModuleHardware,
                swerveModuleHardware.frontRightDriveMotor, swerveModuleHardware.frontRightTurningMotor,
                swerveModuleHardware.frontRightEncoder);
        public SwerveModule rearRight = new SwerveModule(swerveModuleHardware,
                swerveModuleHardware.rearRightDriveMotor, swerveModuleHardware.rearRightTurningMotor,
                swerveModuleHardware.rearRightEncoder);

        public AHRS gyro = new AHRS();
    }

    /** File for storing the hardware of the swerve module. */
    public class SwerveModuleHardware {
        public CANSparkMax frontLeftTurningMotor = new CANSparkMax(
                SwerveConstants.kFrontLeftTurningMotorPort, MotorType.kBrushless);
        public CANSparkMax frontLeftDriveMotor = new CANSparkMax(
                SwerveConstants.kFrontLeftDriveMotorPort, MotorType.kBrushless);
        public WPI_CANCoder frontLeftEncoder = new WPI_CANCoder(SwerveConstants.kFrontLeftTurningEncoderPort);

        public CANSparkMax rearLeftTurningMotor = new CANSparkMax(
                SwerveConstants.kRearLeftTurningMotorPort, MotorType.kBrushless);
        public CANSparkMax rearLeftDriveMotor = new CANSparkMax(
                SwerveConstants.kRearLeftDriveMotorPort, MotorType.kBrushless);
        public WPI_CANCoder rearLeftEncoder = new WPI_CANCoder(SwerveConstants.kRearLeftTurningEncoderPort);

        public CANSparkMax frontRightTurningMotor = new CANSparkMax(
                SwerveConstants.kFrontRightTurningMotorPort, MotorType.kBrushless);
        public CANSparkMax frontRightDriveMotor = new CANSparkMax(
                SwerveConstants.kFrontRightDriveMotorPort, MotorType.kBrushless);
        public WPI_CANCoder frontRightEncoder = new WPI_CANCoder(SwerveConstants.kFrontRightTurningEncoderPort);

        public CANSparkMax rearRightTurningMotor = new CANSparkMax(
                SwerveConstants.kRearRightTurningMotorPort, MotorType.kBrushless);
        public CANSparkMax rearRightDriveMotor = new CANSparkMax(
                SwerveConstants.kRearRightDriveMotorPort, MotorType.kBrushless);
        public WPI_CANCoder rearRightEncoder = new WPI_CANCoder(SwerveConstants.kRearRightTurningEncoderPort);
        }

    public SwerveModuleHardware swerveModuleHardware;
    public SwerveDrivetrainHardware swerveDrivetrainHardware;

    /** Creates a new {@link HardwareMap}. */
    public HardwareMap() {
        swerveModuleHardware = new SwerveModuleHardware();
        swerveDrivetrainHardware = new SwerveDrivetrainHardware();
    }
}
