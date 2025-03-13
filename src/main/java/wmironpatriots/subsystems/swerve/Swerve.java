package wmironpatriots.subsystems.swerve;

import wmironpatriots.Robot;
import wmironpatriots.Constants.MATRIXID;
import wmironpatriots.subsystems.swerve.module.Module;
import wmironpatriots.subsystems.swerve.module.ModuleIOComp;
import wmironpatriots.subsystems.swerve.module.Module.ModuleConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import wmironpatriots.utils.mechanismUtils.LoggedSubsystem;

public class Swerve implements LoggedSubsystem {
    // * CONSTANTS
    public static final double MASS_KG = 54.8847;
    public static final double MOI = 5.503;
    public static final double SIDE_LENGTH_METERS = 0.7239;
    public static final double BUMPER_WIDTH_METER = 0.0889; // TODO add value
    public static final double TRACK_WIDTH_METERS = 0.596201754;

    public static final Translation2d[] MODULE_LOCS =
        new Translation2d[] {
            new Translation2d(0.381, 0.381),
            new Translation2d(0.381, -0.381),
            new Translation2d(-0.381, 0.381),
            new Translation2d(-0.381, -0.381)
        };

    public static final double MAX_LINEAR_SPEED = 0.0;
    public static final double MAX_ANGULAR_SPEED = 0.0;

    public static final double LINEAR_P = 0.0;
    public static final double LINEAR_I = 0.0;
    public static final double LINEAR_D = 0.0;

    public static final double ANGULAR_P = 0.0;
    public static final double ANGULAR_I = 0.0;
    public static final double ANGULAR_D = 0.0;

    public static final double HEADING_P = 0.0;
    public static final double HEADING_I = 0.0;
    public static final double HEADING_D = 0.0;

    public static final ModuleConfig[] MODULE_CONFIGS = 
    new ModuleConfig[] {
        new ModuleConfig(MATRIXID.BL_PIVOT, MATRIXID.BL_DRIVE, MATRIXID.BL_CANCODER, 0.0, true),
        new ModuleConfig(MATRIXID.FL_PIVOT, MATRIXID.FL_DRIVE, MATRIXID.FL_CANCODER, 0.0, true),
        new ModuleConfig(MATRIXID.FR_PIVOT, MATRIXID.FR_DRIVE, MATRIXID.FR_CANCODER, 0.0, true),
        new ModuleConfig(MATRIXID.BR_PIVOT, MATRIXID.BR_DRIVE, MATRIXID.BR_CANCODER, 0.0, true)
    };

    private final Module[] modules;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odo;

    public Swerve() {
        modules = new ModuleIOComp[MODULE_CONFIGS.length];
        for (int i = 0; i < modules.length; i++) {
            modules[i] = new ModuleIOComp(MODULE_CONFIGS[i]);
        }

        kinematics = new SwerveDriveKinematics(MODULE_LOCS);
        odo = new SwerveDriveOdometry(kinematics, getHeading(), getSwerveModulePoses());
    }

    public Rotation2d getHeading() {
        return new Rotation2d();
    }

    public void runVelocities(ChassisSpeeds velocities) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(velocities);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, );
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getModuleState();
        }
        return states;
    }

    public SwerveModulePosition[] getSwerveModulePoses() {
        SwerveModulePosition[] poses = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            poses[i] = modules[i].getModulePose();
        }
        return poses;
    }
}