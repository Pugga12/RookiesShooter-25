package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSTalonFXSwerveConstants;

public class Constants {
    public static final String canivoreName = "canivore";
    public static final String pigeonCanName = "canivore"; //Pigeon is on separate can loop than others

    public static final class AmpArm {
        public static final int shootMotorID = 23;
        public static final int pivotMotorID = 24;

        public static final int canCoderID = 25; 
        public static final double pivotkS = 0.14;
        public static final double pivotkG = 0.195;
        public static final double pivotkV = 0.85;

        public static final double pivotkP = 1.5;
        public static final double pivotkI = 0;
        public static final double pivotkD = 0;

        public static final double integratorZone = 0;

        public static final double pivotTolerance = 0.01;

        public static final double kMaxVelocityRadPerSecond = 13;
        public static final double kMaxAccelerationRadPerSecSquared = 15;

        public static final double armOffset = -Math.PI / 2;

        // Score positions in radians
        public static final double homePosition = -2.4;
        public static final double danglePosition = -Math.PI / 2;
        public static final double handoffPosition = -2.4;
        public static final double climbIdlePosition = -1.1;
        public static final double ampShootPosition = -0.55;
        public static final double trapPosition = 1.37;

        public static final double manualArmPivotSpeed = 0.1;
        public static final double handoffSpeed = -0.3;
        public static final double shootSpeed = 0.8;
    }
}