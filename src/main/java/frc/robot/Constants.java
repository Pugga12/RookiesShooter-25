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

    public static final class Shooter {
        public static final int bottomShooterMotorID = 15;
        public static final int topShooterMotorID = 16;

        public static final int feederMotorID = 17;
        
        public static final int pivotMotorID = 18;

        public static final int canCoderID = 19;

        public static final double pivotkS = 0.09256; // 0.09256
        public static final double pivotkG = 0.15116; // 0.15116
        public static final double pivotkV = 1.593; // 1.275
        public static final double pivotkA = 0;

        public static final double pivotkP = 5;
        public static final double pivotkI = 0;
        public static final double pivotkD = 0;

        public static final double topShooterkS = 0.19655; 
        public static final double topShooterkV = 0.00212586;
        public static final double topShooterkA = 0.00025997;

        public static final double topShooterkP = 0.01;
        public static final double topShooterkI = 0;
        public static final double topShooterkD = 0;

        public static final double bottomShooterkS = 0.13122;
        public static final double bottomShooterkV = 0.00198405;
        public static final double bottomShooterkA = 0.00071765;

        public static final double bottomShooterkP = 0.01;
        public static final double bottomShooterkI = 0;
        public static final double bottomShooterkD = 0;

        public static final double pivotTolerance = 0.01; // <1 degree
        public static final double shooterTolerance = 50;

        public static final double homePosition = 1.05; // 1.05

        public static final double feedSpeed = 0.85;
        public static final double feedVoltage = 10;

        public static final double feedToIntakeSpeed = -0.5;
        public static final double forkToIntakeSpeed = -0.5;
        public static final double shooterIntakeAngle = 0.95;
        public static final double shooterIntakeSpeed = -0.1;

        // Manual testing values
        public static final double topShooterSpeed = 0.4;
        public static final double bottomShooterSpeed = 0.4;
        public static final double manualShooterPivotSpeed = 0.05;

        public static final double manualShooterPivotVoltage = 0.75;

        public static final double ampForkSpeed = 0.8;
        public static final double ampFeedSpeed = 0.8;

        public static final double topShooterMaxRPM = 4500; 
        public static final double bottomShooterMaxRPM = 4800;

        public static final double kMaxVelocityRadPerSecond = 6;
        public static final double kMaxAccelerationRadPerSecSquared = 8;

        public static final double idleSpeed = 0.07;
    }
}