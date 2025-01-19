// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Feed;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.AmpArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED2025;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
    //Declares/Creates new intake object
    public static Intake intake  = new Intake();
    public static Shooter shooter = new Shooter();
    public static Elevator elevator = new Elevator();
    public static AmpArm arm = new AmpArm();
    public static Feeder feeder = new Feeder();
    public static LED2025 led = new LED2025(40);
    DriverStation.Alliance alliance = DriverStation.Alliance.Blue;
    //Declares the controller
    private final CommandXboxController controller = new CommandXboxController(0);

    public RobotContainer() {
        configureBindings();
        if (DriverStation.isDSAttached()) {
            alliance = DriverStation.getAlliance().get();
        } 
        led.setDefaultCommand(led.setColor(alliance == DriverStation.Alliance.Blue ? Color.kBlue : Color.kRed));
    }

    private void configureBindings() {
        controller.rightBumper().whileTrue(new IntakeCommands());
        controller.leftBumper().whileTrue(
            Commands.parallel(
                Commands.runEnd(
                    () -> shooter.runFlywheels(1000, 1000),
                    () -> shooter.stopMotor(),
                    shooter
                ),
                new SequentialCommandGroup(
                    new WaitUntilCommand(() -> shooter.flywheelsAtSpeed()),
                    Commands.runEnd(
                        () -> intake.setMotorPower(0.5, 0.8),
                        () -> intake.stopMotor(),
                        intake
                    )
                ),
                new Feed()
            )
        );
        controller.a().onTrue(elevator.getHomeCommand());
        controller.y().onTrue(elevator.getFullExtendCommand());
        controller.povDown().onTrue(arm.getHomeCommand());
        controller.povUp().onTrue(arm.getAmpShootCommand());
        controller.b().onTrue(
            new SequentialCommandGroup(
                arm.getHandoffPosCommand(),
                new WaitUntilCommand(() -> arm.pivotAtGoal())
                .raceWith(new WaitCommand(1.5)),
                arm.handoffArmToIntake()
            )
        );
        controller.povLeft().whileTrue(shooter.feedToIntakeFromShooter());
        controller.povRight().onTrue(led.setColor(Color.kRed));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
