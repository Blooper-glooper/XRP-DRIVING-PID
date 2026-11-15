// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Subsystems.Drivetrain.Drivetrain;

public class RobotContainer {

  private final Drivetrain drivetrain;
  private final CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    drivetrain = new Drivetrain();
    configureBindings();
  }

  private void configureBindings() {
    RobotModeTriggers.disabled().onTrue(Commands.runOnce(() -> drivetrain.stop(), drivetrain));

    controller.a().onTrue(Commands.runOnce(() -> drivetrain.driveDistance(4), drivetrain));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
