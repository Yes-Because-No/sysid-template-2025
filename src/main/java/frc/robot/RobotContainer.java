// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
  Elevator elevator = new Elevator();
  CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    controller.x().whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
    controller.y().whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
    controller.a().whileTrue(elevator.sysIdDynamic(Direction.kForward));
    controller.b().whileTrue(elevator.sysIdDynamic(Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
