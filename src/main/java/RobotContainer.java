// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.TeleOp;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //drivetrain creation
  private DriveTrainSubsystem driveTrain = new DriveTrainSubsystem(OperatorConstants.leftMotor1, OperatorConstants.leftMotor2, OperatorConstants.rightMotor1, OperatorConstants.rightMotor2);
  private Arm arm = new Arm(OperatorConstants.leftArmMotor, OperatorConstants.rightArmMotor, OperatorConstants.winchMotor);
  private ClawSubsystem claw = new ClawSubsystem(OperatorConstants.intakeMotor1, OperatorConstants.intakeMotor2, OperatorConstants.wristMotor);

  //creates new teleop command
  private final Command m_teleOp = new TeleOp(driveTrain, arm, claw, OperatorConstants.controller0, OperatorConstants.controller1);
  private final Command m_Auton = new Auton(driveTrain);

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_Auton;
  }

  public Command getTelOpCommand() {
    return m_teleOp;
  }
}
