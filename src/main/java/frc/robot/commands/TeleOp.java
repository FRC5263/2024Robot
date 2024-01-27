// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;

public class TeleOp extends CommandBase {
  //init variables
  DriveTrainSubsystem differentialDriveTrain;
  DriveTrainSubsystem driveTrainEncoders;
  Arm arm;
  ClawSubsystem claw;
  XboxController controller0;
  XboxController controller1;
  boolean driveTrainOnly;
  double xSpeed;
  double zSpeed;

  //docstring
  /**
   * creates the teleop command for the teleop period of the robot
   * @param differentialDriveTrain  the robots drive train motors
   * @param driveTrainEncoders  the robots drive train encoders
   * @param arm the arm motors
   * @param claw  the claw motors
   * @param controller0 driver controller
   * @param controller1 operator controller
   */
  public TeleOp(DriveTrainSubsystem differentialDriveTrain, Arm arm, ClawSubsystem claw, XboxController controller0, XboxController controller1) {
    System.out.print("Creating new Teleop\n");

    //assingning method inputs to class variables
    this.differentialDriveTrain = differentialDriveTrain;
    this.arm = arm;
    this.claw = claw;
    this.controller0 = controller0;
    this.controller1 = controller1;

    System.out.print("Teleop created\n");

    //setting drive train only to false
    driveTrainOnly = false;
  }

  //docstring
  /**
   * creates a teleop command for the teleop period that only has the drive train
   * @param differentialDriveTrian the robots drive train motors 
   * @param driveTrainEncoders the robots drive train encoders
   * @param controller0 driver controller
   * @param controller1 operator controller
   */
  public TeleOp(DriveTrainSubsystem differentialDriveTrian, XboxController controller0, XboxController controller1) {
    System.out.print("Creating new Drive Train only TeleOp\n");

    //assinging method inputs to class variables
    this.differentialDriveTrain = differentialDriveTrian;
    this.controller0 = controller0;
    this.controller1 = controller1;

    System.out.print("TeleOp Created!!!!!!\n");

    //setting drive train only to true
    driveTrainOnly = true;
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.print("Teleop initialized\n");
    xSpeed = 0;
    zSpeed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //deadzones x dircetion
    if (controller0.getRawAxis(1) < .301 && controller0.getRawAxis(1) > -.301) {
      xSpeed = 0;
    } else {
      xSpeed = -controller0.getRawAxis(1) - (controller0.getRawAxis(3)*.5);
    }

    //deadzones y direction
    if (controller0.getRawAxis(0) < .301 && controller0.getRawAxis(0) > -.301) {
      zSpeed = 0;
    } else {
      zSpeed = -controller0.getRawAxis(0) - (controller0.getRawAxis(3)*.5);
    }

    //drive train uses curvature drive, one stick operation
    differentialDriveTrain.DriveDifferentialCurvature(xSpeed, zSpeed);

    //left vibrator indicates robot is rotating
    controller0.setRumble(RumbleType.kRightRumble, controller0.getRawAxis(0) * 2);

    //checks if drive train only is true and the decidies wether or not to execute code for the other subsystems
    if (driveTrainOnly == false) {

      //moves wrist
      if (controller1.getRawButton(7) == true){
        claw.setwristmotor(.4);
      } else if (controller1.getRawButton(8) == true) {
        claw.setwristmotor(-.4);
      } else if (controller1.getRawButton(7) == true && controller1.getRawButton(8) == true)  {
        claw.setwristmotor(0);
      } else if (controller1.getRawButton(7) == false && controller1.getRawButton(8) == false) {
        claw.setwristmotor(0);
      } else {
        claw.setwristmotor(0);
      }
      
      
      //moves arm up and down
      if (controller1.getPOV(0) == 0) {
        arm.setArmSpeed(-.3);
        System.out.println("dpad up\n");
      } else if (controller1.getPOV(0) == 180) {
        arm.setArmSpeed(.3);
        System.out.println("dpad down\n");
      } else if (controller1.getPOV(0) == -1 ) {
        arm.setArmSpeed(0);
        System.out.println("dpad no direction");
      }  else {
        arm.setArmSpeed(-controller1.getRawAxis(1));
      }

      //arm length
      
      //arm lengths blblblblblblblbllblblllblbl
      if (controller1.getRawButton(2) == true) {
        arm.setArmLength(.6);
        System.out.println("button 1 pressed\n");
      } else if(controller1.getRawButton(4) == true) {
        arm.setArmLength(-.6);
        System.out.println("button 2 pressed\n");
      } else if(controller1.getRawButton(2) == true && controller1.getRawButton(4) == true || controller1.getRawButton(2) == false && controller1.getRawButton(4) == false) {
        arm.setArmLength(0);
      }

      //intake shit
      if (controller1.getRawButton(1) == true) {
        claw.setintakemotors(.5);
      } else if (controller1.getRawButton(3) == true) {
        claw.setintakemotors(-.5);
      } else if (controller1.getRawButton(1) == true && controller1.getRawButton(3) == true || controller1.getRawButton(1) == false && controller1.getRawButton(3) == false) {
        claw.setintakemotors(0);
      } 
     }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //indicates teleop is over
    System.out.print("TeleOp ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

