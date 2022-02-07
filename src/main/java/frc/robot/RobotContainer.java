// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AimCommand;
import frc.robot.commands.FieldDriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.LowerCommand;
import frc.robot.commands.RaiseCommand;
import frc.robot.commands.RobotDriveCommand;
import frc.robot.subsystems.DriveTrainSubsystems;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Joystick driver = new Joystick(0);

  DriveTrainSubsystems driveSub = new DriveTrainSubsystems();
  TurretSubsystem turretSub = new TurretSubsystem();
  LimelightSubsystem limeSub = new LimelightSubsystem();
  HopperSubsystem hopperSub = new HopperSubsystem();
  IntakeSubsystem intakeSub = new IntakeSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    // configureButtonBindings();

    // turretSub.setDefaultCommand(new AimCommand(turretSub, limeSub));

    // creates conditional command
    // ConditionalCommand driveCommand = new ConditionalCommand(
    //     new RobotDriveCommand
    //     (
    //       () -> modifyAxis(driver.getRawAxis(1)) * DriveTrainSubsystems.maxVelocityPerSecond,
    //       () -> modifyAxis(driver.getRawAxis(0)) * DriveTrainSubsystems.maxVelocityPerSecond,
    //       () -> modifyAxis(driver.getRawAxis(4)) * DriveTrainSubsystems.maxAngularVelocityPerSecond,
    //       driveSub
    //     ),
    //     new FieldDriveCommand
    //     (
    //       () -> modifyAxis(driver.getRawAxis(1)) * DriveTrainSubsystems.maxVelocityPerSecond,
    //       () -> modifyAxis(driver.getRawAxis(0)) * DriveTrainSubsystems.maxVelocityPerSecond,
    //       () -> modifyAxis(driver.getRawAxis(4)) * DriveTrainSubsystems.maxAngularVelocityPerSecond,
    //       driveSub
    //     ),
    //     () -> driver.getRawButtonPressed(1));
    // () -> ture;

    //passes conditional command into the default command of drive
    driveSub.setDefaultCommand(new FieldDriveCommand
    (
      () -> modifyAxis(driver.getRawAxis(1)) * DriveTrainSubsystems.maxVelocityPerSecond,
      () -> modifyAxis(driver.getRawAxis(0)) * DriveTrainSubsystems.maxVelocityPerSecond,
      () -> modifyAxis(driver.getRawAxis(2)) * DriveTrainSubsystems.maxAngularVelocityPerSecond,
      driveSub
    ));
        
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton reset = new JoystickButton(driver, 2);
    // JoystickButton intake = new JoystickButton(driver, 3);
    // JoystickButton raise = new JoystickButton(driver, 6);
    // JoystickButton lower = new JoystickButton(driver, 5);


    // raise.whenActive(new InstantCommand(() -> intakeSub.setRaiseMotor(0.1), intakeSub));
    // lower.whenActive(new InstantCommand(() -> intakeSub.setRaiseMotor(-0.1), intakeSub));
    // raise.whenPressed(new RaiseCommand(intakeSub));
    // lower.whenPressed(new LowerCommand(intakeSub));
    // intake.whenActive(new IntakeCommand(driveSub, intakeSub));
    reset.whenPressed(new InstantCommand(driveSub::zeroGyroscope, driveSub));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // driveSub.resetOdometry(Robot.autoTrajectory.getInitialPose());

    // var thetaController = new ProfiledPIDController(Constants.ThetaController, .6, .1, Constants.thetaControllerConstraints);
    //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //   Robot.autoTrajectory,
    //   driveSub::getPose, // Functional interface to feed supplier
    //   Constants.m_kinematics,
    //   new PIDController(Constants.XController, 0, 0),
    //   new PIDController(Constants.YController, 0, 0),
    //   thetaController,
    //   // (SwerveModuleState[] states) -> driveSub.setModules(states),
    //   driveSub::setModules,
    //   driveSub);

    // // Reset odometry to the starting pose of the trajectory.
    // driveSub.resetOdometry(Robot.autoTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> driveSub.drive(new ChassisSpeeds(0,0,0)));
    return null;
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.2) {
        return (value - deadband) / (1.0 - deadband);
      } 
      else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.1);

    // Square the axis
    value = Math.copySign(value * value, value);

    // SmartDashboard.putNumber("left x", driver.getLeftX());
    // SmartDashboard.putNumber("left y", driver.getLeftY());
    // SmartDashboard.putNumber("right x", driver.getRawAxis(4));

    return value;
  }
}

// this is where possible scrap code is stored

        // this is possible conditional command code
        // () -> driveSub.robotOrientedDrive(
        //   () -> modifyAxis(driver.getRawAxis(1)) * DriveTrainSubsystems.maxVelocityPerSecond,
        //   () -> modifyAxis(driver.getRawAxis(0)) * DriveTrainSubsystems.maxVelocityPerSecond,
        //   () -> modifyAxis(driver.getRawAxis(4)) * DriveTrainSubsystems.maxAngularVelocityPerSecond),
        // () -> driveSub.fieldOrientedDrive(
        //   () -> modifyAxis(driver.getRawAxis(1)) * DriveTrainSubsystems.maxVelocityPerSecond,
        //   () -> modifyAxis(driver.getRawAxis(0)) * DriveTrainSubsystems.maxVelocityPerSecond,
        //   () -> modifyAxis(driver.getRawAxis(4)) * DriveTrainSubsystems.maxAngularVelocityPerSecond),
        // true));