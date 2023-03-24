// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.AutoCommands.Auto_01;
import frc.robot.SequentialCommands.DeliverGamePiece;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Arm_To_Setpoint;
//import frc.robot.commands.Autos;
import frc.robot.commands.ChargingStationAutoBalance;
import frc.robot.commands.DriveForwardDistance;
import frc.robot.commands.DriveToTrackedTarget;
//import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Pivot_To_Setpoint;
import frc.robot.commands.RotateAngle;
import frc.robot.commands.TurnToTrackedTarget;
import frc.robot.subsystems.Arm_MM;
import frc.robot.subsystems.Drivetrain;
//import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Pivot_MM;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SendableChooser<Command> autoCommandSelector = new SendableChooser<>();
  private final Drivetrain drivetrain = new Drivetrain();
  private final Pivot_MM m_pivot_MM = new Pivot_MM();
  private final Arm_MM m_arm_MM = new Arm_MM();
  private final Grabber grabber = new Grabber();
  private final Vision vision = new Vision();


  private final CommandXboxController driverController = new CommandXboxController(Constants.DRIVER_CONTROLLER);
  public static final CommandJoystick extremeController = new CommandJoystick(Constants.EXTREME_CONTROLLER);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    addAutoCommands();
    SmartDashboard.putData(autoCommandSelector);
    drivetrain.setDefaultCommand(new ArcadeDrive(
      drivetrain, 
      () -> driverController.getLeftY()*.80,
      () -> driverController.getRightX()*-0.6  //change as needed
    )
    );


  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
   // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    //DRIVER CONTROLLER
    driverController.povRight().onTrue(new RotateAngle(drivetrain, 90));
    driverController.povLeft().onTrue(new RotateAngle(drivetrain, -90));
    driverController.povDown().onTrue(new RotateAngle(drivetrain, 180));
    driverController.a().whileTrue(new TurnToTrackedTarget(drivetrain, vision));
    driverController.b().whileTrue(new DriveToTrackedTarget(2, true));
    driverController.leftBumper().whileTrue(new ChargingStationAutoBalance(drivetrain));
    driverController.x().onTrue(new InstantCommand(() -> drivetrain.invert_drivetrain()));
    driverController.rightTrigger().onTrue(new InstantCommand(()-> drivetrain.toggleSnailSpeed()));
    driverController.rightTrigger().onFalse(new InstantCommand(()-> drivetrain.toggleSnailSpeed()));


    //EXTREME CONTROLLER
    
    extremeController.button(1).onTrue(new InstantCommand(() -> grabber.toggle()));
    extremeController.button(9).onTrue(new DeliverGamePiece(m_pivot_MM, m_arm_MM, grabber));

    extremeController.button(7).onTrue(new Arm_To_Setpoint(30, m_arm_MM));
    extremeController.button(8).onTrue(new Pivot_To_Setpoint(56, m_pivot_MM));

    extremeController.button(3).onTrue(new Arm_To_Setpoint(1, m_arm_MM));
    extremeController.button(4).onTrue(new Pivot_To_Setpoint(5, m_pivot_MM));

    extremeController.button(10).onTrue(new SequentialCommandGroup(
      new Arm_To_Setpoint(1, m_arm_MM),
      new Pivot_To_Setpoint(5, m_pivot_MM),
      new InstantCommand(() -> grabber.closeGrabber())
    ));

    extremeController.button(12).onTrue(
      new ParallelCommandGroup(
        new InstantCommand(() -> m_arm_MM.my_resetEncoder()),
        new InstantCommand(() -> m_pivot_MM.my_resetEncoder())
      )
    );
   //extremeController.button(3).whileTrue(new ManualRetract(arm, -Constants.ARM_POWER));
    //extremeController.button(3).onFalse(new InstantCommand(() -> arm.stop()));
    //extremeController.button(4).whileTrue(new ManualExtend(arm, Constants.ARM_POWER));
    //extremeController.button(4).onFalse(new InstantCommand(() -> arm.stop()));
    //MANUAL CONTROL:  LOWER/RAISE PIVOT ARM
    //extremeController.button(5).whileTrue(new ManualPivotDown(pivot, -Constants.PIVOT_POWER));
    //extremeController.button(5).onFalse(new InstantCommand(() -> pivot.stop()));
    //extremeController.button(6).whileTrue(new ManualPivotUp(pivot, Constants.PIVOT_POWER));
    //extremeController.button(6).onFalse(new InstantCommand(() -> arm.stop()));  
    //MANUALLY ZERO ARM AND PIVOT SENSORS



  }


  private void addAutoCommands()  {
    autoCommandSelector.addOption("Auto Nothing", new WaitCommand(0));
    autoCommandSelector.setDefaultOption("Auto Center", new Auto_01(m_pivot_MM, m_arm_MM, grabber, drivetrain));
    
  }
  //autoCommandSelector.addOption();

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return Autos.exampleAuto(m_exampleSubsystem);
    return autoCommandSelector.getSelected();

  }

  public Command my_Disable_All_MotionMagic(){
    return Commands.parallel(new InstantCommand(()-> m_pivot_MM.my_PercentOutput_Run(0),m_pivot_MM).ignoringDisable(true),
                              new WaitCommand(0),
                              new InstantCommand(()-> m_arm_MM.my_PercentOutput_Run(0),m_arm_MM).ignoringDisable(true),
                              new WaitCommand(0)
                               // Add each Subsystem here
                              );
  }
}
