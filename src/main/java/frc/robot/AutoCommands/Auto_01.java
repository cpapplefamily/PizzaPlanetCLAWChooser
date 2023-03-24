// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm_To_Setpoint;
import frc.robot.commands.ChargingStationAutoBalance;
import frc.robot.commands.DriveForwardDistance;
import frc.robot.commands.Grabber_Close;
import frc.robot.commands.Grabber_Open;
import frc.robot.commands.Pivot_To_Setpoint;
import frc.robot.subsystems.Arm_MM;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Pivot_MM;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto_01 extends SequentialCommandGroup {
  /** Creates a new Auto_01. */
  public Auto_01(Pivot_MM m_pivot_MM, Arm_MM m_arm_MM, Grabber m_grabber, Drivetrain m_driverTrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        //new InstantCommand(() -> m_arm_MM.my_resetEncoder()),
        //new WaitCommand(0.1),
        //new InstantCommand(() -> m_pivot_MM.my_resetEncoder()),
       // new WaitCommand(0.1),
        new Pivot_To_Setpoint(70, m_pivot_MM),
        new WaitCommand(0.5),
        new Arm_To_Setpoint(37, m_arm_MM),
        new WaitCommand(0.0),
        new Grabber_Open(m_grabber),
        new WaitCommand(0.0),
        new Arm_To_Setpoint(10.0, m_arm_MM),
        new WaitCommand(1.0),
        new Grabber_Close(m_grabber),
        new WaitCommand(0.0),

        Commands.parallel(
          new Pivot_To_Setpoint(5, m_pivot_MM),
          new DriveForwardDistance(m_driverTrain, -7.5)
        ),
        new ChargingStationAutoBalance(m_driverTrain)

    );
  }
}
