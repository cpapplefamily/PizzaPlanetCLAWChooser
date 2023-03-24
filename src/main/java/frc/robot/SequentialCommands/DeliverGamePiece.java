// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SequentialCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm_To_Setpoint;
import frc.robot.commands.Grabber_Open;
import frc.robot.commands.Pivot_To_Setpoint;
import frc.robot.subsystems.Arm_MM;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.Pivot_MM;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DeliverGamePiece extends SequentialCommandGroup {
  /** Creates a new DeliverGamePiece. */
  public DeliverGamePiece(Pivot_MM m_pivot_MM, Arm_MM m_arm_MM, Grabber m_grabber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new Pivot_To_Setpoint(56, m_pivot_MM),
      new WaitCommand(0.5), 
      //new Pivot_To_Setpoint(58, m_pivot_MM),
      new Arm_To_Setpoint(20, m_arm_MM),
      new WaitCommand(0.0),
      new Grabber_Open(m_grabber),
      new WaitCommand(0.0)

    );
  }
}
