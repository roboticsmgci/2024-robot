
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SwerveSubsystem;
/**
 * Changes a slowness factor 
 */
public class SlowDown extends InstantCommand {
  public SlowDown(SwerveSubsystem subsystem) {
    addRequirements(subsystem);
  }
  @Override
  public void execute() {
    if(DefaultDrive.slowFactor == 1){
        DefaultDrive.slowFactor = 0.5;
    }
    else{
        DefaultDrive.slowFactor = 1;
    }
  }
}
