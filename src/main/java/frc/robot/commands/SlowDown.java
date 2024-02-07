
package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
/**
 * Changes a slowness factor 
 */
public class SlowDown extends InstantCommand {
  public SlowDown(DriveSubsystem subsystem) {
    addRequirements(subsystem);
  }
  @Override
  public void execute() {
    if(SwerveDrive.slowFactor == 1){
        SwerveDrive.slowFactor = 0.5;
    }
    else{
        SwerveDrive.slowFactor = 1;
    }
  }
}
