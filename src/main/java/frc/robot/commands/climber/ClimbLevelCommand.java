package frc.robot.commands.climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.turret.MoveSpinnerCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.Climber.ControlLoopType;

import static frc.robot.Constants.Climber.*;

public class ClimbLevelCommand extends SequentialCommandGroup{

    private boolean firstTime = true;

    public ClimbLevelCommand(Climber c, Drivetrain d, Turret t, BooleanSupplier continueSupplier) {
        addRequirements(c, t);

        addCommands(
            new InstantCommand(() -> d.setUseOdometry(false)),
            new ConditionalCommand(
                new MoveSpinnerCommand(t, -10).withTimeout(3), //Move the turret out of the way
                new InstantCommand(),
                () -> t.knowsLocation()
            ),
            new ParallelCommandGroup(
                new MoveClimberCommand(c, d, ClimberState.FullExtension, ControlLoopType.NoLoad, true), //Fully extend the climber
                new ConditionalCommand(
                    new ExtendClimberSolenoidsCommand(c),
                    new SequentialCommandGroup(
                        c.waitForMovementCommand(EXTEND_SOLENOIDS_THRESHOLD),
                        new ExtendClimberSolenoidsCommand(c),
                        c.waitForMovementCommand(RETRACT_SOLENOIDS_THRESHOLD),
                        new RetractClimberSolenoidsCommand(c)
                    ),
                    () -> firstTime
                )
            ),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new FunctionalCommand(() -> {}, () -> {}, (interrupted) -> {}, continueSupplier, c), //Wait for the supplier to be true
                    new ExtendClimberSolenoidsCommand(c), //Extend the soleonids to reach the next bar
                    new WaitCommand(1) //TODO: Get this to the actual right time
                ),
                new InstantCommand(() -> {firstTime = false;}),
                () -> !firstTime
            ),
            new FunctionalCommand(() -> {}, () -> {}, (interrupted) -> {}, continueSupplier, c),
            new MoveClimberCommand(c, d, ClimberState.Lowered, ControlLoopType.Load, false),
            new RetractClimberSolenoidsCommand(c)
        );
    }
    
}
