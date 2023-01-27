package frc3512.lib.swerve;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class SpartanSwerveAutonBuilder extends BaseAutoBuilder {
  private final SwerveDriveKinematics kinematics;
  private final PIDConstants xConstants;
  private final PIDConstants yConstants;
  private final PIDConstants rotationConstants;
  private final Consumer<SwerveModuleState[]> outputModuleStates;
  private final Consumer<Double> yawConsumer;
  private final Subsystem[] driveRequirements;

  public SpartanSwerveAutonBuilder(
      SwerveDriveKinematics kinematics,
      PIDConstants xConstants,
      PIDConstants yConstants,
      PIDConstants rotationConstants,
      Supplier<Pose2d> poseSupplier,
      Consumer<Pose2d> resetPose,
      Consumer<SwerveModuleState[]> outputModuleStates,
      Consumer<Double> yawConsumer,
      Map<String, Command> eventMap,
      Subsystem... driveRequirements) {
    super(poseSupplier, resetPose, eventMap, DrivetrainType.HOLONOMIC, true);

    this.kinematics = kinematics;
    this.xConstants = xConstants;
    this.yConstants = yConstants;
    this.rotationConstants = rotationConstants;
    this.yawConsumer = yawConsumer;
    this.outputModuleStates = outputModuleStates;
    this.driveRequirements = driveRequirements;
  }

  @Override
  public CommandBase followPath(PathPlannerTrajectory trajectory) {
    return new PPSwerveControllerCommand(
        trajectory,
        poseSupplier,
        kinematics,
        pidControllerFromConstants(xConstants),
        pidControllerFromConstants(yConstants),
        pidControllerFromConstants(rotationConstants),
        outputModuleStates,
        driveRequirements);
  }

  @Override
  public CommandBase fullAuto(List<PathPlannerTrajectory> pathGroup) {
    List<CommandBase> commands = new ArrayList<>();

    commands.add(resetPose(pathGroup.get(0)));

    for (PathPlannerTrajectory traj : pathGroup) {
      commands.add(stopEventGroup(traj.getStartStopEvent()));
      commands.add(followPathWithEvents(traj));
    }

    commands.add(stopEventGroup(pathGroup.get(pathGroup.size() - 1).getEndStopEvent()));
    commands.add(
        Commands.runOnce(
            () ->
                yawConsumer.accept(
                    pathGroup
                        .get(pathGroup.size() - 1)
                        .getEndState()
                        .holonomicRotation
                        .getDegrees()),
            driveRequirements));

    return Commands.sequence(commands.toArray(CommandBase[]::new));
  }
}
