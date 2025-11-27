# ------------------------------------------------------------------------ #
#      o-o      o                o                                         #
#     /         |                |                                         #
#    O     o  o O-o  o-o o-o     |  oo o--o o-o o-o                        #
#     \    |  | |  | |-' |   \   o | | |  |  /   /                         #
#      o-o o--O o-o  o-o o    o-o  o-o-o--O o-o o-o                        #
#             |                           |                                #
#          o--o                        o--o                                #
#                        o--o      o         o                             #
#                        |   |     |         |  o                          #
#                        O-Oo  o-o O-o  o-o -o-    o-o o-o                 #
#                        |  \  | | |  | | |  |  | |     \                  #
#                        o   o o-o o-o  o-o  o  |  o-o o-o                 #
#                                                                          #
#    Jemison High School - Huntsville Alabama                              #
# ------------------------------------------------------------------------ #

import logging
from typing import Optional

from commands2 import Command

logger = logging.getLogger(__name__)


class AutoBalanceCommand(Command):
    """
    From Auto Balance command using a simple PID controller. Created by Team 3512 (java)
    """

    def __init__(self, container, indent: Optional[int] = 0):
        self._swerve_subsystem
#
#   private final SwerveSubsystem swerveSubsystem;
#   private final PIDController   controller;
#
#   public AutoBalanceCommand(SwerveSubsystem swerveSubsystem)
#   {
#     this.swerveSubsystem = swerveSubsystem;
#     controller = new PIDController(1.0, 0.0, 0.0);
#     controller.setTolerance(1);
#     controller.setSetpoint(0.0);
#     // each subsystem used by the command must be passed into the
#     // addRequirements() method (which takes a vararg of Subsystem)
#     addRequirements(this.swerveSubsystem);
#   }
#
#   /**
#    * The initial subroutine of a command.  Called once when the command is initially scheduled.
#    */
#   @Override
#   public void initialize()
#   {
#
#   }
#
#   /**
#    * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
#    * until {@link #isFinished()}) returns true.)
#    */
#   @Override
#   public void execute()
#   {
#     SmartDashboard.putBoolean("At Tolerance", controller.atSetpoint());
#
#     double translationVal = MathUtil.clamp(controller.calculate(swerveSubsystem.getPitch().getDegrees(), 0.0), -0.5,
#                                            0.5);
#     swerveSubsystem.drive(new Translation2d(translationVal, 0.0), 0.0, true);
#   }
#
#   /**
#    * <p>
#    * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
#    * the scheduler will call its {@link #end(boolean)} method.
#    * </p><p>
#    * Returning false will result in the command never ending automatically. It may still be cancelled manually or
#    * interrupted by another command. Hard coding this command to always return true will result in the command executing
#    * once and finishing immediately. It is recommended to use *
#    * {@link edu.wpi.first.wpilibj2.command.InstantCommand InstantCommand} for such an operation.
#    * </p>
#    *
#    * @return whether this command has finished.
#    */
#   @Override
#   public boolean isFinished()
#   {
#     return controller.atSetpoint();
#   }
#
#   /**
#    * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
#    * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
#    * up loose ends, like shutting off a motor that was being used in the command.
#    *
#    * @param interrupted whether the command was interrupted/canceled
#    */
#   @Override
#   public void end(boolean interrupted)
#   {
#     swerveSubsystem.lock();
#   }
# }
