package frc.robot.subsystems;

import frc.robot.Constants;

public class SwerveMath {

        public static class WheelState {
                public double speed;
                public double angle;

                public WheelState(double speed, double angle) {
                        this.speed = speed;
                        this.angle = angle;
                }
        }

        public static WheelState[] calculate(
                        double forward,
                        double strafe,
                        double rotation,
                        double wheelbase,
                        double trackwidth) {

                double radius = Math.sqrt(
                                wheelbase * wheelbase +
                                                trackwidth * trackwidth);

                double a = strafe - rotation * (wheelbase / radius);
                double b = strafe + rotation * (wheelbase / radius);
                double c = forward - rotation * (trackwidth / radius);
                double d = forward + rotation * (trackwidth / radius);

                WheelState frontLeft = new WheelState(
                                Math.sqrt(b * b + c * c),
                                Math.atan2(b, c));

                WheelState frontRight = new WheelState(
                                Math.sqrt(b * b + d * d),
                                Math.atan2(b, d));

                WheelState backLeft = new WheelState(
                                Math.sqrt(a * a + c * c),
                                Math.atan2(a, c));

                WheelState backRight = new WheelState(
                                Math.sqrt(a * a + d * d),
                                Math.atan2(a, d));

                // Keep the wheel speeds the same and within the max range of speed
                double maxWheelSpeed = Math.max(
                                Math.max(frontLeft.speed, frontRight.speed),
                                Math.max(backLeft.speed, backRight.speed));

                if (maxWheelSpeed > Constants.MAX_SPEED) {

                        frontLeft.speed = frontLeft.speed / maxWheelSpeed * Constants.MAX_SPEED;
                        frontRight.speed = frontRight.speed / maxWheelSpeed * Constants.MAX_SPEED;
                        backLeft.speed = backLeft.speed / maxWheelSpeed * Constants.MAX_SPEED;
                        backRight.speed = backRight.speed / maxWheelSpeed * Constants.MAX_SPEED;
                }

                return new WheelState[] {
                                frontLeft,
                                frontRight,
                                backLeft,
                                backRight
                };
        }

        // short path for wheel
        public static WheelState optimize(
                        double currentAngle,
                        WheelState desiredState) {

                double angleDifference = desiredState.angle - currentAngle;

                // Angle difference between -180 and 180 degrees.
                angleDifference = Math.atan2(
                                Math.sin(angleDifference),
                                Math.cos(angleDifference));

                double speed = desiredState.speed;
                double targetAngle = desiredState.angle;

                // When more then 90 degrees wheel direction changes
                if (Math.abs(angleDifference) > Math.PI / 2) {
                        speed = -speed;

                        targetAngle += Math.PI;
                }

                // Keeps angle between -180 and 180 degrees
                targetAngle = Math.atan2(
                                Math.sin(targetAngle),
                                Math.cos(targetAngle));

                return new WheelState(speed, targetAngle);
        }
}