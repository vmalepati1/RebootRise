package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

public class Constants {

    public static class CANBusIDs {
        public static final int DRIVE_RIGHT_MASTER_ID = 10;
        public static final int DRIVE_RIGHT_SLAVE_ID = 11;
        public static final int DRIVE_LEFT_MASTER_ID = 12;
        public static final int DRIVE_LEFT_SLAVE_ID = 13;

        public static final int FLYWHEEL_MASTER_ID = 8;
        public static final int FLYWHEEL_SLAVE_ID = 9;
    }

    public static class RamseteConstants {

        public static double TRACK_WIDTH = 0.584;

    }

    public static class SmartDashboardKeys {

        public static final String AUTON_SELECT_ID = "Auton Select Id";
        public static final String IS_BLUE = "Is Blue";

    }

    public static class FieldConfiguration {

        public static final double DISTANCE_TO_REFLECTION_LINE = Units.feetToMeters(54) / 2;
        public static final double LIVE_DASHBOARD_FIELD_HEIGHT = Units.feetToMeters(27);

    }

    /**
     * The Joysticks class contains port mappings for the HID controllers.
     */
    public static class Joysticks {

        public static final int LEFT_JOYSTICK_PORT = 0;
        public static final int RIGHT_JOYSTICK_PORT = 1;
        public static final int GAMEPAD_PORT = 2;

    }

    public static class PneumaticIDs {

        public static final int SHOOTER_TOGGLE_ID = 0;

    }

}
