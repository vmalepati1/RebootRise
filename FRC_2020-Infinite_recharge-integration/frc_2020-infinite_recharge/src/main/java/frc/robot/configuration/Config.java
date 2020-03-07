package frc.robot.configuration;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.InputStream;

public class Config {

    private class RobotConfig {
        private int testVal;
        private String testString;
        private boolean testBool;
    }

    InputStream input;

    RobotConfig rConfig;
}