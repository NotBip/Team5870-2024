// package frc.robot.subsystems.swerve;

// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;
// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
// import com.ctre.phoenix.sensors.SensorTimeBase;
// import frc.robot.RobotMap;


// public class SwerveConfig
// {

//     public CANCoderConfiguration canCoderConfig;


//     /* Swerve Profiling Values */


//     public SwerveConfig() {
//         canCoderConfig = new CANCoderConfiguration();
//         canCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
//         canCoderConfig.sensorDirection = RobotMap.Swerve.canCoderInvert;
//         canCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
//         canCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
//     }
// }
