package frc.robot.utils.preferences;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.Map;

public class RobotPreferences {
    private static RobotPreferences instance;
    private final PowerDistribution pdp;
    private final Map<String, Object> defaultValues = new HashMap<>();
    private final Map<String, Object> lastSmartDashboardValues = new HashMap<>();
    private double lastUpdateTime = 0;
    private double lastSaveTime = 0;
    private final double updateInterval = 0.1; // Check every 100ms
    private final double saveInterval = 1.0; // Save every second if changed

    // Preference keys
    public static final String ROBOT_SPEED_MULTIPLIER = "Robot Speed Multiplier";
    public static final String TELEMETRY_ENABLED = "Telemetry Enabled";
    public static final String BATTERY_MONITORING = "Battery Monitoring";
    public static final String DRIVE_MODE = "Drive Mode";
    public static final String AUTO_BRAKE_ENABLED = "Auto Brake Enabled";
    public static final String FIELD_CENTRIC_ENABLED = "Field Centric Enabled";
    public static final String PRECISION_MODE_SPEED = "Precision Mode Speed";
    public static final String TURBO_MODE_SPEED = "Turbo Mode Speed";
    public static final String ROTATION_SPEED_MULTIPLIER = "Rotation Speed Multiplier";
    public static final String DEADBAND_THRESHOLD = "Deadband Threshold";
    public static final String VOLTAGE_WARNING_THRESHOLD = "Voltage Warning Threshold";
    public static final String CURRENT_WARNING_THRESHOLD = "Current Warning Threshold";
    public static final String TEMPERATURE_WARNING_THRESHOLD = "Temperature Warning Threshold";
    public static final String DEBUG_MODE = "Debug Mode";
    public static final String SIGNAL_LOGGING = "Signal Logging";

    private RobotPreferences() {
        pdp = new PowerDistribution();
        initializeDefaults();
        loadPreferencesToSmartDashboard();
    }

    public static RobotPreferences getInstance() {
        if (instance == null) {
            instance = new RobotPreferences();
        }
        return instance;
    }

    private void initializeDefaults() {
        // Drive preferences
        defaultValues.put(ROBOT_SPEED_MULTIPLIER, 1.0);
        defaultValues.put(PRECISION_MODE_SPEED, 0.3);
        defaultValues.put(TURBO_MODE_SPEED, 1.0);
        defaultValues.put(ROTATION_SPEED_MULTIPLIER, 0.75);
        defaultValues.put(DEADBAND_THRESHOLD, 0.1);
        
        // System preferences
        defaultValues.put(TELEMETRY_ENABLED, true);
        defaultValues.put(BATTERY_MONITORING, true);
        defaultValues.put(DEBUG_MODE, false);
        defaultValues.put(SIGNAL_LOGGING, true);
        
        // Safety preferences
        defaultValues.put(AUTO_BRAKE_ENABLED, true);
        defaultValues.put(FIELD_CENTRIC_ENABLED, true);
        defaultValues.put(VOLTAGE_WARNING_THRESHOLD, 10.5);
        defaultValues.put(CURRENT_WARNING_THRESHOLD, 120.0);
        defaultValues.put(TEMPERATURE_WARNING_THRESHOLD, 80.0);
        
        // Drive mode (0 = Normal, 1 = Precision, 2 = Turbo)
        defaultValues.put(DRIVE_MODE, 0);
    }

    private void loadPreferencesToSmartDashboard() {
        // Load from persistent storage and put into SmartDashboard
        for (Map.Entry<String, Object> entry : defaultValues.entrySet()) {
            String key = entry.getKey();
            Object defaultValue = entry.getValue();
            
            if (defaultValue instanceof Double) {
                double value = Preferences.getDouble(key, (Double) defaultValue);
                SmartDashboard.putNumber(key, value);
                lastSmartDashboardValues.put(key, value);
            } else if (defaultValue instanceof Boolean) {
                boolean value = Preferences.getBoolean(key, (Boolean) defaultValue);
                SmartDashboard.putBoolean(key, value);
                lastSmartDashboardValues.put(key, value);
            } else if (defaultValue instanceof Integer) {
                int value = Preferences.getInt(key, (Integer) defaultValue);
                if (key.equals(DRIVE_MODE)) {
                    SmartDashboard.putNumber("Drive Mode (0=Normal, 1=Precision, 2=Turbo)", value);
                    lastSmartDashboardValues.put("Drive Mode (0=Normal, 1=Precision, 2=Turbo)", (double) value);
                } else {
                    SmartDashboard.putNumber(key, value);
                    lastSmartDashboardValues.put(key, (double) value);
                }
            } else if (defaultValue instanceof String) {
                String value = Preferences.getString(key, (String) defaultValue);
                SmartDashboard.putString(key, value);
                lastSmartDashboardValues.put(key, value);
            }
        }
    }

    public void updateSmartDashboard() {
        double currentTime = Timer.getFPGATimestamp();
        
        // Check for changes more frequently
        if (currentTime - lastUpdateTime >= updateInterval) {
            checkAndSaveChanges();
            lastUpdateTime = currentTime;
        }
        
        // Update read-only status every 500ms
        if (currentTime - lastSaveTime >= 0.5) {
            updateBatteryStatus();
            lastSaveTime = currentTime;
        }
    }

    private void checkAndSaveChanges() {
        boolean hasChanges = false;
        
        // Check each preference for changes
        for (Map.Entry<String, Object> entry : defaultValues.entrySet()) {
            String key = entry.getKey();
            Object defaultValue = entry.getValue();
            
            if (defaultValue instanceof Double) {
                double currentValue = SmartDashboard.getNumber(key, (Double) defaultValue);
                double lastValue = (Double) lastSmartDashboardValues.getOrDefault(key, defaultValue);
                
                if (Math.abs(currentValue - lastValue) > 0.001) { // Small threshold for doubles
                    Preferences.setDouble(key, currentValue);
                    lastSmartDashboardValues.put(key, currentValue);
                    hasChanges = true;
                }
            } else if (defaultValue instanceof Boolean) {
                boolean currentValue = SmartDashboard.getBoolean(key, (Boolean) defaultValue);
                boolean lastValue = (Boolean) lastSmartDashboardValues.getOrDefault(key, defaultValue);
                
                if (currentValue != lastValue) {
                    Preferences.setBoolean(key, currentValue);
                    lastSmartDashboardValues.put(key, currentValue);
                    hasChanges = true;
                }
            } else if (defaultValue instanceof Integer) {
                String dashboardKey = key.equals(DRIVE_MODE) ? "Drive Mode (0=Normal, 1=Precision, 2=Turbo)" : key;
                int currentValue = (int) SmartDashboard.getNumber(dashboardKey, (Integer) defaultValue);
                int lastValue = ((Double) lastSmartDashboardValues.getOrDefault(dashboardKey, (double) (Integer) defaultValue)).intValue();
                
                if (currentValue != lastValue) {
                    Preferences.setInt(key, currentValue);
                    lastSmartDashboardValues.put(dashboardKey, (double) currentValue);
                    hasChanges = true;
                }
            } else if (defaultValue instanceof String) {
                String currentValue = SmartDashboard.getString(key, (String) defaultValue);
                String lastValue = (String) lastSmartDashboardValues.getOrDefault(key, defaultValue);
                
                if (!currentValue.equals(lastValue)) {
                    Preferences.setString(key, currentValue);
                    lastSmartDashboardValues.put(key, currentValue);
                    hasChanges = true;
                }
            }
        }
        
        if (hasChanges) {
            SmartDashboard.putString("Preferences Status", "Auto-saved at " + String.format("%.1f", Timer.getFPGATimestamp()));
        }
    }

    private void updateBatteryStatus() {
        if (getBoolean(BATTERY_MONITORING)) {
            double voltage = RobotController.getBatteryVoltage();
            double totalCurrent = pdp.getTotalCurrent();
            double temperature = pdp.getTemperature();
            
            // Battery Status (Read-only)
            SmartDashboard.putNumber("Battery Voltage", voltage);
            SmartDashboard.putNumber("Total Current", totalCurrent);
            SmartDashboard.putNumber("PDP Temperature", temperature);
            SmartDashboard.putNumber("Battery Percentage", calculateBatteryPercentage(voltage));
            
            // Warning indicators
            SmartDashboard.putBoolean("Voltage Warning", voltage < getDouble(VOLTAGE_WARNING_THRESHOLD));
            SmartDashboard.putBoolean("Current Warning", totalCurrent > getDouble(CURRENT_WARNING_THRESHOLD));
            SmartDashboard.putBoolean("Temperature Warning", temperature > getDouble(TEMPERATURE_WARNING_THRESHOLD));
            
            // System Status (Read-only)
            SmartDashboard.putNumber("CAN Utilization", RobotController.getCANStatus().percentBusUtilization);
            SmartDashboard.putNumber("CPU Temperature", RobotController.getCPUTemp());
            SmartDashboard.putBoolean("Brownout", RobotController.isBrownedOut());
            SmartDashboard.putBoolean("User Button", RobotController.getUserButton());
            
            // Individual channel currents (first 16 channels)
            for (int i = 0; i < Math.min(16, pdp.getNumChannels()); i++) {
                SmartDashboard.putNumber("Channel " + i + " Current", pdp.getCurrent(i));
            }
        }
    }

    private double calculateBatteryPercentage(double voltage) {
        // Simple battery percentage calculation (12.8V = 100%, 10.5V = 0%)
        double maxVoltage = 12.8;
        double minVoltage = 10.5;
        double percentage = ((voltage - minVoltage) / (maxVoltage - minVoltage)) * 100.0;
        return Math.max(0, Math.min(100, percentage));
    }

    // Getter methods - read directly from Preferences for current values
    public double getDouble(String key) {
        return Preferences.getDouble(key, (Double) defaultValues.getOrDefault(key, 0.0));
    }

    public boolean getBoolean(String key) {
        return Preferences.getBoolean(key, (Boolean) defaultValues.getOrDefault(key, false));
    }

    public int getInt(String key) {
        return Preferences.getInt(key, (Integer) defaultValues.getOrDefault(key, 0));
    }

    public String getString(String key) {
        return Preferences.getString(key, (String) defaultValues.getOrDefault(key, ""));
    }

    // Convenience methods for common operations
    public double getRobotSpeedMultiplier() {
        int driveMode = getInt(DRIVE_MODE);
        switch (driveMode) {
            case 1: return getDouble(PRECISION_MODE_SPEED); // Precision mode
            case 2: return getDouble(TURBO_MODE_SPEED); // Turbo mode
            default: return getDouble(ROBOT_SPEED_MULTIPLIER); // Normal mode
        }
    }

    public double getRotationSpeedMultiplier() {
        return getDouble(ROTATION_SPEED_MULTIPLIER);
    }

    public boolean isTelemetryEnabled() {
        return getBoolean(TELEMETRY_ENABLED);
    }

    public boolean isBatteryMonitoringEnabled() {
        return getBoolean(BATTERY_MONITORING);
    }

    public boolean isFieldCentricEnabled() {
        return getBoolean(FIELD_CENTRIC_ENABLED);
    }

    public boolean isAutoBrakeEnabled() {
        return getBoolean(AUTO_BRAKE_ENABLED);
    }

    public boolean isDebugMode() {
        return getBoolean(DEBUG_MODE);
    }

    public boolean isSignalLoggingEnabled() {
        return getBoolean(SIGNAL_LOGGING);
    }

    public double getDeadbandThreshold() {
        return getDouble(DEADBAND_THRESHOLD);
    }

    // Reset to defaults
    public void resetToDefaults() {
        for (Map.Entry<String, Object> entry : defaultValues.entrySet()) {
            String key = entry.getKey();
            Object value = entry.getValue();
            
            if (value instanceof Double) {
                Preferences.setDouble(key, (Double) value);
            } else if (value instanceof Boolean) {
                Preferences.setBoolean(key, (Boolean) value);
            } else if (value instanceof Integer) {
                Preferences.setInt(key, (Integer) value);
            } else if (value instanceof String) {
                Preferences.setString(key, (String) value);
            }
        }
        
        // Update SmartDashboard with new values
        loadPreferencesToSmartDashboard();
        SmartDashboard.putString("Preferences Status", "Reset to defaults at " + String.format("%.1f", Timer.getFPGATimestamp()));
    }
}
