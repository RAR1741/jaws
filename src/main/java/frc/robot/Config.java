package frc.robot;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.io.PrintWriter;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Map.Entry;
import java.util.function.DoubleSupplier;

import org.tomlj.Toml;
import org.tomlj.TomlParseError;
import org.tomlj.TomlParseResult;

import edu.wpi.first.wpilibj.Filesystem;

/**
 *
 */
public class Config {
    static TomlParseResult parsedConfig;

    public static void loadFromFile() {
        try {
            Path source = Paths.get(new File(Filesystem.getDeployDirectory(), "config.toml").getPath());
            parsedConfig = Toml.parse(source);
        } catch (IOException e) {
            System.out.println("Failed to load configuration: " + e.getMessage());
            parsedConfig = null; // Gross but a quick way to implement this interface
            // TODO Change this interface
            return;
        }


        if (!parsedConfig.errors().isEmpty()) {
            System.out.println("Error(s) parsing configuration!");
            for (TomlParseError e : parsedConfig.errors()) {
                System.out.println(e.getMessage());
            }
        }

        System.out.println("Config loaded.");
    }

    public static double getSetting(String name, double reasonableDefault) {
        if (parsedConfig == null) {
            return reasonableDefault;
        }
        return parsedConfig.getDouble(name, new DoubleSupplier() {
            @Override
            public double getAsDouble() {
                return reasonableDefault;
            }
        });
    }

    public static void dump(final PrintStream out) {
        out.println("Config dump:");
        if (parsedConfig == null) {
            out.println("ERROR, file was not parsed properly, all settings will be the default!");
        } else {
            for(Entry<String, Object> e: parsedConfig.entrySet()) {
                out.print("  " + e.getKey() + ": ");
                out.println(e.getValue());
            }
        }
    }
}
