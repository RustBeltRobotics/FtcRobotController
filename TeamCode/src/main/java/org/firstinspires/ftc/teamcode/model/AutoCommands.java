package org.firstinspires.ftc.teamcode.model;

import java.util.List;

/**
 * List of commands to execute (in order) during autonomous period
 */
public class AutoCommands {

    private final List<String> commands;

    public AutoCommands(List<String> commands) {
        this.commands = commands;
    }

    public List<String> getCommands() {
        return commands;
    }
}
