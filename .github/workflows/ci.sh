#!/bin/sh -l

# Make sure that Gradlew is executable.
chmod +x gradlew

# Start simulation.
./gradlew simulateJavaRelease &
sim_pid=$!

# Wait for simulation to start.
sleep 10

# Kill the simulation.
kill $sim_pid