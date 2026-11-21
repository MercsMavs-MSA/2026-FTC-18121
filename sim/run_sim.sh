#!/bin/sh
# Laptop simulation of the Maverick robot code (no robot needed).
# Put this "sim" folder in the project root, next to TeamCode (NOT inside TeamCode).
# Run:   ./run_sim.sh               all scenarios
#        ./run_sim.sh shoot fixed   only scenarios whose name contains a word
# Needs a JDK. Uses JAVA_HOME, then Android Studio's bundled JDK, then whatever is on PATH.
cd "$(dirname "$0")" || exit 1
MAV="../TeamCode/src/main/java/org/firstinspires/ftc/Maverick"
JDK=""
if [ -n "$JAVA_HOME" ] && [ -x "$JAVA_HOME/bin/javac" ]; then JDK="$JAVA_HOME/bin/"
elif [ -x "/Applications/Android Studio.app/Contents/jbr/Contents/Home/bin/javac" ]; then JDK="/Applications/Android Studio.app/Contents/jbr/Contents/Home/bin/"
fi
rm -rf build && mkdir build
{ find fakesdk src "$MAV/Subsystems" -name '*.java'; echo "$MAV/Robot.java"; } | sed 's/.*/"&"/' > build/sources.txt
"${JDK}javac" -nowarn -encoding UTF-8 -d build @build/sources.txt || { echo "Compile failed."; exit 1; }
"${JDK}java" -cp build sim.SimRunner "$@"
