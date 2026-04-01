#!/bin/bash
# Compile all Arduino examples in the TinyRobotics/examples directory and log results

EXAMPLES_DIR="$(dirname "$0")"
LOG_FILE="$EXAMPLES_DIR/compile_all_examples_results.log"

# Set your FQBN here (update as needed for your board)
FQBN="esp32:esp32:esp32"

arduino-cli lib install "pubsubclient3" "FastAccelStepper" "Servo" "ESP32Servo" "RadioLIB"

# Clear previous log
> "$LOG_FILE"

echo "Compilation finished. See $LOG_FILE for details."

EXIT_CODE=0
find "$EXAMPLES_DIR" -type f -name "*.ino" \
  ! -path "*IEEE802_15_4 -receive/*" \
  ! -path "*IEEE802_15_4 -send/*" | while read -r example; do
  echo "Compiling $example..." | tee -a "$LOG_FILE"
  arduino-cli compile --fqbn "$FQBN" "$example" &>> "$LOG_FILE"
  if [ $? -eq 0 ]; then
    echo "SUCCESS: $example" | tee -a "$LOG_FILE"
  else
    echo "FAIL: $example" | tee -a "$LOG_FILE"
    EXIT_CODE=1
  fi
  echo "-----------------------------" >> "$LOG_FILE"
done

if [ $EXIT_CODE -eq 0 ]; then
  echo "All examples compiled successfully. See $LOG_FILE for details."
else
  echo "Some examples failed to compile. See $LOG_FILE for details."
fi
exit $EXIT_CODE
