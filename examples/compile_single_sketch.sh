#!/bin/bash
# Compile a single sketch (others/compile/compile.ino) for multiple FQBNs and log results

SKETCH_PATH="$(dirname "$0")/others/compile/compile.ino"
LOG_FILE="$(dirname "$0")/compile_single_sketch_results.log"

# List of FQBNs to test (add more as needed)
FQBN_LIST=(
  "esp32:esp32:esp32"
  "esp8266:esp8266:arduino-esp8266"
  "rp2040:rp2040:rpipico:os=freertos"
  "arduino:renesas_uno:unor4wifi"
  "arduino:zephyr_main:nano33ble"
  "STMicroelectronics:stm32:GenF4:pnum=BLACKPILL_F411CE"
)


> "$LOG_FILE"
EXIT_CODE=0

for FQBN in "${FQBN_LIST[@]}"; do
  echo "Compiling $SKETCH_PATH for $FQBN..." | tee -a "$LOG_FILE"
  arduino-cli compile --fqbn "$FQBN" "$SKETCH_PATH" &>> "$LOG_FILE"
  if [ $? -eq 0 ]; then
    echo "SUCCESS: $SKETCH_PATH ($FQBN)" | tee -a "$LOG_FILE"
  else
    echo "FAIL: $SKETCH_PATH ($FQBN)" | tee -a "$LOG_FILE"
    EXIT_CODE=1
  fi
  echo "-----------------------------" >> "$LOG_FILE"
done

if [ $EXIT_CODE -eq 0 ]; then
  echo "Sketch compiled successfully for all FQBNs. See $LOG_FILE for details."
else
  echo "Sketch failed to compile for one or more FQBNs. See $LOG_FILE for details."
fi
exit $EXIT_CODE
