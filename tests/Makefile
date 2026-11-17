# Host-side tests for RoadSense. Run from this folder: `make` (build + checks that need no data)
# and `make equivalence` (needs the three raw drive logs in ../datasets/raw_drives/).

CC  ?= gcc
CXX ?= g++
ESP  = ../firmware/esp32/RoadSense_v4_ML
STM  = ../firmware/stm32
LOGS = ../datasets/raw_drives
RES  = ../edge_ml/results
VEH  = hatchback sedan suv
BUILD = build

.PHONY: all compile_esp32 gps equivalence clean

all: compile_esp32 $(BUILD)/stm32_pc $(BUILD)/sim $(BUILD)/test_ml gps

$(BUILD):
	mkdir -p $(BUILD)

# 1. The ESP32 sketch type-checks against mock Arduino headers, with and without the forest.
compile_esp32: | $(BUILD)
	$(CXX) -std=gnu++17 -Wall -fsyntax-only -I esp32_compile_check/mock -I $(ESP) -include mock_arduino.h -x c++ -DUSE_ML=1 $(ESP)/RoadSense_v4_ML.ino
	$(CXX) -std=gnu++17 -Wall -fsyntax-only -I esp32_compile_check/mock -I $(ESP) -include mock_arduino.h -x c++ -DUSE_ML=0 $(ESP)/RoadSense_v4_ML.ino
	@echo "ESP32 sketch: USE_ML=1 and USE_ML=0 compile"

# 2. STM32 signal chain (no HAL) built for the PC.
$(BUILD)/stm32_pc: stm32_pc/pc_main.c $(STM)/Src/rs_core.c $(STM)/Src/rs_baseline.c $(STM)/Src/rs_ml_features.c | $(BUILD)
	$(CC) -O2 -Wall -Wextra -I $(STM)/Inc -o $@ $^ -lm

# 3. Real ESP32 sketch run on the PC with mocked MPU/Serial/time.
$(BUILD)/sim: esp32_firmware_sim/sim_main.cpp | $(BUILD)
	$(CXX) -std=gnu++17 -O2 -I esp32_firmware_sim -I $(ESP) -x c++ $< -o $@

# 4. C feature extraction + forest vs the Python reference vectors.
$(BUILD)/test_ml: esp32_ml_equivalence/test_ml.cpp | $(BUILD)
	$(CXX) -O2 -I $(ESP) $< -o $@

# 5. NMEA parser on sample RMC/GGA sentences.
gps: | $(BUILD)
	$(CC) -O2 -Wall -I $(STM)/Inc -DGPS_PC_TEST -o $(BUILD)/gps_test stm32_pc/gps_test.c $(STM)/Src/gps_nmea.c
	./$(BUILD)/gps_test

# Needs the raw logs: ESP32 sim and STM32 port must reproduce the Python pipeline event for event.
equivalence: all
	cd ../edge_ml && python3 export_model.py
	./$(BUILD)/test_ml $(RES)/test_vectors.txt
	for v in $(VEH); do ./$(BUILD)/sim $(LOGS)/$$v.txt > $(RES)/fw_$$v.txt; done
	cd ../edge_ml && python3 compare_firmware.py
	for v in $(VEH); do ./$(BUILD)/stm32_pc $(LOGS)/$$v.txt > $(BUILD)/stm32_$$v.txt; \
	  grep -E '^(E|Q|K|C),' $(RES)/fw_$$v.txt | diff -q - $(BUILD)/stm32_$$v.txt && echo "$$v: STM32 == ESP32"; done

clean:
	rm -rf $(BUILD)
