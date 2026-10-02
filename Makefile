# Use bash instead of sh
SHELL := /bin/bash

# Define the relative paths to project within main_folder
ROOMIE_PROJECT_PATH := $(CURDIR)/ros2_ws
# Define the launch file
GETUP_LAUNCH := surge_et_ambula getup.launch.py
# Policy and launch options for `make getup`, e.g.
#   make getup POLICY=/abs/path/model.onnx GETUP_ARGS="enable_lowcmd:=true"
POLICY ?= $(HOME)/getup_models/g1_getup.onnx
GETUP_ARGS ?=

# Default target to clean, build, start roscore, and launch both projects
all: build

# Rule to clean project's build and devel folders
clean:
	@echo "Cleaning Project..."
	sleep 2
	rm -rf $(ROOMIE_PROJECT_PATH)/build $(ROOMIE_PROJECT_PATH)/install $(ROOMIE_PROJECT_PATH)/log

build:
	@echo "Building Project..."
	cd $(ROOMIE_PROJECT_PATH) && colcon build

# Launch the getup policy, low-level bridge and e-stop GUI (http://<robot-ip>:8082)
getup:
	@echo "Launching getup policy + GUI..."
	cd $(ROOMIE_PROJECT_PATH) && source install/setup.bash && ros2 launch $(GETUP_LAUNCH) policy_path:=$(POLICY) $(GETUP_ARGS)

.PHONY: all clean build getup
