# Use bash instead of sh
SHELL := /bin/bash

# Define the relative paths to project within main_folder
ROOMIE_PROJECT_PATH := $(CURDIR)/ros2_ws
# Define the launch file

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