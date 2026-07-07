BUILD_DIR ?= build
CMAKE ?= cmake
GENERATOR ?= Unix Makefiles
NPROC := $(shell nproc 2>/dev/null || echo 1)

.PHONY: help configure build rebuild clean

help:
	@echo "Targets:"
	@echo "  configure  - Configure CMake in $(BUILD_DIR)"
	@echo "  build      - Build plugin from $(BUILD_DIR)"
	@echo "  rebuild    - Remove $(BUILD_DIR), then configure and build"
	@echo "  clean      - Remove $(BUILD_DIR)"

configure:
	@mkdir -p $(BUILD_DIR)
	@$(CMAKE) -S . -B $(BUILD_DIR) -G "$(GENERATOR)"

build: configure
	@$(MAKE) -C $(BUILD_DIR) -j$(NPROC)

rebuild:
	@rm -rf $(BUILD_DIR)
	@$(MAKE) configure build

clean:
	@rm -rf $(BUILD_DIR)
