BUILD_DIR := build
TARGET    := H_FOC_V3.0

.PHONY: all $(TARGET) H_FOC clean cmake flash

all: $(TARGET)

$(TARGET) H_FOC:
	@mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake .. 2>/dev/null || true
	$(MAKE) -C $(BUILD_DIR)

clean:
	$(MAKE) -C $(BUILD_DIR) clean 2>/dev/null || true

cmake:
	@mkdir -p $(BUILD_DIR)
	cd $(BUILD_DIR) && cmake ..

flash:
	openocd -f interface/stlink.cfg -f target/stm32g4x.cfg \
		-c "program build/$(TARGET).elf verify reset exit"
