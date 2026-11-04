BUILD_DIR := build
TARGET    := H_MC_V3.0

.PHONY: all $(TARGET) H_MC clean cmake flash test

all: $(TARGET)

$(TARGET) H_MC:
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

test:
	./tools/test_mc_weak.sh
	./tools/test_mc_parse.sh
	./tools/test_mc_splan.sh
	./tools/test_printf.sh
