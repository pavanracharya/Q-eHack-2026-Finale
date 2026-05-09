# QNX ADAS Multi-Process Build
# Produces 5 separate binaries — one per service.
#
# Usage:
#   make                    # build all (no HAM)
#   make USE_HAM=1          # build all with HAM support
#   make clean
#   make rebuild
#
# Deploy to Pi (copy all binaries to same directory):
#   scp adas_supervisor camera_service sensor_service \
#       decision_service safety_service root@<pi_ip>:/tmp/adas/
#
# Run on Pi:
#   ham &                   # start HAM daemon (if USE_HAM=1)
#   ./adas_supervisor       # starts all other services automatically

PLATFORM     ?= aarch64le
BUILD_PROFILE ?= debug
USE_HAM      ?= 1

CONFIG_NAME  = $(PLATFORM)-$(BUILD_PROFILE)
OUTPUT_DIR   = build/$(CONFIG_NAME)

CC = qcc -Vgcc_nto$(PLATFORM)
LD = $(CC)

# Build profile flags
CCFLAGS_debug    += -g -O0 -fno-builtin
CCFLAGS_release  += -O2
CCFLAGS_profile  += -g -O0 -finstrument-functions
LIBS_profile     += -lprofilingS

CCFLAGS_all  = -Wall -fmessage-length=0
CCFLAGS_all += $(CCFLAGS_$(BUILD_PROFILE))
CCFLAGS_all += -Isrc

ifeq ($(USE_HAM),1)
    CCFLAGS_all += -DUSE_HAM
    LIBS_ham     = -lham
endif

DEPS = -Wp,-MMD,$(@:%.o=%.d),-MT,$@

# -----------------------------------------------------------------------
# Shared object files (GPIO + I2C + sensors)
# -----------------------------------------------------------------------
SHARED_SRCS = src/rpi_gpio.c \
              src/lib_i2c.c \
              src/ultrasonic_hcsr04.c \
              src/imu_lsm6dsox.c

SHARED_OBJS = $(patsubst src/%.c,$(OUTPUT_DIR)/src/%.o,$(SHARED_SRCS))

# -----------------------------------------------------------------------
# Targets
# -----------------------------------------------------------------------
TARGETS = $(OUTPUT_DIR)/adas_supervisor \
          $(OUTPUT_DIR)/camera_service  \
          $(OUTPUT_DIR)/sensor_service  \
          $(OUTPUT_DIR)/decision_service \
          $(OUTPUT_DIR)/safety_service

.PHONY: all clean rebuild instructions

all: $(TARGETS)

# adas_supervisor — no sensor/GPIO deps, links HAM if enabled
$(OUTPUT_DIR)/adas_supervisor: $(OUTPUT_DIR)/src/adas_supervisor.o
	$(LD) -o $@ $^ $(LIBS_ham) $(LIBS_$(BUILD_PROFILE))

# camera_service — no hardware deps
$(OUTPUT_DIR)/camera_service: $(OUTPUT_DIR)/src/camera_service.o
	$(LD) -o $@ $^ $(LIBS_ham) $(LIBS_$(BUILD_PROFILE))

# sensor_service — needs GPIO + I2C + ultrasonic + IMU
$(OUTPUT_DIR)/sensor_service: $(OUTPUT_DIR)/src/sensor_service.o $(SHARED_OBJS)
	$(LD) -o $@ $^ $(LIBS_ham) $(LIBS_$(BUILD_PROFILE)) -lm

# decision_service — math only
$(OUTPUT_DIR)/decision_service: $(OUTPUT_DIR)/src/decision_service.o
	$(LD) -o $@ $^ $(LIBS_ham) $(LIBS_$(BUILD_PROFILE)) -lm

# safety_service — needs GPIO, no HAM (never registered)
$(OUTPUT_DIR)/safety_service: $(OUTPUT_DIR)/src/safety_service.o \
                               $(OUTPUT_DIR)/src/rpi_gpio.o
	$(LD) -o $@ $^ $(LIBS_$(BUILD_PROFILE))

# -----------------------------------------------------------------------
# Generic compile rule
# -----------------------------------------------------------------------
$(OUTPUT_DIR)/src/%.o: src/%.c
	@mkdir -p $(dir $@)
	$(CC) -c $(DEPS) -o $@ $(CCFLAGS_all) $<

# -----------------------------------------------------------------------
# Utility targets
# -----------------------------------------------------------------------
instructions:
	@echo ""
	@echo "=== Deploy to Raspberry Pi ==="
	@echo "  scp $(OUTPUT_DIR)/adas_supervisor $(OUTPUT_DIR)/camera_service \\"
	@echo "      $(OUTPUT_DIR)/sensor_service $(OUTPUT_DIR)/decision_service \\"
	@echo "      $(OUTPUT_DIR)/safety_service root@<pi_ip>:/tmp/adas/"
	@echo ""
	@echo "=== Run on Pi ==="
	@echo "  cd /tmp/adas"
	@echo "  ham &                    # only if built with USE_HAM=1"
	@echo "  ./adas_supervisor        # spawns all services automatically"
	@echo ""
	@echo "=== Fault injection (judge demo) ==="
	@echo "  pidin | grep camera_service   # find PID"
	@echo "  kill -9 <pid>                 # inject fault"
	@echo "  # Watch supervisor log for DEATH DETECTED + RTO"

clean:
	-rm -fr "$(OUTPUT_DIR)"
	-rm -fr build

rebuild: clean all

# Include generated dependency files
DEPSFILES := $(wildcard $(OUTPUT_DIR)/src/*.d)
-include $(DEPSFILES)
