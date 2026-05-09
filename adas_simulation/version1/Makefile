# ADAS V3 Makefile for QNX
CC = qcc
CFLAGS = -Vgcc_ntoarmv7le -Wall -O2 -I./common
# For x86_64, use: -Vgcc_ntox86_64

# List all source files
SRCS = system/main_orchestrator.c \
       core/fusion_engine.c \
       core/health_monitor.c \
       sensors/radar_proc.c \
       sensors/vision_proc.c \
       sensors/lidar_proc.c \
       decision/decision_logic.c

OBJS = $(SRCS:.c=.o)
TARGET = adas_v3_system

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $(TARGET) $(OBJS) -lm

clean:
	rm -f $(OBJS) $(TARGET)
