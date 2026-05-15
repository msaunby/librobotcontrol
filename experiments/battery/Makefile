CC       := arm-linux-gnueabihf-gcc
CFLAGS   := -Wall -Wextra -O2 -I/usr/include/robotcontrol
LDFLAGS  := -lrobotcontrol -lm -lpthread
PREFIX   ?= /usr
DESTDIR  ?=

# Component source paths (will be set by parent)
MOTOR_CONTROL_OBJS     ?= ../../components/motor_control/build/*.o
HEADING_CONTROL_OBJS   ?= ../../components/heading_control/build/*.o

# Default app
MAIN     ?= rc_explore_preflight.c
TARGET   = $(basename $(MAIN))
SOURCES  = $(MAIN)

# Support for additional sources (e.g., helper modules)
APP_SOURCES ?= 

.PHONY: all build-preflight build-endurance preflight endurance-log endurance-start-bg endurance-status endurance-stop preflight-watch endurance-watch run install uninstall clean

all: build-preflight build-endurance

# Preflight targets
build-preflight:
	$(CC) $(CFLAGS) -o rc_explore_preflight rc_explore_preflight.c $(LDFLAGS)

preflight: build-preflight
	./rc_explore_preflight

preflight-watch:
	while true; do \
		clear; \
		./rc_explore_preflight; \
		sleep 2; \
	done

# Battery endurance targets
build-endurance:
	$(CC) $(CFLAGS) -o rc_battery_endurance_log rc_battery_endurance_log.c $(LDFLAGS)

endurance-log: build-endurance
	./rc_battery_endurance_log

endurance-watch: build-endurance
	while ./rc_battery_endurance_log; do \
		sleep 1; \
	done

endurance-start-bg: build-endurance
	@if [ -f battery_endurance.pid ]; then \
		echo "Endurance logger already running (PID: $$(cat battery_endurance.pid))"; \
		exit 1; \
	fi
	@./rc_battery_endurance_log > battery_endurance.out 2>&1 & \
	echo $$! > battery_endurance.pid && \
	echo "Battery endurance logger started (PID: $$(cat battery_endurance.pid))"

endurance-status:
	@if [ ! -f battery_endurance.pid ]; then \
		echo "Battery endurance logger not running"; \
		exit 1; \
	fi
	@PID=$$(cat battery_endurance.pid); \
	if kill -0 $$PID 2>/dev/null; then \
		echo "Battery endurance logger running (PID: $$PID)"; \
		echo "Log file: battery_endurance.csv"; \
		if [ -f battery_endurance.csv ]; then \
			echo "Rows: $$(wc -l < battery_endurance.csv)"; \
			echo "Last row: $$(tail -1 battery_endurance.csv)"; \
		fi; \
	else \
		echo "Battery endurance logger not running (stale PID: $$PID)"; \
	fi

endurance-stop:
	@if [ ! -f battery_endurance.pid ]; then \
		echo "Battery endurance logger not running"; \
		exit 0; \
	fi
	@PID=$$(cat battery_endurance.pid); \
	if kill -0 $$PID 2>/dev/null; then \
		kill $$PID || true; \
		sleep 1; \
		if kill -0 $$PID 2>/dev/null; then \
			kill -9 $$PID || true; \
		fi; \
		echo "Battery endurance logger stopped"; \
	else \
		echo "Battery endurance logger not running"; \
	fi; \
	rm -f battery_endurance.pid

install: build-preflight build-endurance
	@mkdir -p $(DESTDIR)$(PREFIX)/bin
	@$(INSTALL) rc_explore_preflight $(DESTDIR)$(PREFIX)/bin/
	@$(INSTALL) rc_battery_endurance_log $(DESTDIR)$(PREFIX)/bin/
	@echo "Battery experiments installed to $(DESTDIR)$(PREFIX)/bin/"

uninstall:
	@rm -f $(DESTDIR)$(PREFIX)/bin/rc_explore_preflight
	@rm -f $(DESTDIR)$(PREFIX)/bin/rc_battery_endurance_log
	@echo "Battery experiments uninstalled"

clean:
	@rm -f rc_explore_preflight rc_battery_endurance_log
	@rm -f battery_endurance.pid battery_endurance.csv battery_endurance.out
	@echo "Battery experiments cleaned"
