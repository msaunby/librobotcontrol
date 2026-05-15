CC       := arm-linux-gnueabihf-gcc
CFLAGS   := -Wall -Wextra -O2 -I/usr/include/robotcontrol
LDFLAGS  := -lrobotcontrol -lm -lpthread
PREFIX   ?= /usr
DESTDIR  ?=

# Component source paths (will be set by parent)
MOTOR_CONTROL_OBJS     ?= ../../components/motor_control/build/*.o
HEADING_CONTROL_OBJS   ?= ../../components/heading_control/build/*.o

# Makefile parameters for shuffle test
DUTY       ?= 50
INTERVAL   ?= 10

.PHONY: all build-shuffle shuffle-log shuffle-start-bg shuffle-status shuffle-stop shuffle-watch run install uninstall clean

all: build-shuffle

# Shuffle endurance targets
build-shuffle:
	$(CC) $(CFLAGS) -o rc_shuffle_endurance_log rc_shuffle_endurance_log.c $(LDFLAGS)

shuffle-log: build-shuffle
	./rc_shuffle_endurance_log --duty $(DUTY) --interval $(INTERVAL)

shuffle-watch: build-shuffle
	while ./rc_shuffle_endurance_log --duty $(DUTY) --interval $(INTERVAL); do \
		sleep 1; \
	done

shuffle-start-bg: build-shuffle
	@if [ -f shuffle_endurance.pid ]; then \
		echo "Shuffle logger already running (PID: $$(cat shuffle_endurance.pid))"; \
		exit 1; \
	fi
	@./rc_shuffle_endurance_log --duty $(DUTY) --interval $(INTERVAL) > shuffle_endurance.out 2>&1 & \
	echo $$! > shuffle_endurance.pid && \
	echo "Shuffle endurance logger started (PID: $$(cat shuffle_endurance.pid), duty: $(DUTY)%, interval: $(INTERVAL)s)"

shuffle-status:
	@if [ ! -f shuffle_endurance.pid ]; then \
		echo "Shuffle endurance logger not running"; \
		exit 1; \
	fi
	@PID=$$(cat shuffle_endurance.pid); \
	if kill -0 $$PID 2>/dev/null; then \
		echo "Shuffle endurance logger running (PID: $$PID)"; \
		echo "Log file: shuffle_endurance.csv"; \
		if [ -f shuffle_heartbeat.txt ]; then \
			echo "Last heartbeat:"; \
			cat shuffle_heartbeat.txt; \
		fi; \
		if [ -f shuffle_endurance.csv ]; then \
			echo "Rows: $$(wc -l < shuffle_endurance.csv)"; \
			echo "Last row: $$(tail -1 shuffle_endurance.csv)"; \
		fi; \
	else \
		echo "Shuffle endurance logger not running (stale PID: $$PID)"; \
	fi

shuffle-stop:
	@if [ ! -f shuffle_endurance.pid ]; then \
		echo "Shuffle endurance logger not running"; \
		exit 0; \
	fi
	@PID=$$(cat shuffle_endurance.pid); \
	if kill -0 $$PID 2>/dev/null; then \
		kill $$PID || true; \
		sleep 1; \
		if kill -0 $$PID 2>/dev/null; then \
			kill -9 $$PID || true; \
		fi; \
		echo "Shuffle endurance logger stopped"; \
	else \
		echo "Shuffle endurance logger not running"; \
	fi; \
	rm -f shuffle_endurance.pid

install: build-shuffle
	@mkdir -p $(DESTDIR)$(PREFIX)/bin
	@$(INSTALL) rc_shuffle_endurance_log $(DESTDIR)$(PREFIX)/bin/
	@echo "Motor experiments installed to $(DESTDIR)$(PREFIX)/bin/"

uninstall:
	@rm -f $(DESTDIR)$(PREFIX)/bin/rc_shuffle_endurance_log
	@echo "Motor experiments uninstalled"

clean:
	@rm -f rc_shuffle_endurance_log
	@rm -f shuffle_endurance.pid shuffle_endurance.csv shuffle_endurance.out shuffle_heartbeat.txt
	@echo "Motor experiments cleaned"
