CC       := arm-linux-gnueabihf-gcc
CFLAGS   := -Wall -Wextra -O2 -I/usr/include/robotcontrol -I../../components/motor_control/include -I../../components/heading_control/include
LDFLAGS  := -lrobotcontrol -lm -lpthread
PREFIX   ?= /usr
DESTDIR  ?=

# Component object files (built by parent makefiles)
MOTOR_CONTROL_OBJS     = ../../components/motor_control/build/rc_motor_ramp.o
HEADING_CONTROL_OBJS   = ../../components/heading_control/build/rc_gyro_heading.o

.PHONY: all build-server server-start-bg server-stop server-status run install uninstall clean

all: build-server

# Robot server targets
build-server: $(MOTOR_CONTROL_OBJS) $(HEADING_CONTROL_OBJS)
	$(CC) $(CFLAGS) -o rc_robot_server rc_robot_server.c $(MOTOR_CONTROL_OBJS) $(HEADING_CONTROL_OBJS) $(LDFLAGS)

server-start-bg: build-server
	@if [ -f robot_server.pid ]; then \
		echo "Robot server already running (PID: $$(cat robot_server.pid))"; \
		exit 1; \
	fi
	@./rc_robot_server > robot_server.out 2>&1 & \
	echo $$! > robot_server.pid && \
	echo "Robot server started (PID: $$(cat robot_server.pid), listening on port 5000)"

server-status:
	@if [ ! -f robot_server.pid ]; then \
		echo "Robot server not running"; \
		exit 1; \
	fi
	@PID=$$(cat robot_server.pid); \
	if kill -0 $$PID 2>/dev/null; then \
		echo "Robot server running (PID: $$PID, port 5000)"; \
		if [ -f robot_server.out ]; then \
			echo "Last log lines:"; \
			tail -3 robot_server.out; \
		fi; \
	else \
		echo "Robot server not running (stale PID: $$PID)"; \
	fi

server-stop:
	@if [ ! -f robot_server.pid ]; then \
		echo "Robot server not running"; \
		exit 0; \
	fi
	@PID=$$(cat robot_server.pid); \
	if kill -0 $$PID 2>/dev/null; then \
		kill $$PID || true; \
		sleep 1; \
		if kill -0 $$PID 2>/dev/null; then \
			kill -9 $$PID || true; \
		fi; \
		echo "Robot server stopped"; \
	else \
		echo "Robot server not running"; \
	fi; \
	rm -f robot_server.pid

install: build-server
	@mkdir -p $(DESTDIR)$(PREFIX)/bin
	@$(INSTALL) rc_robot_server $(DESTDIR)$(PREFIX)/bin/
	@echo "Robot server installed to $(DESTDIR)$(PREFIX)/bin/rc_robot_server"

uninstall:
	@rm -f $(DESTDIR)$(PREFIX)/bin/rc_robot_server
	@echo "Robot server uninstalled"

clean:
	@rm -f rc_robot_server
	@rm -f robot_server.pid robot_server.out
	@echo "Robot server cleaned"
