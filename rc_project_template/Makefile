# This is a general use makefile for librobotcontrol projects written in C.
# MAIN selects the source file containing main().
MAIN		?= rc_project_template.c
TARGET		?= $(basename $(MAIN))
APP_SOURCES	?= $(MAIN)

# compiler and linker binaries
CC		:= gcc
LINKER		:= gcc

# compiler and linker flags
WFLAGS		:= -Wall -Wextra -Werror=float-equal -Wuninitialized -Wunused-variable -Wdouble-promotion
CFLAGS		:= -g -c -Wall
LDFLAGS		:= -pthread -lm -lrt -l:librobotcontrol.so.1

INCLUDES	:= $(wildcard *.h)
OBJECTS		:= $(APP_SOURCES:.c=.o)

prefix		:= /usr/local
RM		:= rm -f
INSTALL		:= install -m 4755
INSTALLDIR	:= install -d -m 755

SYMLINK		:= ln -s -f
SYMLINKDIR	:= /etc/robotcontrol
SYMLINKNAME	:= link_to_startup_program


# linking Objects
$(TARGET): $(OBJECTS)
	@$(LINKER) -o $@ $(OBJECTS) $(LDFLAGS)
	@echo "Made: $@"


# compiling command
$(OBJECTS): %.o : %.c $(INCLUDES)
	@$(CC) $(CFLAGS) $(WFLAGS) $(DEBUGFLAG) $< -o $@
	@echo "Compiled: $@"

all:	$(TARGET)

run:	all
	@./$(TARGET)

build-preflight:
	@$(MAKE) --no-print-directory MAIN=rc_explore_preflight.c TARGET=rc_explore_preflight APP_SOURCES=rc_explore_preflight.c

preflight:	build-preflight
	@./rc_explore_preflight

preflight-watch:	build-preflight
	@./rc_explore_preflight --watch

build-endurance:
	@$(MAKE) --no-print-directory MAIN=rc_battery_endurance_log.c TARGET=rc_battery_endurance_log APP_SOURCES=rc_battery_endurance_log.c

endurance-log:	build-endurance
	@./rc_battery_endurance_log

endurance-log-watch:	build-endurance
	@./rc_battery_endurance_log --interval-sec 30

endurance-start-bg:	build-endurance
	@nohup ./rc_battery_endurance_log --log battery_endurance.csv --interval-sec 30 > battery_endurance.out 2>&1 & echo $$! > battery_endurance.pid
	@sync
	@echo "Started endurance logger in background, pid=$$(cat battery_endurance.pid)"

endurance-stop:
	@if [ -f battery_endurance.pid ]; then \
		kill $$(cat battery_endurance.pid) 2>/dev/null || true; \
		rm -f battery_endurance.pid; \
		echo "Stopped endurance logger"; \
	else \
		echo "No battery_endurance.pid found"; \
	fi

endurance-status:
	@if [ -f battery_endurance.pid ] && ps -p $$(cat battery_endurance.pid) >/dev/null 2>&1; then \
		echo "Running, pid=$$(cat battery_endurance.pid)"; \
		tail -n 5 battery_endurance.csv 2>/dev/null || true; \
	else \
		echo "Not running"; \
	fi

build-shuffle:
	@$(MAKE) --no-print-directory MAIN=rc_shuffle_endurance_log.c TARGET=rc_shuffle_endurance_log APP_SOURCES=rc_shuffle_endurance_log.c

shuffle-log:	build-shuffle
	@./rc_shuffle_endurance_log

shuffle-start-bg:	build-shuffle
	@nohup ./rc_shuffle_endurance_log --log shuffle_endurance.csv --heartbeat shuffle_heartbeat.txt --no-adc --interval-sec 5 --samples 20 --duty 0.35 --shuffle-sec 4 --watchdog-sec 30 --reverse-verify-threshold 0.25 --max-unverified-toggles 20 > shuffle_endurance.out 2>&1 & echo $$! > shuffle_endurance.pid
	@sync
	@echo "Started shuffle logger in background, pid=$$(cat shuffle_endurance.pid)"

shuffle-status:
	@if [ -f shuffle_endurance.pid ] && ps -p $$(cat shuffle_endurance.pid) >/dev/null 2>&1; then \
		echo "Running, pid=$$(cat shuffle_endurance.pid)"; \
		tail -n 5 shuffle_endurance.csv 2>/dev/null || true; \
	else \
		echo "Not running"; \
	fi

shuffle-stop:
	@if [ -f shuffle_endurance.pid ]; then \
		kill $$(cat shuffle_endurance.pid) 2>/dev/null || true; \
		rm -f shuffle_endurance.pid; \
		echo "Stopped shuffle logger"; \
	else \
		echo "No shuffle_endurance.pid found"; \
	fi

build-server:
	@$(MAKE) --no-print-directory MAIN=rc_robot_server.c TARGET=rc_robot_server APP_SOURCES="rc_robot_server.c rc_motor_ramp.c rc_gyro_heading.c"

server-start-bg:	build-server
	@nohup ./rc_robot_server --port 5000 --max-duty 0.6 --accel-rate 0.5 --decel-rate 1.0 --heading-gain 0.1 --heading-tol 2.0 --watchdog-sec 5 > robot_server.out 2>&1 & echo $$! > robot_server.pid
	@sync
	@echo "Started robot server in background, pid=$$(cat robot_server.pid)"

server-stop:
	@if [ -f robot_server.pid ]; then \
		kill $$(cat robot_server.pid) 2>/dev/null || true; \
		rm -f robot_server.pid; \
		echo "Stopped robot server"; \
	else \
		echo "No robot_server.pid found"; \
	fi

server-status:
	@if [ -f robot_server.pid ] && ps -p $$(cat robot_server.pid) >/dev/null 2>&1; then \
		echo "Running, pid=$$(cat robot_server.pid)"; \
		tail -n 5 robot_server.out 2>/dev/null || true; \
	else \
		echo "Not running"; \
	fi

devloop:
	@while true; do \
		clear; \
		date; \
		$(MAKE) --no-print-directory preflight || true; \
		echo "Press Ctrl+C to stop"; \
		sleep 2; \
	done

debug:
	$(MAKE) $(MAKEFILE) DEBUGFLAG="-g -D DEBUG"
	@echo " "
	@echo "$(TARGET) Make Debug Complete"
	@echo " "

install:
	@$(MAKE) --no-print-directory
	@$(INSTALLDIR) $(DESTDIR)$(prefix)/bin
	@$(INSTALL) $(TARGET) $(DESTDIR)$(prefix)/bin
	@echo "$(TARGET) Install Complete"

clean:
	@$(RM) *.o
	@$(RM) $(basename $(wildcard *.c))
	@echo "$(TARGET) Clean Complete"

uninstall:
	@$(RM) $(DESTDIR)$(prefix)/bin/$(TARGET)
	@echo "$(TARGET) Uninstall Complete"

runonboot:
	@$(MAKE) install --no-print-directory
	@$(SYMLINK) $(DESTDIR)$(prefix)/bin/$(TARGET) $(SYMLINKDIR)/$(SYMLINKNAME)
	@echo "$(TARGET) Set to Run on Boot"

