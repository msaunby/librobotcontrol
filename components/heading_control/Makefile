CC       := arm-linux-gnueabihf-gcc
CFLAGS   := -Wall -Wextra -O2 -c
LDFLAGS  := -shared
PREFIX   ?= /usr
DESTDIR  ?=

# Directories
SRCDIR   := src
INCDIR   := include
BUILDDIR := build
BINDIR   := bin

# Source and object files
SOURCES  := $(wildcard $(SRCDIR)/*.c)
OBJECTS  := $(patsubst $(SRCDIR)/%.c, $(BUILDDIR)/%.o, $(SOURCES))
HEADERS  := $(wildcard $(INCDIR)/*.h)

# Component library (optional; for now just build objects)
COMPONENT_OBJS := $(OBJECTS)

.PHONY: all install uninstall clean

all: $(COMPONENT_OBJS)

$(BUILDDIR):
	@mkdir -p $(BUILDDIR)

$(BUILDDIR)/%.o: $(SRCDIR)/%.c $(HEADERS) | $(BUILDDIR)
	$(CC) $(CFLAGS) -I$(INCDIR) -I/usr/include/robotcontrol $< -o $@

install:
	@$(INSTALL) -d $(DESTDIR)$(PREFIX)/include/robotcontrol_components
	@$(INSTALL) -m 644 $(HEADERS) $(DESTDIR)$(PREFIX)/include/robotcontrol_components/

uninstall:
	@$(RM) $(DESTDIR)$(PREFIX)/include/robotcontrol_components

clean:
	@$(RM) $(BUILDDIR)
	@echo "heading_control component cleaned"
