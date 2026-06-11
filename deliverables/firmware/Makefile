# =============================================================================
# Project : WETI - Bezkontaktowy miernik odleglosci
# MCU     : ATtiny1616 (tinyAVR 1-series, UPDI)
# Clock   : 10 MHz (internal 20 MHz oscillator, prescaled /2 in code)
# Toolchain: avr-gcc, avr-objcopy, avr-objdump, avr-size
# Programmer: SerialUPDI via avrdude (>= 7.1) on a USB-serial adapter
#
# Quick start:
#   make                 # build main.hex (default)
#   make flash PORT=COM7 # upload via SerialUPDI on given COM port
#   make fuses PORT=COM7 # program fuse bytes (run once, if needed)
#   make clean           # remove build artifacts
#   make help            # list all available targets
#
# Override PORT on the command line, e.g.:  make flash PORT=COM3
# =============================================================================

# ----- Project configuration ------------------------------------------------
MCU         := attiny1616
F_CPU       := 10000000UL
TARGET      := main
SRC         := main.c

# ----- SerialUPDI / avrdude configuration -----------------------------------
# Default serial port for the UPDI adapter. Override from CLI: make flash PORT=COM3
PORT        ?= COM7
BAUD        ?= 230400
AVRDUDE     := avrdude
PROGRAMMER  := serialupdi

# Fallback programmer (uncomment/use manually):
#   pymcuprog write -t uart -u $(PORT) -d $(MCU) -f $(TARGET).hex --erase --verify
#   pymcuprog read  -t uart -u $(PORT) -d $(MCU) -m fuses

# ----- Toolchain ------------------------------------------------------------
CC          := avr-gcc
OBJCOPY     := avr-objcopy
OBJDUMP     := avr-objdump
SIZE        := avr-size

# ----- Compiler flags -------------------------------------------------------
CFLAGS  := -mmcu=$(MCU) \
           -DF_CPU=$(F_CPU) \
           -Os \
           -std=gnu11 \
           -Wall -Wextra \
           -ffunction-sections -fdata-sections \
           -flto

LDFLAGS := -mmcu=$(MCU) \
           -Wl,--gc-sections \
           -Wl,-Map,$(TARGET).map \
           -flto

# ----- Fuse values for ATtiny1616 -------------------------------------------
# OSCCFG  = 0x02 : FREQSEL = 20 MHz internal oscillator (divided /2 in code)
# SYSCFG0 = 0xF6 : CRC disabled, RSTPINCFG = UPDI (keep UPDI on PA0, no reset)
# SYSCFG1 = 0x07 : SUT = 64 ms startup delay
# BODCFG  = 0x00 : BOD disabled (enable later if needed)
FUSE_OSCCFG  := 0x02
FUSE_SYSCFG0 := 0xF6
FUSE_SYSCFG1 := 0x07
FUSE_BODCFG  := 0x00

# ============================================================================
# Build rules
# ============================================================================

.PHONY: all build flash fuses read-fuses size clean help

all: build

build: $(TARGET).hex $(TARGET).lst size

# Compile + link in one step (LTO-friendly)
$(TARGET).elf: $(SRC)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ -lm

# Intel HEX for flashing
$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex -R .eeprom $< $@

# Disassembly listing for debugging / size inspection
$(TARGET).lst: $(TARGET).elf
	$(OBJDUMP) -h -S $< > $@

# ----- Size report ----------------------------------------------------------
size: $(TARGET).elf
	@echo ""
	@echo "---- Size report ($(MCU)) ----"
	$(SIZE) -A $(TARGET).elf

# ----- Flash via SerialUPDI -------------------------------------------------
flash: $(TARGET).hex
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -b $(BAUD) \
	           -U flash:w:$(TARGET).hex:i

# ----- Fuses (write) --------------------------------------------------------
# NOT run by default. Invoke explicitly: make fuses PORT=COMx
fuses:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -b $(BAUD) \
	           -U fuse2:w:$(FUSE_OSCCFG):m \
	           -U fuse5:w:$(FUSE_SYSCFG0):m \
	           -U fuse6:w:$(FUSE_SYSCFG1):m \
	           -U fuse1:w:$(FUSE_BODCFG):m

# ----- Fuses (read-back for verification) -----------------------------------
read-fuses:
	$(AVRDUDE) -c $(PROGRAMMER) -p $(MCU) -P $(PORT) -b $(BAUD) \
	           -U fuse0:r:-:h \
	           -U fuse1:r:-:h \
	           -U fuse2:r:-:h \
	           -U fuse4:r:-:h \
	           -U fuse5:r:-:h \
	           -U fuse6:r:-:h \
	           -U fuse7:r:-:h \
	           -U fuse8:r:-:h

# ----- Clean ----------------------------------------------------------------
clean:
	rm -f *.elf *.hex *.o *.lst *.map

# ----- Help -----------------------------------------------------------------
help:
	@echo "WETI - Bezkontaktowy miernik odleglosci (ATtiny1616 @ 10 MHz)"
	@echo ""
	@echo "Usage: make <target> [PORT=COMx]"
	@echo ""
	@echo "Targets:"
	@echo "  all         Build $(TARGET).hex (default)"
	@echo "  build       Same as 'all' - compile, link, listing, size report"
	@echo "  flash       Upload $(TARGET).hex via SerialUPDI (avrdude -c serialupdi)"
	@echo "  fuses       Write ATtiny1616 fuse bytes (run once, if needed)"
	@echo "  read-fuses  Read current fuse values for verification"
	@echo "  size        Print avr-size report for $(TARGET).elf"
	@echo "  clean       Remove *.elf *.hex *.o *.lst *.map"
	@echo "  help        Show this help message"
	@echo ""
	@echo "Variables (override on command line):"
	@echo "  PORT   UPDI serial port (default: $(PORT)) - e.g. make flash PORT=COM3"
	@echo "  BAUD   avrdude baud rate (default: $(BAUD))"
	@echo ""
	@echo "Example:"
	@echo "  make flash PORT=COM7"
