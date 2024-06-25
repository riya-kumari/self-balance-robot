BOARD?=arduino:avr:uno
PORT?=/dev/cu.usbserial-110
BUILD=build
# LIBRARIES=./MPU6050

.PHONY: default  all flash clean

default:  all flash

all:
	arduino-cli compile --fqbn $(BOARD) --output-dir $(BUILD) --libraries $(LIBRARIES) ./

flash:
	arduino-cli upload --fqbn $(BOARD) --port $(PORT) --input-dir $(BUILD)

clean:
	rm -r build


# To view serial output
# screen /dev/cu.usbserial-110 9600
#
# lsof | grep /dev/cu.usbserial-120
