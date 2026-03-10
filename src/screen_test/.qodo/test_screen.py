import board
import busio
import digitalio
from adafruit_ili9341 import ILI9341
from PIL import Image, ImageDraw

# SPI setup
spi = busio.SPI(clock=board.SCK, MOSI=board.MOSI, MISO=board.MISO)

# Pin setup
cs = digitalio.DigitalInOut(board.CE0)      # Pin 24 - TCS
dc = digitalio.DigitalInOut(board.D12)      # Pin 15 - DC
rst = digitalio.DigitalInOut(board.D01)     # Pin 29 - RST

# Init display
display = ILI9341(spi, cs=cs, dc=dc, rst=rst)

# Draw something
img = Image.new("RGB", (320, 240), color=(0, 0, 0))
draw = ImageDraw.Draw(img)
draw.rectangle([(0, 0), (320, 40)], fill=(0, 180, 120))
draw.text((10, 10), "HELLO ROBOT!", fill=(255, 255, 255))

display.image(img)
print("Done!")