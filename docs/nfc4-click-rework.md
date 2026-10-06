# NFC 4 Click - rework request

Three small jobs on one circuit board. Everything is identified by the white silkscreen printing on the board itself, so nothing needs to be counted or measured.

- **Board:** MikroElektronika NFC 4 Click (MIKROE-4842), 57 x 25 mm, green
- **Quantity:** 2 identical boards supplied - please work on ONE only and return the second untouched as a spare
- **Contact:** Marcus Adolfsson - phone: ____________________

## The three jobs

1. Move the COMM SEL jumpers from SPI to I2C (three 0 ohm resistors)
2. Remove R1, to disable the power LED
3. Remove both 8-pin headers and clear the holes

### Job 1 - COMM SEL: change from SPI to I2C

**Where:** lower middle of the component side, under the silkscreen **"COMM SEL"**. There are two rows of three footprints. The silkscreen to their right labels the **upper row "I2C"** and the **lower row "SPI"**.

**As supplied:** three 0 ohm resistors are fitted in the **SPI** row. The **I2C** row is empty.

**Required:**

- Remove all three 0 ohm resistors from the **SPI** row, and leave those pads clear and open.
- Make a connection across all three footprints in the **I2C** row - either refit the salvaged 0 ohm resistors, or simply solder-bridge each footprint. A 0 ohm link is electrically just a wire, so a bridge is equivalent and nothing has to match a value.

> **IMPORTANT:** All three links must end up in the I2C row. The manufacturer's documentation states that if the jumpers are not all on the same side, the board will not respond at all.


### Job 2 - Remove R1, to disable the power LED

**Where:** bottom edge of the component side, by the silkscreen **"PWR"** and **"R1"**. R1 is the small black chip resistor marked **4700**.

**Required:** remove **R1** and leave its pads open and unbridged. Please leave the LED itself in place.

**Why:** this board is going into a battery-powered device. The power LED draws a few milliamps continuously - far more than everything else in the device put together - and would flatten the battery in a couple of weeks.

> **IMPORTANT:** Please do not bridge the R1 pads after removing it.


### Job 3 - Remove the two 8-pin headers

**Where:** the two 8-pin male pin headers running along the long edges, on the underside of the board.

**Required:**

- Remove both headers completely.
- Clear all 16 through-holes of solder (braid or solder sucker) so that wire can be passed through them.

> **IMPORTANT:** I will be soldering my own wires into these holes afterwards, so please take care not to lift any pads or damage the plated-through holes.

## Please do NOT

- **Do not touch the top half of the board.** The large rectangular spiral there is the NFC antenna. Scratches, solder splashes or flux residue on it will detune the antenna and reduce the read range.
- **Do not apply hot air to the antenna half** of the board.
- **Do not remove, move or reflow any other component** - in particular the crystal (**Y1**), the chip itself, or the small capacitors and inductors sitting between the chip and the antenna. Those form a tuned matching network.

## Checks before returning the board

- All three I2C links solid; all three SPI positions clear and open.
- R1 removed, its pads open and unbridged.
- No solder bridges anywhere around the chip.
- All 16 header holes clear and undamaged, with no lifted pads.
- Flux residue cleaned off (IPA is fine) and the board visually inspected.

---

**Background, if useful.** The board is a 13.56 MHz NFC reader. It ships configured to talk to a host computer over SPI, and the host it is going into talks I2C only - hence job 1. It is then built into a small battery-powered enclosure with the antenna facing outwards, which is why the LED and the headers have to go. Manufacturer product page: www.mikroe.com/nfc-4-click
