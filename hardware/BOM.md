# ESPLeafSOC — Bill of Materials

This covers parts for the main ESPLeafSOC build and the optional OLED Adapter PCB. Quantities are per single unit.

---

## Main build

| Item | Qty | Notes | Source |
|---|---|---|---|
| LilyGo T-CAN485 | 1 | ESP32, onboard CAN transceiver, WiFi | https://lilygo.cc/products/t-can485 |
| 1.3" 128×64 OLED, SPI, SH1106 | 1 | e.g. Keyestudio KS0056 — confirmed working | Aliexpress / Amazon / generic |
| 1.3" 128×64 OLED, SPI, SSD1306 | 1 | Alternative driver, also supported (select in web portal) | Aliexpress / Amazon / generic |
| OBD2 plug (male) | 1 | To tap into Leaf's EV-CAN bus + switched 12V | Aliexpress / Amazon |
| Wire, ~500mm | 7 cores | OLED connection (VCC, GND, CLK, MOSI, RES, DC, CS) | Generic hookup wire |
| Case/enclosure | 1 | See `/hardware/case/` for printable STL | 3D printed |

No separate MCP2515 CAN module, buck converter, or 5V regulator is required — these were needed for Paul Kennett's original Arduino-based build but are made redundant by the T-CAN485's onboard CAN transceiver and direct 12V input.

---

## ESPLeafSOC OLED Adapter PCB (optional)

For upgrading an existing Paul Kennett LeafSOCdisplay install without re-pinning the OLED ribbon cable. Gerber files: [`/hardware/oled-adapter/`](./oled-adapter/).

| Item | Qty | LCSC Part # | Notes |
|---|---|---|---|
| PCB | 1 | — | Order from Gerber_ESPLeafSOC-OLED-Adapter.zip |
| H1 — 12-pin header, 2×6, T-CAN485 side | 1 | *TBC* | Pin pitch and connector type to be confirmed against received boards |
| P1 — 10-pin header, 2×5, OLED side | 1 | *TBC* | Pin pitch and connector type to be confirmed against received boards |

LCSC part numbers and any assembly notes (pin pitch, keying/orientation) to be added once boards have arrived and been physically checked against the schematic.

---

## Notes

- This BOM will be updated as parts are confirmed against received hardware — check back before ordering a full set if building more than one unit.
- Driver chip (SH1106/SSD1306) is selectable from the web portal at runtime — no need to commit to one before ordering, both are supported by the same firmware.
