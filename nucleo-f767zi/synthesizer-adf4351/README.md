Notes

- Need to send MSB first
- Need manual SS control because we want to send 32 bits, using PB8
- Clock polarity and phase: `CPOL=0` and `CPHA=0` (rising edge), i.e. `mode 0`
- Had to do pin remapping from the default CubeMX SP1 config to make it match the board schematic
- Used LSB first to match on-the-wire to in-memory format

Links

- [ADF4351 Datasheet](http://www.analog.com/media/en/technical-documentation/data-sheets/ADF4351.pdf)
- [Helpful video by OpenLabTech](https://www.youtube.com/channel/UCeF7JKNXOy0jpMOxpgbZcpg)
- [Helpful code by OpenLabTech](https://github.com/jhol/pyadf435x/blob/master/adf435x/core.py)
