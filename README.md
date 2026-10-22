# I2C / SMBus 3.0 Controller - SystemVerilog Design

A synthesisable **I2C / SMBus 3.0 bus controller** implemented in SystemVerilog. This project includes both a master and a slave controller, a top-level integration module, and an extensive self-checking testbench suite covering standard I2C, SMBus 3.0 features (PEC, ARP, SMBALERT#, clock stretching, the 35 ms timeout), and several randomized/mutation-tested regressions.

## Features

### I2C Master
- **7-bit addressing** with configurable slave address
- **Standard (100 kHz)** and **Fast (400 kHz)** mode operation
- **Read/Write** single-byte and **multi-byte burst** transactions (up to 256 bytes)
- **Repeated START** condition support
- **Clock stretching** support (real slave-driven, not just forced)
- **ACK/NACK** detection and error reporting
- **Arbitration loss** detection (address-byte collisions)
- **SMBus 3.0 PEC** (Packet Error Checking — CRC-8) generation, cross-checked against an independent reference implementation
- **SMBus 3.0 35 ms SCL-low timeout** detection, sticky until genuinely resolved (does not self-clear as a side effect of the FSM returning to idle)
- **Configurable SCL frequency** via an asymmetric system clock divider (meets I2C `t_LOW` at both 100 kHz and 400 kHz)

### I2C Slave
- **Programmable 7-bit address**, plus SMBus default address (`0x61`) and Alert Response Address (`0x0C`) support
- **General call (0x00)** address detection (optional)
- **Read/Write** transaction handling with double-buffered TX/RX handshake
- **ACK/NACK** generation
- **START/STOP** condition detection, bus busy tracking
- **Clock stretching**: genuinely holds SCL low on write (waiting for `rx_ready`) and on the first byte of a read (waiting for `tx_valid`) — see [Known Limitations](#known-limitations) for stretching on byte 2+ of a read
- **SMBus 3.0 PEC** verification (CRC-8 check at STOP)
- **SMBus 3.0 SMBALERT#** open-drain interrupt signaling
- **SMBus 3.0 ARP** (Address Resolution Protocol): Prepare-to-ARP, Get UDID (general and directed), Assign Address, Reset Device (general and directed)

### Protocol Support
- START and STOP condition generation/detection
- Data transfer MSB-first, 8-bit + ACK
- Repeated START (combined transactions)
- Bus arbitration (multi-master aware)

## Architecture

```
                         ┌─────────────────────────┐
                         │       i2c_top            │
                         │                          │
  Host Interface ──────► │  ┌──────────┐            │
  (Master)               │  │ I2C      │            │
                         │  │ Master   │───┬───► SCL│──► to bus
                         │  └──────────┘   │       │
                         │                 ├───► SDA│──► to bus
  Host Interface ──────► │  ┌──────────┐   │       │
  (Slave)                │  │ I2C      │   │       │
                         │  │ Slave    │───┘       │
                         │  └──────────┘           │
                         └─────────────────────────┘
```

### Module Descriptions

| Module | File | Description |
|--------|------|-------------|
| `i2c_pkg` | `rtl/i2c_pkg.sv` | Package with parameters, typedefs, and constants (incl. ARP command codes, UDID width) |
| `smbus_pec` | `rtl/smbus_pec.sv` | SMBus 3.0 PEC CRC-8 accumulator, shared by master and slave |
| `i2c_master` | `rtl/i2c_master.sv` | I2C Master controller (FSM-based) |
| `i2c_slave` | `rtl/i2c_slave.sv` | I2C Slave controller (FSM-based) |
| `i2c_top` | `rtl/i2c_top.sv` | Top-level integration with tri-state pads |
| `i2c_top_verilator` | `rtl/i2c_top_verilator.sv` | Verilator-only variant using `logic` bus ports instead of `inout`, plus a `tb_sda_force_low` fault-injection input for arbitration testing |
| `tb_i2c` | `tb/tb_i2c.sv` | Self-checking testbench |

## Getting Started

### Prerequisites

- **Icarus Verilog** (>= 10.0 recommended) for simulation
- **Verilator** (>= 5.0 recommended) — optional, alternative simulator
- **GTKWave** (optional) for waveform viewing

Install on Ubuntu/Debian:
```bash
sudo apt-get install iverilog verilator gtkwave
```

Install on macOS:
```bash
brew install icarus-verilog verilator gtkwave
```

### Quick Start

Compile and simulate:
```bash
make run
```

Or step by step:
```bash
make        # Compile all sources
make sim    # Run simulation
make waves  # View waveforms (if GTKWave is installed)
```

### Clean Up
```bash
make clean
```

### Running with Different Parameters

Override parameters from the command line:
```bash
# Fast-mode (400 kHz)
make sim SCL_FREQ=400000

# Custom slave address (0x20)
make sim SLAVE_ADDR=32
```

### Available Testbenches

The `tb/` directory contains one self-contained testbench per module/mode
combination — each has its own `initial` block and is meant to be compiled
and run on its own (not all bundled together):

| Testbench | Covers |
|-----------|--------|
| `tb_i2c.sv` | Core I2C master/slave transactions, standard mode (100 kHz) |
| `tb_i2c_fastmode.sv` | Core I2C transactions, fast mode (400 kHz) |
| `tb_i2c_smbus.sv` | SMBus 3.0 features (PEC, SMBALERT#/ARA, ARP, clock stretching); randomized burst write/read (Tests 19-20); randomized PEC burst write with a reference-CRC cross-check and mutation-tested corruption detection (Test 21); randomized ARP Assign Address including UDID-mismatch rejection (Test 22); genuine arbitration-loss detection (Test 23); standard mode |
| `tb_i2c_smbus_fastmode.sv` | Same SMBus 3.0 feature set and Tests 19-23, fast mode |
| `tb_i2c_smbus_timeout.sv` | 35 ms SCL-low timeout (forced bus hang), standard mode |
| `tb_i2c_smbus_timeout_fastmode.sv` | Same timeout test, fast mode |
| `tb_i2c_smbus_stretch_timeout.sv` | Real slave-driven clock stretching interacting with the 35 ms timeout; randomized burst write with interspersed legitimate stretching; randomized burst read regression; real over-limit stretch at a random position within a burst; standard mode |
| `tb_i2c_smbus_stretch_timeout_fastmode.sv` | Same stretch/timeout interaction and randomized-burst tests, fast mode |
| `tb_i2c_read_stretch_regression.sv` | Standalone regression for genuine clock stretching on byte 2+ of a multi-byte read (first/middle/last byte scenarios), standard mode — see [Known Limitations](#known-limitations) |
| `tb_i2c_read_stretch_regression_fastmode.sv` | Same read-stretch scenarios, fast mode |

### Running a Testbench with Icarus Verilog

The default `make sim` target only elaborates `tb_i2c`. To compile and run
any individual testbench directly with `iverilog`/`vvp`:

```bash
iverilog -g2012 -Wall -Winfloop -I rtl -I tb \
  -o build/<testbench>.vvp \
  rtl/i2c_pkg.sv rtl/smbus_pec.sv rtl/i2c_master.sv rtl/i2c_slave.sv rtl/i2c_top.sv \
  tb/<testbench>.sv

vvp build/<testbench>.vvp
```

For example, to run the SMBus 3.0 fast-mode testbench:
```bash
iverilog -g2012 -Wall -Winfloop -I rtl -I tb \
  -o build/tb_i2c_smbus_fastmode.vvp \
  rtl/i2c_pkg.sv rtl/smbus_pec.sv rtl/i2c_master.sv rtl/i2c_slave.sv rtl/i2c_top.sv \
  tb/tb_i2c_smbus_fastmode.sv

vvp build/tb_i2c_smbus_fastmode.vvp
```

Repeat for each file listed in [Available Testbenches](#available-testbenches)
to run the full regression suite.

#### Viewing Waveforms from an Icarus Verilog Run

Each testbench already calls `$dumpfile`/`$dumpvars` in its source, so no
extra flags are needed — running it with `vvp` writes a waveform to the
current directory named after the testbench (e.g. `tb_i2c_smbus_fastmode.vcd`).
Open it with GTKWave once the run finishes:
```bash
vvp build/tb_i2c_smbus_fastmode.vvp
gtkwave tb_i2c_smbus_fastmode.vcd
```

### Running a Testbench with Verilator

Verilator can run these testbenches too, using its `--binary` mode (which
auto-generates a C++ driver so the SystemVerilog `initial`/`$finish`
testbench runs standalone) with `--timing` enabled (required for the
`#delay` statements the testbenches use):

```bash
verilator --binary --timing -Wno-fatal \
  --top-module <testbench> -Mdir build/verilator_<testbench> \
  -Irtl -Itb \
  rtl/i2c_pkg.sv rtl/smbus_pec.sv rtl/i2c_master.sv rtl/i2c_slave.sv rtl/i2c_top.sv \
  tb/<testbench>.sv \
  -o <testbench>

./build/verilator_<testbench>/<testbench>
```

For example:
```bash
verilator --binary --timing -Wno-fatal \
  --top-module tb_i2c_smbus_fastmode -Mdir build/verilator_tb_i2c_smbus_fastmode \
  -Irtl -Itb \
  rtl/i2c_pkg.sv rtl/smbus_pec.sv rtl/i2c_master.sv rtl/i2c_slave.sv rtl/i2c_top.sv \
  tb/tb_i2c_smbus_fastmode.sv \
  -o tb_i2c_smbus_fastmode

./build/verilator_tb_i2c_smbus_fastmode/tb_i2c_smbus_fastmode
```

Notes:
- Always give each testbench its **own `-Mdir`** — reusing one output
  directory across multiple `verilator` invocations will overwrite files
  from the previous build.
- `-Wno-fatal` silences non-fatal width/pin warnings so the build completes;
  it does not suppress `%Error`s.
- All 10 `tb/` testbenches pass identically under both Icarus Verilog and
  Verilator's `--binary --timing` mode, 0 failures.

#### Viewing Waveforms from a Verilator Run

Unlike Icarus, Verilator ignores `$dumpvars` unless the build is compiled
with tracing enabled — add `--trace` to the command above:
```bash
verilator --binary --timing --trace -Wno-fatal \
  --top-module tb_i2c_smbus_fastmode -Mdir build/verilator_tb_i2c_smbus_fastmode \
  -Irtl -Itb \
  rtl/i2c_pkg.sv rtl/smbus_pec.sv rtl/i2c_master.sv rtl/i2c_slave.sv rtl/i2c_top.sv \
  tb/tb_i2c_smbus_fastmode.sv \
  -o tb_i2c_smbus_fastmode

./build/verilator_tb_i2c_smbus_fastmode/tb_i2c_smbus_fastmode
gtkwave tb_i2c_smbus_fastmode.vcd
```
Without `--trace`, the run still succeeds but prints
`$dumpvar ignored, as Verilated without --trace` and no `.vcd` file is written.

#### Pre-Loading Signals into GTKWave

Opening a `.vcd` directly shows the signal tree but draws nothing — GTKWave
never auto-populates the waveform pane. Each testbench has a matching Tcl
script (`add_<testbench>_signals.tcl`) that adds a useful signal set — the
top-level bus/control ports plus the master/slave FSM state, stretch, PEC,
ARP, and SMBALERT# internals — and zooms to fit. Load it with `-S`:
```bash
gtkwave tb_i2c_smbus_fastmode.vcd -S add_tb_i2c_smbus_fastmode_signals.tcl
```

These scripts only use GTKWave Tcl commands confirmed to exist in this
build (`addSignalsFromList`, `highlightSignalsFromList`, `Zoom_Full` — check
with `info commands gtkwave::*` in a script if you add more, since commands
like `setWindowPrefix`/`setNewRadix`/`getCurrentDump` referenced in some
GTKWave tutorials are not available here and will abort the rest of the
script on first use).

### Native Verilator C++ Flow (`make veri-sim`)

Separate from running the `tb/` SystemVerilog testbenches directly under
Verilator, `verilator/testbench.cpp` drives `rtl/i2c_top_verilator.sv` (the
`inout`-free variant) via a small hand-written C++ driver:
```bash
make veri-sim
```
This flow includes a genuine arbitration-loss test: since Verilator has no
`force`/`release` and this C++ testbench has no hierarchy introspection, a
dedicated `tb_sda_force_low` input on `i2c_top_verilator` lets the driver act
as a second, competing master pulling the shared bus low — timed off the one
signal the driver can observe (`sda_o`, watching for the START-to-address-bit
transition) rather than any internal FSM state.

## Project Structure

```
i2c_controller/
├── rtl/                       # RTL source files
│   ├── i2c_pkg.sv             # Package definitions
│   ├── smbus_pec.sv           # SMBus 3.0 PEC CRC-8 accumulator
│   ├── i2c_master.sv          # I2C Master controller
│   ├── i2c_slave.sv           # I2C Slave controller
│   ├── i2c_top.sv             # Top-level integration (inout ports)
│   └── i2c_top_verilator.sv   # Verilator-only variant (logic ports)
├── tb/                        # Testbench files (10 self-contained testbenches)
├── verilator/
│   └── testbench.cpp          # Native Verilator C++ driver (make veri-sim)
├── Makefile                   # Build automation
└── README.md                  # This file
```

## I2C Protocol Reference

The I2C bus uses two bidirectional open-drain lines (SCL, SDA) with external pull-up resistors.

```
START:  SDA ↓ while SCL high
STOP:   SDA ↑ while SCL high
DATA:   SDA changes while SCL low, sampled while SCL high
ACK:    Receiver pulls SDA low on 9th SCL pulse
NACK:   Receiver leaves SDA high on 9th SCL pulse
```

Transaction format:
```
START | 7-bit Address | R/W | ACK | Data Byte | ACK | ... | STOP
```

## Test Plan

Baseline scenarios covered by `tb_i2c.sv`/`tb_i2c_fastmode.sv`:

| Test | Description |
|------|-------------|
| Reset | Verify initial conditions after reset |
| Master Write | Single byte write to slave, verify data |
| Master Read | Single byte read from slave, verify data |
| Repeated START | Write followed by read without STOP |
| Burst Write | Multi-byte write transaction (4 bytes) |
| Error Detection | NACK detection for non-existent address |
| Multiple Transactions | 5 consecutive write transactions |
| Back-to-back | Immediate write then read |

`tb_i2c_smbus.sv`/`tb_i2c_smbus_fastmode.sv` add SMBALERT#/ARA, PEC, ARP
(Prepare-to-ARP, Get UDID, Assign Address, Reset Device — both general and
directed variants), clock stretching, and five randomized/mutation-tested
regressions (Tests 19-23): randomized burst write/read, randomized PEC burst
write cross-checked against an independent reference CRC-8 with a mutation
loop that deliberately corrupts the PEC byte and confirms detection,
randomized ARP Assign Address including deliberate UDID-mismatch rejection,
and genuine arbitration-loss detection. `tb_i2c_smbus_stretch_timeout.sv`
adds real (not forced) slave-driven clock stretching interacting with the
35 ms timeout, plus randomized burst write/read regressions under a
stretch-capable configuration and a real over-limit stretch at a random
burst position. `tb_i2c_read_stretch_regression.sv` is a dedicated
standalone regression for stretching byte 2+ of a multi-byte read — see
[Known Limitations](#known-limitations).

## Known Limitations

- **Clock stretching on byte 2+ of a multi-byte read is not fully
  randomized-tested.** The RTL correctly stretches when the slave isn't
  ready with data — for the first byte of a read (`i2c_slave.sv`'s
  `S_ACK_REF`-based path) and, after a fix, for any single later byte too
  (`S_TX_DATA`-based path, verified via `tb_i2c_read_stretch_regression.sv`
  for the first, a middle, and the last byte of a burst). A prior version of
  this same condition was dead code (keyed on a `bit_pos` value that state
  never actually holds), so stretching past byte 0 silently never engaged at
  all until this fix. What remains untested is *multiple* bytes stretching
  within the same burst and doing so as part of a large randomized/looped
  regression — orchestrating that correctly from a testbench (timing when to
  withdraw `tx_valid` relative to the RTL's stretch-engage check, which reads
  it at the same clock edge `tx_ready` sets on) proved to have escalating
  edge cases with more consecutive stretches or more prior iterations in the
  same simulation, unrelated to the RTL itself. `tb_i2c_smbus_stretch_timeout.sv`'s
  read-burst test therefore does not deliberately stretch any byte.
- **`mst_pec_error` (master) is dead code.** Its set condition depends on the
  slave NACKing a bad PEC byte, but the slave only ever detects a PEC
  mismatch post-hoc via CRC at STOP — it never NACKs because of PEC. PEC
  correctness is validated through other means (Test 21 cross-checks the
  master's computed PEC against an independent reference CRC-8, and checks
  the slave's own `pec_error`).
- **`arbitration_lost` has narrow test coverage.** The dedicated arbitration
  tests (`tb_i2c_smbus.sv` Test 23, and `make veri-sim`'s Test 7) only cover
  a second master colliding on the address byte. No test exercises
  arbitration loss mid-transaction (e.g. during a data byte) or its
  interaction with clock stretching.

## Synthesis

This design is intended for FPGA implementation. The I2C bus signals (`scl`, `sda`) should be connected to bidirectional I/O buffers (e.g., Xilinx IOBUF, Intel ALT_IOBUF) in the target device.

For synthesis, set the following parameters based on your system clock:
- `CLK_FREQ`: System clock frequency (e.g., 50_000_000 for 50 MHz)
- `SCL_FREQ`: Desired SCL frequency (e.g., 100_000 for standard mode)

## License

This project is open-source hardware.

## Contributing

Contributions, bug reports, and feature requests are welcome.
