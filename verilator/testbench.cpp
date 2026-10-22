//==============================================================================
// testbench.cpp — Verilator C++ Testbench for i2c_top
//
// Drives the I2C controller via the host interface, models external I2C bus
// pull-ups, and runs basic write/read/NACK transactions.
//
// Build (from project root):
//   verilator --cc --exe --build -j 4 --timing \
//     --top-module i2c_top \
//     -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-SYNCASYNCNET -Wno-WIDTHEXPAND \
//     -CFLAGS "-std=c++17 -O2" \
//     --trace \
//     rtl/i2c_pkg.sv rtl/smbus_pec.sv \
//     rtl/i2c_master.sv rtl/i2c_slave.sv rtl/i2c_top.sv \
//     verilator/testbench.cpp
//
// Then run: obj_dir/Vi2c_top
//==============================================================================

#include <verilated.h>
#include <verilated_vcd_c.h>
#include "Vi2c_top_verilator.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>

//==============================================================================
// Simulation state
//==============================================================================
static Vi2c_top_verilator  *top;
static uint64_t   sim_time = 0;
static uint64_t   posedge_cnt = 0;
static VerilatedVcdC *tfp = nullptr;

//==============================================================================
// Clock & reset helpers
// NB: I2C pullups are handled by the wrapper (i2c_top_verilator.sv)
// via === 1'b0 conversion. No C++ pullup modeling needed.
//==============================================================================
static void toggle_clk() {
    top->clk = !top->clk;
    top->eval();
    if (tfp) tfp->dump(sim_time);
    sim_time += 10;  // 10 ns half-period → 20 ns = 50 MHz
    if (top->clk) posedge_cnt++;
}

static void wait_clks(int n) {
    for (int i = 0; i < n * 2; i++) toggle_clk();
}

static void pulse_reset() {
    top->rst_n = 0;
    wait_clks(10);
    top->rst_n = 1;
    wait_clks(10);
}

//==============================================================================
// Test checks (declared before wait_mst_done which uses them)
//==============================================================================
static int pass_cnt = 0, fail_cnt = 0;

static void check(const char *desc, bool cond) {
    if (cond) {
        printf("  PASS: %s\n", desc);
        pass_cnt++;
    } else {
        printf("  FAIL: %s\n", desc);
        fail_cnt++;
    }
}

//==============================================================================
// Testbench control signals (master)
//==============================================================================
static void master_start(uint8_t addr, bool rw, uint8_t data_tx, uint8_t nbytes) {
    top->mst_addr      = addr;
    top->mst_rw        = rw ? 1 : 0;
    top->mst_data_tx   = data_tx;
    top->mst_num_bytes = nbytes;
    top->mst_start     = 1;
    wait_clks(1);
    top->mst_start     = 0;
}

static void wait_mst_done() {
    // Wait for mst_busy to go high then low (transaction complete)
    // Timeout after 5M cycles (~100ms) to avoid infinite hangs
    int cyc = 5000000;
    while (!top->mst_busy && --cyc > 0) wait_clks(1);
    if (cyc <= 0) { printf("  TIMEOUT waiting for mst_busy to go high!\n"); fail_cnt++; return; }
    cyc = 5000000;
    while (top->mst_busy && --cyc > 0) wait_clks(1);
    if (cyc <= 0) { printf("  TIMEOUT waiting for mst_busy to go low!\n"); fail_cnt++; return; }
    wait_clks(2);
}

//==============================================================================
// MAIN
//==============================================================================
int main(int argc, char **argv) {
    // Disable stdout buffering for real-time output
    setbuf(stdout, NULL);
    setbuf(stderr, NULL);

    Verilated::commandArgs(argc, argv);
    Verilated::traceEverOn(true);

    printf("[VERI] Initializing...\n");
    top = new Vi2c_top_verilator;

    // Trace (VCD) // ----no trace for now
    // tfp = new VerilatedVcdC;
    // top->trace(tfp, 99);
    // tfp->open("verilator/vlt_i2c.vcd");

    // -------- Defaults (reset asserted) --------
    top->clk       = 0;
    top->rst_n     = 0;  // Start with reset asserted
    top->mst_start = 0;
    top->mst_reset = 0;
    top->mst_addr  = 0;
    top->mst_rw    = 0;
    top->mst_data_tx = 0;
    top->mst_num_bytes = 0;
    top->tb_sda_force_low = 0;

    // Slave data interface
    top->slv_tx_data  = 0;
    top->slv_tx_valid = 0;
    top->slv_rx_ready = 1;

    top->eval();

    printf("\n");
    printf("========================================\n");
    printf("  Verilator I2C Controller Testbench\n");
    printf("========================================\n");
    printf("\n");

    // -------- Reset --------
    printf("--- Test 1: Reset and idle conditions ---\n");
    pulse_reset();
    check("Bus idle (mst_busy=0)", top->mst_busy == 0);
    check("Slave idle (slv_busy=0)", top->slv_busy == 0);
    check("No ack error", top->mst_ack_error == 0);

    // -------- Master Write --------
    printf("\n--- Test 2: Master Write to Slave (addr=0x50, data=0xAB) ---\n");
    // Pre-load slave TX data (for reads)
    top->slv_tx_data  = 0xA5;
    top->slv_tx_valid = 1;
    wait_clks(2);

    master_start(0x50, false, 0xAB, 1);
    // Capture signals while transaction is active
    wait_mst_done();

    // Note: slv_addressed and slv_dir_rw are registered outputs that clear
    // when the slave returns to S_IDLE. Capture them during the transaction
    // by checking mst_ack_error (registered and persists) and data signals.
    check("No ACK error on write", top->mst_ack_error == 0);
    // Capture slave RX data (updated at the end of each byte)
    uint8_t slv_rx = top->slv_rx_data;
    check("Slave received correct data (0xAB)", slv_rx == 0xAB);

    // -------- Master Read --------
    printf("\n--- Test 3: Master Read from Slave (addr=0x50) ---\n");
    top->slv_tx_data  = 0x3C;
    top->slv_tx_valid = 1;
    wait_clks(2);

    master_start(0x50, true, 0, 1);
    wait_mst_done();

    check("No ACK error on read", top->mst_ack_error == 0);
    uint8_t mst_rx = top->mst_data_rx;
    check("Master read correct data (0x3C)", mst_rx == 0x3C);
    top->slv_tx_valid = 0;

    // -------- Repeated START --------
    printf("\n--- Test 4: Repeated START (write+read) ---\n");
    top->slv_tx_data  = 0x99;
    top->slv_tx_valid = 1;
    wait_clks(2);

    master_start(0x50, false, 0x11, 1);
    wait_mst_done();
    check("Write in repeated START - no NACK", top->mst_ack_error == 0);

    top->slv_tx_data  = 0x99;
    top->slv_tx_valid = 1;
    wait_clks(5);

    master_start(0x50, true, 0, 1);
    wait_mst_done();
    check("Read in repeated START - no NACK", top->mst_ack_error == 0);
    check("Repeated START read data (0x99)", top->mst_data_rx == 0x99);
    top->slv_tx_valid = 0;

    // -------- Burst Write --------
    printf("\n--- Test 5: Burst Write (4 bytes) ---\n");
    master_start(0x50, false, 0x01, 4);
    wait_mst_done();
    check("Burst write - no NACK", top->mst_ack_error == 0);

    // -------- NACK Detection --------
    printf("\n--- Test 6: NACK detection (non-existent address 0x7F) ---\n");
    master_start(0x7F, false, 0xFF, 1);
    wait_mst_done();
    check("Master detected NACK for non-existent slave", top->mst_ack_error == 1);

    // -------- Genuine Arbitration Loss Detection --------
    // Every check above only confirms mst_arb_lost stays 0 during normal
    // single-master traffic -- the set path (a second master winning
    // arbitration) was never actually exercised. Verilator's C++ flow has
    // no hierarchy introspection (no --public here) and no force/release,
    // so instead of peeking at the master's internal FSM state or forcing
    // the bus net directly (as the iverilog testbenches do), we drive a
    // dedicated tb_sda_force_low input added to i2c_top_verilator.sv,
    // timed off the one resolved bus signal we *can* observe: sda_o.
    //
    // Address 0x50 = 7'b1010000 -- its MSB is '1', so right after the
    // START condition (which pulls SDA low) the master releases SDA high
    // to send that first bit. Watching for the first 0->1 transition of
    // sda_o after START catches exactly that moment.
    printf("\n--- Test 7: Genuine Arbitration Loss Detection ---\n");
    master_start(0x50, false, 0x00, 1);

    {
        int cyc = 5000000;
        while (!top->mst_busy && --cyc > 0) wait_clks(1);
        check("Transaction started (mst_busy=1)", top->mst_busy == 1);

        bool saw_low = false;
        cyc = 5000000;
        while (--cyc > 0) {
            wait_clks(1);
            if (!top->sda_o) saw_low = true;
            if (saw_low && top->sda_o) break;
        }
        check("Observed SDA pulled low (START) then released high (address MSB)",
              saw_low && top->sda_o);

        // Inject: a second master driving SDA low right as ours releases it.
        top->tb_sda_force_low = 1;

        // Margin for the master's own scl_rising_event to sample the
        // corrupted bus -- several SCL periods' worth comfortably covers
        // it regardless of internal FSM pipeline depth.
        wait_clks(600);

        check("Arbitration loss detected (mst_arb_lost)", top->mst_arb_lost == 1);

        top->tb_sda_force_low = 0;
    }

    // Drain the deliberately-corrupted transaction before moving on.
    wait_mst_done();

    // -------- Summary --------
    printf("\n");
    printf("========================================\n");
    printf("  VERILATOR TEST RESULTS\n");
    printf("  PASS: %d   FAIL: %d\n", pass_cnt, fail_cnt);
    printf("========================================\n");
    printf("\n");

    if (tfp) tfp->close();
    delete top;
    return (fail_cnt == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}
