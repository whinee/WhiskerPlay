import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles


@cocotb.test()
async def test_project(dut):
    # 25.175 MHz nominal VGA clock.
    clock = Clock(dut.clk, 39.72, unit="ns")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    # Default payload is encoded automatically.  Wait long enough for the
    # eight-mask scorer to finish before sampling VGA timing.
    await ClockCycles(dut.clk, 15000)

    # Basic VGA sync sanity checks over one full line.
    # 640 active + 16 front + 96 sync + 48 back = 800 clocks.
    for i in range(800):
        hsync = int(dut.uo_out.value[7])
        expected = 0 if 656 <= i < 752 else 1
        assert hsync == expected, f"bad hsync at {i}: {hsync} != {expected}"
        await ClockCycles(dut.clk, 1)

    # Exercise the byte loader and forced mask path.  Bytes are entered MSB first.
    payload = bytes.fromhex("0123456789abcdef")
    dut.uio_in.value = 0
    for b in payload:
        dut.ui_in.value = b
        dut.uio_in.value = int(dut.uio_in.value) | (1 << 5)
        await ClockCycles(dut.clk, 1)
        dut.uio_in.value = int(dut.uio_in.value) & ~(1 << 5)
        await ClockCycles(dut.clk, 1)

    # Force mask 7, EC=H (11), then GO.
    dut.uio_in.value = (3 << 0) | (7 << 2) | (1 << 6)
    await ClockCycles(dut.clk, 1)
    dut.uio_in.value = (3 << 0) | (7 << 2)
    await ClockCycles(dut.clk, 1000)
