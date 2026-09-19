import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

# Same power-up demo constant as project.v's DEFAULT_SR (HTTPS://TINYTAPEOUT.COM,
# level L, mask 5). Reusing it here means this test doesn't need to import
# qr_host.py separately -- it builds its own known-good push stream.
DEFAULT_SR = 0x20bb1aa65463dd52b85b48e39c56a868d160ecba9456988a668c
DEFAULT_ECC = 0b00    # L
DEFAULT_MASK = 0b101  # 5


def chunks_for(codeword_bits: int, width: int = 208):
    """Build the 35 x 6-bit push stream: 2 dummy zero bits + `width` codeword
    bits, MSB first -- per project.v's documented data protocol."""
    stream = "00" + format(codeword_bits, f"0{width}b")
    assert len(stream) % 6 == 0
    return [int(stream[i:i + 6], 2) for i in range(0, len(stream), 6)]


async def send_cmd(dut, addr, data):
    """One command per project.v's protocol:
       - drive ui_in = 00_dddddd (the "idle"/address-00 phase) to arm the decoder
       - drive ui_in = aa_dddddd (the real command) and hold it
       Each phase is held for 4 cycles, comfortably over the 2-cycle glitch
       filter, and well over the ~4-cycle minimum the header comment asks for."""
    dut.ui_in.value = 0
    await ClockCycles(dut.clk, 4)
    dut.ui_in.value = ((addr & 0b11) << 6) | (data & 0b111111)
    await ClockCycles(dut.clk, 4)


@cocotb.test()
async def test_project(dut):
    # 25.175 MHz nominal VGA pixel clock.
    clock = Clock(dut.clk, 39.72, unit="ns")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 10)

    # --- Power-up demo: the shift register / ecc / mask should already hold
    # the built-in DEFAULT_SR pattern, with no data sent yet. ---
    assert int(dut.user_project.sr.value) == DEFAULT_SR, "power-up demo pattern wrong"
    assert int(dut.user_project.ecc.value) == DEFAULT_ECC, "power-up ECC wrong"
    assert int(dut.user_project.mask.value) == DEFAULT_MASK, "power-up mask wrong"

    # --- VGA sync sanity check over one full line ---
    # 640 active + 16 front porch + 96 sync + 48 back porch = 800 clocks.
    # Wait a multiple of 800 cycles so the check loop below starts on a line
    # boundary (hc == 0). Waiting a non-multiple (the original bug: 15000,
    # which is 15000 % 800 == 600 cycles off) misaligns loop index "i" against
    # the chip's real hc counter and produces false hsync mismatches.
    await ClockCycles(dut.clk, 800 * 4)

    for i in range(800):
        hsync = int(dut.uo_out.value[7])
        expected = 0 if 656 <= i < 752 else 1
        assert hsync == expected, f"bad hsync at {i}: {hsync} != {expected}"
        await ClockCycles(dut.clk, 1)

    # --- Protocol test: re-load the same demo payload via the real ui_in
    # command interface (ee = ecc, mmm = mask, dddddd = 6 data bits per push)
    # and confirm the shift register round-trips correctly. This replaces the
    # old uio_in "byte loader" / GO-strobe section, which drove a protocol
    # project.v never implements (uio_in is entirely unused by the chip). ---
    await send_cmd(dut, 0b01, DEFAULT_ECC)     # set ECC level
    await send_cmd(dut, 0b10, DEFAULT_MASK)    # set mask

    for chunk in chunks_for(DEFAULT_SR):
        await send_cmd(dut, 0b11, chunk)

    await ClockCycles(dut.clk, 2)
    assert int(dut.user_project.sr.value) == DEFAULT_SR, "reloaded payload didn't match"
    assert int(dut.user_project.ecc.value) == DEFAULT_ECC
    assert int(dut.user_project.mask.value) == DEFAULT_MASK

    # --- Load a different mask + an easy-to-eyeball all-zero codeword, and
    # confirm both land correctly. ---
    NEW_MASK = 0b111
    await send_cmd(dut, 0b10, NEW_MASK)
    for chunk in chunks_for(0x0):
        await send_cmd(dut, 0b11, chunk)

    await ClockCycles(dut.clk, 2)
    assert int(dut.user_project.mask.value) == NEW_MASK, "mask change didn't take"
    assert int(dut.user_project.sr.value) == 0, "all-zero payload didn't load"

    await ClockCycles(dut.clk, 100)