#!/usr/bin/env python3
"""Compile the actual PX4 DShot functions with host stubs and check valid-sample handling.

DMA capture and interrupt scheduling are not exercised by this host check.
"""
import argparse
from pathlib import Path
import subprocess


def function(source, signature):
    start = source.index(signature + "\n{")
    body = source.index("{", start)
    depth = 1
    end = body + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


PRELUDE = r'''
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#define MAX_TIMER_IO_CHANNELS 4
#define PX4_OK 0
#define PX4_ERROR -1
static int32_t _erpms[4]{};
static bool _erpms_ready[4]{};
static uint32_t read_ok[4]{}, read_fail_nibble[4]{}, read_fail_crc[4]{};
static unsigned next_period = 0;
static int critical_depth = 0;
using irqstate_t = int;
irqstate_t px4_enter_critical_section() { return critical_depth++; }
void px4_leave_critical_section(irqstate_t flags) { assert(critical_depth == flags + 1); critical_depth = flags; }
unsigned calculate_period(uint8_t, uint8_t) { return next_period; }
uint8_t output_channel_from_timer_channel(uint8_t, uint8_t channel) { return channel; }
struct TimerConfig { bool initialized_channels[4]; } timer_configs[] = {{{true,true,true,true}}};
struct Channel { uint8_t timer_index, timer_channel; } timer_io_channels[] = {{0,1},{0,2},{0,3},{0,4}};
static uint64_t clock_us = 100000;
uint64_t hrt_absolute_time() { return ++clock_us; }
void perf_count(int) {}
struct EscReport { uint64_t timestamp{}; int esc_rpm{}, actuator_function{}; };
struct esc_status_s {
    enum { ESC_CONNECTION_TYPE_DSHOT = 1 };
    uint64_t timestamp{};
    unsigned counter{}, esc_connectiontype{}, esc_armed_flags{}, esc_online_flags{};
    EscReport esc[4];
};
struct Publication { esc_status_s data; esc_status_s &get() { return data; } };
struct Mixing { bool isFunctionSet(unsigned i) { return i < 4; } };
struct Param { int get() { return 14; } };
struct DShot {
    Publication esc_status_pub;
    Mixing _mixing_output;
    Param _param_mot_pole_count;
    unsigned _num_motors{4}, _num_outputs{4}, _esc_status_counter{}, _outputs_on{1};
    int _actuator_functions[4]{101,102,103,104}, _bdshot_rpm_perf{};
    int handle_new_bdshot_erpm(void);
};
'''

TESTS = r'''
void reset() {
    memset(_erpms, 0, sizeof(_erpms));
    memset(_erpms_ready, 0, sizeof(_erpms_ready));
}
void capture(unsigned channel, unsigned period) {
    next_period = period;
    process_capture_results(0, channel);
}
int main() {
    unsigned map[16]{};
    for (unsigned symbol=0; symbol<32; ++symbol) {
        unsigned nibble = nibbles_from_mapped(symbol);
        if (nibble != 0xff) map[nibble] = symbol;
    }
    unsigned legal_accepted=0, invalid_accepted=0;
    for (unsigned gcr=0; gcr<(1u<<20); ++gcr) {
        bool valid_symbols=true;
        for (unsigned pos=0; pos<4; ++pos)
            valid_symbols &= nibbles_from_mapped((gcr>>(5*pos))&31) != 0xff;
        unsigned before=read_ok[0];
        decode_gcr(gcr);
        if (read_ok[0] != before) {
            if (valid_symbols) ++legal_accepted;
            else ++invalid_accepted;
        }
    }
    assert(legal_accepted == 4096);
    assert(invalid_accepted == 0);
    printf("All 1048576 GCR words: valid-symbol CRC passes=%u, invalid-symbol CRC passes=%u\n",
        legal_accepted, invalid_accepted);
    for (unsigned payload=0; payload<4096; ++payload) {
        unsigned checksum=(~(payload^(payload>>4)^(payload>>8)))&15;
        unsigned word=(payload<<4)|checksum;
        unsigned gcr=0;
        for (unsigned pos=0; pos<4; ++pos) gcr |= map[(word>>(pos*4))&15] << (pos*5);
        assert(decode_gcr(gcr) == ((payload&511) << (payload>>9)));
    }
    puts("All 4096 valid payload periods match the reference.");
    reset(); capture(0,0);
    int erpm=-123;
    int rc=up_bdshot_get_erpm(0,&erpm);
    assert(rc==-1 && erpm==-123);
    printf("Empty capture: return=%d erpm=%d\n",rc,erpm);
    reset(); capture(0,200); capture(0,0);
    rc=up_bdshot_get_erpm(0,&erpm);
    assert(rc==0 && erpm==3000);
    printf("Valid sample then invalid before read: erpm=%d\n",erpm);
    rc=up_bdshot_get_erpm(0,&erpm);
    assert(rc==-1);
    printf("Read same sample again: return=%d\n",rc);
    reset(); capture(0,65408);
    assert(up_bdshot_get_erpm(0,&erpm)==0 && erpm==0);
    puts("Valid stopped-motor sample remains readable as zero.");
    reset(); capture(0,200);
    assert(up_bdshot_get_erpm(0,&erpm)==0);
    capture(0,0);
    assert(up_bdshot_num_erpm_ready()==0 && up_bdshot_get_erpm(0,&erpm)==-1);
    reset(); capture(0,200); timer_configs[0].initialized_channels[0]=false;
    assert(up_bdshot_get_erpm(0,&erpm)==-1);
    timer_configs[0].initialized_channels[0]=true;
    reset(); DShot driver;
    for (unsigned i=0;i<4;++i) capture(i,200+i*10);
    assert(driver.handle_new_bdshot_erpm()==4);
    uint64_t old_timestamp=driver.esc_status_pub.data.esc[3].timestamp;
    int old_rpm=driver.esc_status_pub.data.esc[3].esc_rpm;
    for (unsigned i=0;i<3;++i) capture(i,300+i*10);
    capture(3,0);
    unsigned count=driver.handle_new_bdshot_erpm();
    assert(count==3u);
    auto &missing=driver.esc_status_pub.data.esc[3];
    assert(missing.timestamp==old_timestamp && missing.esc_rpm==old_rpm);
    printf("One failed channel: updated=%u; failed channel rpm=%d; timestamp advanced=%d\n", count, missing.esc_rpm, missing.timestamp>old_timestamp);
    assert(driver.handle_new_bdshot_erpm()==0);
    assert(critical_depth==0);
    puts("PASS");
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--build-dir", type=Path, required=True)
    args = parser.parse_args()
    c = (args.source / "platforms/nuttx/src/px4/stm/stm32_common/dshot/dshot.c").read_text()
    cpp = (args.source / "src/drivers/dshot/DShot.cpp").read_text()
    decoder = function(c, "unsigned calculate_period(uint8_t timer_index, uint8_t channel_index)")
    # Exercise the unmodified GCR decoding body, after pulse capture conversion.
    decoder = "unsigned decode_gcr(unsigned gcr)\n{\nunsigned channel_index = 0;\n" + decoder[decoder.index("\tuint32_t data = 0;"):]
    functions = "\n".join([
        function(c, "uint8_t nibbles_from_mapped(uint8_t mapped)"), decoder,
        function(c, "void process_capture_results(uint8_t timer_index, uint8_t channel_index)"),
        function(c, "int up_bdshot_num_erpm_ready(void)"),
        function(c, "int up_bdshot_get_erpm(uint8_t output_channel, int *erpm)"),
        function(cpp, "int DShot::handle_new_bdshot_erpm(void)"),
    ])
    args.build_dir.mkdir(parents=True, exist_ok=True)
    source = args.build_dir / "dshot_telemetry_check.cpp"
    executable = args.build_dir / "dshot_telemetry_check"
    source.write_text(PRELUDE + functions + TESTS)
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-fsanitize=undefined",
                    str(source), "-o", str(executable)], check=True)
    subprocess.run([str(executable.resolve())], check=True)


if __name__ == "__main__":
    main()
