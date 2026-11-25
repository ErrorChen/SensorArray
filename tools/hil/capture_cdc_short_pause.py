"""Pause the VOLT2 formal CDC application reader once while BLE/debug keep reading.

Linux can buffer bytes during the pause. This injects host application latency,
with no claim that USB URBs, the controller or the device stopped for 650 ms.
"""
import sys
import time

import capture_cdc_benchmark as cdc


class PausedReader(cdc.CdcBenchmarkCollector):
    runner_source_file = __file__
    variant = 'formal_cdc_single_650ms_application_pause'
    pause_bracket = None

    def pump(self, deadline):
        now = time.monotonic()
        if (self.phase == 'capture' and not self.periodic_active and self.pause_bracket is None
                and now >= self.capture_started + 17 and min(deadline, self.capture_deadline) - now >= 1):
            self.event('user_space_read_pause_start', requested_seconds=0.650)
            start = time.monotonic()
            time.sleep(0.650)
            self.pause_bracket = (start, time.monotonic())
            self.event('user_space_read_pause_end', duration_seconds=self.pause_bracket[1] - start)
        super().pump(deadline)

    def run(self):
        result = super().run()
        bracket = self.pause_bracket
        result['formal_reader_pause_monotonic'] = bracket
        result['formal_reader_pause_verified'] = bool(
            bracket and self.capture_started <= bracket[0] and bracket[1] <= self.capture_finished
            and bracket[1] - bracket[0] >= 0.650)
        return result


if __name__ == '__main__':
    with cdc.shared.bulk_collector_selected(PausedReader):
        raise SystemExit(cdc.shared.base.main(['--transport', 'usb_cdc', *sys.argv[1:]], require_image_identity=True))
