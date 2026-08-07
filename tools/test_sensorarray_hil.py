import asyncio
import dataclasses
import unittest
import zlib

from sensorarray_hil import (
    BLE_BASE_SUFFIX,
    CTRL_RX_UUID,
    CTRL_TX_UUID,
    DATA_TX_UUID,
    LOG_TX_UUID,
    SERVICE_UUID,
    BleObserver,
    FaultDetector,
    HilFailure,
    HilOutput,
    HilSkipped,
    KnownResistance,
    LineRecord,
    SerialCandidate,
    assertBleChannelQuietAfterUnsubscribe,
    buildArgumentParser,
    buildBleProfile,
    checkKnownResistances,
    expectedCommandPrefix,
    fastHighLoadIncompleteAssemblyBudget,
    formatSubscriptionCase,
    isPreApplyFrame,
    isRuntimeReadyFrame,
    isTransientSnapshotBusy,
    normalizeUuid,
    parseKnownResistance,
    parseSubscriptionCase,
    parseSubscriptionCases,
    requiredLogTagsForSubscription,
    selectSerialPort,
    serialMissingEmissions,
    startSubscriptions,
    stopSubscriptions,
    switchBleMode,
    validateBleWindow,
    validateBleFirmwareCounters,
    validateGatt,
    validateModeFrame,
    validateArguments,
    validateStackTelemetry,
    waitBleCommand,
    waitForBleTraffic,
)
from text_protocol import MeasurementFrame


class SerialSequenceTests(unittest.TestCase):
    def testAcceptsNonBlockingUsbPublicationDrops(self):
        self.assertEqual(serialMissingEmissions(100, 110), 0)
        self.assertEqual(serialMissingEmissions(100, 130), 2)

    def testRejectsStaleOrNonIntegralStride(self):
        for current in (100, 90, 115):
            with self.subTest(current=current):
                with self.assertRaises(HilFailure):
                    serialMissingEmissions(100, current)


class SnapshotBusyTests(unittest.TestCase):
    def testRecognizesOnlyModeStateSeqlockCollision(self):
        line = "ERR,cmd=MODE,reason=snapshot_busy"
        self.assertTrue(isTransientSnapshotBusy("STATE?", line))
        self.assertTrue(isTransientSnapshotBusy("MODE?", line))
        self.assertFalse(isTransientSnapshotBusy("TX?", line))
        self.assertFalse(isTransientSnapshotBusy(
            "STATE?", "ERR,cmd=MODE,reason=response_too_long"))


def fragmentMessage(channel, messageId, payload, fragmentCount, crcOverride=None):
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    if crcOverride is not None:
        crc = crcOverride
    base = len(payload) // fragmentCount
    extra = len(payload) % fragmentCount
    fragments = []
    offset = 0
    for index in range(fragmentCount):
        length = base + (1 if index < extra else 0)
        body = payload[offset:offset + length]
        offset += length
        header = ("G,%s,%d,%d,%d,%d,%d,%08X\n" %
                  (channel, messageId, index, fragmentCount, len(body),
                   len(payload), crc)).encode("ascii")
        fragments.append(header + body)
    return fragments


def capPayload(sequence=7, generation=3, requestId=9, corruptInnerCrc=False):
    lines = [
        ("C,seq=%d,ts=123,rows=1,cells=8,gen=%d,rid=%d,rf=01,pf=01,sf=01,"
         "bad=0/0/0,fmt=pf6,n=8") % (sequence, generation, requestId),
        "D0,1,2,3,4,5,6,7,8",
    ]
    crc = zlib.crc32("".join(item + "\n" for item in lines).encode("ascii")) & 0xFFFFFFFF
    if corruptInnerCrc:
        crc ^= 0x1
    lines.append("K,seq=%d,gen=%d,rid=%d,crc=%08X" %
                 (sequence, generation, requestId, crc))
    return "".join(item + "\n" for item in lines).encode("ascii")


def resistanceFrame(valueOhms=10000.0, requestId=5, generation=2):
    fixedValue = int(round(valueOhms * 1000.0))
    return MeasurementFrame(
        sequence=10,
        timestamp_us=0,
        rows=8,
        cells=64,
        mode="RES",
        unit="ohm",
        scale=-3,
        values_fixed=[fixedValue] * 64,
        error_reasons=[0] * 64,
        pga_gains=[1] * 64,
        valid_mask=(1 << 64) - 1,
        fresh_mask=(1 << 64) - 1,
        error_mask=0,
        reference="INTREF",
        rail_valid=True,
        generation=generation,
        request_id=requestId,
        rail_age_frames=0,
        frame_duration_us=1000,
        transition_duration_us=0,
        gain_change_count=0,
        overrange_count=0,
        autorange_attempt_count=0,
        autorange_fallback_count=0,
        io_retry_count=0,
        drdy_timeout_count=0,
        stale_count=0,
        spi_error_count=0,
        crc_ok=True,
    )


class UuidAndSelectionTest(unittest.TestCase):
    def test_normalizes_short_and_full_uuids(self):
        self.assertEqual(SERVICE_UUID, "000000ff" + BLE_BASE_SUFFIX)
        self.assertEqual(CTRL_RX_UUID, "0000ff10" + BLE_BASE_SUFFIX)
        self.assertEqual(normalizeUuid("0xFF11"), CTRL_TX_UUID)
        self.assertEqual(normalizeUuid("ff20"), DATA_TX_UUID)
        self.assertEqual(normalizeUuid("{0000FF30-0000-1000-8000-00805F9B34FB}"),
                         LOG_TX_UUID)
        with self.assertRaises(ValueError):
            normalizeUuid("not-a-uuid")

    def test_serial_selection_never_guesses_multiple_ports(self):
        candidates = [
            SerialCandidate("COM3", "first", "A"),
            SerialCandidate("COM12", "second", "B"),
        ]
        self.assertEqual(selectSerialPort("COM9", "COM8", candidates), "COM9")
        self.assertEqual(selectSerialPort(None, "COM8", candidates), "COM8")
        with self.assertRaises(HilSkipped):
            selectSerialPort(None, None, candidates)
        self.assertEqual(selectSerialPort(None, None, candidates[1:]), "COM12")
        with self.assertRaises(HilSkipped):
            selectSerialPort(None, None, [])

    def test_subscription_aliases_and_matrix(self):
        self.assertEqual(parseSubscriptionCase("FF11+20+log"), {"C", "D", "L"})
        self.assertEqual(parseSubscriptionCase("none"), set())
        self.assertEqual(formatSubscriptionCase({"L", "C"}), "C+L")
        cases = parseSubscriptionCases("none,11,20+30")
        self.assertEqual(cases, [set(), {"C"}, {"D", "L"}])

    def test_explicit_known_resistor_replaces_defaults(self):
        parser = buildArgumentParser()
        defaults = parser.parse_args(["serial", "--modes", "RES"])
        validateArguments(parser, defaults)
        self.assertEqual([item.label for item in defaults.knownResistor],
                         ["S1D1", "S8D8"])

        explicit = parser.parse_args([
            "serial", "--modes", "RES",
            "--known-resistor", "S2D3:4000:18000",
        ])
        validateArguments(parser, explicit)
        self.assertEqual([item.label for item in explicit.knownResistor],
                         ["S2D3"])


class FaultDetectionTest(unittest.TestCase):
    def test_ignores_startup_then_flags_requested_reset_and_panic_markers(self):
        detector = FaultDetector()
        detector.observe("ESP-ROM:esp32s3-20210327", "serial")
        self.assertEqual(detector.events(), [])
        self.assertEqual(len(detector.ignoredEvents()), 1)
        detector.arm()
        samples = (
            ("rst:0x3 (RTC_SW_SYS_RST)", "reset"),
            ("RST,reason=power_on", "reset"),
            ("Guru Meditation Error: Core 0 panic'ed", "panic"),
            ("Stack canary watchpoint triggered (sensorarrayLogT)", "panic"),
            ("LoadProhibited", "panic"),
            ("Task watchdog got triggered", "watchdog"),
            ("BLECORRUPT,ch=D,reason=crc", "firmware_fault"),
            ("LOGTRUNC,packet=summary", "firmware_fault"),
        )
        for line, expected in samples:
            event = detector.observe(line, "serial")
            self.assertIsNotNone(event)
            self.assertEqual(event.kind, expected)
        self.assertEqual(len(detector.events()), len(samples))

    def test_health_telemetry_does_not_false_trigger_watchdog(self):
        detector = FaultDetector()
        detector.arm()
        self.assertIsNone(detector.observe("STK100,watchdog=0,log=8192", "serial"))
        self.assertIsNone(detector.observe("BL50,conn=1,sub=000", "serial"))
        detector.assertHealthy()

    def test_pre_ready_panic_is_never_ignored(self):
        detector = FaultDetector()
        detector.observe("Guru Meditation Error: Core 0 panic'ed", "serial")
        with self.assertRaises(HilFailure):
            detector.assertHealthy()
        self.assertEqual(detector.ignoredEvents(), [])


class FrameContractTest(unittest.TestCase):
    def test_known_resistor_sanity_uses_physical_ohms(self):
        ranges = [KnownResistance(1, 1, 5000.0, 20000.0),
                  KnownResistance(8, 8, 5000.0, 20000.0)]
        evidence = checkKnownResistances([resistanceFrame()], ranges)
        self.assertEqual(evidence, {"S1D1": 10000.0, "S8D8": 10000.0})
        with self.assertRaises(HilFailure):
            checkKnownResistances([resistanceFrame(25000.0)], ranges)

    def test_mode_frame_validates_rid_generation_and_crc(self):
        frame = resistanceFrame()
        validateModeFrame(frame, "RES", requestId=5, generation=2,
                          appliedSequence=10)
        for changes in ({"request_id": 6}, {"generation": 3}, {"crc_ok": False}):
            with self.assertRaises(HilFailure):
                validateModeFrame(dataclasses.replace(frame, **changes), "RES", 5, 2, 10)
        with self.assertRaisesRegex(HilFailure, "stale/mixed"):
            validateModeFrame(frame, "VOLT", 5, 2, 10)

    def test_cap_mode_uses_mapp_not_scan_config_rid(self):
        observer = BleObserver(HilOutput(None), FaultDetector())
        for fragment in fragmentMessage("D", 8, capPayload(), 3):
            observer.feedNotification("D", fragment)
        frame = observer.frames[0].frame
        validateModeFrame(frame, "CAP", requestId=123, generation=456,
                          appliedSequence=frame.sequence)

    def test_ble_discards_any_frame_before_authoritative_apply_boundary(self):
        resFrame = resistanceFrame()
        oldResFrame = dataclasses.replace(resFrame, sequence=9)
        self.assertTrue(isPreApplyFrame(oldResFrame, 10))
        self.assertFalse(isPreApplyFrame(resFrame, 10))

    def test_runtime_readiness_requires_fully_fresh_cap(self):
        observer = BleObserver(HilOutput(None), FaultDetector())
        for fragment in fragmentMessage("D", 8, capPayload(), 3):
            observer.feedNotification("D", fragment)
        frame = observer.frames[0].frame
        self.assertTrue(isRuntimeReadyFrame(frame))
        self.assertFalse(isRuntimeReadyFrame(
            dataclasses.replace(frame, secondary_fresh_mask=0)))

    def test_command_reply_contracts(self):
        self.assertEqual(expectedCommandPrefix("STATE?"), "MODE,")
        self.assertEqual(expectedCommandPrefix("MODE?"), "MODE,")
        self.assertEqual(expectedCommandPrefix("BTX=SAFE"), "ACK,cmd=BTX")
        self.assertEqual(expectedCommandPrefix("WIFI?"), "ACK,cmd=WIFI")
        parsed = parseKnownResistance("s8d8:5000:20000")
        self.assertEqual(parsed.index, 63)

    def test_stack_pool_and_heap_telemetry_contract(self):
        def record(sequence, logFree, heapFree, fragmentErrors=0):
            return LineRecord(
                float(sequence), "L",
                ("STK50,unit=bytes,seq=%d,log=16384/%d,transport=6144/2048,"
                 "usb=4096/1536,bleTx=6144/1800,bleCtrl=6144/1700,"
                 "serialCtrl=6144/1600,heap=300000/%d/260000,tSlot=0/3,"
                 "ta=0,ts=0,tr=0,tq=0/0,bSlot=0/3,ba=0,bs=0,br=0,"
                 "bc=0,bf=%d,trunc=0") %
                (sequence, logFree, heapFree, fragmentErrors))

        records = [record(50, 4096, 280000), record(100, 3500, 279500)]
        result = validateStackTelemetry(records, 2048, 512, 4096, required=True)
        self.assertEqual(result["tasks"]["log"]["minimumRemainingBytes"], 3500)
        self.assertEqual(result["heap"]["warmupToFinalDeltaBytes"], -500)
        self.assertEqual(result["transportSlots"]["highWater"], 3)

        with self.assertRaisesRegex(HilFailure, "log stack minimum"):
            validateStackTelemetry([record(100, 1024, 280000)],
                                   2048, 512, 4096, required=True)
        with self.assertRaisesRegex(HilFailure, "counter bf increased"):
            validateStackTelemetry([record(50, 4096, 280000, 0),
                                    record(100, 4096, 280000, 1)],
                                   2048, 512, 4096, required=True)
        self.assertEqual(validateStackTelemetry([], 2048, 512, 4096,
                                                required=False)["samples"], 0)

    def test_ble_firmware_drop_counters_are_delta_checked(self):
        first = LineRecord(1.0, "S",
                           "BL50,conn=1,sub=010,mtu=247,mode=FAST,mq=10,ms=10,md=2,dropD=2,dropL=0,dropC=0,ctrlExhaust=0,fs=50,fe=0,cg=0,tiny=0")
        same = LineRecord(2.0, "S",
                          "BL50,conn=1,sub=010,mtu=247,mode=FAST,mq=20,ms=20,md=2,dropD=2,dropL=0,dropC=0,ctrlExhaust=0,fs=100,fe=0,cg=0,tiny=0")
        result = validateBleFirmwareCounters([first, same], required=True)
        self.assertEqual(result["counterDelta"],
                         {"md": 0, "fe": 0, "tiny": 0, "cg": 0,
                          "dropD": 0, "dropL": 0, "dropC": 0,
                          "ctrlExhaust": 0})
        self.assertEqual(result["channelDropDelta"],
                         {"DATA": 0, "LOG": 0, "CTRL": 0})
        dropped = LineRecord(3.0, "S",
                             "BL50,conn=1,sub=010,mtu=247,mode=FAST,mq=30,ms=29,md=3,dropD=3,dropL=0,dropC=0,ctrlExhaust=0,fs=150,fe=0,cg=0,tiny=0")
        with self.assertRaisesRegex(HilFailure, "counter md increased"):
            validateBleFirmwareCounters([first, dropped], required=True)
        tolerated = validateBleFirmwareCounters(
            [first, dropped], required=True, allowedMessageDrops=1)
        self.assertEqual(tolerated["counterDelta"]["md"], 1)

        controlDropped = LineRecord(
            4.0, "S",
            "BL50,conn=1,sub=111,mtu=247,mode=FAST,mq=30,ms=29,md=3,dropD=2,dropL=0,dropC=1,ctrlExhaust=1,fs=150,fe=0,cg=0,tiny=0")
        with self.assertRaisesRegex(HilFailure, "counter dropC increased"):
            validateBleFirmwareCounters(
                [first, controlDropped], required=True, allowedMessageDrops=1)


class BleWireValidationTest(unittest.TestCase):
    def setUp(self):
        self.output = HilOutput(None)
        self.detector = FaultDetector()
        self.detector.arm()

    def tearDown(self):
        self.output.close()

    def test_reassembles_fragmented_log_and_validates_crc(self):
        observer = BleObserver(self.output, self.detector)
        payload = b"SF50,seq=50,drop=0\n"
        for fragment in fragmentMessage("L", 77, payload, 3):
            observer.feedNotification("L", fragment)
        self.assertEqual(observer.completeMessages["L"], 1)
        self.assertEqual(observer.lines["L"][0].line, "SF50,seq=50,drop=0")
        stats = validateBleWindow(observer)
        self.assertEqual(stats["fragment"]["L"]["ok"], 1)
        self.assertEqual(stats["fragment"]["L"]["crc_fail"], 0)

    def test_raw_messages_between_envelopes_are_not_false_id_gaps(self):
        observer = BleObserver(self.output, self.detector)
        for fragment in fragmentMessage("L", 2, b"SF50,seq=1\n", 2):
            observer.feedNotification("L", fragment)
        observer.feedNotification("L", b"BL50,conn=1\n")
        for fragment in fragmentMessage("L", 4, b"ADS50,seq=2\n", 2):
            observer.feedNotification("L", fragment)
        stats = validateBleWindow(observer, strictSequence=False)
        self.assertEqual(stats["fragment"]["L"]["missing"], 0)
        self.assertEqual(stats["fragment"]["L"]["gap"], 0)

    def test_nonblocking_window_allows_positive_gaps_but_never_regressions(self):
        observer = BleObserver(self.output, self.detector)
        for sequence in (7, 9):
            for fragment in fragmentMessage("D", sequence,
                                            capPayload(sequence=sequence), 3):
                observer.feedNotification("D", fragment)
        stats = validateBleWindow(observer, strictSequence=False)
        self.assertEqual(stats["protocol"]["missing_frames"], 1)
        self.assertEqual(stats["protocol"]["sequence_regressions"], 0)

        for fragment in fragmentMessage("D", 10, capPayload(sequence=8), 3):
            observer.feedNotification("D", fragment)
        with self.assertRaisesRegex(HilFailure, "sequence regressions"):
            validateBleWindow(observer, strictSequence=False)

    def test_rejects_outer_crc_mismatch(self):
        observer = BleObserver(self.output, self.detector)
        payload = b"ADS50,seq=50\n"
        for fragment in fragmentMessage("L", 5, payload, 2, crcOverride=0x12345678):
            observer.feedNotification("L", fragment)
        self.assertEqual(observer.completeMessages["L"], 0)
        with self.assertRaisesRegex(HilFailure, "crc_fail"):
            validateBleWindow(observer)

    def test_fragmented_data_checks_internal_frame_crc(self):
        observer = BleObserver(self.output, self.detector)
        payload = capPayload()
        for fragment in fragmentMessage("D", 8, payload, 4):
            observer.feedNotification("D", fragment)
        self.assertEqual(len(observer.frames), 1)
        frame = observer.frames[0].frame
        validateModeFrame(frame, "CAP", 9, 3, 7)
        validateBleWindow(observer)

        corrupt = BleObserver(self.output, self.detector)
        payload = capPayload(corruptInnerCrc=True)
        for fragment in fragmentMessage("D", 9, payload, 4):
            corrupt.feedNotification("D", fragment)
        with self.assertRaisesRegex(HilFailure, "internal frame crc"):
            validateBleWindow(corrupt)

    def test_rejects_cross_characteristic_envelope(self):
        observer = BleObserver(self.output, self.detector)
        for fragment in fragmentMessage("D", 4, capPayload(), 3):
            observer.feedNotification("L", fragment)
        with self.assertRaisesRegex(HilFailure, "characteristic carried"):
            validateBleWindow(observer)

    def test_stale_incomplete_fragment_is_failure(self):
        observer = BleObserver(self.output, self.detector)
        observer.feedNotification("L", fragmentMessage("L", 1, b"TR50,x=1\n", 2)[0])
        for item in observer.reassembler._messages.values():
            item["created"] -= 10.0
        observer.expireStaleFragments(5.0)
        with self.assertRaisesRegex(HilFailure, "stale fragment"):
            validateBleWindow(observer)
        stats = validateBleWindow(observer, maximumStaleAssemblies=1)
        self.assertEqual(stats["staleFragments"], 1)

        with self.assertRaisesRegex(HilFailure, "must be non-negative"):
            validateBleWindow(observer, maximumStaleAssemblies=-1)


class FakeCommandClient:
    def __init__(self, observer):
        self.observer = observer
        self.commands = []

    async def write_gatt_char(self, characteristicUuid, payload, response):
        self.assertedUuid = characteristicUuid
        self.assertedResponse = response
        command = bytes(payload).decode("ascii").strip()
        self.commands.append(command)
        if command == "BTX?":
            self.observer.feedNotification("C", b"ACK,cmd=BTX,v=FAST\n")
        elif command == "MODE=CAP":
            self.observer.feedNotification(
                "C", b"MACK,id=9,old=RES,new=CAP,state=accepted\n")
            self.observer.feedNotification(
                "L", b"MAPP,id=9,gen=3,old=RES,new=CAP,seq=7,state=applied\n")
            for fragment in fragmentMessage("D", 22, capPayload(), 3):
                self.observer.feedNotification("D", fragment)


class FakeNotifyClient:
    def __init__(self):
        self.started = []
        self.stopped = []

    async def start_notify(self, characteristicUuid, callback, **options):
        self.started.append((characteristicUuid, callback, options))

    async def stop_notify(self, characteristicUuid):
        self.stopped.append(characteristicUuid)


class BleCommandCorrelationTest(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.output = HilOutput(None)
        self.detector = FaultDetector()
        self.detector.arm()
        self.observer = BleObserver(self.output, self.detector)
        self.client = FakeCommandClient(self.observer)

    async def asyncTearDown(self):
        self.output.close()

    async def test_wait_command_and_mode_boundary_correlation(self):
        line = await waitBleCommand(self.client, self.observer, "BTX?", 0.2)
        self.assertEqual(line, "ACK,cmd=BTX,v=FAST")
        self.assertFalse(self.client.assertedResponse)
        frames, evidence = await switchBleMode(
            self.client, self.observer, "CAP", 1, 0.2, logSubscribed=True)
        self.assertEqual(len(frames), 1)
        self.assertEqual(evidence,
                         {"requestId": 9, "generation": 3, "appliedSequence": 7})
        self.assertEqual(self.client.commands, ["BTX?", "MODE=CAP"])

    async def test_low_load_write_with_response_probe_is_selectable(self):
        line = await waitBleCommand(
            self.client, self.observer, "BTX?", 0.2,
            writeWithResponse=True)
        self.assertEqual(line, "ACK,cmd=BTX,v=FAST")
        self.assertTrue(self.client.assertedResponse)

    async def test_safe_subscriptions_force_indicate_cccd(self):
        client = FakeNotifyClient()
        await startSubscriptions(client, self.observer, {"C", "D", "L"},
                                 forceIndicate=True)
        self.assertEqual([item[0] for item in client.started],
                         [CTRL_TX_UUID, DATA_TX_UUID, LOG_TX_UUID])
        self.assertTrue(all(item[2] == {"force_indicate": True}
                            for item in client.started))
        await stopSubscriptions(client, {"C", "D", "L"})
        self.assertEqual(client.stopped,
                         [LOG_TX_UUID, DATA_TX_UUID, CTRL_TX_UUID])

    async def test_full_log_case_requires_periodic_diagnostic_tags(self):
        class ConnectedClient:
            is_connected = True

        required = requiredLogTagsForSubscription("full", {"L"})
        self.assertEqual(required,
                         {"SF50", "TR50", "ADS50", "ADST50", "AB50"})
        self.assertEqual(requiredLogTagsForSubscription("smoke", {"L"}), set())
        self.assertEqual(requiredLogTagsForSubscription("full", {"C", "L"}), set())

        for line in (b"SF50,seq=50\n", b"TR50,r=8\n", b"ADS50,mode=RES\n",
                     b"ADST50,mode=RES\n", b"AB50,bt=4100\n"):
            self.observer.feedNotification("L", line)
        await waitForBleTraffic(
            ConnectedClient(), self.observer, 0, 3, 0.0, 0.1,
            self.detector, requiredLogTags=required)

        self.observer.resetWindow()
        self.observer.feedNotification("L", b"SF50,seq=50\n")
        self.observer.feedNotification("L", b"TR50,r=8\n")
        with self.assertRaisesRegex(HilFailure, "missingTags=.*ADS50"):
            await waitForBleTraffic(
                ConnectedClient(), self.observer, 0, 2, 0.0, 0.01,
                self.detector, requiredLogTags=required)

    async def test_unsubscribe_quiet_check_tolerates_grace_only(self):
        class ConnectedClient:
            is_connected = True

        async def deliverAfter(delay):
            await asyncio.sleep(delay)
            self.observer.feedNotification("L", b"BL50,conn=1\n")

        graceDelivery = asyncio.create_task(deliverAfter(0.005))
        result = await assertBleChannelQuietAfterUnsubscribe(
            ConnectedClient(), self.observer, "L", self.detector,
            settleSeconds=0.02, quietSeconds=0.02)
        await graceDelivery
        self.assertEqual(result,
                         {"graceNotifications": 1, "trailingNotifications": 0})

        self.observer.resetWindow()
        trailingDelivery = asyncio.create_task(deliverAfter(0.025))
        with self.assertRaisesRegex(HilFailure,
                                    "notifications after unsubscribe settle"):
            await assertBleChannelQuietAfterUnsubscribe(
                ConnectedClient(), self.observer, "L", self.detector,
                settleSeconds=0.01, quietSeconds=0.05)
        await trailingDelivery


class FakeCharacteristic:
    def __init__(self, characteristicUuid, properties):
        self.uuid = characteristicUuid
        self.properties = properties


class FakeService:
    def __init__(self, serviceUuid, characteristics):
        self.uuid = serviceUuid
        self.characteristics = characteristics


class FakeClient:
    def __init__(self, services):
        self.services = services


class GattAndProfileTest(unittest.TestCase):
    def test_gatt_contract_requires_read_notify_and_indicate(self):
        characteristics = [
            FakeCharacteristic(CTRL_RX_UUID, ["write", "write-without-response"]),
            FakeCharacteristic(CTRL_TX_UUID, ["read", "notify", "indicate"]),
            FakeCharacteristic(DATA_TX_UUID, ["read", "notify", "indicate"]),
            FakeCharacteristic(LOG_TX_UUID, ["read", "notify", "indicate"]),
        ]
        client = FakeClient([FakeService(SERVICE_UUID, characteristics)])
        result = validateGatt(client)
        self.assertIn("indicate", result[DATA_TX_UUID])

        characteristics[-1] = FakeCharacteristic(LOG_TX_UUID, ["read", "notify"])
        with self.assertRaisesRegex(HilFailure, "missing properties"):
            validateGatt(FakeClient([FakeService(SERVICE_UUID, characteristics)]))

    def test_ff10_contract_requires_both_write_properties(self):
        transmitCharacteristics = [
            FakeCharacteristic(CTRL_TX_UUID, ["read", "notify", "indicate"]),
            FakeCharacteristic(DATA_TX_UUID, ["read", "notify", "indicate"]),
            FakeCharacteristic(LOG_TX_UUID, ["read", "notify", "indicate"]),
        ]
        for properties, missingProperty in (
                (["write"], "write_without_response"),
                (["write-without-response"], "write")):
            with self.subTest(properties=properties):
                characteristics = [
                    FakeCharacteristic(CTRL_RX_UUID, properties),
                    *transmitCharacteristics,
                ]
                with self.assertRaisesRegex(HilFailure, missingProperty):
                    validateGatt(FakeClient([
                        FakeService(SERVICE_UUID, characteristics)]))

    def test_fast_high_load_incomplete_budget_is_strict_point_two_percent(self):
        expected = {
            0: 0,
            1: 0,
            499: 0,
            500: 1,
            501: 1,
            999: 1,
            1000: 2,
            2000: 4,
        }
        for completedFrames, budget in expected.items():
            with self.subTest(completedFrames=completedFrames):
                self.assertEqual(
                    fastHighLoadIncompleteAssemblyBudget(completedFrames), budget)
        with self.assertRaisesRegex(HilFailure, "must be non-negative"):
            fastHighLoadIncompleteAssemblyBudget(-1)

    def test_full_profile_encodes_acceptance_counts(self):
        parser = buildArgumentParser()
        args = parser.parse_args(["ble", "--profile", "full"])
        profile = buildBleProfile(args)
        self.assertEqual(profile.ff20OnlyFrames, 1000)
        self.assertEqual(profile.modeCycles, 20)
        self.assertEqual(profile.subscribeCycles, 100)
        self.assertEqual(profile.reconnectCycles, 30)
        self.assertEqual(profile.longRunFrames, 2000)


if __name__ == "__main__":
    unittest.main()
