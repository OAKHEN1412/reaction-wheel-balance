// Mega 2560 as a USB-serial bridge for the Nano, replacing the Nano's own
// CH340 whose inbound side stopped working on 2026-09-23 (board TX fine,
// RX dead, DTR reset ineffective; 5 PC ports, 2 cables, driver unplugged).
//
// Wiring (Nano USB UNPLUGGED, 12V OFF):
//   Mega TX1 (D18) -> Nano RX0 (D0)
//   Mega RX1 (D19) -> Nano TX1 (D1)
//   Mega GND       -> Nano GND
//   Mega 5V        -> Nano 5V      (powers the Nano)
//   Mega D2        -> Nano RST     (optional: lets the 'reset' trick below work)
//
// Then run tuning/tools/bridge.ps1 against the Mega's COM port exactly as
// before -- every byte is forwarded both ways at 115200.
//
// Reset: the Nano's bootloader needs a reset pulse right before an upload,
// and no DTR reaches it through this bridge. Two options:
//   (a) press the Nano's RESET button ~1 s after starting the upload, or
//   (b) send the line "~RESET~" through this bridge: the Mega pulses D2 low
//       for 100 ms. bridge.ps1 can send it from the command file. Only that
//       exact line is intercepted; everything else passes through untouched.
// For flashing without any timing at all, use firmware/README.md's
// "Mega as ISP" procedure instead of this sketch.

const uint8_t PIN_TARGET_RESET = 2;

void setup() {
  pinMode(PIN_TARGET_RESET, INPUT);   // high-impedance: never hold the Nano in reset
  Serial.begin(115200);
  Serial1.begin(115200);
}

// Watch the PC->Nano stream for the reset token without adding latency to
// ordinary traffic: bytes are forwarded immediately and the matcher only
// tracks how much of the token has been seen so far.
static const char kToken[] = "~RESET~";
static uint8_t matched = 0;

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == kToken[matched]) {
      if (++matched == sizeof(kToken) - 1) {
        matched = 0;
        pinMode(PIN_TARGET_RESET, OUTPUT);
        digitalWrite(PIN_TARGET_RESET, LOW);
        delay(100);
        pinMode(PIN_TARGET_RESET, INPUT);
        Serial.println(F("[mega] pulsed target RESET"));
        continue;   // the token itself is not forwarded
      }
    } else {
      // Flush any partial token that turned out to be ordinary text.
      for (uint8_t k = 0; k < matched; k++) Serial1.write(kToken[k]);
      matched = (c == kToken[0]) ? 1 : 0;
      if (matched) continue;
    }
    if (!matched) Serial1.write(c);
  }
  while (Serial1.available()) Serial.write(Serial1.read());
}
