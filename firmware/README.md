# Firmware -- Reaction Wheel Balance

เฟิร์มแวร์สำหรับ **Arduino Mega 2560** คุมมอเตอร์ BLDC หมุน Reaction Wheel เพื่อทรงตัวแกนเดียวด้วย
feedback จาก MPU-6050 (เปลี่ยนจาก ESP32-C3 มาเป็น Mega เมื่อ 2026-09-19 แบบถาวร — ดูเหตุผลใน
`../hardware/wiring.md`) โครงสร้างไฟล์:

```
firmware/
  reaction_wheel_balance/   เฟิร์มแวร์หลัก (state machine + control loop)
  motor_test/                เครื่องมือหา polarity ของมอเตอร์/ไดรเวอร์
  imu_test/                  เครื่องมือหาแกน/เครื่องหมาย IMU ตามการติดตั้งจริง
  legacy_esp32/               เวอร์ชัน ESP32-C3 เดิม (เก็บไว้อ้างอิงเท่านั้น ไม่ใช้งานต่อ)
```

Toolchain: `arduino-cli`, board `arduino:avr:mega` (avr core, ติดตั้งผ่าน
`arduino-cli core install arduino:avr` ถ้ายังไม่มี) ไม่ต้องตั้งออปชันพิเศษเหมือน ESP32:

```
arduino-cli compile --fqbn arduino:avr:mega firmware/reaction_wheel_balance
arduino-cli compile --fqbn arduino:avr:mega firmware/motor_test
arduino-cli compile --fqbn arduino:avr:mega firmware/imu_test

arduino-cli upload -p COM3 --fqbn arduino:avr:mega firmware/reaction_wheel_balance
arduino-cli monitor -p COM3 -c baudrate=115200
```

**ก่อนอัปโหลดทุกครั้ง: ปิดสวิตช์ไฟ 12V ของวงจรมอเตอร์ก่อน** เพราะขา PWM (D11) ของไดร์เวอร์นี้
กลับขั้ว (HIGH = หยุด) ถ้าขาลอยตอน Mega รีเซ็ต/กำลังอัปโหลด มอเตอร์อาจวิ่งได้ชั่วขณะ (ดู
`../hardware/wiring.md` §5 risk a)

## ขั้นตอน bring-up (ต้องทำตามลำดับ)

### 1. `imu_test` -- หาแกน/เครื่องหมายของ IMU

MPU-6050 ติดตั้งในทิศทางที่ยังไม่รู้ล่วงหน้า ต้องหาว่าคู่แกน accel ไหน (ผ่าน
`atan2`) ให้มุมเอียงที่ถูกต้องสำหรับแกนที่ต้องการทรงตัว:

1. Flash `firmware/imu_test` แล้วเปิด serial monitor (115200)
2. ตั้งโครงให้ตั้งตรง ดูว่าคอลัมน์มุมไหนอ่านได้ใกล้ 0 องศา
3. ค่อย ๆ เอียงโครงไปทางที่ต้องการเรียกว่า "บวก" ดูว่าคอลัมน์ไหนขึ้น/ลงตามที่ต้องการ
   แบบราบรื่นไม่กระโดด (ระวัง gimbal ที่แกนอื่นเกิน ±90°)
4. จดคู่แกน (NUM, DEN) และเครื่องหมายที่ได้ แมปตามหัวคอลัมน์:

   | คอลัมน์ | IMU_ACCEL_AXIS_NUM | IMU_ACCEL_AXIS_DEN |
   |---|---|---|
   | atan2(ay,az) | 1 (Y) | 2 (Z) |
   | atan2(az,ay) | 2 (Z) | 1 (Y) |
   | atan2(ax,az) | 0 (X) | 2 (Z) |
   | atan2(az,ax) | 2 (Z) | 0 (X) |
   | atan2(ax,ay) | 0 (X) | 1 (Y) |
   | atan2(ay,ax) | 1 (Y) | 0 (X) |

   ถ้ามุมมีเครื่องหมายกลับด้าน ให้ตั้ง `IMU_ACCEL_ANGLE_SIGN = -1.0f` แทนที่จะไปสลับคู่แกน
5. ดูค่า gyro (dps) 3 แกนตอนเอียง เลือกแกนที่ตอบสนองต่อการเอียงรอบแกนเดียวกัน
   ใส่เป็น `IMU_GYRO_AXIS` และปรับ `IMU_GYRO_SIGN` ให้เครื่องหมายตรงกับมุมจาก accel
   (theta_dot ควรมีเครื่องหมายเดียวกับอัตราการเปลี่ยนของ theta)
6. กรอกค่าทั้งหมดลงใน `firmware/reaction_wheel_balance/config.h`

### 2. `motor_test` -- หา polarity ของมอเตอร์/ไดรเวอร์ (ยึดล้อให้แน่นก่อน!)

**ปิดไฟ 12V ก่อนแฟลชเสมอ** (PWM กลับขั้ว เสี่ยงมอเตอร์วิ่งตอน reset — ดู `../hardware/wiring.md` risk a)

1. **ยึด/ประกบล้อให้แน่น** หรือถอดล้อออกก่อน เพราะยังไม่รู้ทิศทาง/เบรกจะทำงานอย่างไร
2. Flash `firmware/motor_test` เปิด serial monitor (115200)
3. หา `PWM_INVERT`: สั่ง `duty 0` แล้วดูว่ามอเตอร์หมุนเร็วสุดหรือหยุด, แล้วลอง
   `duty 1023` เทียบกัน -- ถ้า `duty 0` หมุนเร็วสุด แปลว่า `PWM_INVERT = true`
   (ยืนยันแล้วกับมอเตอร์ตัวจริง: `PWM_INVERT = true`)
4. หา `DIR_CW_LEVEL`: ตั้ง duty ปานกลาง (เช่น `duty 400`) แล้วสลับ `dir 0` / `dir 1`
   สังเกตทิศทางการหมุนจริงของ "ล้อ" (ไม่ใช่มอเตอร์เปล่า ถ้ามีเกียร์/สายพาน) --
   เลือกระดับไหนก็ได้เป็น "positive" แล้วตั้ง `DIR_CW_LEVEL` เป็นระดับนั้นในไฟล์ config
5. หา `BRAKE_ACTIVE_LEVEL`: ปั่นล้อด้วย duty ปานกลาง แล้วลอง `brake 0` / `brake 1`
   ดูว่าระดับไหนที่ทำให้เพลาหยุด/ฝืดจริง (บางบอร์ดเป็น active-low)
6. หา `FG_PULSES_PER_REV`: หมุนเพลามอเตอร์ด้วยมือ **ครบ 1 รอบพอดี ช้า ๆ** แล้วสั่ง
   `count` -- ค่าที่ได้คือ pulses/รอบ (วัดจริงแล้ว: ~70 พัลส์ต่อรอบ**ล้อ**)
7. ถ้ามีเกียร์/สายพาน ให้หมุน "ล้อ" ครบ 1 รอบแล้วนับพัลส์ที่ "เพลามอเตอร์" หมุนไปกี่รอบ
   (สังเกตด้วยตา หรือทำเครื่องหมายบนเพลามอเตอร์) เพื่อคำนวณ `GEAR_RATIO`
8. หา `MOTOR_MIN_DUTY_FRACTION`: ค่อย ๆ เพิ่ม `duty` จาก 0 ทีละน้อย สังเกตว่า
   duty เท่าไหร่ที่มอเตอร์เริ่มหมุนจริง (FG เริ่มมีพัลส์) -- ค่า duty/1023 ตรงนั้นคือ
   `MOTOR_MIN_DUTY_FRACTION` (วัดจริงแล้ว: เริ่มหมุนที่ duty ~10%)
9. กรอกค่าทั้งหมดลงใน `config.h`

`motor_test` เวอร์ชัน Mega ปัจจุบันรองรับคำสั่ง: `duty <0-1023>` (0 = หยุด), `dir <0|1>`,
`brake <0|1>`, `stop`, `count`, `resetcount`, `status`, `help` (ไม่มีคำสั่ง `coastdown` /
`freq` / `probe` / `od` เหมือนเวอร์ชัน ESP32 เดิมใน `legacy_esp32/` แล้ว เพราะไม่จำเป็นอีก
ต่อไปเมื่อขับสายมอเตอร์ตรงจาก Mega ไม่ต้องผ่านทรานซิสเตอร์)

### 3. Flash เฟิร์มแวร์หลัก + ตั้งศูนย์ + จูนเกน

1. Flash `firmware/reaction_wheel_balance`
2. เปิด serial monitor รอจน state เป็น `WAIT_UPRIGHT`
3. ประคองโครงให้ตั้งตรงจริง ๆ (ด้วยตา/ฉาก) แล้วพิมพ์ `zero` เพื่อบันทึกมุมปัจจุบัน
   เป็นจุดศูนย์ (upright offset) จากนั้น `save` เพื่อเก็บลง EEPROM
4. พิมพ์ `selftest` เพื่อตรวจ control law + wheel-speed estimator เบื้องต้น
   (ควรได้ `selftest: balance_controller PASS` และ `selftest: wheel_speed_estimator PASS`)
5. **ขั้นตอนจูนเกน** — ⚠️ ห้ามใช้วิธีจูน PID ทั่วไปแบบ "เริ่ม Kp=0 แล้วค่อยเพิ่ม"
   เพราะ reaction wheel มี **Kp ขั้นต่ำ** ≈ (m·g·l)/I_w — ถ้า Kp ต่ำกว่านี้ ล้อสร้างแรงบิดชนะ
   แรงโน้มถ่วงไม่ได้เลย จะล้มทุกครั้งไม่ว่า Kd/Kw เป็นเท่าไร
   1. **วัดของจริง** (ดู `sim/README.md` หัวข้อ 6.1): มวลเฟรม+ล้อ, ความสูง COM (หาจุดสมดุล),
      ความเฉื่อยล้อ (Fusion → Properties หรือแขวนแบบ bifilar), อัตราทดเกียร์, tau_m จาก motor_test
   2. **คำนวณเกนจาก sim**:
      `py -X utf8 sim/design_gains.py --set m_b=... l_b=... m_w=... l_w=... gear_ratio=... tau_m=... --quick`
      แล้ววางบรรทัด `#define` ที่พิมพ์ออกมาลง `config.h` (หรือตั้งผ่าน serial `kp/kd/kw/ki`)
      — เกนทั้งหมด Kp, Kd, Kw, Ki **เครื่องหมายเดียวกัน** (บวก) ตาม LQR
   3. **เช็คทิศทางแรงบิดก่อนวางบนพื้น**: ถือโครงแน่น ๆ ด้วยมือ ตั้งตรงจน state = BALANCING
      แล้วเอียงโครงเล็กน้อย — ต้องรู้สึกว่าล้อ "ดัน" โครงกลับเข้าหาแนวตั้ง ถ้าดันไปทางเดียวกับที่เอียง
      (ช่วยให้ล้มเร็วขึ้น) ให้สั่ง `sign -1`
   4. วางบนพื้นราบ เปิด `tel 1` เก็บ CSV แล้วปรับละเอียด:
      - แกว่งเร็ว/สั่น → ลด `kd` ลงเล็กน้อย (noise จากไจโร) หรือเช็คว่า loop overrun (`get`)
      - แกว่งช้า ๆ ขยายขึ้นจนล้ม → เพิ่ม `kd` 10–20%
      - ยืนได้แต่ `wheel_rpm` ไหลขึ้นเรื่อย ๆ → ตั้ง `zero` ใหม่ให้แม่นขึ้นก่อน แล้วค่อยเพิ่ม `kw`
      - ล้อชน max rpm ทุกครั้งแม้มุมเล็ก → ฮาร์ดแวร์แรงบิดไม่พอ (ดูตาราง `sim/out/hardware_requirements.csv`: ต้องการ gear ≥ 3, tau_m ≤ 0.08 s)
   5. จูนเสร็จพิมพ์ `save` (เก็บลง EEPROM) — **หมายเหตุ**: ค่าใน EEPROM จะทับค่า default ใน config.h
      ทุกครั้งที่บูต ถ้าแก้ config.h แล้วค่าไม่เปลี่ยน ให้ตั้งผ่าน serial แล้ว `save` ใหม่

## คำสั่ง Serial (115200 baud)

| คำสั่ง | ความหมาย |
|---|---|
| `kp [v]` | อ่าน/ตั้งค่า Kp |
| `kd [v]` | อ่าน/ตั้งค่า Kd |
| `kw [v]` | อ่าน/ตั้งค่า Kw |
| `ki [v]` | อ่าน/ตั้งค่า Ki (0 = ปิด integral term) |
| `sign [v]` | อ่าน/ตั้งค่า control sign โดยรวม (+1/-1) |
| `zero` | ตั้งมุมปัจจุบันเป็นจุดศูนย์ (upright offset) |
| `save` | บันทึกเกน + offset ลง EEPROM |
| `start` | เปิดการทรงตัว (ต้อง hold ตั้งตรงใหม่ตาม state machine) |
| `stop` | ปิดการทรงตัว, เบรก+ปล่อยมอเตอร์ทันที |
| `jump` | สั่งโหมดทดลอง JUMP_UP (ต้องเปิด `ENABLE_JUMP_UP` ใน config.h ก่อน) |
| `tel 0|1` | เปิด/ปิด telemetry CSV (ไม่ใส่ argument = สลับค่า) |
| `selftest` | รันชุดทดสอบ control law + wheel-speed estimator แบบ pure-logic |
| `get` | พิมพ์เกน/สถานะปัจจุบัน + จำนวน loop overrun |
| `help` | รายการคำสั่ง |

## Telemetry

เปิดด้วย `tel 1` จะพิมพ์บรรทัด CSV ที่ ~50 Hz:

```
t_ms,state,theta_deg,theta_dot_dps,wheel_rpm,cmd_rpm,duty
```

เก็บลงไฟล์เพื่อทำกราฟการทดลองได้ เช่น:

```
arduino-cli monitor -p <PORT> -c baudrate=115200 | tee telemetry.csv
```

(ตัดบรรทัดข้อความอื่นที่ไม่ใช่ CSV ออกก่อนโหลดเข้า pandas/Excel)

## Control law and sign convention (updated 2026-09-19)

Default `CONTROL_MODE_VOLTAGE true` uses the measured voltage-mode motor model:

```
a = controlSign * (Kp*theta + Kd*theta_dot + Kw*omega_w + Ki*integral(theta))
u = clamp((omega_signed_est + MOTOR_TAU_S*a) / MOTOR_FULL_SCALE_WHEEL_RADPS, -1, 1)
```

`omega_w` comes from the signed WheelSpeedEstimator, not FG magnitude times DIR.
The estimator now predicts from previous **applied** signed duty, including
reversal blanks and brake/coast, rather than requested controller output.
FG direction remains model-derived; FG only emits while driving, and this
estimator does not model current limiting or asymmetric coast/brake torque.
Its sign near physical reversal must still be checked on hardware.

`MOTOR_TAU_S=0.18f` applies to the light wheel (no rim nuts). The weighted wheel
measured 0.45?0.59 s and needs a different estimate and gain verification.
Voltage mode bypasses the legacy minimum-duty affine boost, retaining the
small motor deadband. Startup below ~10% duty remains a hardware uncertainty.
`MAX_WHEEL_SPEED_RADPS` inhibits further outward acceleration when measured
speed exceeds the guard, while allowing reverse torque. It does not clamp
voltage to 85%; full duty remains available. Duty saturation freezes integration
when the angle error would push further into saturation.

Set `CONTROL_MODE_VOLTAGE false` for the old integrated-speed law and its
minimum-duty mapping. Gains, offset and sign still load from EEPROM; existing
saved gains override config defaults (use `get`, then set gains and `save`).

DIR changes only after a complete zero-duty control step (2 ms at 500 Hz).
The following step writes DIR before applying the opposite duty, even while
the wheel still spins. Waiting for zero RPM would withhold reverse torque too
long. Actual reverse-voltage braking ability is unmeasured; the sim assumes it
works. `ACTIVE_DECEL_BRAKE` remains optional (default false): same-direction
voltage reduction, including zero target, beyond `DECEL_BRAKE_MARGIN_RADPS`
pulses BRAKE for that step. Brake torque has not been characterized.

CSV columns remain unchanged. In voltage mode `cmd_rpm` means `u*omega_fs`
converted to rpm, **not a desired actual wheel rpm**; `duty` is applied unsigned
duty after the interlock. Thus wheel rpm need not track cmd_rpm during torque
transients. Hardware pins, PWM polarity, DIR/BRAKE levels, IMU axes, control sign,
and FG calibration are unchanged.

## State machine

```
CALIBRATING -> WAIT_UPRIGHT -> BALANCING <-> FALLEN
                    ^--------------------------|
```

- **CALIBRATING**: ตอนบูต, เก็บ gyro bias ~2 วินาที (ต้องถือนิ่ง) มอเตอร์ยังปิดอยู่
- **WAIT_UPRIGHT**: มอเตอร์ปล่อย (coast) รอจน |theta| < 3° ค้างไว้ 0.5 วินาที
  (ผู้ใช้ประคองโครงให้ตั้งตรงเอง) จึงเข้า BALANCING
- **BALANCING**: รัน control law ตามด้านบน ถ้า |theta| > 20° ถือว่าล้ม -> FALLEN
- **FALLEN**: เบรกสั้น ๆ แล้วปล่อย (coast), reset integrator, รอ hold ตั้งตรงใหม่
  แล้วกลับไป WAIT_UPRIGHT (ไม่ข้ามตรงไป BALANCING เลย)
- **JUMP_UP** (ทดลอง, ปิดโดย default): ปั่นล้อขึ้นความเร็วที่ตั้งไว้ แล้วเบรกกะทันหัน
  ให้โมเมนตัมที่สะสมส่งให้โครงเงยขึ้น จากนั้นถ้า |theta| < 15° หลังเบรก จะส่งต่อให้
  BALANCING ทันที **โหมดนี้พึ่งพากลไก/แรงเสียดทานของโครงจริงมาก อาจไม่ทำงานเลย
  หรือทำให้โครงกระแทกแรงได้ ต้องเข้าใจความเสี่ยงก่อนเปิดใช้**

## Safety

- มอเตอร์ถูกปิด (brake) ตั้งแต่บูตจนกว่าจะ calibrate เสร็จ
- ถ้าอ่าน IMU ผิดพลาดติดกัน 10 ครั้ง (`IMU_FAIL_LIMIT`) เฟิร์มแวร์จะสั่งเบรกมอเตอร์
  ทันที **และเปลี่ยน state เป็น FALLEN** (ไม่ใช่แค่เบรกเฉย ๆ แล้วค้าง state เดิมไว้กับ
  `omega_cmd` ที่ค้างอยู่) เพื่อไม่ให้ตอน IMU กลับมาอ่านได้ปกติ ระบบ resume การทรงตัว
  ด้วยค่าที่ไม่สดแล้ว -- ต้อง hold ตั้งตรงใหม่ผ่าน WAIT_UPRIGHT ตามปกติ พิมพ์
  ข้อความเตือนหนึ่งครั้งตอนเข้า fault (ไม่สแปม)
- คำสั่ง `stop` เบรก+ปล่อยมอเตอร์ทันทีทุก state (ยกเว้น CALIBRATING)
- การสลับทิศทาง (F/R) ทำผ่านสถานะเครื่องที่บังคับให้ duty เป็น 0 ก่อนเปลี่ยน
  ระดับพิน DIR เสมอ (ไม่มีทางสั่งสลับทิศตอน duty ไม่เป็นศูนย์) ตรวจได้ด้วย
  `selftest`
- `Motor::setSpeed()` จะปลดเบรก (เขียน brake pin เป็น inactive) ทุกครั้งที่กำลังจะ
  สั่ง duty ที่ไม่เป็นศูนย์ ไม่ว่าจะถูกเรียกจาก state ไหนก็ตาม -- ป้องกันกรณีเช่น
  สั่ง `jump` ระหว่างช่วง brake 200ms ของ FALLEN แล้ว PWM ไปสู้กับเบรกที่ยังค้างอยู่
- ลูป control ทำงานที่ 500 Hz ด้วย `micros()` แบบ fixed-timestep ถ้า loop() ถูกบล็อก
  นาน (เช่น Serial write ค้างตอนไม่มีใครเปิด monitor) เฟิร์มแวร์จะนับ
  `loop_overruns` (ดูได้จาก `get`) และถ้าตกขบวนเกิน 10 คาบจะ resync เวลาแทนที่จะ
  รันหลาย control step รัวติดกันด้วยข้อมูลเซนเซอร์ที่ไม่สด (จึงตั้ง
  `Serial.setTxTimeoutMs(0)` ไว้ตั้งแต่ setup() เพื่อไม่ให้ Serial write บล็อกด้วย)

## หมายเหตุค่า gain ตั้งต้น

`DEFAULT_KP/KD/KW/KI` ใน `config.h` เป็น **placeholder = 0 ทั้งหมด** รอค่าจากงาน
simulation (Python, ทำโดยทีมอื่นแบบขนาน) มาใส่ก่อนเริ่มจูนบนฮาร์ดแวร์จริง ห้ามเชื่อ
ค่าตั้งต้นเหล่านี้ว่าทรงตัวได้
