# Handoff — Reaction Wheel Balance

## 2026-09-19 ? Voltage/torque controller implemented

This supersedes the older speed-command diagnosis below: light-wheel steps
156/291/425 rpm at duty .29/.49/.68, t63 .155/.175/.215 s; adding rim nuts
increases tau to .45?.59 s with almost unchanged final rpm. Driver is voltage
mode, not a soft-start speed servo. Sim and firmware now default to signed
back-EMF compensation `(omega_est + .18*a)/omega_fs`; old mode stays selectable.
Measured electrical/IMU/FG constants preserved. DIR uses one zero-duty interval
then reverses even while spinning; estimator follows applied duty. CSV unchanged
but cmd_rpm now represents voltage-equivalent speed. No upload or COM3 access.

Keep Kp=1137.4, Kd=144.3, Kw=1, Ki=.0387; CONTROL_MODE_VOLTAGE=true,
MOTOR_TAU_S=.18 (light wheel), ACTIVE_DECEL_BRAKE=false. Saved EEPROM gains
still override config defaults. Six-case nominal recovery: 20/20 except
(.25 s, coast=.3) = 18/20; fixed-gain wheel-inertia sweep worst = 17/20,
one saturated run. Soft gains/higher Kd did not help.
Validation: 15/15 Python tests pass; Mega compile passes (24078 B flash,
1613 B SRAM). Extended firmware self-test compiled, not executed on board. See sim/README.md and
sim/voltage_results.json for protocol, limits and complete results. Need wheel
mass/inertia, low-duty behavior, FG sign accuracy and reverse/braking torque
measurements before treating these model results as hardware validation.


อัปเดตล่าสุด: 2026-09-19 · repo: `OAKHEN1412/reaction-wheel-balance` (private)
local path: `C:\Users\Pishe\Desktop\Projects\self-balancing-robot`

## เป้าหมายโปรเจค
โครงตัว U ทรงตัวบนขอบฐานด้วยล้อเหวี่ยง 20 cm (reaction wheel) บนแกน X — **Arduino Mega 2560** อ่าน MPU-6050 แล้วสั่ง BLDC-3640 ขอบเขตในโครงงาน: เอียงไม่เกิน 5° บนพื้นราบ
ภาพรวม สเปก และงบประมาณอยู่ใน `project-brief.md` (สรุปจากสไลด์ PDF และ xlsx ของผู้ใช้ — สไลด์เดิมระบุ ESP32-C3 แต่โปรเจคเปลี่ยนเป็น Mega ถาวรแล้ว ดูหมายเหตุในไฟล์นั้น) และ `README.md`

## ทำเสร็จแล้ว (อ่านรายละเอียดจากไฟล์ ไม่ต้องทำซ้ำ)
| ส่วน | ไฟล์ | สถานะ |
|---|---|---|
| วงจร | `hardware/wiring.md`, `hardware/schematic.svg/.png` | รีวิวแล้ว ขาตรงตามสเปก |
| เฟิร์มแวร์ | `firmware/reaction_wheel_balance/`, `firmware/imu_test/`, `firmware/motor_test/`, `firmware/README.md` | compile ผ่านทุก sketch (รวม `sign_test`) · รีวิวและแก้ไปแล้ว 1 รอบ (8 จุด) |
| Simulation | `sim/` (`design_gains.py --set … --quick`, `hardware_requirements.py`, `README.md`) | unittest ผ่าน 8/8 |
| หน้าเว็บภาพรวม | `project-overview.html` + `assets/` · เผยแพร่ที่ https://claude.ai/code/artifact/0f8a08c6-c1ac-4857-b9ba-71d55fbd3f8d | v2 มีรูปโมดูลจริงและภาพ CAD แล้ว |

คำสั่ง compile: `arduino-cli compile --fqbn arduino:avr:mega firmware/<sketch>` (avr core ติดตั้งอยู่แล้ว) · upload ที่ `COM3` · Python ใช้ `py -X utf8`

## ข้อสรุปสำคัญ (ไม่ชัดเจนถ้าอ่านจากโค้ดอย่างเดียว)
- **กฎควบคุม:** ใช้ state feedback แล้วคำนวณเป็นความเร่งล้อ `a = Kp·θ + Kd·θ̇ + Kw·ω + Ki·∫θ` จากนั้นสะสมเป็น ω_cmd **ห้ามแปลงมุมเป็น PWM ตรง ๆ** เพราะแรงบิดปฏิกิริยาเป็นสัดส่วนกับความเร่งของล้อ
- **เกนทุกตัวเครื่องหมายบวกเหมือนกัน** ตาม LQR · Kp มีค่าต่ำสุดประมาณ m·g·l / I_w จึงห้ามจูนแบบเริ่มจาก Kp = 0 · เกนตั้งต้นใน `config.h` คือ Kp 1137, Kd 144.3, Kw 1.0, Ki 0.0387 ตรวจซ้ำที่อัตราทดจริง 6 แล้ว ได้ค่าเท่าเดิม (tau_m 0.06 s ยังเป็นค่าประมาณ)
- **Simulation:** ถ้าออกแบบเกนให้ตรงกับพารามิเตอร์แต่ละชุด จะฟื้นจาก 5° ได้ 19/20 รอบ แต่ต้องมีอัตราทด ≥ 3, แรงบิดล้อ ≥ 1.5 เท่าของแรงโน้มถ่วง และ tau_m ≤ 0.08 s · ฟื้นจาก 10° ไม่ได้ · **โหมดลุกเองจากท่านอน (jump-up) ทำไม่ได้** กับมอเตอร์นี้ (`ENABLE_JUMP_UP=false`)
- ยอด Monte Carlo ต่ำ ๆ ที่ worker รายงานรอบแรกเกิดจากใช้เกนชุดเดียวกับพารามิเตอร์ทุกช่วง แก้วิธีวัดแล้ว ดู `sim/README.md` §3.5–3.7
- รูป ESP32 และแบตในสไลด์เดิมไม่ตรงรุ่นที่ใช้จริง · รูปใน `assets/modules/` มาจากลิงก์ร้านใน xlsx

## ทดสอบฮาร์ดแวร์จริง 2026-09-19
- **MPU-6050 (GY-521) ตัวที่มีเสีย:** ตอบ WHO_AM_I=0x68, ACK ทุก byte (ยืนยันด้วย bit-bang I2C) แต่เขียน register ไม่ติด ค้าง sleep (PWR_MGMT_1=0x40) ค่า accel/gyro เป็น 0 — ต้องซื้อใหม่ · มี `tools/imu_viewer.html` (Codex สร้าง, Web Serial + 3D + Auto-suggest แกน/เครื่องหมาย) รอใช้กับตัวใหม่
- **มอเตอร์ BLDC-3640 วัดครบแล้ว** (ใส่ใน `config.h` + `wiring.md` แล้ว): สีสายจริง แดง +12V / ดำ GND / น้ำเงิน PWM **กลับขั้ว** (เริ่มหมุน ~10%) / เหลือง FG (~70 พัลส์ต่อรอบล้อ, ส่งเฉพาะตอนไดร์เวอร์ขับ) / ขาว DIR (LOW = CW) / เขียว BRAKE (LOW = เบรก, ผลน้อยกว่า duty 0 นิดเดียว)
- **เปลี่ยนคอนโทรลเลอร์เป็น Arduino Mega 2560 ถาวรแล้ว** — ขาว (DIR) และเขียว (BRAKE) มี pull-up ในไดร์เวอร์แรง (~1kΩ ไป 5V) ESP32 (3.3V) ผ่าน R ดึงลงไม่ได้ ต้องใช้ NPN ช่วย ส่วน Mega เป็น 5V logic ต่อตรงได้เลย **ไม่ต้องใช้ NPN อีกต่อไป** เฟิร์มแวร์พอร์ตเสร็จแล้ว เวอร์ชัน ESP32 เดิมย้ายไปเก็บที่ `firmware/legacy_esp32/`
- **ความปลอดภัย:** PWM กลับขั้ว → ถ้าขา PWM ลอย/LOW มอเตอร์วิ่งเต็มสปีด · **ปิดไฟ 12V ทุกครั้งก่อนแฟลช/รีเซ็ตบอร์ด** (Mega ก็ยังมีความเสี่ยงนี้ตอน reset/upload เหมือนกัน) · แนะนำใส่ pull-up 10k จาก D11 ไป 5V เป็น hardware safety net เพิ่มเติม
- `motor_test.ino` (Mega) รองรับคำสั่ง `duty`, `dir`, `brake`, `stop`, `count`, `resetcount`, `status`, `help`

## ยังค้าง / ขั้นต่อไป
0. **อัตราทดวัดแล้ว = 6** (2026-09-19 นับด้วยมือ: ล้อ 1 รอบ = มอเตอร์ 6 รอบ) ใส่ใน `config.h` และ `sim/params.py` แล้ว · รัน sim ใหม่ได้เกนเท่าเดิม ฟื้นจาก 5° ได้ 19/20 รอบ ไม่อิ่มตัว ล้อหมุนสูงสุด 157 จากขีดจำกัด 667 rpm แรงบิดล้อ 0.46 N·m เท่ากับ ~4 เท่าของแรงโน้มถ่วงที่ 5° · FG น่าจะเป็น 12 พัลส์ต่อรอบมอเตอร์ (72 ต่อรอบล้อ) ไม่ใช่ 70 ถ้าจะให้แม่นให้นับซ้ำ
1. **MPU-6050 ตัวใหม่ใช้งานได้แล้ว (2026-09-19)**: WHO_AM_I=0x68 ตอนวางนิ่ง az≈1.05g และ gyro bias ≈ (-1.1, 0.5, 1.7) dps · **ยืนยันแกนแล้ว:** accel ใช้ atan2(ay,az), gyro ใช้ X และเครื่องหมาย +1 ทั้งคู่ (Δθ +9.4°, corr 0.74) ใส่ใน `config.h` แล้ว ถ้าย้ายตำแหน่ง IMU ต้องวัดใหม่
1b. **ทิศควบคุมยืนยันแล้ว** (2026-09-19 ด้วย `firmware/sign_test/`): ล้อเร่งไป +ω แล้วโครงเอียงไป θ ลบ -10.3° · ล้อเร่งไป -ω แล้วโครงเอียงไป θ บวก +7.0° → `DEFAULT_CONTROL_SIGN = +1` ถูกต้อง · ทิศทางจริง: **เอียงซ้าย = θ ลบ** · IMU ตอนตั้งตรงอ่านได้ประมาณ -2 ถึง -3° ใช้คำสั่ง `zero` ในเฟิร์มแวร์หลัก
1c. **ทดสอบทรงตัวครั้งแรก (2026-09-19) ไม่สำเร็จ:** zero offset = -0.28° (บันทึกลง EEPROM แล้ว) · ทิศที่ controller ดันกลับถูกต้อง แต่โครงแกว่งแรงขึ้นทุกรอบ (+2.8 → -7.8 → +10.8 → -16.9°, คาบ ~0.85 s) แล้วล้มใน ~1.5 s เพราะล้อหมุนตามคำสั่งได้แค่ประมาณ 1/5 · `firmware/step_test/` วัดซ้ำ 2 รอบได้ผลเหมือนกัน: t63 = **0.45–0.59 s** (sim สมมติไว้ 0.06 s), ที่ duty เต็มล้อหมุนได้ประมาณ 585 rpm, ความเร่งสูงสุดของล้อประมาณ 50 rad/s² ≈ แรงบิด 0.125 N·m (ถ้า I_w = 0.00245) ซึ่งแค่พอสู้แรงโน้มถ่วงที่ 5° (0.108 N·m) sim เคยสมมติไว้ 0.46 · sim ที่ tau_m 0.15–0.45 ได้ 0/20 · ขั้วเวลาไม่เสถียรของโครงประมาณ 0.13 s เร็วกว่ามอเตอร์ประมาณ 4 เท่า → **ปัญหาอยู่ที่ฮาร์ดแวร์ ไม่ใช่การจูน** ดูทางเลือกในข้อ 2 · FG ส่งพัลส์เฉพาะตอนขับ จึงวัด coast ไม่ได้
2. **ขั้นต่อไป: เลือกทางแก้ actuator** (ก) หาว่าไดรเวอร์มี soft-start/accel ramp ไหม (ข) เพิ่มอัตราทดเป็นประมาณ 10–12 เพราะความเร็วล้อยังเหลือเยอะ (ค) ลดแรงโน้มถ่วง m·l: ย้ายแบตหรือ Mega ลงต่ำ (ง) ถ้ามอเตอร์รับได้ ลองใช้ไฟ 24V · เดิม: **ทดสอบทรงตัวครบระบบครั้งแรก** แกน IMU, อัตราทด, `MOTOR_MIN_DUTY_FRACTION` และทิศควบคุมวัดครบแล้ว · แฟลช `reaction_wheel_balance` ตอนปิด 12V → ตั้งตรงแล้วใช้คำสั่ง `zero` → `start` โดยมีมือกันล้มไว้
3. หลังวัดมวล COM I_w และ tau_m จริง (รวมมวล/COM ที่เปลี่ยนไปจาก Mega ที่หนักกว่า ESP32-C3 Super Mini มาก — ~37g เทียบ ~2g, ขนาด ~101.5×53.3mm เทียบ ~22×18mm ต้องคิดในตำแหน่งจริงบนโครง) → รัน `sim/design_gains.py --set … --quick` → อัปเดตเกน
4. สไลด์ยังว่างสองหน้า: "ทบทวนงานวิจัยที่เกี่ยวข้อง" และ "สรุปและข้อเสนอแนะ" · ตัวเลขงบในสองชีตของ xlsx ไม่ตรงกัน

## ข้อตกลงการทำงานกับผู้ใช้
- สื่อสารภาษาไทย · ผู้ใช้สั่งงานแบบหัวหน้าคุมลูกน้อง ผ่าน skill `/engineer`: Opus วางแผนและรีวิว ส่วน `worker-general` (Sonnet) ลงมือทำ
- งานยาวให้แจ้งผ่าน PushNotification เมื่อเสร็จ
- ถ้าแก้หน้าเว็บให้ republish ที่ URL เดิม (ใช้ `url` ถ้าอยู่ต่าง session) อย่าสร้าง artifact ใหม่

## Suggested skills
- `engineer` — เมื่อต้องแตกงานให้ worker ทำ (แก้วงจรเพิ่มไดโอด, bring-up tooling)
- `artifact-design` — ก่อนแก้ `project-overview.html` แล้ว republish
- `diagnose` / `debug-mantra` — ตอนทดสอบกับเครื่องจริงแล้วเจอบั๊ก (บูตไม่ขึ้น, ทรงตัวไม่ได้, IMU อ่านไม่ได้)
- `tdd` — ถ้าเพิ่ม logic ใหม่ใน header pure-logic ของเฟิร์มแวร์ หรือใน `sim/`
- `tailsync` — ถ้าต้องย้ายไฟล์ใหญ่ เช่นไฟล์ CAD หรือ log telemetry ไปเครื่อง hub

## ทดสอบทรงตัวรอบ 2 (voltage mode, 2026-09-19)
- ทรงตัวได้ประมาณ 1–7 วินาทีต่อครั้ง (20 ครั้ง) ดีกว่ารอบแรก แต่ยังล้มทุกครั้ง · log: scratchpad `balance_attempt2.log`
- **สาเหตุหลัก: FG มีพัลส์หลอก** (ห่างกันประมาณ 25–35 µs ช่วงที่กลับทิศ) ทำให้อ่านความเร็วล้อได้ 30,000–42,000 rpm → ในโหมด voltage ทำให้ duty ชน 100% ไปทิศผิดนานประมาณ 300 ms (เกิด 588 จาก 7204 sample)
  - แก้แล้วใน `fg_tach.cpp`: ทิ้งพัลส์ที่ห่างกันน้อยกว่า `FG_MIN_PERIOD_US` (700 µs) และให้ค่าความเร็วลดลงเองเมื่อไม่มีพัลส์ใหม่ · ถ้ายังมีปัญหา: ใส่ pull-up 4.7k จากสายเหลืองไป 5V (+ C 1nF ลง GND)
- ปัญหารอง: 20–40% ของเวลา duty อยู่ใต้ 0.10 ซึ่งเป็นช่วงที่มอเตอร์ไม่ออกแรง ถ้าแก้ FG แล้วยังส่ายไปมาช้า ๆ ให้เพิ่มการชดเชย deadzone ในโหมด voltage

## 🎉 ทรงตัวได้สำเร็จครั้งแรก (2026-09-19, รอบ 3 + จูน Kd)
- ใช้โหมด voltage + ตัวกรอง FG · Kd 144.3 (ค่าจาก LQR) → ทรงได้ 1.5–8 วินาที · Kd 220 → ดีขึ้นมาก · **Kd 300 → ทรงได้ต่อเนื่องมากกว่า 2 นาทีโดยไม่ล้ม**
- 10 วินาทีสุดท้าย: θ เฉลี่ย -0.33°, SD 1.7°, อยู่ในช่วง -3.7 ถึง +3.1° · ส่ายด้วยคาบ ~0.47 s · ล้อหมุนเฉลี่ย +137 rpm (ช่วง 40–223) · duty สูงสุด 0.47 · ไม่พบค่าหลอกจาก FG
- บันทึก Kd 300 ลง EEPROM แล้ว และตั้งเป็น `DEFAULT_KD` ใน `config.h` แล้ว (Kp 1137.4, Kw 1.0, Ki 0.0387 เท่าเดิม)
- ขั้นต่อไป: ลดการส่าย (ลอง Kd 350–400 หรือปรับ Kp), ล้อค้างอยู่ฝั่งบวกประมาณ 137 rpm (ลองเพิ่ม Kw เพื่อดึงความเร็วล้อกลับศูนย์), ทดสอบรับแรงผลัก, ใส่ R pull-up 4.7k ที่สาย FG

### จูนลดการส่าย (Kd, เปลี่ยนค่าระหว่างที่ทรงตัวอยู่)
| Kd | SD ของ θ | ช่วงของ θ | คาบการส่าย |
|---|---|---|---|
| 300 | 1.59° | -4.3 ถึง +3.7° | 0.46 s |
| 350 | 0.71° | -2.4 ถึง +1.9° | 0.35 s |
| 400 | 0.49° | -2.5 ถึง +2.1° | 0.28 s |
| **500** | **0.34°** | **-1.4 ถึง +0.8°** | 0.20 s |
- **ใช้ Kd 500** บันทึกลง EEPROM และตั้งเป็น `DEFAULT_KD` แล้ว · duty กระตุกเพิ่มขึ้นประมาณ 20% แต่ยังไม่มีอาการสั่นหรือเสียงหึ่ง · ถ้าเพิ่ม Kd ต่อ ได้ประโยชน์น้อยลงเรื่อย ๆ และเสี่ยงสัญญาณรบกวน
- ความเร็วล้อค้างเฉลี่ยประมาณ -91 rpm (ไม่กลับศูนย์) → ถ้าจะแก้ ลองเพิ่ม Kw
