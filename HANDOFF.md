# Handoff — Reaction Wheel Balance

อัปเดตล่าสุด: 2026-09-13 · repo: `OAKHEN1412/reaction-wheel-balance` (private)
local path: `C:\Users\Pishe\Desktop\Projects\self-balancing-robot`

## เป้าหมายโปรเจค
โครงตัว U ทรงตัวบนขอบฐานด้วยล้อเหวี่ยง 20 cm (reaction wheel) บนแกน X — ESP32-C3 อ่าน MPU-6050 แล้วสั่ง BLDC-3640 ขอบเขตในโครงงาน: เอียงไม่เกิน 5° บนพื้นราบ
ภาพรวม สเปก และงบประมาณอยู่ใน `project-brief.md` (สรุปจากสไลด์ PDF และ xlsx ของผู้ใช้) และ `README.md`

## ทำเสร็จแล้ว (อ่านรายละเอียดจากไฟล์ ไม่ต้องทำซ้ำ)
| ส่วน | ไฟล์ | สถานะ |
|---|---|---|
| วงจร | `hardware/wiring.md`, `hardware/schematic.svg/.png` | รีวิวแล้ว ขาตรงตามสเปก |
| เฟิร์มแวร์ | `firmware/reaction_wheel_balance/`, `firmware/imu_test/`, `firmware/motor_test/`, `firmware/README.md` | compile ผ่านทั้ง 3 sketch · รีวิวและแก้ไปแล้ว 1 รอบ (8 จุด) |
| Simulation | `sim/` (`design_gains.py --set … --quick`, `hardware_requirements.py`, `README.md`) | unittest ผ่าน 8/8 |
| หน้าเว็บภาพรวม | `project-overview.html` + `assets/` · เผยแพร่ที่ https://claude.ai/code/artifact/0f8a08c6-c1ac-4857-b9ba-71d55fbd3f8d | v2 มีรูปโมดูลจริงและภาพ CAD แล้ว |

คำสั่ง compile: `arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc firmware/<sketch>` (esp32 core 3.2.1 ติดตั้งอยู่แล้ว) · Python ใช้ `py -X utf8`

## ข้อสรุปสำคัญ (ไม่ชัดเจนถ้าอ่านจากโค้ดอย่างเดียว)
- **กฎควบคุม:** ใช้ state feedback แล้วคำนวณเป็นความเร่งล้อ `a = Kp·θ + Kd·θ̇ + Kw·ω + Ki·∫θ` จากนั้นสะสมเป็น ω_cmd **ห้ามแปลงมุมเป็น PWM ตรง ๆ** เพราะแรงบิดปฏิกิริยาเป็นสัดส่วนกับความเร่งของล้อ
- **เกนทุกตัวเครื่องหมายบวกเหมือนกัน** ตาม LQR · Kp มีค่าต่ำสุดประมาณ m·g·l / I_w จึงห้ามจูนแบบเริ่มจาก Kp = 0 · เกนตั้งต้นใน `config.h` คือ Kp 1137, Kd 144.3, Kw 1.0, Ki 0.0387 ออกแบบไว้ที่อัตราทด 2.5 และ tau_m 0.06 s
- **Simulation:** ถ้าออกแบบเกนให้ตรงกับพารามิเตอร์แต่ละชุด จะฟื้นจาก 5° ได้ 19/20 รอบ แต่ต้องมีอัตราทด ≥ 3, แรงบิดล้อ ≥ 1.5 เท่าของแรงโน้มถ่วง และ tau_m ≤ 0.08 s · ฟื้นจาก 10° ไม่ได้ · **โหมดลุกเองจากท่านอน (jump-up) ทำไม่ได้** กับมอเตอร์นี้ (`ENABLE_JUMP_UP=false`)
- ยอด Monte Carlo ต่ำ ๆ ที่ worker รายงานรอบแรกเกิดจากใช้เกนชุดเดียวกับพารามิเตอร์ทุกช่วง แก้วิธีวัดแล้ว ดู `sim/README.md` §3.5–3.7
- รูป ESP32 และแบตในสไลด์เดิมไม่ตรงรุ่นที่ใช้จริง · รูปใน `assets/modules/` มาจากลิงก์ร้านใน xlsx

## ยังค้าง / ขั้นต่อไป
1. **ไดโอดกันไฟย้อนที่ขา 5V ของ C3** — ขา 5V ต่อกับ USB VBUS ตรง ๆ ถ้าเสียบ USB ขณะ buck จ่ายไฟอยู่ ไฟจะชนกัน เสนอผู้ใช้ให้ใส่ Schottky (1N5819/SS34) ระหว่าง buck OUT+ กับขา 5V **ผู้ใช้ยังไม่ได้ตอบว่าจะให้เพิ่มลง `wiring.md`, schematic และ HTML หรือไม่** — ถามก่อนแก้
2. ยังไม่เคยทดสอบบนฮาร์ดแวร์จริง ทุกค่าที่ขึ้นต้นด้วย `// placeholder` / `MEASURE` ใน `firmware/reaction_wheel_balance/config.h` ต้องหาจาก `imu_test` / `motor_test`: ขั้ว PWM/DIR/BRAKE, พัลส์ FG ต่อรอบ, แกน IMU, อัตราทด, `MOTOR_MIN_DUTY_FRACTION`
3. หลังวัดมวล COM I_w และ tau_m จริง → รัน `sim/design_gains.py --set … --quick` → อัปเดตเกน
4. ข้อสงสัยด้านฮาร์ดแวร์ที่ยังไม่ยืนยัน: ขาควบคุมมอเตอร์มี pull-up 5V ภายในหรือไม่ (ดู `hardware/wiring.md` risk b) · GPIO2 (F/R) เป็นขาบูต
5. สไลด์ยังว่างสองหน้า: "ทบทวนงานวิจัยที่เกี่ยวข้อง" และ "สรุปและข้อเสนอแนะ" · ตัวเลขงบในสองชีตของ xlsx ไม่ตรงกัน

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
