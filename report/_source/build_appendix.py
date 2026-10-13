from docxlib import *
import os
import segno

REPO_URL = 'https://github.com/OAKHEN1412/reaction-wheel-balance'
COMMIT = '1758122'          # firmware used for the final results (2026-10-13)

os.makedirs('figs', exist_ok=True)
segno.make(REPO_URL, error='m').save('figs/qr_repo.png', scale=20, border=2)

doc = new_document()
para(doc, 'ภาคผนวก ก', align='center', first=0, bold=True, keep_next=True)
para(doc, 'ซอร์สโค้ดและข้อมูลของโครงงาน', align='center', first=0, bold=True, keep_next=True)
blank(doc)

para(doc, 'ซอร์สโค้ดของโปรแกรมควบคุมการทรงตัว โปรแกรมทดสอบอุปกรณ์ แบบจำลองทางคณิตศาสตร์ ผังการต่อวงจร และข้อมูลที่บันทึกจากการทดลอง'
          'ในปริญญานิพนธ์นี้ เผยแพร่เป็นสาธารณะบน GitHub สามารถเข้าถึงได้โดยสแกนคิวอาร์โค้ดในรูปที่ ก.1 หรือเปิดที่อยู่ ',
     REPO_URL)
figure(doc, 'figs/qr_repo.png', 'ก.1', 'คิวอาร์โค้ดสำหรับเข้าถึงซอร์สโค้ดของโครงงาน', 5.0)
para(doc, f'โปรแกรมที่ใช้ในผลการทดลองฉบับสุดท้ายตรงกับคอมมิต {COMMIT} ลงวันที่ 13 ตุลาคม 2569 '
          'โครงสร้างของคลังโค้ดแสดงดังตารางที่ ก.1')
table(doc, 'ก.1', 'โครงสร้างของคลังโค้ด',
      ['โฟลเดอร์', 'เนื้อหา'],
      [['firmware/reaction_wheel_balance', 'โปรแกรมควบคุมการทรงตัวสำหรับ Arduino Nano ค่าพารามิเตอร์ทั้งหมดอยู่ในไฟล์ config.h'],
       ['firmware/imu_test, motor_test, step_test, sign_test', 'โปรแกรมทดสอบเซนเซอร์และมอเตอร์ที่ใช้ในหัวข้อ 4.2'],
       ['firmware/tests', 'ชุดทดสอบตรรกะของตัวควบคุมและการประมาณความเร็วล้อบนคอมพิวเตอร์'],
       ['firmware/mega_isp, mega_passthrough', 'โปรแกรมสำหรับใช้ Arduino Mega 2560 เป็นตัวกลางอัปโหลดและสื่อสารกับ Arduino Nano'],
       ['firmware/legacy_esp32', 'โปรแกรมฉบับเดิมสำหรับ ESP32-C3 ก่อนเปลี่ยนหน่วยประมวลผล'],
       ['sim', 'แบบจำลองทางคณิตศาสตร์และการออกแบบค่าอัตราขยาย'],
       ['tuning', 'ข้อมูลที่บันทึกจากการทดลอง (logs) และสคริปต์วิเคราะห์ผล'],
       ['hardware', 'ผังการต่อวงจรและรายละเอียดการต่อสาย'],
       ['tools', 'หน้าเว็บแสดงค่าจากเซนเซอร์ MPU-6050 ผ่านพอร์ตอนุกรม']],
      [6.2, 8.8], align=['l', 'l'], size=14)
para(doc, 'การอัปโหลดโปรแกรมให้เปิดโฟลเดอร์ firmware/reaction_wheel_balance ด้วยโปรแกรม Arduino IDE หรือ arduino-cli '
          'เลือกบอร์ด Arduino Nano (ATmega328P) แล้วอัปโหลดผ่านพอร์ต USB โดยต้องปิดไฟ 12 V ก่อนอัปโหลดหรือรีเซ็ตบอร์ดทุกครั้ง '
          'ตามหัวข้อ 5.2 เมื่อเปิดไฟ บอร์ดจะปรับเทียบไจโรสโคปประมาณ 2 วินาที (ต้องวางโครงสร้างนิ่ง) จากนั้นเมื่อยกโครงสร้างให้ตั้งตรง'
          'และค้างไว้ 0.5 วินาที ระบบจะเริ่มควบคุมการทรงตัวโดยอัตโนมัติ')

doc.save('out/ภาคผนวก.docx')
print('saved appendix')
