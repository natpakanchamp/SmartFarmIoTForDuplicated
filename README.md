# SmartFarmIoT

Repository นี้สร้างขึ้นมาเพื่อส่งงานรายวิชา **261214 + 215 Microprocessor and Interfacing + Lab, Computer Engineering, Chiang Mai University** โดยเป็นระบบ Smart Farm อัตโนมัติที่ใช้ ESP32 ควบคุมการให้น้ำและแสงสำหรับการปลูกพืชในโรงเรือน

---

## ภาพรวมระบบ (System Overview)

ระบบประกอบด้วย ESP32 จำนวน 2 ตัว สื่อสารกันผ่านโปรโตคอล ESP-NOW:

| Node | หน้าที่ | ไฟล์หลัก |
|------|---------|----------|
| **Node A** (Sensor TX) | อ่านค่าเซ็นเซอร์ความชื้นดิน 4 ช่อง + แสง BH1750 แล้วส่งไป Node B | `NodeA/NodeA.ino` |
| **Node B** (Controller) | รับข้อมูล, ตัดสินใจควบคุมวาล์วน้ำ/ไฟ, ส่ง Telemetry ขึ้น MQTT/ThingsBoard | `SmartFarmIoT.ino` |

---

## ฟีเจอร์หลัก

### Node A - Sensor Transmitter
- อ่านค่าความชื้นดิน 4 ช่อง (Capacitive Soil Moisture Sensor)
- กรองสัญญาณด้วย Median-of-5 + Exponential Moving Average (EMA)
- อ่านค่าแสง BH1750 พร้อม auto-retry เมื่อ sensor ขัดข้อง
- ส่งข้อมูลผ่าน ESP-NOW แบบ fixed channel

### Node B - Controller
- **Soil Aggregation** - รวมค่าดิน 4 ช่องด้วย trimmed mean/median + EMA
- **Growth Phase Management** - 3 ระยะ (Establish / Vegetative / Finishing) คำนวณจากวันหลังย้ายปลูก
- **DLI Integration** - คำนวณ Daily Light Integral แบบ real-time พร้อม photoperiod control
- **Irrigation Control** - anti-chatter, ignore window หลังเปิดวาล์ว, emergency mode, dryback mode, pulse irrigation, autotune kWet model
- **Light Control** - เปิด/ปิดไฟเสริมตาม DLI target และ photoperiod
- **Valve Guard** - ป้องกันการสั่งเปิด/ปิดวาล์วถี่เกินไป (minimum ON/OFF time)
- **MQTT Telemetry** - ส่งข้อมูลทุก parameter ขึ้น MQTT broker
- **ThingsBoard Integration** - รับคำสั่งจาก dashboard ผ่าน HTTP long-poll
- **OTA Update** - อัปเดต firmware ผ่าน HTTPS
- **TFT Display** - แสดงผล 3 หน้า (Operate / Health / Control) บนหน้าจอ TFT
- **RTC + NTP** - เวลาจาก DS3231 RTC พร้อม NTP sync รายชั่วโมง
- **RGB Status LED** - แสดงสถานะระบบผ่าน LED สี

---

## Hardware ที่ใช้

- ESP32 DevKit x2
- Capacitive Soil Moisture Sensor v1.2 x4
- BH1750 Light Sensor
- DS3231 RTC Module
- TFT Display (ILI9341 / ST7789)
- Relay Module 2 Channel (วาล์วน้ำ + ไฟเสริม)
- RGB LED

---

## โครงสร้างไฟล์

```
SmartFarmIoTForDuplicated/
├── SmartFarmIoT.ino      # Node B - Controller (main)
├── SmartFarmTypes.h       # Shared type definitions (GrowPhase enum)
├── User_Setup.h           # TFT display configuration
├── NodeA/
│   └── NodeA.ino          # Node A - Sensor Transmitter
├── secrets.h.example      # Template สำหรับตั้งค่าข้อมูลลับ
├── docs/
│   └── SmartFarmIoT.ino.bin  # Pre-built firmware (OTA)
└── README.md
```

---

## การตั้งค่าก่อนใช้งาน

1. คัดลอก `secrets.h.example` เป็น `secrets.h`
   ```bash
   cp secrets.h.example secrets.h
   ```

2. แก้ไข `secrets.h` ใส่ค่าของคุณ:
   - WiFi SSID / Password
   - MQTT Broker, Port, Username, Password
   - ThingsBoard Host / Device Token
   - OTA Firmware URL
   - MAC Address ของ Node A และ Node B

3. อัปโหลดโค้ดไปยัง ESP32 ผ่าน Arduino IDE หรือ PlatformIO

---

## Libraries ที่ต้องติดตั้ง

- `BH1750` - เซ็นเซอร์วัดแสง
- `PubSubClient` - MQTT client
- `TFT_eSPI` - TFT display driver
- `RTClib` - DS3231 RTC
- `WiFiMulti` - จัดการหลาย WiFi network
- `ESP-NOW` (built-in ESP32)

---

## Git Log จาก Repository ต้นฉบับ

<details>
<summary>คลิกเพื่อดู commit history ทั้งหมด (85 commits)</summary>

```
add0eb9 2026-03-12 OTA reduce gain
f4ecdb8 2026-03-12 Merge pull request #18 from natpakanchamp/testOTA
6752037 2026-03-12 this is final versiongit add .(hope to be)
ef11429 2026-03-11 last
1e0a043 2026-03-05 changed reason
f52784d 2026-03-04 Merge pull request #17 from natpakanchamp/testOTA
097f658 2026-03-04 fix for new board db
e992034 2026-03-04 change MAC esp23 and Board
2003813 2026-03-01 ota
bd41ceb 2026-03-01 update
d8d9cc1 2026-03-01 ota
cecbc77 2026-03-01 ota
e25919b 2026-03-01 Add files via upload
1bf1bb8 2026-03-01 Create .keep
28d955a 2026-03-01 OTA
f7a4c42 2026-03-01 ota
4eda6e5 2026-03-01 UPDATE: OTA
bf34c3d 2026-03-01 Add redirect to latest SmartFarmIoT release
a4eb5b5 2026-03-01 Merge pull request #16 from natpakanchamp/Led
aa81156 2026-03-01 Lastets
4ea3b16 2026-03-01 idontknow
3623ef7 2026-02-28 UPDATE: more BAR to Node A
55ca7a5 2026-02-28 update arduino code
775030a 2026-02-26 Merge pull request #15 from natpakanchamp/master
8bfd779 2026-02-25 UPDATE: push NodeA to folder
a87adc7 2026-02-25 Prob Finish
62b80e1 2026-02-24 UPDATE: fix wifi and dashboard 12:50
4a3fc1c 2026-02-23 OOP to Node A
2887d1f 2026-02-23 Merge pull request #14 from natpakanchamp/master
5c61320 2026-02-23 update: IPAddress mqtt_broker_ip — IP fallback
a8c9bb7 2026-02-18 Pre-release
1045497 2026-02-18 THIS IS MAGIC
bb509ce 2026-02-18 FIX: channel Node A
36276a5 2026-02-18 FIX: channel esp32 node B
c3fa6e2 2026-02-18 I don't know
6bc4303 2026-02-18 update to static
9f1eb21 2026-02-18 update to static
96230da 2026-02-18 update to static
150f414 2026-02-18 ...
c8f9f0f 2026-02-18 UODATE
9627a75 2026-02-18 UPDATE: RESCAN
4480253 2026-02-18 Dynamic Node A
b2539d4 2026-02-18 UPDATE: Dynamic Channel
b7df587 2026-02-18 changed channel to 9
a4f3bdc 2026-02-18 WIP on UI
36e607e 2026-02-17 uodate
cae2e1e 2026-02-17 UPDATE: return readLuxSafe()
9a84bae 2026-02-17 UPDATE: ...
c1efefc 2026-02-17 UPDATE: 4Soil
1d6bd62 2026-02-17 UPDATE: 4soil
d08ee98 2026-02-17 I dont what it changed
d9316a1 2026-02-17 UPDATE: add topic light
7867fe8 2026-02-17 UPDATE: Seperate exacly part
fc31633 2026-02-17 update msgType
57f4fd8 2026-02-17 update patch MsgType
994cc14 2026-02-17 UPDATE
3378e77 2026-02-17 UPDATE
baf7ed5 2026-02-17 UPDATE
d811340 2026-02-17 update
281e882 2026-02-17 UPDATE: init MAC Address
3c5b35f 2026-02-17 Update Node A
dbe7ec4 2026-02-17 update Node B
2b4f7c8 2026-02-12 UPDATE
a650b91 2026-02-12 UPDATE 1.0
f106bed 2026-02-11 Merge pull request #13 from natpakanchamp/test
f205f10 2026-02-11 UPDATE
33d06ff 2026-02-11 ERROR wait for fix
6cfcc36 2026-02-11 Source
70e0c5c 2026-02-11 Merge pull request #12 from natpakanchamp/test
eb494f7 2026-02-11 UPDATE TFT ROTATE
d6c95ad 2026-02-10 Merge pull request #11 from natpakanchamp/test
100572d 2026-02-10 add User_Setup.h
0c668da 2026-02-10 Merge pull request #10 from natpakanchamp/test
4465e03 2026-02-10 NEW UPDATE
e825bfb 2026-02-10 Merge pull request #9 from natpakanchamp/test
a7bba17 2026-02-10 UPDATE LASTET
a264c4b 2026-02-09 Create doc
cb45088 2026-02-09 Create SmartFarmIoTv3.ino
6bc28be 2026-02-09 Create SmartFarmIoTv2.ino
f1912a7 2026-02-09 Merge pull request #8 from natpakanchamp/champxxx
0ba8d0f 2026-02-06 UPDATE
499473e 2026-02-06 Merge pull request #7 from natpakanchamp/test
d6dcd5e 2026-02-06 FIXED NON-SINGNAL TFT
a1f269c 2026-02-05 Merge pull request #6 from natpakanchamp/dashboard
10f2dc1 2026-02-05 dashboard fixed
2bbd251 2026-02-05 Merge pull request #5 from natpakanchamp/feature/TFT
a7079ba 2026-02-05 UPDATE_TFT2.0
15db708 2026-02-05 Try to connect TFT
64e788d 2026-02-05 Merge pull request #4 from natpakanchamp/feature/Mode
5cb8b8e 2026-02-05 UPDATE_MQTT
f56304e 2026-02-05 Merge pull request #3 from natpakanchamp/feature/valve_main
25c2d74 2026-02-05 Update ManualMode
83f19d1 2026-02-05 Create README.md
7d209d0 2026-02-04 Merge pull request #2 from natpakanchamp/feature/valve_main
42ae685 2026-02-04 UPDATE_VALVE_MAIN
a04db14 2026-02-04 UPDATE
d1f7698 2026-02-04 update soil&waterSystem
0481ab6 2026-02-03 initail code
```

</details>

---

> **หมายเหตุ:** Repository นี้ถูก Duplicate มาจาก repository ต้นฉบับโดยมีการใช้ **Claude AI** ช่วยในการแยกข้อมูลที่เป็นความลับ (WiFi credentials, API tokens, MAC addresses, server endpoints) ออกเป็นไฟล์ `secrets.h` ที่ไม่ถูก commit ขึ้น Git เพื่อปกป้องข้อมูลที่เป็นความลับและไม่สามารถเผยแพร่ได้
