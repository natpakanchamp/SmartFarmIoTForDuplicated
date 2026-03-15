# SmartFarmIoT

Repository นี้สร้างขึ้นมาเพื่อส่งงานรายวิชา **261214 + 215 Microprocessor and Interfacing + Lab** โดยเป็นระบบ Smart Farm อัตโนมัติที่ใช้ ESP32 ควบคุมการให้น้ำและแสงสำหรับการปลูกพืชในโรงเรือน

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

> **หมายเหตุ:** Repository นี้ถูก Duplicate มาจาก repository ต้นฉบับโดยมีการใช้ **Claude AI** ช่วยในการแยกข้อมูลที่เป็นความลับ (WiFi credentials, API tokens, MAC addresses, server endpoints) ออกเป็นไฟล์ `secrets.h` ที่ไม่ถูก commit ขึ้น Git เพื่อปกป้องข้อมูลที่เป็นความลับและไม่สามารถเผยแพร่ได้
