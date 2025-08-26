/**
 * PTKidsBITRobot (BLE-safe)
 * This fork removes NeoPixel/ws2812b usage to reduce flash/RAM and avoid MakeCode Bluetooth conflicts.
 * Public APIs are kept, LED-related calls are converted to harmless no-ops (or onboard LED hints).
 */

let Sensor_All_PIN = [0, 1, 2, 3, 4, 5]
let Sensor_PIN = [1, 2, 3, 4]
let Sensor_Left = [0]
let Sensor_Right = [5]
let Num_Sensor = 4
let LED_PIN = 0

let Servo_Version = 1
let ADC_Version = 1
let Read_Servo_Version = false
let Read_ADC_Version = false
let PCA = 0x40
let initI2C = false
let initLED = false
let SERVOS = 0x06
let Line_LOW = [0, 0, 0, 0, 0, 0, 0, 0]
let Line_HIGH = [0, 0, 0, 0, 0, 0, 0, 0]
let Color_Line_All: number[] = []
let Color_Background_All: number[] = []
let Color_Line: number[] = []
let Color_Background: number[] = []
let Color_Line_Left: number[] = []
let Color_Background_Left: number[] = []
let Color_Line_Right: number[] = []
let Color_Background_Right: number[] = []
let Line_All = [0, 0, 0, 0, 0, 0]
let Line_Mode = 0
let Last_Position = 0
let Compensate_Left = 0
let Compensate_Right = 0
let error = 0
let P = 0
let D = 0
let previous_error = 0
let PD_Value = 0
let left_motor_speed = 0
let right_motor_speed = 0
let Servo_8_Enable = 0
let Servo_12_Enable = 0
let Servo_8_Degree = 0
let Servo_12_Degree = 0
let distance = 0
let timer = 0

enum Motor_Write {
    //% block="Left"
    Motor_Left,
    //% block="Right"
    Motor_Right
}

enum _Turn {
    //% block="Left"
    Left,
    //% block="Right"
    Right
}

enum _Spin {
    //% block="Left"
    Left,
    //% block="Right"
    Right
}

enum Servo_Write {
    //% block="P8"
    P8,
    //% block="P12"
    P12
}

enum Button_Status {
    //% block="Pressed"
    Pressed,
    //% block="Released"
    Released
}

enum ADC_Read {
    //% block="0"
    ADC0 = 0x84,
    //% block="1"
    ADC1 = 0xC4,
    //% block="2"
    ADC2 = 0x94,
    //% block="3"
    ADC3 = 0xD4,
    //% block="4"
    ADC4 = 0xA4,
    //% block="5"
    ADC5 = 0xE4,
    //% block="6"
    ADC6 = 0xB4,
    //% block="7"
    ADC7 = 0xF4
}

enum Forward_Direction {
    //% block="Forward"
    Forward,
    //% block="Backward"
    Backward
}

enum Find_Line {
    //% block="Left"
    Left,
    //% block="Center"
    Center,
    //% block="Right"
    Right
}

enum Turn_Line {
    //% block="Left"
    Left,
    //% block="Right"
    Right
}

enum Turn_Sensor {
    //% block="Center"
    Center,
    //% block="ADC1"
    ADC1,
    //% block="ADC2"
    ADC2,
    //% block="ADC3"
    ADC3,
    //% block="ADC4"
    ADC4
}

enum Turn_ADC {
    //% block="ADC0"
    ADC0 = 0,
    //% block="ADC1"
    ADC1 = 1,
    //% block="ADC2"
    ADC2 = 2,
    //% block="ADC3"
    ADC3 = 3,
    //% block="ADC4"
    ADC4 = 4,
    //% block="ADC5"
    ADC5 = 5
}

enum LED_Color {
    //% block=Red
    Red = 0xFF0000,
    //% block=Green
    Green = 0x00FF00,
    //% block=Blue
    Blue = 0x0000FF,
    //% block=Black
    Black = 0x000000
}

//% color="#51cb57" icon="\u2B99"
namespace PTKidsBITRobot {

    // ====== I2C SERVO (PCA9685) ======
    function initPCA(): void {
        let i2cData = pins.createBuffer(2)
        initI2C = true
        i2cData[0] = 0
        i2cData[1] = 0x10
        pins.i2cWriteBuffer(PCA, i2cData, false)

        i2cData[0] = 0xFE
        i2cData[1] = 101
        pins.i2cWriteBuffer(PCA, i2cData, false)

        i2cData[0] = 0
        i2cData[1] = 0x81
        pins.i2cWriteBuffer(PCA, i2cData, false)

        for (let servo = 0; servo < 16; servo++) {
            i2cData[0] = SERVOS + servo * 4 + 0
            i2cData[1] = 0x00
            pins.i2cWriteBuffer(PCA, i2cData, false);
            i2cData[0] = SERVOS + servo * 4 + 1
            i2cData[1] = 0x00
            pins.i2cWriteBuffer(PCA, i2cData, false);
        }
    }

    function setServoPCA(servo: number, angle: number): void {
        if (initI2C == false) initPCA()
        let i2cData = pins.createBuffer(2)
        let angle_input = pins.map(angle, 0, 180, -90, 90)
        angle = Math.max(Math.min(90, angle_input), -90)
        let stop = 369 + angle * 235 / 90
        i2cData[0] = SERVOS + servo * 4 + 2
        i2cData[1] = (stop & 0xff)
        pins.i2cWriteBuffer(PCA, i2cData, false)
        i2cData[0] = SERVOS + servo * 4 + 3
        i2cData[1] = (stop >> 8)
        pins.i2cWriteBuffer(PCA, i2cData, false)
    }

    // ====== MOTOR CONTROL ======
    //% group="Motor Control"
    //% block="Motor Stop"
    export function motorStop(): void {
        pins.digitalWritePin(DigitalPin.P13, 1)
        pins.analogWritePin(AnalogPin.P14, 0)
        pins.digitalWritePin(DigitalPin.P15, 1)
        pins.analogWritePin(AnalogPin.P16, 0)
    }

    //% group="Motor Control"
    //% block="Spin %_Spin|Speed %Speed"
    //% speed.min=0 speed.max=100
    //% speed.defl=50
    export function Spin(spin: _Spin, speed: number): void {
        if (spin == _Spin.Left) motorGo(-speed, speed)
        else if (spin == _Spin.Right) motorGo(speed, -speed)
    }

    //% group="Motor Control"
    //% block="Turn %_Turn|Speed %Speed"
    //% speed.min=0 speed.max=100
    //% speed.defl=50
    export function Turn(turn: _Turn, speed: number): void {
        if (turn == _Turn.Left) motorGo(0, speed)
        else if (turn == _Turn.Right) motorGo(speed, 0)
    }

    //% group="Motor Control"
    //% block="Motor Left %Motor_Left|Motor Right %Motor_Right"
    //% speed1.min=-100 speed1.max=100
    //% speed2.min=-100 speed2.max=100
    //% speed1.defl=50
    //% speed2.defl=50
    export function motorGo(speed1: number, speed2: number): void {
        speed1 = Math.max(-100, Math.min(100, speed1 + Compensate_Left))
        speed2 = Math.max(-100, Math.min(100, speed2 + Compensate_Right))
        speed1 = pins.map(speed1, -100, 100, -1023, 1023)
        speed2 = pins.map(speed2, -100, 100, -1023, 1023)

        if (speed1 < 0) {
            pins.digitalWritePin(DigitalPin.P13, 1)
            pins.analogWritePin(AnalogPin.P14, -speed1)
            pins.analogSetPeriod(AnalogPin.P14, 2000)
        } else {
            pins.digitalWritePin(DigitalPin.P13, 0)
            pins.analogWritePin(AnalogPin.P14, speed1)
            pins.analogSetPeriod(AnalogPin.P14, 2000)
        }

        if (speed2 < 0) {
            pins.digitalWritePin(DigitalPin.P15, 1)
            pins.analogWritePin(AnalogPin.P16, -speed2)
            pins.analogSetPeriod(AnalogPin.P16, 2000)
        } else {
            pins.digitalWritePin(DigitalPin.P15, 0)
            pins.analogWritePin(AnalogPin.P16, speed2)
            pins.analogSetPeriod(AnalogPin.P16, 2000)
        }
    }

    //% group="Motor Control"
    //% block="motorWrite %Motor_Write|Speed %Speed"
    //% speed.min=-100 speed.max=100
    //% speed.defl=50
    export function motorWrite(motor: Motor_Write, speed: number): void {
        if (motor == Motor_Write.Motor_Left) motorGo(speed, 0)
        else if (motor == Motor_Write.Motor_Right) motorGo(0, speed)
    }

    // ====== LED Indicator Control (BLE-safe no-ops) ======
    // Keep APIs but avoid ws2812b to stay Bluetooth-compatible.

    //% group="LED Indicator Control"
    //% block="LED Color Red %red|Green %green|Blue %blue|Brightness %brightness"
    //% red.min=0 red.max=255
    //% green.min=0 green.max=255
    //% blue.min=0 blue.max=255
    //% brightness.min=0 brightness.max=255
    //% red.defl=0
    //% green.defl=255
    //% blue.defl=0
    //% brightness.defl=100
    export function setColorRGB(red: number, green: number, blue: number, brightness: number): void {
        // BLE-safe: no external NeoPixel driving. Optionally flash onboard LED matrix.
        basic.clearScreen()
        if (brightness > 0) basic.showLeds(`
            . . . . .
            . # # # .
            . # . # .
            . # # # .
            . . . . .
        `)
    }

    //% group="LED Indicator Control"
    //% block="LED Color %colors|Brightness %brightness"
    //% brightness.min=0 brightness.max=255
    //% brightness.defl=100
    export function setColor(color: number, brightness: number): void {
        // BLE-safe no-op (optional tiny feedback)
        if (brightness > 0) basic.showIcon(IconNames.SmallSquare)
        else basic.clearScreen()
    }

    //% group="Sensor and ADC"
    //% block="Sensor Color 1 $color_1|Sensor Color 2 $color_2|Sensor Color 3 $color_3|Sensor Color 4 $color_4|Sensor Color 5 $color_5|Sensor Color 6 $color_6"
    export function setSensorColor(color_1: number, color_2: number, color_3: number, color_4: number, color_5: number, color_6: number): void {
        // BLE-safe no-op
    }

    //% group="Sensor and ADC"
    //% block="Sensor Color $color"
    export function setSensorColorAll(color: number): void {
        // BLE-safe no-op
    }

    // ====== SENSOR & ADC ======
    //% group="Sensor and ADC"
    //% block="ADCRead %ADC_Read"
    export function ADCRead(ADCRead: ADC_Read): number {
        if (Read_ADC_Version == false) {
            let i2cData = pins.createBuffer(1)
            i2cData[0] = 132
            if (pins.i2cWriteBuffer(0x49, i2cData, false) == 0) ADC_Version = 2
            else ADC_Version = 1
            Read_ADC_Version = true
        }

        if (ADC_Version == 1) {
            control.waitMicros(2000)
            pins.i2cWriteNumber(0x48, ADCRead, NumberFormat.UInt8LE, false)
            control.waitMicros(2000)
            return ADCRead = pins.i2cReadNumber(0x48, NumberFormat.UInt16BE, false)
        } else if (ADC_Version == 2) {
            control.waitMicros(2000)
            pins.i2cWriteNumber(0x49, ADCRead, NumberFormat.UInt8LE, false)
            control.waitMicros(2000)
            return ADCRead = pins.i2cReadNumber(0x49, NumberFormat.UInt8LE, false)
        } else {
            return 0
        }
    }

    //% group="Sensor and ADC"
    //% block="GETDistance"
    export function distanceRead(): number {
        let duration
        pins.digitalWritePin(DigitalPin.P1, 1)
        basic.pause(1)
        pins.digitalWritePin(DigitalPin.P1, 0)
        if (pins.digitalReadPin(DigitalPin.P2) == 0) {
            pins.digitalWritePin(DigitalPin.P1, 0)
            pins.digitalWritePin(DigitalPin.P1, 1)
            pins.digitalWritePin(DigitalPin.P1, 0)
            duration = pins.pulseIn(DigitalPin.P2, PulseValue.High, 500 * 58)
        } else {
            pins.digitalWritePin(DigitalPin.P1, 0)
            pins.digitalWritePin(DigitalPin.P1, 1)
            duration = pins.pulseIn(DigitalPin.P2, PulseValue.Low, 500 * 58)
        }
        let x = duration / 39
        if (x <= 0 || x > 400) return 400
        return Math.round(x)
    }

    // ====== LINE FOLLOWER ======
    function readAdcAll(): void {
        let ADC_PIN = [
            ADC_Read.ADC0, ADC_Read.ADC1, ADC_Read.ADC2, ADC_Read.ADC3,
            ADC_Read.ADC4, ADC_Read.ADC5, ADC_Read.ADC6, ADC_Read.ADC7
        ]
        for (let i = 0; i < Sensor_All_PIN.length; i++) {
            let Value_Sensor = 0;
            if (Line_Mode == 0) {
                Value_Sensor = pins.map(ADCRead(ADC_PIN[Sensor_All_PIN[i]]), Color_Line_All[i], Color_Background_All[i], 1000, 0)
            } else {
                Value_Sensor = pins.map(ADCRead(ADC_PIN[Sensor_All_PIN[i]]), Color_Background_All[i], Color_Line_All[i], 1000, 0)
            }
            if (Value_Sensor < 0) Value_Sensor = 0
            else if (Value_Sensor > 1000) Value_Sensor = 1000
            Line_All[i] = Value_Sensor
        }
    }

    //% group="Servo Control"
    //% block="Servo %Servo_Write|Degree %Degree"
    //% degree.min=0 degree.max=180
    export function servoWrite(servo: Servo_Write, degree: number): void {
        if (Read_Servo_Version == false) {
            let i2cData = pins.createBuffer(2)
            i2cData[0] = 0
            i2cData[1] = 16
            if (pins.i2cWriteBuffer(64, i2cData, false) == 0) Servo_Version = 2
            else Servo_Version = 1
            Read_Servo_Version = true
        }
        if (servo == Servo_Write.P8) {
            if (Servo_Version == 1) {
                Servo_8_Enable = 1
                Servo_8_Degree = degree
                pins.servoWritePin(AnalogPin.P8, Servo_8_Degree)
                pins.servoWritePin(AnalogPin.P12, Servo_12_Degree)
                basic.pause(100)
            } else {
                setServoPCA(1, degree)
            }
        } else if (servo == Servo_Write.P12) {
            if (Servo_Version == 1) {
                Servo_12_Enable = 1
                Servo_12_Degree = degree
                pins.servoWritePin(AnalogPin.P8, Servo_8_Degree)
                pins.servoWritePin(AnalogPin.P12, Servo_12_Degree)
                basic.pause(100)
            } else {
                setServoPCA(0, degree)
            }
        }
    }

    //% group="Line Follower"
    //% block="TurnLINE %turn|Speed\n %speed|Sensor %sensor|Fast Time %time"
    //% speed.min=0 speed.max=100
    //% time.shadow="timePicker"
    //% time.defl=200
    export function TurnLINE(turn: Turn_Line, speed: number, sensor: number, time: number) {
        let error = 0
        let motor_speed = 0
        let motor_slow = 20
        let timer = control.millis()
        let _position = 0

        if (sensor == 0) _position = 0
        else if (sensor == 1) _position = 3000
        else if (sensor == 2) _position = 2400
        else if (sensor == 3) _position = 1800
        else if (sensor == 4) _position = 1200
        else if (sensor == 5) _position = 600

        while (true) {
            error = timer - (control.millis() - time)
            motor_speed = error
            if (motor_speed > speed) motor_speed = speed
            else if (motor_speed < 0) motor_speed = motor_slow

            if (turn == Turn_Line.Left) {
                if (GETPosition() <= 500) break
                motorGo(-motor_speed, motor_speed)
            } else if (turn == Turn_Line.Right) {
                if (GETPosition() >= 2500) break
                motorGo(motor_speed, -motor_speed)
            }
        }
        while (true) {
            if (GETPosition() >= _position - 300 && GETPosition() <= _position + 300) {
                motorStop()
                break
            }
            error = timer - (control.millis() - time)
            motor_speed = error
            if (motor_speed > speed) motor_speed = speed
            else if (motor_speed < 0) motor_speed = motor_slow

            if (turn == Turn_Line.Left) motorGo(-motor_speed, motor_speed)
            else if (turn == Turn_Line.Right) motorGo(motor_speed, -motor_speed)
        }
    }

    //% group="Line Follower"
    //% block="Time %time|Min Speed %base_speed|Max Speed %max_speed|KP %kp|KD %kd"
    //% min_speed.min=0 min_speed.max=100
    //% max_speed.min=0 max_speed.max=100
    //% time.shadow="timePicker"
    //% time.defl=200
    export function ForwardTIME(time: number, min_speed: number, max_speed: number, kp: number, kd: number) {
        let timer = control.millis()
        while (control.millis() - timer < time) {
            error = GETPosition() - (((Num_Sensor - 1) * 1000) / 2)
            P = error
            D = error - previous_error
            PD_Value = (kp * P) + (kd * D)
            previous_error = error

            left_motor_speed = min_speed - PD_Value
            right_motor_speed = min_speed + PD_Value

            if (left_motor_speed > max_speed) left_motor_speed = max_speed
            else if (left_motor_speed < -max_speed) left_motor_speed = -max_speed
            if (right_motor_speed > max_speed) right_motor_speed = max_speed
            else if (right_motor_speed < -max_speed) right_motor_speed = -max_speed

            motorGo(left_motor_speed, right_motor_speed)
        }
        motorStop()
    }

    //% group="Line Follower"
    //% block="Find %sensor|Min Speed %base_speed|Max Speed %max_speed|KP %kp|KD %kd"
    //% min_speed.defl=30
    //% max_speed.defl=100
    //% kp.defl=0.01
    //% min_speed.min=0 min_speed.max=100
    //% max_speed.min=0 max_speed.max=100
    export function ForwardLINECustom(sensor: string, min_speed: number, max_speed: number, kp: number, kd: number) {
        let set_sensor = [0, 0, 0, 0, 0, 0]
        let on_line_setpoint = 800
        let out_line_setpoint = 100
        let sensor_interesting = 6
        for (let i = 0; i < Sensor_All_PIN.length; i ++) {
            if (sensor.charAt(i) == '-') {
                sensor_interesting -= 1
                set_sensor[i] = 2
            } else {
                set_sensor[i] = parseFloat(sensor.charAt(i))
            }
        }

        while (true) {
            let found = 0
            readAdcAll()
            for (let i = 0; i < Sensor_All_PIN.length; i++) {
                if (set_sensor[i] == 0) {
                    if (Line_All[i] < out_line_setpoint) found += 1
                } else if (set_sensor[i] == 1) {
                    if (Line_All[i] > on_line_setpoint) found += 1
                }
            }
            if (found >= sensor_interesting) {
                motorStop()
                break
            }

            error = GETPosition() - (((Num_Sensor - 1) * 1000) / 2)
            P = error
            D = error - previous_error
            PD_Value = (kp * P) + (kd * D)
            previous_error = error

            left_motor_speed = min_speed - PD_Value
            right_motor_speed = min_speed + PD_Value

            if (left_motor_speed > max_speed) left_motor_speed = max_speed
            else if (left_motor_speed < -max_speed) left_motor_speed = -max_speed
            if (right_motor_speed > max_speed) right_motor_speed = max_speed
            else if (right_motor_speed < -max_speed) right_motor_speed = -max_speed

            motorGo(left_motor_speed, right_motor_speed)
        }
    }

    //% group="Line Follower"
    //% block="Find %Find_Line|Count %count|Min Speed %base_speed|Max Speed %max_speed|KP %kp|KD %kd"
    //% find.defl=Find_Line.Center
    //% count.defl=1
    //% min_speed.defl=30
    //% max_speed.defl=100
    //% kp.defl=0.01
    //% min_speed.min=0 min_speed.max=100
    //% max_speed.min=0 max_speed.max=100
    export function ForwardLINECount(find: Find_Line, count: number, min_speed: number, max_speed: number, kp: number, kd: number) {
        let on_line_setpoint = 800
        let _count = 0
        while (true) {
            let found = 0
            readAdcAll()
            for (let i = 0; i < Sensor_All_PIN.length; i++) if (Line_All[i] > on_line_setpoint) found += 1

            if (find == Find_Line.Center) {
                if (found >= 5) {
                    _count += 1
                    if (_count >= count) { motorStop(); break }
                    else {
                        while (true) {
                            let found2 = 0
                            readAdcAll()
                            for (let i = 0; i < Sensor_All_PIN.length; i++) if (Line_All[i] > on_line_setpoint) found2 += 1
                            if (found2 >= 5) motorGo(min_speed, min_speed)
                            else break
                        }
                    }
                }
            } else if (find == Find_Line.Left) {
                if (Line_All[0] > on_line_setpoint && Line_All[1] > on_line_setpoint && Line_All[2] > on_line_setpoint && Line_All[5] < 500) {
                    _count += 1
                    if (_count >= count) { motorStop(); break }
                    else {
                        while (true) {
                            readAdcAll()
                            if (Line_All[0] > on_line_setpoint && Line_All[1] > on_line_setpoint && Line_All[2] > on_line_setpoint && Line_All[5] < 500) motorGo(min_speed, min_speed)
                            else break
                        }
                    }
                }
            } else if (find == Find_Line.Right) {
                if (Line_All[3] > on_line_setpoint && Line_All[4] > on_line_setpoint && Line_All[5] > on_line_setpoint && Line_All[0] < 500) {
                    _count += 1
                    if (_count >= count) { motorStop(); break }
                    else {
                        while (true) {
                            readAdcAll()
                            if (Line_All[3] > on_line_setpoint && Line_All[4] > on_line_setpoint && Line_All[5] > on_line_setpoint && Line_All[0] < 500) motorGo(min_speed, min_speed)
                            else break
                        }
                    }
                }
            }

            error = GETPosition() - (((Num_Sensor - 1) * 1000) / 2)
            P = error
            D = error - previous_error
            PD_Value = (kp * P) + (kd * D)
            previous_error = error

            left_motor_speed = min_speed - PD_Value
            right_motor_speed = min_speed + PD_Value
            if (left_motor_speed > max_speed) left_motor_speed = max_speed
            else if (left_motor_speed < -max_speed) left_motor_speed = -max_speed
            if (right_motor_speed > max_speed) right_motor_speed = max_speed
            else if (right_motor_speed < -max_speed) right_motor_speed = -max_speed

            motorGo(left_motor_speed, right_motor_speed)
        }
    }

    //% group="Line Follower"
    //% block="Find %Find_Line|Min Speed %base_speed|Max Speed %max_speed|KP %kp|KD %kd"
    //% find.defl=Find_Line.Center
    //% min_speed.defl=30
    //% max_speed.defl=100
    //% kp.defl=0.01
    //% min_speed.min=0 min_speed.max=100
    //% max_speed.min=0 max_speed.max=100
    export function ForwardLINE(find: Find_Line, min_speed: number, max_speed: number, kp: number, kd: number) {
        let on_line_setpoint = 500
        while (true) {
            let found = 0
            readAdcAll()
            for (let i = 0; i < Sensor_All_PIN.length; i++) if (Line_All[i] > on_line_setpoint) found += 1
            if (found >= 5) motorGo(min_speed, min_speed)
            else { motorGo(50, 50); basic.pause(20); break }
        }
        while (true) {
            let found = 0
            readAdcAll()
            for (let i = 0; i < Sensor_All_PIN.length; i++) if (Line_All[i] > on_line_setpoint) found += 1
            if (find == Find_Line.Center) {
                if (found >= 5) { motorGo(50, 50); basic.pause(20); motorStop(); break }
            } else if (find == Find_Line.Left) {
                if (Line_All[0] > on_line_setpoint && Line_All[1] > on_line_setpoint && Line_All[2] > on_line_setpoint && Line_All[5] < 500) { motorGo(50, 50); basic.pause(20); motorStop(); break }
            } else if (find == Find_Line.Right) {
                if (Line_All[3] > on_line_setpoint && Line_All[4] > on_line_setpoint && Line_All[5] > on_line_setpoint && Line_All[0] < 500) { motorGo(50, 50); basic.pause(20); motorStop(); break }
            }

            error = GETPosition() - (((Num_Sensor - 1) * 1000) / 2)
            P = error
            D = error - previous_error
            PD_Value = (kp * P) + (kd * D)
            previous_error = error

            left_motor_speed = min_speed - PD_Value
            right_motor_speed = min_speed + PD_Value
            if (left_motor_speed > max_speed) left_motor_speed = max_speed
            else if (left_motor_speed < -max_speed) left_motor_speed = -max_speed
            if (right_motor_speed > max_speed) right_motor_speed = max_speed
            else if (right_motor_speed < -max_speed) right_motor_speed = -max_speed
            motorGo(left_motor_speed, right_motor_speed)
        }
    }

    //% group="Line Follower"
    //% block="GETPosition"
    export function GETPosition() {
        let ADC_PIN = [
            ADC_Read.ADC0, ADC_Read.ADC1, ADC_Read.ADC2, ADC_Read.ADC3,
            ADC_Read.ADC4, ADC_Read.ADC5, ADC_Read.ADC6, ADC_Read.ADC7
        ]
        let Average = 0
        let Sum_Value = 0
        let ON_Line = 0

        for (let i = 0; i < Num_Sensor; i++) {
            let Value_Sensor = 0;
            if (Line_Mode == 0) {
                Value_Sensor = pins.map(ADCRead(ADC_PIN[Sensor_PIN[i]]), Color_Line[i], Color_Background[i], 1000, 0)
            } else {
                Value_Sensor = pins.map(ADCRead(ADC_PIN[Sensor_PIN[i]]), Color_Background[i], Color_Line[i], 1000, 0)
            }
            if (Value_Sensor < 0) Value_Sensor = 0
            else if (Value_Sensor > 1000) Value_Sensor = 1000

            if (ADC_Version == 1) {
                if (Value_Sensor > 200) ON_Line = 1
            }
            else if (ADC_Version == 2) {
                if (Value_Sensor > 500) ON_Line = 1
            }

            Average += Value_Sensor * (i * 1000)
            Sum_Value += Value_Sensor
        }
        if (ON_Line == 0) {
            if (Last_Position < (Num_Sensor - 1) * 1000 / 2) return (Num_Sensor - 1) * 1000
            else return 0
        }
        Last_Position = Average / Sum_Value;
        return Math.round(((Num_Sensor - 1) * 1000) - Last_Position)
    }

    //% group="Line Follower"
    //% block="SETColorLine $line|Ground $ground"
    export function ValueSensorSET(line: number[], ground: number[]): void {
        if (Read_ADC_Version == false) {
            let i2cData = pins.createBuffer(1)
            i2cData[0] = 132
            if (pins.i2cWriteBuffer(0x49, i2cData, false) == 0) ADC_Version = 2
            else ADC_Version = 1
            Read_ADC_Version = true
        }
        Color_Line_Left[0] = line[0]
        Color_Line[0] = line[1]
        Color_Line[1] = line[2]
        Color_Line[2] = line[3]
        Color_Line[3] = line[4]
        Color_Line_Right[5] = line[5]
        Color_Background_Left[0] = ground[0]
        Color_Background[0] = ground[1]
        Color_Background[1] = ground[2]
        Color_Background[2] = ground[3]
        Color_Background[3] = ground[4]
        Color_Background_Right[5] = ground[5]
        Color_Line_All = line
        Color_Background_All = ground
    }

    //% group="Line Follower"
    //% block="SensorCalibrate"
    export function SensorCalibrate(): void {
        serial.writeLine("")
        let ADC_PIN = [
            ADC_Read.ADC0, ADC_Read.ADC1, ADC_Read.ADC2, ADC_Read.ADC3,
            ADC_Read.ADC4, ADC_Read.ADC5, ADC_Read.ADC6, ADC_Read.ADC7
        ]
        let _Sensor_PIN = [0, 1, 2, 3, 4, 5]
        let _Num_Sensor = _Sensor_PIN.length
        let Line_Cal = [0, 0, 0, 0, 0, 0, 0, 0]
        let Background_Cal = [0, 0, 0, 0, 0, 0, 0, 0]

        basic.pause(300)
        pins.digitalWritePin(DigitalPin.P8, 0)
        pins.digitalWritePin(DigitalPin.P12, 0)
        basic.pause(100)
        music.playTone(587, music.beat(BeatFraction.Quarter))
        music.playTone(784, music.beat(BeatFraction.Quarter))
        basic.pause(200)

        while (!input.buttonIsPressed(Button.A));
        pins.digitalWritePin(DigitalPin.P8, 0)
        pins.digitalWritePin(DigitalPin.P12, 0)
        basic.pause(100)
        music.playTone(784, music.beat(BeatFraction.Quarter))
        basic.pause(200)

        for (let i = 0; i < 20; i++) {
            for (let j = 0; j < _Num_Sensor; j++) {
                Line_Cal[j] += ADCRead(ADC_PIN[_Sensor_PIN[j]])
            }
            basic.pause(50)
        }
        for (let i = 0; i < _Num_Sensor; i++) {
            Line_Cal[i] = Line_Cal[i] / 20
            for (let j = 0; j < 8; j++) {
                if (_Sensor_PIN[i] == j) Line_HIGH[j] = Line_Cal[i]
            }
        }
        pins.digitalWritePin(DigitalPin.P8, 0)
        pins.digitalWritePin(DigitalPin.P12, 0)
        basic.pause(100)
        music.playTone(784, music.beat(BeatFraction.Quarter))
        basic.pause(200)

        while (!input.buttonIsPressed(Button.A));
        pins.digitalWritePin(DigitalPin.P8, 0)
        pins.digitalWritePin(DigitalPin.P12, 0)
        basic.pause(100)
        music.playTone(784, music.beat(BeatFraction.Quarter))
        basic.pause(200)

        for (let i = 0; i < 20; i++) {
            for (let j = 0; j < _Num_Sensor; j++) {
                Background_Cal[j] += ADCRead(ADC_PIN[_Sensor_PIN[j]])
            }
            basic.pause(50)
        }
        for (let i = 0; i < _Num_Sensor; i++) {
            Background_Cal[i] = Background_Cal[i] / 20
            for (let j = 0; j < 8; j++) {
                if (_Sensor_PIN[i] == j) Line_LOW[j] = Background_Cal[i]
            }
        }

        for (let i = 0; i < Num_Sensor; i++) {
            Color_Line[i] = Line_HIGH[Sensor_PIN[i]]
            Color_Background[i] = Line_LOW[Sensor_PIN[i]]
        }
        for (let i = 0; i < Sensor_Left.length; i++) {
            Color_Line_Left[i] = Line_HIGH[Sensor_Left[i]]
            Color_Background_Left[i] = Line_LOW[Sensor_Left[i]]
        }
        for (let i = 0; i < Sensor_Right.length; i++) {
            Color_Line_Right[i] = Line_HIGH[Sensor_Right[i]]
            Color_Background_Right[i] = Line_LOW[Sensor_Right[i]]
        }

        Color_Line_All = [Color_Line_Left[0], Color_Line[0], Color_Line[1], Color_Line[2], Color_Line[3], Color_Line_Right[0]]
        Color_Background_All = [Color_Background_Left[0], Color_Background[0], Color_Background[1], Color_Background[2], Color_Background[3], Color_Background_Right[0]]

        pins.digitalWritePin(DigitalPin.P8, 0)
        pins.digitalWritePin(DigitalPin.P12, 0)
        basic.pause(100)
        music.playTone(784, music.beat(BeatFraction.Quarter))
        music.playTone(587, music.beat(BeatFraction.Quarter))
        if (Servo_8_Enable == 1) pins.servoWritePin(AnalogPin.P8, Servo_8_Degree)
        if (Servo_12_Enable == 1) pins.servoWritePin(AnalogPin.P12, Servo_12_Degree)
        basic.pause(200)

        serial.writeLine("------------------------------------")
        let sensor_value = "Color Line:"
        for (let i = 0; i < Color_Line_All.length; i++) sensor_value += " " + Math.round(Color_Line_All[i])
        serial.writeLine("" + sensor_value)

        sensor_value = "Color Ground:"
        for (let i = 0; i < Color_Background_All.length; i++) sensor_value += " " + Math.round(Color_Background_All[i])
        serial.writeLine("" + sensor_value)
    }
}
