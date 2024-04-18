package mks42d

import (
	"bytes"
	"encoding/binary"
	"errors"
	"fmt"
	"strconv"
	"sync"
	"time"

	"github.com/tarm/serial"
)

/*
This module references MKS SERVO42D/57D_RS485 V1.0.4 User Manual
Work Mode must be SR_vFOC
*/

var S42DDriverConfig = serial.Config{
	Name:        "/dev/ttyUSB1",
	Baud:        115200,
	ReadTimeout: 3 * time.Millisecond, // Adjust as needed
}

type S42DDriver struct {
	// config
	serialMutex  sync.Mutex // used to police the serial communication
	SerialConfig *serial.Config
	port         *serial.Port

	EncoderAddition   uint // encoder value +/- for a full rotation
	ScrewPitch        uint // mm
	StepsPerRotation  uint // usually 200 without microsteps
	StepsPerMm        uint // steps_per_rotation / screw_pitch
	distanceTravelled uint // in mm

	// top and bottom maxes
	zeroTopSet bool // true/false we set the top of the system as 0 axis
	endStop    int  // in encoder value
}

////
// exported functions
////

func (driver *S42DDriver) Connect() error {
	if driver.SerialConfig.Name == "" {
		return errors.New("serial config must be set before connecting")
	}

	port, err := connectToDriver(driver.SerialConfig)
	if err != nil {
		return err
	}
	driver.port = port
	return nil
}

func (driver *S42DDriver) Close() error {
	err := driver.port.Close()
	if err != nil {
		return err
	}

	return nil
}

func (driver *S42DDriver) MoveOneRotation(dir Direction, speed uint16) error {
	tryUnlockProtectedState(driver)

	fmt.Printf("moving %d steps\n", driver.StepsPerRotation)

	err := startMotorPositionMode(driver, dir, speed, uint32(driver.StepsPerRotation), 5000)
	if err != nil {
		return err
	}

	return nil
}

// go to top of system. Note this resets all pulse counting
func (driver *S42DDriver) GoHome() error {
	lockedChan := make(chan int, 3)
	defer close(lockedChan)

	tryUnlockProtectedState(driver)

	const speed = 200
	err := startMotorSpeedMode(driver, CLOCKWISE, speed)
	if err != nil {
		fmt.Println(err.Error())
		return errors.New("we failed to move")
	}

	// goroutine to continuously read the motor locked state
	go func() {
		for {
			status, err := readMotorLockedState(driver)
			if err != nil {
				fmt.Println(err.Error())
				lockedChan <- -1 // -1 means error
				return
			}

			lockedChan <- status
			if status == 1 {
				return
			}
		}
	}()

	for {
		select {
		case message := <-lockedChan:
			if message == 0 { // unlocked state, leaving this case in for explicityness
				continue
			}
			if message == 1 { // locked state
				err = tryUnlockProtectedState(driver)
				if err != nil {
					fmt.Println(err.Error())
					return errors.New("could not unlock shaft at the top")
				}

				err = setCurrentAxisToZero(driver)
				if err != nil {
					fmt.Println(err.Error())
					return errors.New("error setting top of shaft to 0")
				}

				driver.zeroTopSet = true
				return nil
			}
			if message == -1 { // error state
				err = cleanSpeedParameter(driver)
				if err != nil {
					emergencyStopDriver(driver)
				}

				fmt.Println(err.Error())
				return errors.New("error while searching for top of shaft")
			}
		}
	}
}

func (driver *S42DDriver) FindMaxBottom() (int, error) {
	lockedChan := make(chan int, 3)
	defer close(lockedChan)

	tryUnlockProtectedState(driver)

	const speed = 10
	err := startMotorSpeedMode(driver, COUNTERCLOCKWISE, speed)
	if err != nil {
		fmt.Println(err.Error())
		return 0, errors.New("we failed to move")
	}

	// goroutine to continuously read the motor locked state
	go func() {
		for {
			status, err := readMotorLockedState(driver)
			if err != nil {
				fmt.Println(err.Error())
				lockedChan <- -1 // -1 means error
				return
			}

			lockedChan <- status
			if status == 1 {
				return
			}
		}
	}()

	for {
		select {
		case message := <-lockedChan:
			if message == 0 { // unlocked state, leaving this case in for explicityness
				continue
			}
			if message == 1 { // locked state
				err = tryUnlockProtectedState(driver)
				if err != nil {
					fmt.Println(err.Error())
					return 0, errors.New("could not unlock shaft at the top")
				}

				value, err := readEncoderAdditionValue(driver)
				if err != nil {
					return 0, err
				}

				driver.endStop = value
				return value, nil
			}
			if message == -1 { // error state
				err = cleanSpeedParameter(driver)
				if err != nil {
					emergencyStopDriver(driver)
				}

				fmt.Println(err.Error())
				return 0, errors.New("error while searching for top of shaft")
			}
		}
	}
}

func (driver *S42DDriver) StartGoingDown(speed uint16) error {
	err := tryUnlockProtectedState(driver)
	if err != nil {
		fmt.Println(err.Error())
		return err
	}

	err = startMotorSpeedMode(driver, COUNTERCLOCKWISE, speed)
	if err != nil {
		return err
	}

	return nil
}

func (driver *S42DDriver) StartGoingUp(speed uint16) error {
	err := tryUnlockProtectedState(driver)
	if err != nil {
		fmt.Println(err.Error())
		return err
	}

	err = startMotorSpeedMode(driver, CLOCKWISE, speed)
	if err != nil {
		return err
	}

	return nil
}

// this assumes we traveled in one direction from a 0 axis
func (driver *S42DDriver) ReadDistance() (float64, error) {
	encoder, err := readEncoderAdditionValue(driver)
	if err != nil {
		return 0, err
	}

	// we just need the abs, so turn negative to positive
	if encoder < 0 {
		encoder = -encoder
	}

	numberOfRotations := float64(encoder) / float64(driver.EncoderAddition)
	totalSteps := driver.StepsPerRotation * uint(numberOfRotations)
	distanceTraveled := float64(totalSteps) / float64(driver.StepsPerMm)

	return distanceTraveled, nil
}

func (driver *S42DDriver) ResetCounters() error {
	err := setCurrentAxisToZero(driver)
	if err != nil {
		fmt.Println(err.Error())
		return errors.New("error setting top of shaft to 0")
	}

	return nil
}

////
// private functions
////

func tryUnlockProtectedState(driver *S42DDriver) error {
	status, err := readMotorLockedState(driver)
	if err != nil {
		fmt.Println(err.Error())
		return err
	}

	if status == 1 {
		err = releaseMotorLockedState(driver)
		if err != nil {
			fmt.Println(err.Error())
			return err
		}
	}

	return nil
}

func readCurrentPosition(driver *S42DDriver, servoChan chan<- ServoMessage) {
	value, err := readEncoderAdditionValue(driver)
	if err != nil {
		servoChan <- ServoMessage{
			Type:    ServoError,
			Message: err.Error(),
		}
	}

	servoChan <- ServoMessage{
		Type: ServoEncoderMessage,
		Data: value,
	}

	value2, err := readNumberOfPulses(driver)
	if err != nil {
		servoChan <- ServoMessage{
			Type:    ServoPulseCountMessage,
			Message: err.Error(),
		}
	}

	servoChan <- ServoMessage{
		Type: ServoEncoderMessage,
		Data: value2,
	}
}

func (driver *S42DDriver) sendCommand(command DriverCommand) ([]byte, error) {
	driver.serialMutex.Lock()
	defer driver.serialMutex.Unlock()

	err := writeToDriver(driver.port, command.payload, true)
	if err != nil {
		fmt.Println(err.Error())
		return nil, fmt.Errorf("error writing payload for command: %s", command.name)
	}

	if command.waitForResponseMs > 0 {
		time.Sleep(time.Duration(command.waitForResponseMs) * time.Millisecond)
	}

	numberOfBytesReceived, buffer, err := readFromDriver(driver.port, make([]byte, command.expectedResponseSize), true)
	if err != nil {
		fmt.Println(err.Error())
		return nil, fmt.Errorf("could not read return value from sending command: %s", command.name)
	}

	if command.expectedResponseSize != numberOfBytesReceived {
		return nil, fmt.Errorf("unexpected size of response from command: %s", command.name)
	}

	return buffer, nil
}

func (driver *S42DDriver) sendCommandAndWaitForResponse(command DriverCommand, waitForResponse []byte, timeoutMs int64) ([]byte, error) {
	driver.serialMutex.Lock()
	defer driver.serialMutex.Unlock()

	err := writeToDriver(driver.port, command.payload, false)
	if err != nil {
		fmt.Println(err.Error())
		return nil, fmt.Errorf("error writing payload for command: %s", command.name)
	}

	timestart := msSinceEpoch()
	for (msSinceEpoch() - timestart) < timeoutMs {
		numberOfBytesReceived, buffer, err := readFromDriver(driver.port, make([]byte, command.expectedResponseSize), false)
		if err != nil {
			if err.Error() == "EOF" {
				continue
			}
			fmt.Println(err.Error())
			return nil, fmt.Errorf("could not read return value from sending command: %s", command.name)
		}

		if command.expectedResponseSize != numberOfBytesReceived {
			return nil, fmt.Errorf("unexpected size of response from command: %s", command.name)
		}

		// fmt.Printf("expected: %s\n", ByteArrayToString(waitForResponse))
		// fmt.Printf("recieved: %s\n", ByteArrayToString(buffer))
		if bytes.Equal(buffer, waitForResponse) {
			return buffer, nil
		}
	}

	return nil, errors.New("expected response was never recieved")
}

const MAX_MOTOR_SPEED = 3000
const MAX_MOTOR_ACCELERATION = 255
const MOTOR_TOTAL_STEPS = 200 // 200 for 1.8

type Direction int

const (
	COUNTERCLOCKWISE Direction = iota // COUNTERCLOCKWISE is assigned 0 (DOWN)
	CLOCKWISE                         // CLOCKWISE is assigned 1 (UP)
)

const DOWNLINK_HEAD byte = 0xFA
const UPLINK_HEAD byte = 0xFB
const SLAVE_ADDRESS byte = 0x01
const DEFAULT_ACCELERATION uint8 = 0

/*
Byte 4: The highest bit indicates the direction, the lower 4 bits
and byte 5 together indicate the speed
The parameter description is as follows:

	ddr: slave address, the value range is 0-255
	dir: the value range is 0/1 (CCW/CW)
	speed: the speed, the value range is 0-3000
	acc: the acceleration, the value range is 0-255

for example：

	Send “FA 01 F6 01 40 02 34”, the motor rotates forward at acc=2, speed=320RPM
	Send “FA 01 F6 81 40 02 B4”, the motor reverses at acc=2, speed=320RPM
*/
func createSpeedModePayload(direction Direction, speed uint16, acceleration uint8) ([]byte, error) {
	var payload []byte
	const SPEED_FUNCTION_BYTE byte = 0xF6

	if speed > MAX_MOTOR_SPEED {
		return nil, fmt.Errorf("speed was outside 0-3000 constraint of the motor: %d", speed)
	}

	if acceleration > MAX_MOTOR_ACCELERATION {
		return nil, fmt.Errorf("acceleration was outside 0-255 limit of the motor: %d", acceleration)
	}

	const downByte = 0x80 // 1000 0000 in binary, clockwise CW
	const upByte = 0x00   // 0000 0000, counter clockwise CCW
	var directionByte byte
	if direction == 0 {
		directionByte = upByte
	} else {
		directionByte = downByte
	}

	byteAcceleration := byte(acceleration)

	if speed <= 255 {
		// the speed can fit into byte 5 alone, so we leave the last 4 bits of byte 4 all 0
		byteSpeed := byte(speed)
		payload = []byte{DOWNLINK_HEAD, SLAVE_ADDRESS, SPEED_FUNCTION_BYTE, directionByte, byteSpeed, byteAcceleration}
	} else {
		// we need to use the last 4 bits of byte 4 to communicate the speed
		speedBinaryString := strconv.FormatInt(int64(speed), 2)
		byte5AsString := speedBinaryString[len(speedBinaryString)-8:]
		byte5AsInt, err := strconv.ParseUint(byte5AsString, 2, 8)
		if err != nil {
			fmt.Printf("could not convert string %s to int\n", byte5AsString)
			return nil, err
		}

		byte5 := byte(byte5AsInt)

		// figure out the last 4 bytes of the direction byte (byte 4)
		allButLast8 := speedBinaryString[:len(speedBinaryString)-8]
		speedRemainder, err := strconv.ParseUint(allButLast8, 2, 8)
		if err != nil {
			fmt.Printf("could not convert string %s to int\n", byte5AsString)
			return nil, err
		}
		if speedRemainder > 15 {
			return nil, fmt.Errorf("speed remainder bits cannot exceed 15, it won't fit in byte4")
		}

		// set the last 4 bits to intValue.
		directionByte |= byte(speedRemainder & 0x0F) // ensure speedRemainder is masked to only 4 bits just in case

		payload = []byte{DOWNLINK_HEAD, SLAVE_ADDRESS, SPEED_FUNCTION_BYTE, directionByte, byte5, byteAcceleration}
	}

	return payload, nil
}

func createPositionMode1Payload(direction Direction, speed uint16, acceleration uint8, numberOfPulses uint32) ([]byte, error) {
	var payload []byte
	const POSITION_MODEL_FUNCTION byte = 0xFD

	const downByte = 0x80 // 1000 0000 in binary, clockwise CW
	const upByte = 0x00   // 0000 0000, counter clockwise CCW
	var directionByte byte
	if direction == 0 {
		directionByte = upByte
	} else {
		directionByte = downByte
	}

	byteAcceleration := byte(acceleration)
	numberOfPulseBytes := make([]byte, 4)
	binary.BigEndian.PutUint32(numberOfPulseBytes, numberOfPulses)

	if speed <= 255 {
		// the speed can fit into byte 5 alone, so we leave the last 4 bits of byte 4 all 0
		byteSpeed := byte(speed)
		payload = []byte{DOWNLINK_HEAD, SLAVE_ADDRESS, POSITION_MODEL_FUNCTION, directionByte, byteSpeed, byteAcceleration}
		payload = append(payload, numberOfPulseBytes...)
	} else {
		// we need to use the last 4 bits of byte 4 to communicate the speed
		speedBinaryString := strconv.FormatInt(int64(speed), 2)
		byte5AsString := speedBinaryString[len(speedBinaryString)-8:]
		byte5AsInt, err := strconv.ParseUint(byte5AsString, 2, 8)
		if err != nil {
			fmt.Printf("could not convert string %s to int\n", byte5AsString)
			return nil, err
		}

		byte5 := byte(byte5AsInt)

		// figure out the last 4 bytes of the direction byte (byte 4)
		allButLast8 := speedBinaryString[:len(speedBinaryString)-8]
		speedRemainder, err := strconv.ParseUint(allButLast8, 2, 8)
		if err != nil {
			fmt.Printf("could not convert string %s to int\n", byte5AsString)
			return nil, err
		}
		if speedRemainder > 15 {
			return nil, fmt.Errorf("speed remainder bits cannot exceed 15, it won't fit in byte4")
		}

		// set the last 4 bits to intValue.
		directionByte |= byte(speedRemainder & 0x0F) // ensure speedRemainder is masked to only 4 bits just in case

		payload = []byte{DOWNLINK_HEAD, SLAVE_ADDRESS, POSITION_MODEL_FUNCTION, directionByte, byte5, byteAcceleration}
		payload = append(payload, numberOfPulseBytes...)
	}

	return payload, nil
}

func readEncoderCarryValue(driver *S42DDriver) (int32, uint16, error) {
	command := DriverCommand{
		name:                 "read_encoder_carry_value",
		payload:              []byte{DOWNLINK_HEAD, SLAVE_ADDRESS, 0x30},
		expectedResponseSize: 10,
	}

	response, err := driver.sendCommand(command)
	if err != nil {
		return 0, 0, err
	}

	// var head = buffer[0]
	// var slaveAddress = buffer[1]
	// var function = buffer[2]
	var carryValue = response[3:7]
	fmt.Printf("Carry Value: %s\n", ByteArrayToString(carryValue))
	carryDestination := int32(binary.BigEndian.Uint32(carryValue))

	var encoderValue = response[7:9]
	fmt.Printf("Encoder Value: %s\n", ByteArrayToString(encoderValue))
	valueDestination := binary.BigEndian.Uint16(encoderValue)
	// var crc = buffer[9]

	return carryDestination, valueDestination, nil
}

func readEncoderAdditionValue(driver *S42DDriver) (int, error) {
	command := DriverCommand{
		name:                 "read_encoder_value_addition",
		payload:              []byte{DOWNLINK_HEAD, SLAVE_ADDRESS, 0x31},
		expectedResponseSize: 10,
	}

	response, err := driver.sendCommand(command)
	if err != nil {
		return 0, err
	}

	// var head = buffer[0]
	// var slaveAddress = buffer[1]
	// var function = buffer[2]
	var value = response[3:9]
	// var crc = buffer[9]

	/*
		Golang does not have an int48 which this call returns, so
		we must do all this fun stuff next with bits.
	*/

	var signedInt int64

	// check if the most significant bit is set of this big endianess
	// bitwise AND between the first byte and 0x80 (1000 0000) and checking if
	// the result is 0x80, if so, the bit is set and its a negative number
	isNegative := value[0]&0x80 == 0x80

	// construct the int64, we technically only have enough bytes for
	// int48, so we need to build it ourselves (yay)
	// each loop we shift the bytes of the destination int64 over by 8 making room for the new byte
	// then the new byte is OR'd into the lowest 8 bits of the destination
	for _, b := range value {
		signedInt = (signedInt << 8) | int64(b)
	}

	// if its negative, flip those bits, also some sign extension
	if isNegative {
		signedInt |= ^int64(0xFFFFFFFFFFFF)
	}

	fmt.Printf("Encoder (addition) Value: %s\n", ByteArrayToString(value))
	fmt.Printf("Parsed encoder value decimmal: %d\n", signedInt)

	return int(signedInt), nil
}

func readNumberOfPulses(driver *S42DDriver) (int32, error) {
	command := DriverCommand{
		name:                 "read_number_of_pulses",
		payload:              []byte{DOWNLINK_HEAD, SLAVE_ADDRESS, 0x33},
		expectedResponseSize: 8,
	}

	response, err := driver.sendCommand(command)
	if err != nil {
		return 0, err
	}

	// var head = buffer[0]
	// var slaveAddress = buffer[1]
	// var function = buffer[2]
	var value = response[3:7]
	// var crc = buffer[7]
	fmt.Printf("Number of pulses: %s\n", ByteArrayToString(value))
	valueDestination := int32(binary.BigEndian.Uint32(value))

	return valueDestination, nil
}

func readErrorOfMotorShaftAngle(driver *S42DDriver) (int, error) {
	command := DriverCommand{
		name:                 "read_shaft_angle_error",
		payload:              []byte{DOWNLINK_HEAD, SLAVE_ADDRESS, 0x39},
		expectedResponseSize: 8,
	}

	response, err := driver.sendCommand(command)
	if err != nil {
		return 0, err
	}

	// var head = buffer[0]
	// var slaveAddress = buffer[1]
	// var function = buffer[2]
	var value = response[3:7]
	// var crc = buffer[7]

	// TODO -> need to correctly convert this
	var intValue uint64
	binary.Read(bytes.NewReader(value), binary.BigEndian, &intValue)

	return int(intValue), nil
}

func startMotorSpeedMode(driver *S42DDriver, direction Direction, speed uint16) error {
	payload, err := createSpeedModePayload(direction, speed, 2)
	if err != nil {
		fmt.Println(err.Error())
		return errors.New("error could create speed mode payload")
	}

	command := DriverCommand{
		name:                 "start_motor_speed_mode",
		payload:              payload,
		expectedResponseSize: 5,
	}

	response, err := driver.sendCommand(command)
	if err != nil {
		return err
	}

	// var head = buffer[0]
	// var slaveAddress = buffer[1]
	// var function = buffer[2]
	var status = response[3]
	if status != 0x01 {
		return errors.New("command to start failed")
	}

	return nil
}

// Move the motor the amount of given pulses. This function blocks until the motor has completed the movement.
func startMotorPositionMode(driver *S42DDriver, direction Direction, speed uint16, numberOfPulses uint32, moveTimeoutMs int64) error {
	payload, err := createPositionMode1Payload(direction, speed, 0, numberOfPulses)
	if err != nil {
		fmt.Println(err.Error())
		return errors.New("error could not create position mode payload")
	}

	command := DriverCommand{
		name:                 "start_motor_position_mode1",
		payload:              payload,
		expectedResponseSize: 5,
	}

	doneResponse := addCRC([]byte{0xFB, SLAVE_ADDRESS, 0xFD, 0x02})
	response, err := driver.sendCommandAndWaitForResponse(command, doneResponse, moveTimeoutMs)
	if err != nil {
		return err
	}

	// var head = buffer[0]
	// var slaveAddress = buffer[1]
	// var function = buffer[2]
	var status = response[3]
	if status == 0x00 {
		return errors.New("command to start failed")
	}

	return nil
}

/*
query the motor status
can return:
status = 0 query fail.
status = 1 motor stop
status = 2 motor speed up
status = 3 motor speed down
status = 4 motor full speed
status = 5 motor is homing
status = 5 motor is Cal…
*/
func queryMotorStatus(driver *S42DDriver) (int, error) {
	command := DriverCommand{
		name:                 "query_motor_status",
		payload:              []byte{DOWNLINK_HEAD, SLAVE_ADDRESS, 0xF1},
		expectedResponseSize: 5,
	}

	response, err := driver.sendCommand(command)
	if err != nil {
		return 0, err
	}

	// var head = buffer[0]
	// var slaveAddress = buffer[1]
	// var function = buffer[2]
	var status = response[3]
	if status != 0x01 {
		return 0, errors.New("command to query motor failed")
	}

	return int(status), nil
}

/*
read the motor shaft protection state
status = 1 protected
status = 0 no protected
*/
func readMotorLockedState(driver *S42DDriver) (int, error) {
	command := DriverCommand{
		name:                 "read_motor_locked_state",
		payload:              []byte{DOWNLINK_HEAD, SLAVE_ADDRESS, 0x3E},
		expectedResponseSize: 5,
	}

	response, err := driver.sendCommand(command)
	if err != nil {
		return 0, err
	}

	// var head = buffer[0]
	// var slaveAddress = buffer[1]
	// var function = buffer[2]
	var status = response[3]
	return int(status), nil
}

func releaseMotorLockedState(driver *S42DDriver) error {
	command := DriverCommand{
		name:                 "release_motor_locked_state",
		payload:              []byte{DOWNLINK_HEAD, SLAVE_ADDRESS, 0x3D},
		expectedResponseSize: 5,
	}

	response, err := driver.sendCommand(command)
	if err != nil {
		return err
	}

	// var head = buffer[0]
	// var slaveAddress = buffer[1]
	// var function = buffer[2]
	var status = response[3]
	if status != 0x01 {
		return errors.New("command to release motor locked state failed")
	}
	return nil
}

func enableTheMotor(driver *S42DDriver) error {
	command := DriverCommand{
		name:                 "enable_the_motor",
		payload:              []byte{DOWNLINK_HEAD, SLAVE_ADDRESS, 0xF1, 0x01},
		expectedResponseSize: 5,
	}

	response, err := driver.sendCommand(command)
	if err != nil {
		return err
	}

	var status = response[3]
	if status != 0x01 {
		return errors.New("command to enable failed")
	}
	return nil
}

func cleanSpeedParameter(driver *S42DDriver) error {
	command := DriverCommand{
		name:                 "clean_speed_parameter",
		payload:              []byte{DOWNLINK_HEAD, SLAVE_ADDRESS, 0xFF, 0xCA},
		expectedResponseSize: 5,
	}

	response, err := driver.sendCommand(command)
	if err != nil {
		return err
	}

	var status = response[3]
	if status != 0x01 {
		return errors.New("command to clean speed mode failed")
	}
	return nil
}

func setCurrentAxisToZero(driver *S42DDriver) error {
	command := DriverCommand{
		name:                 "set_current_axis_zero",
		payload:              []byte{DOWNLINK_HEAD, SLAVE_ADDRESS, 0x92},
		expectedResponseSize: 5,
		waitForResponseMs:    100,
	}

	response, err := driver.sendCommand(command)
	if err != nil {
		return err
	}

	var status = response[3]
	if status != 0x01 {
		return errors.New("command to set current axis to zero failed")
	}
	return nil
}

func emergencyStopDriver(driver *S42DDriver) error {
	command := DriverCommand{
		name:                 "set_current_axis_zero",
		payload:              []byte{DOWNLINK_HEAD, SLAVE_ADDRESS, 0xF7},
		expectedResponseSize: 5,
	}

	response, err := driver.sendCommand(command)
	if err != nil {
		return err
	}

	var status = response[3]
	if status != 0x01 {
		fmt.Println("manual intervention required, emergency stop has failed")
		return errors.New("command to emergency stop failed")
	}
	return nil
}
