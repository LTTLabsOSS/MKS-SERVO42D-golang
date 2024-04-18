package mks42d

import (
	"fmt"
	"time"

	"github.com/tarm/serial"
)

/*
This module contains the abstraction for interacting with a stepper motor and its driver board.
Each concrete implementation of this interface for different driver boards should be in their
own files.
*/
type StepperDriver interface {
	Connect() error                  // initialize serial connection
	GetCurrentDistanceTraveled() int // get distance traveled since counter started in mm
	ResetCounters() error            // resets pulse and distance counters
	StartGoingDown() error           // start motor going forward
	StartGoingUp(speed uint16) error // start motor going in reverse
	Stop() error                     // stop all movement
	FindMaxBottom() (int, error)     // find the maximum encoder value to the bottom of the system
	GoHome(speed uint16) error       // go to the very top of the system
}

type DriverCommand struct {
	name                 string
	payload              []byte
	expectedResponseSize int
	waitForResponseMs    int
}

type ServoMessageType int

const (
	ServoError ServoMessageType = iota // 0 ...
	ServoTravelMessage
	ServoEncoderMessage
	ServoPulseCountMessage
)

type ServoMessage struct {
	Type    ServoMessageType
	Data    any
	Message string
}

func msSinceEpoch() int64 {
	return time.Now().UnixNano() / int64(time.Millisecond)
}

func connectToDriver(config *serial.Config) (*serial.Port, error) {
	driverPort, err := serial.OpenPort(config)
	if err != nil {
		fmt.Println(err.Error())
		return nil, fmt.Errorf("failed to connect to %s", config.Name)
	}

	return driverPort, nil
}

func addCRC(data []byte) []byte {
	var checksum byte
	for _, b := range data {
		checksum += b
	}
	checksum &= 0xFF
	data = append(data, checksum)
	return data
}

func writeToDriver(port *serial.Port, payload []byte, flush bool) error {
	if port == nil {
		return fmt.Errorf("cannot write to nil port")
	}
	_, err := port.Write(addCRC(payload))
	if err != nil {
		fmt.Println(err.Error())
		return fmt.Errorf("error writing to serial port")
	}

	if flush {
		err = port.Flush()
		if err != nil {
			fmt.Println(err.Error())
			return fmt.Errorf("error flushing serial port")
		}
	}
	// time.Sleep(50 * time.Millisecond)
	return nil
}

func readFromDriver(port *serial.Port, buffer []byte, flush bool) (int, []byte, error) {
	if port == nil {
		return 0, nil, fmt.Errorf("cannot read from nil port")
	}

	numberOfBytes, err := port.Read(buffer)
	if err != nil {
		fmt.Println(err.Error())
		if err.Error() == "EOF" {
			return 5, buffer, nil
		}
		return 0, nil, fmt.Errorf("error reading from serial port")
	}

	if flush {
		err = port.Flush()
		if err != nil {
			fmt.Println(err.Error())
			return 0, nil, fmt.Errorf("error flushing serial port")
		}
	}
	time.Sleep(50 * time.Millisecond)
	return numberOfBytes, buffer, nil
}

func ByteArrayToString(array []byte) string {
	var hexString string
	for _, byteValue := range array {
		hexString += fmt.Sprintf("0x%02x ", byteValue)
	}

	return hexString
}
