package mks42d

import (
	"fmt"
	"testing"
)

func checkPayloadAgainstExpected(expected []byte, received []byte) bool {
	if len(expected) != len(received) {
		fmt.Println("payloads are not equivalent")
		return false
	}

	// Iterate over the slices and compare each element
	for i := 0; i < len(expected); i++ {
		fmt.Printf("comparing expected %02X to received %02X\n", expected[i], received[i])
		if expected[i] != received[i] {
			fmt.Println("received payload did not match expected payload")
			return false
		}
	}

	return true
}

func printByteArray(printMe []byte) {
	for _, b := range printMe {
		fmt.Printf("%02X ", b)
	}
	fmt.Println()
}

// Send “FA 01 F6 01 40 02”, the motor rotates forward at acc=2, speed=320RPM
func TestCreateSpeedModePayload1(t *testing.T) {
	direction := CLOCKWISE
	var speed uint16 = 320
	var acceleration uint8 = 2

	payload, err := createSpeedModePayload(direction, speed, acceleration)
	if err != nil {
		fmt.Println(err.Error())
		t.FailNow()
	}

	fmt.Println("received")
	printByteArray(payload)

	expectedPayload := []byte{0xFA, 0x01, 0xF6, 0x81, 0x40, 0x02}
	if !checkPayloadAgainstExpected(expectedPayload, payload) {
		t.FailNow()
	}
}

// Send “FA 01 F6 81 40 02”, the motor reverses at acc=2, speed=320RPM
func TestCreateSpeedModePayload2(t *testing.T) {
	direction := COUNTERCLOCKWISE
	var speed uint16 = 320
	var acceleration uint8 = 2

	payload, err := createSpeedModePayload(direction, speed, acceleration)
	if err != nil {
		fmt.Println(err.Error())
		t.FailNow()
	}

	printByteArray(payload)

	expectedPayload := []byte{0xFA, 0x01, 0xF6, 0x01, 0x40, 0x02}
	if !checkPayloadAgainstExpected(expectedPayload, payload) {
		t.FailNow()
	}
}

func TestCreateSpeedPayloadBadSpeed(t *testing.T) {
	direction := COUNTERCLOCKWISE
	var speed uint16 = 6000
	var acceleration uint8 = 2

	_, err := createSpeedModePayload(direction, speed, acceleration)
	if err == nil {
		fmt.Println("expected error for negative speed, but no error")
		t.FailNow()
	}

	speed = 4000
	_, err = createSpeedModePayload(direction, speed, acceleration)
	if err == nil {
		fmt.Println("expected error for out of bounds speed, but no error")
		t.FailNow()
	}
}

func TestCreatePositionModePayload(t *testing.T) {
	direction := COUNTERCLOCKWISE
	var speed uint16 = 320
	var acceleration uint8 = 2
	var numberOfPulses uint32 = 64000

	payload, err := createPositionMode1Payload(direction, speed, acceleration, numberOfPulses)
	if err != nil {
		fmt.Println(err.Error())
		t.FailNow()
	}

	printByteArray(payload)

	expectedPayload := []byte{0xFA, 0x01, 0xFD, 0x01, 0x40, 0x02, 0x00, 0x00, 0xFA, 00}
	if !checkPayloadAgainstExpected(expectedPayload, payload) {
		t.FailNow()
	}
}
