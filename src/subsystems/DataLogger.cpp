/*
 * DataLogger.cpp
 *
 * Created on: Mar 26, 2022
 * Author: Peter Dentch
 */

#include "DataLogger.h"

/*
* Constructor for the DataLogger subsystem object
*/
DataLogger::DataLogger(){};


/*
 * Init function for the system, should be run after instantiation
 */
bool DataLogger::subsystemInit(){

	// Flash memory
	bool flashInit = false;
	flashInit = flashMemoryInit();

	// Transceiver radio
	bool radioInit = false;
	radioInit = transcieverInit();


	//delay(10);

	return flashInit && radioInit;

}


/*
 *
 */
bool DataLogger::transcieverInit() {

	Serial1.begin(9600);

	bool initSuccess = false;
	//initSuccess = transceiver->init(1);

//	//transceiver->SetMode();
//	transceiver->SetAddressH(0);
//	transceiver->SetAddressL(0);
//	transceiver->SetChannel(1);
//	//transceiver->SetOptions();
//
//	transceiver->SetParityBit(0);	 		// SpeedParityBit
//	transceiver->SetUARTBaudRate(3);		// 3 = 9600 baud
//	transceiver->SetAirDataRate(4);			// 2 = B010 =  2.4kbps (default)
//											// 4 = B100 = 9.6kbps
//											// 5 = B101 = 19.2kbps
//	transceiver->SetTransmissionMode(0);	// OptionTrans
//	transceiver->SetPullupMode(1);			// OptionPullup
//	transceiver->SetWORTIming(0);			// OptionWakeup
//	transceiver->SetFECMode(1);				// OptionFEC
//	transceiver->SetTransmitPower(0);		// default
//
//	transceiver->SaveParameters(PERMANENT);
//	transceiver->PrintParameters();

	return initSuccess;

}


/*
 *
 */
bool DataLogger::flashMemoryInit() {

	SerialFlash.begin(PIN_CS);	//TODO check bool

    //Function immediately checks if the file already exists, else does nothing!
    bool fileCreation = SerialFlash.createErasable(bufferFileName, circularBufferSize);

    //Serial.print("Circular buffer size: "); Serial.println(circularBufferSize);
    //Serial.print("Sensor data size: "); Serial.println(dataPacketSize);

    if(fileCreation) {
      Serial.println("File created successfully!");
    } else {
      Serial.println("File not created/File Already Exists");
    }

    bufferFile = SerialFlash.open(bufferFileName);

    //getFlashAddress gives the file address in the memory, blocks of 2^16 for using createErasable
    Serial.print("Address: "); Serial.println(bufferFile.getFlashAddress());	// should be 65536
    Serial.print("File Position: ");Serial.println(bufferFile.position());

    Serial.print("File Size: "); Serial.println(bufferFile.size());


	return fileCreation;

}


/*
 *	Function for locating the
 */
void DataLogger::locateCircBufferAddresses() {

	// First: find the first block that has yet to be fully populated with data

	uint16_t totalBlocks = circularBufferSize / SerialFlash.blockSize();
	//Serial.print("Total blocks: "); Serial.println(totalBlocks);

	uint32_t blockToWriteAddr = 0;		// the first free block (chuck of 2^16 bytes)

	bool foundErasedBlock = false;

	//looping through the buffer
	for(uint16_t i = 0; i < totalBlocks; i++) {

		uint32_t blockAddress = i * SerialFlash.blockSize();
		uint32_t lastByteAddr = blockAddress + (SerialFlash.blockSize() - 1);

		uint8_t lastByteValue = 0;
		bufferFile.seek(lastByteAddr);
		bufferFile.read(&lastByteValue, 1);

		if(lastByteValue == 255){				// found first erased block in the buffer

			foundErasedBlock = true;
			blockToWriteAddr = blockAddress;
			break;
		}

	}

	// Must handle case if no free block is found (system powered off before erase complete)
	if(foundErasedBlock){
		Serial.print("First free block is at: "); Serial.println(blockToWriteAddr);
	} else {
		Serial.println(F("FAILED TO FIND ERASED BLOCK"));
	}


	// Second: find the first byte address in the block to write new data

	uint32_t byteToWriteAddr = 0;			// the first free byte in the block

	// looping backwards through the block
	for(uint32_t i = SerialFlash.blockSize(); i > 0; i--) {

		uint32_t byteAddress = blockToWriteAddr + (i - 1);
		uint8_t byteValue = 255;
		bufferFile.seek(byteAddress);
		bufferFile.read(&byteValue, 1);

		if(byteValue != 255){				// found first erased byte in the block

			endDataAddress = byteAddress;
			byteToWriteAddr = byteAddress + 1;		// the next address is blank, write to it

			break;
		}

	}

	//Serial.print("First free byte is at: "); Serial.println(byteToWriteAddr);

	// Third: find the next block with data in it

	// The first block to search for valid data (start 1 block after the current block)
	uint32_t begBlockAddr = blockToWriteAddr + SerialFlash.blockSize();

	//looping through the remainder of the buffer
	for(uint16_t i = 0; i < totalBlocks; i++) {

		//uint32_t blockAddress = begBlockAddr + (i * SerialFlash.blockSize());
		uint32_t blockAddress = i * SerialFlash.blockSize();

		// Wrap around if out of bounds
		if(blockAddress > circularBufferSize){
			blockAddress = 0;
		}

		uint8_t byteValue = 255;
		bufferFile.seek(blockAddress);
		bufferFile.read(&byteValue, 1);

		//Serial.print("First byte is: "); Serial.println(byteValue);

		if(byteValue != 255){						// found data in first address of block

			begDataAddress = blockAddress;

			break;
		}


	}

	// Move to the byte address where data should resume being written to the buffer
	bufferFile.seek(byteToWriteAddr);

}


/*
 *
 * @return true if data written, false if block needs to be erased
 */
bool DataLogger::updateCircBuffer() {

	uint32_t currentAddress = bufferFile.position();
	Serial.print("Current address: "); Serial.println(currentAddress);

	// First update buffer end position

	// Check if the current buffer position is at the end of the buffer
	if(currentAddress >= circularBufferSize) {

		// Wrap around if out of bounds
		endDataAddress = 0;
		bufferFile.seek(0);			// set the current address for writing back to the start
	} else {

		// otherwise just update to the next position to write
		endDataAddress = currentAddress;
	}

	// Then check if the buffer position has reached the begDataAddress position
	// in which case the current block must be erased and begDataAddress set to the
	// start of the next block
	if(endDataAddress == begDataAddress){

		// Erase block
		Serial.print("Erase block: "); Serial.println(begDataAddress);

		uint32_t fileAddress = bufferFile.getFlashAddress();    // should be 0x00FFFF (2^16)
		SerialFlash.eraseBlock(fileAddress + begDataAddress);	// offset by file location

		// Update new beginning address
		begDataAddress = begDataAddress + SerialFlash.blockSize();

		// Wrap around if out of bounds
		if(begDataAddress >= circularBufferSize){

			begDataAddress = 0;
		}

		// Don't proceed to write if erase block command ran
		//setState(DATALOGGER_ERASE_BUFFER);
		//return false;

	}

	// Don't proceed to write if chip is busy (erase block command ran)
	if(!SerialFlash.ready()){
		return false;
	}


	uint32_t fileWriteLen = bufferFile.write(&currentDataPacket, dataPacketSize);
	//Serial.print("Length of data written: "); Serial.println(fileWriteLen);
	//Serial.print("Current address: "); Serial.println(currentAddress);

//	Serial.print("Buffer begin addr: "); Serial.println(begDataAddress);
//	Serial.print("Buffer end addr: "); Serial.println(endDataAddress);


	return true;

}


/*
 *
 */
bool DataLogger::readCircBuffer() {

	// If there's a next data packet to read in the file, read and print it
	if(bufferFile.available() > 0){

		//uint32_t currentAddress = bufferFile.position();
		//Serial.print(currentAddress); Serial.print(", ");
		//Serial.print("Current address: "); Serial.println(currentAddress);

		uint8_t buffer[dataPacketSize];             // temporary data buffer for reading

		// Read the memory block
		uint32_t fileReadLen = bufferFile.read(buffer, dataPacketSize);
		//Serial.print("Length of data read: "); Serial.println(fileReadLen);

		//Serial.println("Whole buffer of data");
		//for(uint8_t i = 0; i < dataPacketSize; i++) {
		//	Serial.println(buffer[i]);
		//}

		DataPacket bufferPacket;
		// for loop?
		bufferPacket.count0 = buffer[0];
		bufferPacket.count1 = buffer[1];
		bufferPacket.count2 = buffer[2];
		bufferPacket.count3 = buffer[3];
		bufferPacket.count4 = buffer[4];
		bufferPacket.count5 = buffer[5];
		bufferPacket.count6 = buffer[6];
		bufferPacket.count7 = buffer[7];



		printPacketToSerialMonitor(bufferPacket);
//		printPacketToGroundstation(bufferPacket);


		return true;

	} else {
		return false;
	}

}


/*
 *
 */
bool DataLogger::addPacketToSmallBuffer() {

	//Serial.print("INDEX: "); Serial.println(smallBufferIndex);
	dataPacketSmallBuffer[smallBufferIndex] = currentDataPacket;

	// Wrap around if out of bounds
	uint8_t smallBufferPacketNum = sizeof(dataPacketSmallBuffer)
			/ sizeof(dataPacketSmallBuffer[1]);

	if(smallBufferIndex >= smallBufferPacketNum - 1){
		smallBufferIndex = 0;
		return true;
	} else {
		smallBufferIndex++;
		return false;
	}

}


/*
 *
 */
void DataLogger::writeSmallBufferToCircBuffer() {


	for(uint8_t i = 0; i < smallBufferIndex; i++){

		DataPacket packet = dataPacketSmallBuffer[i];

		uint32_t fileWriteLen = bufferFile.write(&packet, dataPacketSize);

		uint32_t currentAddress = bufferFile.position();
		endDataAddress = currentAddress;

	}

	smallBufferIndex = 0;				// reset to start of buffer after copying it

}



/*
 *
 */
bool DataLogger::timeToTransmit() {
	return transmitTimer.check() == 1;			//check if the timer has passed it's interval
}


/*
 *
 */
bool DataLogger::transmitTelemetry() {

	bool sendSuccess = false;

//	dataPacket.count0 = timestampBytes[3];
//	dataPacket.count1 = timestampBytes[2];
//	dataPacket.count2 = timestampBytes[1];
//	dataPacket.count3 = timestampBytes[0];
//
//	dataPacket.count4 = barometerBytes[0];
//	dataPacket.count5 = barometerBytes[1];
//	dataPacket.count6 = barometerBytes[2];
//	dataPacket.count7 = barometerBytes[3];
//
//	dataPacket.count8 = gyroAccelBytes[0];
//	dataPacket.count9 = gyroAccelBytes[1];
//	dataPacket.count10 = gyroAccelBytes[2];
//	dataPacket.count11 = gyroAccelBytes[3];
//	dataPacket.count12 = gyroAccelBytes[4];
//	dataPacket.count13 = gyroAccelBytes[5];
//	dataPacket.count14 = gyroAccelBytes[6];
//	dataPacket.count15 = gyroAccelBytes[7];
//	dataPacket.count16 = gyroAccelBytes[8];
//	dataPacket.count17 = gyroAccelBytes[9];
//	dataPacket.count18 = gyroAccelBytes[10];
//	dataPacket.count19 = gyroAccelBytes[11];

	sendSuccess = transceiver->SendStruct(&currentDataPacket, dataPacketSize);

	return sendSuccess;

}


/*
 *
 */
void DataLogger::setState(DataLoggerState state) {
	loggerState = state;
}


/*
 *
 */
void DataLogger::setCurrentDataPacket(DataPacket packet) {
	currentDataPacket = packet;
}


/*
 *
 * BIG ENDIAN
 * @param packet is the DataPacket to parse and print
 */
void DataLogger::printPacketToGroundstation(DataPacket packet) {

    //Data start bytes
	Serial.print(PRINT_BEG);

    Serial.print(TIMESTAMP);
    Serial.write(packet.count0);
    Serial.write(packet.count1);
    Serial.write(packet.count2);
    Serial.write(packet.count3);

    Serial.print(ALTITUDE);
    Serial.write(packet.count4);
    Serial.write(packet.count5);
    Serial.write(packet.count6);
    Serial.write(packet.count7);

    // Data end bytes
    Serial.print(PRINT_END);


//    Serial.write(83); // S - State
//    Serial.write(84); // T
//    Serial.write(84); // T
//    Serial.write(0);  // state zero hardcode for now
//
//    Serial.write(65); // A - Altitute
//    Serial.write(76); // L
//    Serial.write(84); // T
//    Serial.write(altitudeBytes[3]);
//    Serial.write(altitudeBytes[2]);
//    Serial.write(altitudeBytes[1]);
//    Serial.write(altitudeBytes[0]);
//
//    Serial.write(84); // T - Temperature
//    Serial.write(77); // M
//    Serial.write(80); // P
//    Serial.write(temperatureBytes[3]);
//    Serial.write(temperatureBytes[2]);
//    Serial.write(temperatureBytes[1]);
//    Serial.write(temperatureBytes[0]);
//
//    // Accelerometer x-axis
//	Serial.write(65); // A
//	Serial.write(67); // C
//	Serial.write(88); // X
//	Serial.write(gyroAccelBytes[0]);
//	Serial.write(gyroAccelBytes[1]);
//
//	// Accelerometer y-axis
//	Serial.write(65); // A
//	Serial.write(67); // C
//	Serial.write(89); // Y
//	Serial.write(gyroAccelBytes[2]);
//	Serial.write(gyroAccelBytes[3]);
//
//	// Accelerometer z-axis
//	Serial.write(65); // A
//	Serial.write(67); // C
//	Serial.write(90); // Z
//	Serial.write(gyroAccelBytes[4]);
//	Serial.write(gyroAccelBytes[5]);
//
//	// Gyro x-axis
//	Serial.write(71); // G
//	Serial.write(89); // Y
//	Serial.write(88); // X
//	Serial.write(gyroAccelBytes[6]);
//	Serial.write(gyroAccelBytes[7]);
//
//	// Gyro y-axis
//	Serial.write(71); // G
//	Serial.write(89); // Y
//	Serial.write(89); // Y
//	Serial.write(gyroAccelBytes[8]);
//	Serial.write(gyroAccelBytes[9]);
//
//	// Gyro z-axis
//	Serial.write(71); // G
//	Serial.write(89); // Y
//	Serial.write(90); // Z
//	Serial.write(gyroAccelBytes[10]);
//	Serial.write(gyroAccelBytes[11]);
//

}


/*
 *
 */
void DataLogger::printPacketToSerialMonitor(DataPacket packet) {

	uint32_t ts = 0;
	uint8_t * timestampBytes = (uint8_t *) &ts;
	timestampBytes[3] = packet.count0;
	timestampBytes[2] = packet.count1;
	timestampBytes[1] = packet.count2;
	timestampBytes[0] = packet.count3;
	//Serial.print("TIMESTAMP IS: ");
	//Serial.println(ts);
	Serial.print(ts); Serial.print(", ");

//	float voltage = 0;
//	uint8_t * voltageBytes = (uint8_t *) &voltage;
//	voltageBytes[3] = packet.count4;
//	voltageBytes[2] = packet.count5;
//	voltageBytes[1] = packet.count6;
//	voltageBytes[0] = packet.count7;
	//Serial.print("VOLTAGE IS: "); Serial.println(voltage);
	//Serial.println(voltage);

//	uint8_t sensorVal = 0;
//	uint8_t * pinBytes = (uint8_t *) &sensorVal;
//	pinBytes[0] = packet.count4;
//	Serial.println(sensorVal);

	Serial.println(packet.count4);


}


/*
 * Zero subsystem sensors
 */
void DataLogger::zeroSensors() {

}

/*
 * Register data logger loop
 */
void DataLogger::registerLoops(Looper * runningLooper) {

	runningLooper->registerLoop(loggerLoop);
}

/*
 * Print output of data logger
 */
void DataLogger::printOutput() {
	Serial.println(F("TEST"));
}



