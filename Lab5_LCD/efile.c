#include <stdint.h>
#include <string.h>
#include "ST7735.h"
#include "diskio.h"
#include "efile.h"

/*
I plan to use only 256 blocks for this program

Block 0 is reserved for root directory
Block 255 is reserved for Allocation table

Allocation table will contain address of next block

Special cases: 
	0xFFFF indicates end of file
	0x00FF to 0xFFFE reserved; not to be used
	0x0000 indicates unallocated space
	0x0001 to 0x00FE store address of next block
*/


uint8_t tempBuffer[512];


void diskError(char *errtype, int32_t code, int32_t block){
  ST7735_DrawString(0, 0, "Error:", ST7735_Color565(255, 0, 0));
  ST7735_DrawString(7, 0, errtype, ST7735_Color565(255, 0, 0));
  ST7735_DrawString(0, 1, "Code:", ST7735_Color565(255, 0, 0));
  ST7735_SetCursor(6, 1);
  ST7735_SetTextColor(ST7735_Color565(255, 0, 0));
  ST7735_OutUDec(code);
  ST7735_DrawString(0, 2, "Block:", ST7735_Color565(255, 0, 0));
  ST7735_SetCursor(7, 2);
  ST7735_OutUDec(block);
	while(1) {};
}

allocationTable_t allocationTable;

const uint16_t rootDirectoryBlock = 0x0000;
const uint16_t allocationTableBlock = 0x00FF;

uint16_t currentDirectoryBlock;

filepointer_t writeFile, readFile;

void printDirectoryEntry(directoryEntry_t entry, int lineNumber) {
	ST7735_SetCursor(0, lineNumber);
	if (entry.type)	ST7735_DrawString(0, lineNumber, entry.fileName, ST7735_Color565(255, 0, 0));
	else ST7735_DrawString(0, lineNumber, entry.fileName, ST7735_Color565(0, 255, 0));
}
void ls(int blockNumber) {
	int i, result;
	directory_t dir;
	result = disk_read (0, tempBuffer, blockNumber,1);
	dir = *((directory_t *) tempBuffer);
	if(result) diskError("disk_read ", result, blockNumber); // read from disk
	printDirectoryEntry(dir.parent, 0);
	printDirectoryEntry(dir.self, 1);
	for (i = 0; i < dir.size; i++) {
		printDirectoryEntry(dir.contents[i], i+2);
	}
}

uint8_t nextAvailableBlock() {
	int i;
	for (i = 1; i < 256; i++) {
		if(allocationTable.entry[i] == 0) {
			allocationTable.entry[i] = 0xFFFF;
			return i;
		}
	}
	diskError("out_of_space", 0, 0);
	return 0;
}


//---------- eFile_Init-----------------
// Activate the file system, without formating
// Input: none
// Output: 0 if successful and 1 on failure (already initialized)
// since this program initializes the disk, it must run with 
//    the disk periodic task operating
static enum { INIT = 1, READY = 0 } initStatus = READY;
int eFile_Init(void) {
	int i, result;
	
	// Disk already initialized
	if (initStatus) return 1;
	
	// Start initializing
	initStatus = INIT;
	

	// initialize disk
	result = disk_initialize(0);
	if (result) diskError("disk_initialize", result, 0);
	
	// read allocation table from Disk into memory
	result = disk_read (0,tempBuffer, allocationTableBlock,1);
	if(result) diskError("disk_read ", result, allocationTableBlock); // read from disk
	
	// Init the allocation table
	allocationTable = *((allocationTable_t *) tempBuffer);
	currentDirectoryBlock = rootDirectoryBlock;
	
	// Set read and write as available
	writeFile.available = 1;
	readFile.available = 1;
	
	return 0;	
}

//---------- eFile_Format-----------------
// Erase all files, create blank directory, initialize free space manager
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Format(void) {
	int i = 0, result = 0;
	directory_t root;
	// erase disk, add format]
	
	// Init the allocation table
	for( i = 0; i < 256; i++) allocationTable.entry[i] = 0x00;
	allocationTable.entry[0] = 0xFFFF;
	allocationTable.entry[255] = 0xFFFF;
	// Save allocation block to disk
	result = disk_write (0,(uint8_t *)&allocationTable, allocationTableBlock,1);
	if(result) diskError("disk_write ", result, allocationTableBlock);
	
	//Create root directory
	
	
	// Create entry for self
	strcpy(root.self.fileName, ".");
	root.self.startBlock = rootDirectoryBlock;
	root.self.type = 0;
	strcpy(root.parent.fileName, "..");
	root.parent.startBlock = rootDirectoryBlock;
	root.parent.type = 0;
	root.isRootDirectory = 1;
	
	// Save root directory to disk
	result = disk_write (0,(uint8_t *)&root, rootDirectoryBlock,1);
	if(result) diskError("disk_write ", result, rootDirectoryBlock);
	
	currentDirectoryBlock = rootDirectoryBlock;
	
	return 0;
}
//---------- eFile_Create-----------------
// Create a new, empty file with one allocated block
// Input: file name is an ASCII string up to seven characters 
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Create( char name[]) {
	// create new file, make it empty
	int i, result;
	uint16_t blockNumber;
	directory_t dir;
	
	// open directory
	result = disk_read (0, tempBuffer, currentDirectoryBlock,1);
	dir = *((directory_t *) tempBuffer);
	if(result) diskError("disk_read ", result, currentDirectoryBlock); // read from disk
	
	//check if file name already in use
	for (i =0; i < dir.size; i++) {
		if (strcmp(dir.contents[i].fileName, name)==0) {
			ST7735_DrawString(0, 0, "File name in use", ST7735_Color565(255, 0, 0));
			return 1;
		}
	}
	
	blockNumber = nextAvailableBlock();
	
	if (dir.size == 28) diskError("directory full", 1, 0);
	dir.contents[dir.size].type = 1;
	dir.contents[dir.size].startBlock = blockNumber;
	strcpy(dir.contents[dir.size].fileName, name);
	dir.size++;
	result = disk_write (0,(uint8_t *)&dir, currentDirectoryBlock,1);
	if(result) diskError("disk_write ", result, currentDirectoryBlock);
	
	result = disk_write (0,(uint8_t *)&allocationTable, allocationTableBlock,1);
	if(result) diskError("disk_write ", result, allocationTableBlock);
	
	return 0;
}

//---------- eFile_WOpen-----------------
// Open the file, read into RAM last block
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WOpen(char name[]) {
	// open a file for writing
	int i, result, index;
	uint16_t blockNumber;
	directory_t dir;

	//check if another file in use
	if (!writeFile.available) {
		diskError("Write file already open", result, currentDirectoryBlock);
	}
	// open directory
	result = disk_read (0, tempBuffer, currentDirectoryBlock,1);
	dir = *((directory_t *) tempBuffer);
	if(result) diskError("disk_read ", result, currentDirectoryBlock); // read from disk
	
	
	//check if file exists
	for (i =0; i < dir.size; i++) {
		if (strcmp(dir.contents[i].fileName, name)==0) {
			index = i;
			break;
		}
	}
	if (i == dir.size) {
		ST7735_DrawString(0, 0, "File not found", ST7735_Color565(255, 0, 0));
		return 1;
	}
	
	if (!readFile.available && readFile.startBlockNumber == dir.contents[index].startBlock) {
		diskError("file already open", result, currentDirectoryBlock);
	}
	
	writeFile.filePosition = 0;
	writeFile.fileBlockNumber = dir.contents[index].startBlock;
	writeFile.startBlockNumber = dir.contents[index].startBlock;
	writeFile.available = 0;
	
	return 0;
	
}	

//---------- eFile_Write-----------------
// save at end of the open file
// Input: data to be saved
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Write( char data) {
	int blockNumber, result;
	if (writeFile.filePosition == 512) {
		// Get new block
		blockNumber = nextAvailableBlock();
		
		// Update allocation table
		allocationTable.entry[writeFile.fileBlockNumber] = blockNumber;
		allocationTable.entry[blockNumber] = 0xFFFF;
		
		// Write to disk
		result = disk_write (0,(uint8_t *)&allocationTable, allocationTableBlock,1);
		if(result) diskError("disk_write ", result, allocationTableBlock);
		
		result = disk_write(0, writeFile.fileBuffer, writeFile.fileBlockNumber, 1);
		if(result) diskError("disk_write ", result, writeFile.fileBlockNumber); // read from disk
		
		// Update file pointer
		writeFile.fileBlockNumber = blockNumber;
		writeFile.filePosition = 0;
	}
	writeFile.fileBuffer[writeFile.filePosition] = data;
	writeFile.filePosition++;
	return 0;
}	

//---------- eFile_Close-----------------
// Deactivate the file system
// Input: none
// Output: 0 if successful and 1 on failure (not currently open)
int eFile_Close(void); 


//---------- eFile_WClose-----------------
// close the file, left disk in a state power can be removed
// Input: none
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_WClose(void) {
	int result;
	
	result = disk_write(0, writeFile.fileBuffer, writeFile.fileBlockNumber, 1);
	if(result) diskError("disk_write ", result, writeFile.fileBlockNumber); // read from disk
	
	writeFile.available = 1;
	
	return 0;
}

//---------- eFile_ROpen-----------------
// Open the file, read first block into RAM 
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble read to flash)
int eFile_ROpen( char name[]){
	// open a file for reading
	int i, result, index;
	uint16_t blockNumber;
	directory_t dir;

	//check if another file in use
	if (!readFile.available) {
		diskError("Write file already open", result, currentDirectoryBlock);
	}
	
	// open directory
	result = disk_read (0, tempBuffer, currentDirectoryBlock,1);
	dir = *((directory_t *) tempBuffer);
	if(result) diskError("disk_read ", result, currentDirectoryBlock); // read from disk
	
	//check if file exists
	for (i =0; i < dir.size; i++) {
		if (strcmp(dir.contents[i].fileName, name)==0) {
			index = i;
			break;
		}
	}
	
	if (!writeFile.available && writeFile.startBlockNumber == dir.contents[index].startBlock) {
		diskError("Write file already open", result, currentDirectoryBlock);
	}
	
	if (i == dir.size) {
		ST7735_DrawString(0, 0, "File not found", ST7735_Color565(255, 0, 0));
		return 1;
	}
	
	if (!writeFile.available && writeFile.startBlockNumber == dir.contents[index].startBlock) {
		diskError("Write file already open", result, currentDirectoryBlock);
	}
	
	result = disk_read(0, readFile.fileBuffer, dir.contents[index].startBlock, 1);
	if(result) diskError("disk_read ", result, writeFile.fileBlockNumber); // read from disk
	
	readFile.filePosition = 0;
	readFile.fileBlockNumber = dir.contents[index].startBlock;
	readFile.startBlockNumber = dir.contents[index].startBlock;
	readFile.available = 0;
	
	return 0;
}
   
//---------- eFile_ReadNext-----------------
// retreive data from open file
// Input: none
// Output: return by reference data
//         0 if successful and 1 on failure (e.g., end of file)
int eFile_ReadNext( char *pt) {
	int blockNumber, result;
	if (readFile.filePosition == 512) {
		
		blockNumber = allocationTable.entry[readFile.fileBlockNumber];
		if (blockNumber == 0xFFFF) diskError("end of file", 1, readFile.fileBlockNumber); // read from disk
		
		// Load next block
		result = disk_read(0, readFile.fileBuffer, blockNumber, 1);
		if(result) diskError("disk_read ", result, writeFile.fileBlockNumber); // read from disk
		
		// Update file pointer
		readFile.filePosition = 0;
		readFile.fileBlockNumber = blockNumber;
		
	}
	*pt = readFile.fileBuffer[readFile.filePosition];
	readFile.filePosition++;
	return 0;
}
                              
//---------- eFile_RClose-----------------
// close the reading file
// Input: none
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int eFile_RClose(void) {
	// close the file for writing
	readFile.available = 1;
	return 0;
}

//---------- eFile_Directory-----------------
// Display the directory with filenames and sizes
// Input: pointer to a function that outputs ASCII characters to display
// Output: characters returned by reference
//         0 if successful and 1 on failure (e.g., trouble reading from flash)
int eFile_Directory(void(*fp)(unsigned char)) {
}

//---------- eFile_Delete-----------------
// delete this file
// Input: file name is a single ASCII letter
// Output: 0 if successful and 1 on failure (e.g., trouble writing to flash)
int eFile_Delete( char name[]) {
	// remove this file
	int i, result, index, temp;
	directory_t dir;
	
	// open directory
	result = disk_read (0, tempBuffer, currentDirectoryBlock,1);
	dir = *((directory_t *) tempBuffer);
	if(result) diskError("disk_read ", result, currentDirectoryBlock); // read from disk
	
	//check if file exists
	for (i =0; i < dir.size; i++) {
		if (strcmp(dir.contents[i].fileName, name)==0) {
			index = i;
			break;
		}
	}
	if (i == dir.size) {
		ST7735_DrawString(0, 0, "File not found", ST7735_Color565(255, 0, 0));
		return 1;
	}
	
		
	//clear entries from allocation table
	temp = index;
	while(allocationTable.entry[temp] != 0) {
		i = allocationTable.entry[temp];
		allocationTable.entry[temp] = 0;
		temp = i;
		if (temp == 0xFFFF) break;
	}
	
	// update directory
	dir.contents[index] = dir.contents[dir.size-1];
	dir.size--;
	
	//save directory
	result = disk_write (0,(uint8_t *)&dir, currentDirectoryBlock,1);
	if(result) diskError("disk_write ", result, currentDirectoryBlock);
	
	//save allocation table
	result = disk_write (0,(uint8_t *)&allocationTable, allocationTableBlock,1);
	if(result) diskError("disk_write ", result, allocationTableBlock);
	
	
	return 0;
}

//---------- eFile_RedirectToFile-----------------
// open a file for writing 
// Input: file name is a single ASCII letter
// stream printf data into file
// Output: 0 if successful and 1 on failure (e.g., trouble read/write to flash)
int eFile_RedirectToFile(char *name) {
}

//---------- eFile_EndRedirectToFile-----------------
// close the previously open file
// redirect printf data back to UART
// Output: 0 if successful and 1 on failure (e.g., wasn't open)
int eFile_EndRedirectToFile(void) {
}
