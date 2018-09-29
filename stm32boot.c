/*
 * SPI boot raspberry pi
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <byteswap.h>
#include <math.h>
#include <getopt.h>
#include <stdbool.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))

#define SPI_CHANNEL	1
#define SPI_SPEED	500000

#define BOOT0_PIN	37
#define RESET_PIN	0

#define ACK 	0x79
#define NACK 	0x1F
#define SOF 	0x5A

#define CMD_GET 				0x00 /*works*/
#define CMD_GET_V				0x01 /*works*/
#define CMD_GET_ID				0x02 /*works*/
#define CMD_READ_MEM			0x11 /*works*/
#define CMD_GO					0x21 /*maybe works*/
#define CMD_WRITE_MEM			0x31 /*works*/
#define CMD_ERASE				0x44 /*works*/
// #define CMD_WRITE_PROTECT 		0x63 // DO NOT USE
// #define CMD_WRITE_UNPROTECT		0x73 // DO NOT USE
// #define CMD_READOUT_PROTECT 	0x82 // DO NOT USE
// #define CMD_READOUT_UNPROTECT	0x92 // DO NOT USE

/* Timing Datasheet STM32f401, Flash memory programing */
#define WORD_PROGRAMMING_TIME	0.016 	//[ms] Typ: 0.016ms, Max: 0.1ms
#define SECTOR_ERASE_16_TIME	400	  	//[ms]
#define SECTOR_ERASE_64_TIME	1200	//[ms]
#define SECTOR_ERASE_128_TIME	2000	//[ms]
#define MASS_ERASE_TIME			8000	//[ms]

#define FLASH_MEM_ADDRESS_START 0x08000000
#define FLASH_MEM_ADDRESS_END	 0x0807FFFF	

// Values from 0xFFFC to 0xFFF0 are reserved 
#define ERASE_SPECIAL_GLOBAL	0xFFFF
#define ERASE_SPECIAL_BANK1		0xFFFE
#define ERASE_SPECIAL_BANK2		0xFFFD

#define SECTOR0_START	0x08000000
#define SECTOR0_END		0x08003FFF
#define SECTOR0_SIZE	SECTOR0_END - SECTOR0_START //16

#define SECTOR1_START	0x08004000
#define SECTOR1_END		0x08007FFF
#define SECTOR1_SIZE	SECTOR1_END - SECTOR1_START //16

#define SECTOR2_START	0x08008000
#define SECTOR2_END		0x0800BFFF 
#define SECTOR2_SIZE	SECTOR2_END - SECTOR2_START //16

#define SECTOR3_START	0x0800C000
#define SECTOR3_END		0x0800FFFF 
#define SECTOR3_SIZE	SECTOR3_END - SECTOR3_START //16

#define SECTOR4_START	0x08010000
#define SECTOR4_END		0x0801FFFF
#define SECTOR4_SIZE	SECTOR4_END - SECTOR4_START //64

#define SECTOR5_START	0x08020000
#define SECTOR5_END		0x0803FFFF
#define SECTOR5_SIZE	SECTOR5_END - SECTOR5_START //128

// #define SECTOR6_START	0x08040000
// #define SECTOR6_END		0x0805FFFF
// #define SECTOR6_SIZE	SECTOR6_END - SECTOR6_START //128

// #define SECTOR7_START	0x08060000
// #define SECTOR7_END		0x0807FFFF
// #define SECTOR7_SIZE	SECTOR7_END - SECTOR7_START //128

#define NUMBER_OF_SECTORS 8

#define PAGE_SIZE 256

#define global_erase_cmd() erase_memory_cmd(NULL,ERASE_SPECIAL_GLOBAL,MASS_ERASE_TIME)


int reset = false;
int info = false;
int prog = false;
int verif = false;

unsigned int address;
char *bin_path;

 typedef struct {
 	int addr_start;
 	int addr_end;
 } Sector;

Sector sectors[] = {
	{SECTOR0_START,SECTOR0_END},
	{SECTOR1_START,SECTOR1_END},
	{SECTOR2_START,SECTOR2_END},
	{SECTOR3_START,SECTOR3_END},
	{SECTOR4_START,SECTOR4_END},
	{SECTOR5_START,SECTOR5_END},
	// {SECTOR6_START,SECTOR6_END},
	// {SECTOR7_START,SECTOR7_END}
};

int sector_size(Sector *sector){
	return sector->addr_end - sector->addr_start;
}

int sector_erase_time(Sector* sector){
	switch(sector_size(sector)){
		case 0x3FFF: return SECTOR_ERASE_16_TIME;
		case 0xFFFF: return SECTOR_ERASE_64_TIME;
		case 0x1FFFF: return SECTOR_ERASE_128_TIME;
		default: return MASS_ERASE_TIME;
	}
}

char data[1024];
short ID;
char bl_version;

char spi_write(unsigned char ch)
{
	wiringPiSPIDataRW(SPI_CHANNEL,&ch,1);
	return ch;
}

int spi_write_data(unsigned char* d,int len){
	return wiringPiSPIDataRW(SPI_CHANNEL, d, len);
}

int get_ack(void){
	char ch;
	spi_write(0x0);
	do {
		ch = spi_write(0x0);
	} while(ch != ACK && ch != NACK);
	spi_write(ACK);
	return (ch==ACK)?1:0;
}

void sync_frame(void){
	while(spi_write(SOF) != 0xA5); 
	printf("Synchronization byte recieved\n");
	if(!get_ack()){
		printf("SPI Bootloader is not connected\n");
		exit(-1);
	}
	printf("SPI Bootloader is connected\n");
	return;
}

void stm_reset(int boot)
{
	digitalWrite(RESET_PIN,LOW);
	digitalWrite(BOOT0_PIN,boot);
	delay(200);
	digitalWrite(RESET_PIN,HIGH);
	printf("STM reset, with bootloader %s\n",(boot!=0)?"enabled":"disabled");
	if(boot == 1){
		sync_frame();
	}
}

int cmd_frame(unsigned char cmd){
	spi_write(SOF);
	spi_write(cmd);
	spi_write(~cmd);
	if(!get_ack()){
		printf("Command 0x%02x is not acknowledged\n",cmd);
		return 0;
	}else{
		return 1;
	}
}

void print_bl_version(){
	printf("bootloader version: %d.%d\n",(bl_version&0xF0)>>4,(bl_version&0x0F));
}

int get_cmd(){
	if(!cmd_frame(CMD_GET)) return 0;
	//data
	spi_write(0x0); //dummy
	char N = spi_write(0x0)+1;
	//bootloader version
	bl_version = spi_write(0x0);
	print_bl_version();
	N--;
	for(int i=0;i<N;i++){
		printf("cmd %d: 0x%02x\n",i, spi_write(0x0));
	}
	if(!get_ack()) {
		return 0;
	}else{
		return 1;
	}
}

int get_version_cmd(){
	if(!cmd_frame(CMD_GET_V)) return 0;
	spi_write(0x0); //dummy
	bl_version = spi_write(0x0);
	print_bl_version();
	if(!get_ack()) {
		return 0;
	}else{
		return 1;	
	}
}

int get_id_cmd(){
	if(!cmd_frame(CMD_GET_ID)) return 0;
	spi_write(0x0); //dummy
	char N = spi_write(0x0)+1;
	ID = 0x0;
	for(int i = 1;i<=N;i++){
		ID |= (spi_write(0x0) << 8*(N-i)) & (0xFF<<8*(N-i));
	}
	printf("Product ID: 0x%04x\n",ID );
	if(!get_ack()){
		return 0;
	}else{
		return 1;
	}
}

unsigned char checksum(unsigned char* data,int len){
		unsigned char checksum = 0x00;
		for(int i = 0;i<len;i++){
			checksum ^=data[i];
		}
		return checksum;
}

 unsigned char* send_data_frame(unsigned char* data, int len){
	static unsigned char spi_buffer[1024];
	memcpy(spi_buffer,data,len);
	spi_write_data(spi_buffer,len);
	spi_write(checksum(data,len));
	return spi_buffer;
}

int write_address(int address){
	address = __bswap_32(address);
	send_data_frame((unsigned char*)&address,sizeof(int));
	if(!get_ack()) {
		printf("Address it not valid\n");
		return 0;
	}else{
		return 1;	
	}
	
}

int is_in_range(int value, int min, int max){
	if(value < min || value > max) {
		return 0;
		printf("Number %d is out of range <%d;%d>\n", value,min,max);
	}
	else return 1;
}


int read_memory_cmd(int address, unsigned char* mem,int size){
	if(!cmd_frame(CMD_READ_MEM)) return 0;
	if(!write_address(address)) return 0;
	if(!is_in_range(size,1,256)) return 0;

	char N = size-1;
	spi_write(N);
	spi_write(~N);

	if(!get_ack()) {
		printf("Number %d of bytes to be read is not valid\n",size);
		return 0;
	}

	spi_write(0x0); //dummy
	for(int i=0;i<size;i++){
		mem[i] = spi_write(0x0);
	}

	return 1;
}

int go_cmd(int address){
	if(!cmd_frame(CMD_GO)) return 0;
	if(!write_address(address)) {
		return 0;
	} else{
		printf("Command GO done\n");
		return 1;
	}
}

int write_memory_cmd(unsigned int address, unsigned char* mem, int size){
	/*
	* Max length of block to be written is 256B
	* Write operations to Flash memory muse be WORD(16bit) aligned => data should be multiples of two bytes.
		if less data is written, the remaining bytes should be filled by 0xFF.	
	*/

	/* No error is returned when performing write operations in write-protected sector */

	// Never write to the first RAM memory, there is the bootloader
	// This is address space where program should be. 

	if(!is_in_range(address,FLASH_MEM_ADDRESS_START,FLASH_MEM_ADDRESS_END)) return 0;
	if(!is_in_range(size,1,256)) return 0;

	if(!cmd_frame(CMD_WRITE_MEM)) return 0;
	if(!write_address(address)) return 0;
	// In some operating conditions, the Master has to wait for a delay of 1 ms after receiving the Acknowledge and before sending the data frame 
	delay(1); 

	unsigned char* d = malloc(size+1);
	if(d == NULL) {
		printf("Allocation data failed\n");
		return 0;
	}

	d[0] = size-1;
	memcpy(d+1,mem,size);

	send_data_frame(d,size+1);
	free(d);

	// delay
	int delay_time = ceil(size*WORD_PROGRAMMING_TIME);

	// printf("delay ms: %d, N = %d\n",delay_time,size);
	delay(delay_time);


	if(!get_ack()) {
		printf("Write Memory (addr: 0x%08x, size: %d) failed\n",address,size);
		return 0;
	}else{
		// printf("is completed\n");
		return 1;	
	}
}

int erase_memory_cmd(short* page_numbers, unsigned short N, int delay_ms){

	if(!cmd_frame(CMD_ERASE)) return 0;


	if(N != ERASE_SPECIAL_GLOBAL){
		printf("classic errase does not work, USE GLOBAL erase instead\n");
		
		exit(-1);
		/*
		printf("Erasing page(s): ");
		for(int i=0;i<N;i++){
			if(i==(N-1)){
				printf("%d", page_numbers[i]);
			}else{
				printf("%d,", page_numbers[i]);	
			}
		}
		printf(" ...");
	
	short swapped_N = N-1; 
	swapped_N = __bswap_16(swapped_N); 
	send_data_frame((unsigned char*)&swapped_N,2);
	
	
		if(!get_ack()) {
			printf("Error, pages were NOT accepted\n");
			return 0;
		}

		unsigned char* d = malloc(2*N);

		if(d==NULL){
			printf("Allocation data failed\n");
			return 0;
		}
		for (int i = 0; i < N; i++)
		{
			swapped_N = page_numbers[i];
			swapped_N = __bswap_16(swapped_N);
			memcpy(d+2*i,&swapped_N,2);
		}
		send_data_frame(d,N);
		free(d);
		delay(delay_ms);
		// normal erase
		*/

	}else{
		short swapped_N = __bswap_16(N); 
		send_data_frame((unsigned char*)&swapped_N,2);
		delay(MASS_ERASE_TIME);
	}

	if(!get_ack()){
		printf("failed\n");
		return 0;
	}else{
		printf("done\n");
		return 1;
	}
}

// int write_protect_cmd(unsigned char* sectors, unsigned char sectors_count){
// 	if(!cmd_frame(CMD_WRITE_PROTECT)) return 0;
// 	unsigned char N = sectors_count-1;
// 	spi_write(N);
// 	spi_write(~N);
// 	if(!get_ack()) {
// 		printf("Number of sectors wasn't accepted\n");
// 		return 0;
// 	}

// 	send_data_frame(sectors,sectors_count);
// 	if(!get_ack()) {
// 		printf("Sectors numbers weren't accepted\n");
// 		return 0;
// 	}

// 	printf("Write protect is completed\n");
// 	return 1;
// }

// int write_unprotect_cmd(){
// 	// if(!cmd_frame(CMD_WRITE_UNPROTECT)) return 0;
// 	spi_write(0x5A);
// 	spi_write(0x73);
// 	spi_write(0x8C);
// 	if(!get_ack()) {
// 		printf("Write unprotect failed1\n");
// 		return 0;
// 	}
	
// 	if(!get_ack()) {
// 		printf("Write unprotect failed2\n");
// 		return 0;
// 	}
	
// 	printf("Write unprotect is completed\n");
// 	return 1;
// }

// int readout_protect_cmd(){
// 	cmd_frame(CMD_READOUT_PROTECT);
// 	if(!get_ack()) {
// 		printf("Readout protect failed\n");
// 		return 0;
// 	}
// 	printf("Readout protect is completed\n");
// 	return 1;
// }

// int readout_unprotect_cmd(){
// 	cmd_frame(CMD_READOUT_UNPROTECT);
// 	delay(MASS_ERASE_TIME);
// 	if(!get_ack()) {
// 		printf("Readout unprotect failed\n");
// 		return 0;
// 	}
// 	printf("Readout unprotect is completed\n");
// 	return 1;
// }

unsigned char* read_bin(unsigned int addr_start, long size){
	FILE * f = fopen("read.bin","w");
	if(f == NULL){
		printf("File error\n");
		exit(-1);
	}

	unsigned char* buffer = (unsigned char *) malloc(size);

	unsigned int full_pages_count = size/PAGE_SIZE;
	unsigned int remaining_bytes = size - full_pages_count*PAGE_SIZE;

	/* cteni pameti*/
	for (int i = 0; i <= full_pages_count; i++)
	{
		if(!read_memory_cmd(addr_start + i*PAGE_SIZE,(unsigned char*)(buffer+i*PAGE_SIZE),i==full_pages_count?remaining_bytes:PAGE_SIZE)){
			printf("Reading page %d...failed\n",i);
			return NULL;
		}else{
			// printf("done\n");
		}
	}
	
	/* cteni pameti konec*/
	fwrite(buffer,1,size,f);
	fclose(f);

	return buffer;
}


long load_bin(char* path_to_bin, unsigned char** buffer)
{
  FILE* pFile;
  long size;

  pFile = fopen(path_to_bin, "rb");
  	if (pFile==NULL) { printf("File error\n"); 
  	exit(-1);
  }

  // obtain file size:
  fseek(pFile, 0, SEEK_END);
  size = ftell (pFile);
  rewind (pFile);

  // allocate memory to contain the whole file:
  *buffer = (unsigned char*)malloc (sizeof(unsigned char)*size);
  if (*buffer == NULL) {
  	printf("Memory error"); 
  	exit(-2);
  }

  // copy the file into the buffer:
  if(fread (*buffer,1,size,pFile)!=size){
	printf("Reading error"); 
  	exit(-3);
  }
  fclose (pFile);
  return size;
}


void load_program(unsigned int addr_start, unsigned char* buffer, unsigned int size){

	if(buffer == NULL){
		printf("buffer == null pointer\n");
		return;
	}

	unsigned int addr_end = addr_start + size - 1;
	printf("Program starts at 0x%08x, ends at 0x%08x, size is %d\n", addr_start, addr_end, size);

	if(!is_in_range(addr_start,FLASH_MEM_ADDRESS_START ,FLASH_MEM_ADDRESS_END) || !is_in_range(addr_end,FLASH_MEM_ADDRESS_START ,FLASH_MEM_ADDRESS_END)){
		printf("Adresy jsou mimo rozsah");
		return;
	}

	// /*Erase*/
	// for(int i = 0; i < NUMBER_OF_SECTORS; i++){
	// 	if(is_in_range(sectors[i].addr_start,addr_start,addr_end) || is_in_range(sectors[i].addr_end,addr_start,addr_end)){
	// 		short sec = (short)i;
	// 		erase_memory_cmd(&sec,1,sector_erase_time(sectors + i));
	// 	}
	// }
 	global_erase_cmd();

	/*Loading*/
	printf("Loading program\n");
	int full_pages_count = size/PAGE_SIZE;
	int remaining_bytes = size - full_pages_count*PAGE_SIZE;
	printf("Full pages count: %d\nRemaining bytes count: %d\n", full_pages_count, remaining_bytes);

	for(unsigned int i = 0; i <= full_pages_count; i++){
		int addr = addr_start + i*PAGE_SIZE;
		// printf("Writing page %d\n",i);
	  	if(!write_memory_cmd(addr,(unsigned char*)(buffer + i*PAGE_SIZE), i==full_pages_count?remaining_bytes:PAGE_SIZE)){
	  		printf("Reading failed\n");
	  		return;
	  	}
	}

	printf("Program is fully loaded\n");
}

long int verification(unsigned char* b1, unsigned char* b2, long size){
	long i;
	for (i = 0; i < size; i++)
	{
		if(b1[i] != b2[i]){
			printf("verification failed on addr 0x%08x\n",(unsigned int)i);
			return i;
		}
	}
	printf("verification is succesful\n");
	return ++i;
}

static void print_usage(){
	//TODO add help
	return;
}


static void parse_opts(int argc, char *argv[])
{
	int c;
	int errflg;

	while((c = getopt(argc,argv,":rivp:")) != -1){
		switch(c){
			case 'r':{
				reset = true;
				break;
			}
			case 'i':{
				info = true;
				break;
			}
			case 'v':{
				verif = true;
				break;
			}
			case 'p':{
				printf("%s\n", optarg);
				int input = 0;
				optind--;
        		for( ;optind < argc && *argv[optind] != '-'; optind++){
        			if(input == 0){
        				int l = strlen(argv[optind]);
        				bin_path = malloc(l);	
        				strcpy(bin_path,argv[optind]);
        				input++;
        			} else if(input == 1){
						sscanf(argv[optind], "%x", &address);
						input++;
					} 
				}
        		if(input != 2){
        			printf("Option -%c requires two operand(s)\n",c);
        			errflg++;
        		}else{
        			prog = true;
        		}
        		break;
			}
			case ':':{
				printf("Option -%c requires an operand\n", optopt);
				errflg++;
				break;
			}
			case '?':{
				printf("Unrecognized option: -%c\n", optopt);
				errflg++;
				break;
			}
		}
	}

	if(errflg){
		print_usage();
		exit(-2);
	}

}


int main(int argc, char *argv[])
{
	// Initialization
	wiringPiSetup();
	// wiringPiSetupGpio();
	int channel = 1;
	int speed = 500000;
	int fd = wiringPiSPISetup(channel,speed);
	if(fd == -1) {
		printf("SPI initialization failed\n");
		return -1;
	}
	pinMode(RESET_PIN,OUTPUT);
	pinMode(BOOT0_PIN,OUTPUT);
	parse_opts(argc, argv);

	// Main program
	// /* commands part */
	if(info){
		stm_reset(1);
		get_cmd();
		get_version_cmd();
		get_id_cmd();
		stm_reset(0);
	}

	if(prog){
		stm_reset(1);

		unsigned char* buffer;

		long size = load_bin(bin_path,&buffer);
		load_program(address,buffer,(unsigned int)size);
		if(verif)
		{
			unsigned char* buf2 = read_bin(address,size);
			verification(buf2,buffer,size);
			free(buf2);	
		}
		free(buffer);
		free(bin_path);
		stm_reset(0);
	}

	if(reset){
		stm_reset(0);
	}

	close(fd);

	return 0;
}




