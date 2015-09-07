
#define FlashMem &FlashData[]
//0x0801F800
const unsigned int FlashData[6]={0};
#define SIZE sizeof(FlashData)
unsigned int STMFLASH_ReadHalfWord(unsigned int faddr)
{  
		return *(unsigned int*)faddr; 
}

void flashwrite(unsigned int data,unsigned int num)
{
	HAL_FLASH_Unlock();
	HAL_FLASH_Program(TYPEPROGRAM_WORD,FlashMem+num*SIZE,data);
	FLASH_WaitForLastOperation(100);
	HAL_FLASH_Lock();
}