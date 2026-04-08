#ifndef _ALU_FILE_H_ 
#define _ALU_FILE_H_ 

#include "main.h"
#include "fatfs.h"
//#include "usart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern FATFS    AluSD_FatFs;    // 文件系统对象
extern FRESULT  AluSD_statu;    // 操作结果

extern FIL	    AluFile;		 // 文件对象
extern UINT 	AluFile_len;		//	数据长度

extern DIR      AluDir;         // 目录对象
extern FILINFO  AluFile_info;   // 文件结构体信息


void Alu_SD_mount(void);
void Alu_SD_write(BYTE WriteBuffer[],int Buffer_Size, const char* filename);
void Alu_SD_del_file(const char* filename);

int Alu_SD_csv_num(const TCHAR *sniff_path);

char *Alu_strndup(const char *str, size_t n);
char *Alu_strcopy(const char *str);



typedef struct {   // 自定义动态文件列表
    char** items;  // 文件对象
    int size;      // 共有多少个文件
    int capacity;  // 数组容量
} AluDynList;     

void Alu_list_init(AluDynList* list);
void Alu_list_add(AluDynList* list, const char* fileName);
void Alu_list_del(AluDynList* list, int index);
void Alu_sniff_files(AluDynList* list, const TCHAR *sniff_path);

#endif // _ALU_FILE_H_ 
