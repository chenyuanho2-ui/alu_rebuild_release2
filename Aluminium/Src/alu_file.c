#include "alu_file.h"


FATFS    AluSD_FatFs;    // 文件系统对象
FRESULT  AluSD_statu;    // 操作结果

FIL	     AluFile;			   // 文件对象
UINT 	   AluFile_len;		 //	数据长度

DIR      AluDir;         // 目录对象
FILINFO  AluFile_info;   // 文件结构体信息


void Alu_SD_mount(void)	//判断FatFs是否挂载成功，若没有创建FatFs则格式化SD卡
{
//	FATFS_LinkDriver(&SD_Driver, SDPath);		// 初始化驱动
	AluSD_statu = f_mount(&AluSD_FatFs, "0:",1);	//	挂载SD卡 f_mount(&AluSD_FatFs, SDPath,1);
}


void Alu_SD_write(BYTE WriteBuffer[],int Buffer_Size, const char* filename) {	//文件创建和写入测试
	/* 
		WriteBuffer:待写入的数据          WriteBuffer[] = "STM32H750 SD卡文件系统测试 汪";
		Buffer_Size:写入数据长度					Buffer_Size = sizeof(WriteBuffer) = 20
		filename: 文件名                  filename = "0:Celsius.csv" 
	*/
	AluSD_statu = f_open(&AluFile,filename,FA_OPEN_APPEND | FA_WRITE);	//打开文件，若不存在则创建该文件
	if(AluSD_statu == FR_OK)  //成功打开文件
	{
		AluSD_statu = f_write(&AluFile,WriteBuffer,Buffer_Size,&AluFile_len);	//向文件写入数据  //	WriteBuffer,sizeof(WriteBuffer)
		f_close(&AluFile);	  //关闭文件			
	}
}

void Alu_SD_del_file(const char* filename) { // 使用FatFs删除文件之前，需要打开再关闭文件获取文件对象，从而传递给f_unlink
    AluSD_statu = f_open(&AluFile, filename, FA_READ);
    if (AluSD_statu != FR_OK) {
		printf("Open file failed, statu: %d",AluSD_statu);  // 文件打开失败
    }
    f_close(&AluFile);	// 关闭文件
    AluSD_statu = f_unlink(filename); // 删除文件
}


int Alu_SD_csv_num(const TCHAR *sniff_path)  // 获取对应目录下的文件最大标号数量，默认文件按照 dir_[num].csv存储
{
	// 每个文件的，文件名、文件编号(字符串形式)
	char *fileName;
	char *fileNum;

	int num_dat_csv = 0;   // 有几个文件
	int num_max     = 0;	 // 最大的文件编号
	int num_cache   = 0;	 // 临时的文件编号
	
	AluSD_statu = f_opendir(&AluDir, sniff_path);  // 打开路径
	if (AluSD_statu != FR_OK) {}  // 处理错误 但是没什么用

	while (1) {
		AluSD_statu = f_readdir(&AluDir, &AluFile_info);  // 获取文件结构体信息

		if ((AluSD_statu != FR_OK) || (AluFile_info.fname[0] == 0)) {  // 遍历完成或发生错误，退出循环
			break;
		}
		if (AluFile_info.fattrib & AM_DIR) { // 文件属性是目录,不管了跳过
		} else {                             // 文件属性是文件
			
			char *index_dot = strrchr(AluFile_info.fname, '.');  // 查找最后一个点号，该点号后的部分是文件后缀
			fileName = Alu_strndup(AluFile_info.fname, index_dot - AluFile_info.fname);  // 文件从头复制到index_dot的位置,两个指针相减相当于直接计算文件名长度
			
			char *index_under = strrchr(fileName, '_');          // 查找最后一个"_"号
			
			if ((index_dot != NULL) && (index_under != NULL)) {  // 按照dat_{数字}.csv,如果这两个找到就OK了
				
				fileNum = Alu_strcopy(index_under + 1);             // 新的字符串从"_"后面复制到"\0",刚好是数字部分的字符串
				
				num_cache = atoi(fileNum);                         // 将字符串转换为整数,strtol也可以但那个是long
				
				if (num_cache > num_max) {     // 判断该文件是否为最大
					num_max = num_cache;
				}
				num_dat_csv++;                 // csv文件计数(没用到)
			}
		}  // 判断下一个文件
	} 	   // 跳出文件遍历
	f_closedir(&AluDir);
	return num_max;
}




char *Alu_strndup(const char *str,unsigned int n) {  // 字符串从头向后拷贝n个
	unsigned int len = strlen(str);  // 计算str的长度
	if (n < len) {                   // 如果n小于str长度就按照n的来,如果n大于str的长度那就复制整个字符串
		len = n;
	}
	char *dup_str = (char *)malloc(len + 1);  // 动态分配内存
	if (dup_str) {                            
		memcpy(dup_str, str, len);              // 将str的前len个字符复制到dup_str中
		dup_str[len] = '\0';                    // 最后一个字符位置添加结束符
	}
	return dup_str;
}



char *Alu_strcopy(const char *str) {  // 字符串复制函数
	unsigned int len = strlen(str); 
	char *dup_str = (char *)malloc(len + 1);
	if (dup_str) {
		strcpy(dup_str, str);
	}
	return dup_str;
}


/* 列表文件初始化 */
void Alu_list_init(AluDynList* list) {
    list->items = NULL;
    list->size = 0;
    list->capacity = 0;
}

/* 列表添加文件对象 */
void Alu_list_add(AluDynList* list, const char* fileName) {
    if (list->size >= list->capacity) {
        list->capacity += 10;
        list->items = (char**)realloc(list->items, list->capacity * sizeof(char*));
    }

    list->items[list->size] = (char*)malloc(strlen(fileName) + 1);
    strcpy(list->items[list->size], fileName);
    list->size++;
}

/* 列表删除文件对象 */
void Alu_list_del(AluDynList* list, int index){
    if (index < 0 || index >= list->size) {
        return;  // 索引无效
    }
    free(list->items[index]);  // 释放内存
    // 将后面的元素向前移动
    for (int i = index; i < list->size - 1; i++) {
        list->items[i] = list->items[i + 1];
    }
    list->size--;
}


void Alu_sniff_files(AluDynList* list, const TCHAR *sniff_path){
//    FILINFO fileInfo;
//    DIR dir;
//    FRESULT res;

    AluSD_statu = f_opendir(&AluDir, sniff_path);
    if (AluSD_statu != FR_OK) {return;}  // 打开目录失败
    
    while (1) {
        AluSD_statu = f_readdir(&AluDir, &AluFile_info);
        if (AluSD_statu != FR_OK || AluFile_info.fname[0] == 0) {
            break;  // 读取完毕或出错
        }

        if (AluFile_info.fattrib & AM_DIR) {
            // 忽略文件夹
            continue;
        }else

        Alu_list_add(list, AluFile_info.fname);
    }

    f_closedir(&AluDir);

}













