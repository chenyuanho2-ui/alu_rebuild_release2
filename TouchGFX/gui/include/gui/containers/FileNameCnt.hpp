#ifndef FILENAMECNT_HPP
#define FILENAMECNT_HPP

#include <gui_generated/containers/FileNameCntBase.hpp>

#ifndef SIMULATOR  // 如果没定义这个前缀,引用之前头文件的标准库
	#include "alu_file.h" 
#else  // 在定义前缀的情况下,直接引用,别问，问就是TouchGFX模拟看不到外面
	#include <iostream>
	#include <string>
	#include <stdio.h>
	#include <stdlib.h>
	#include <wchar.h>
#endif

class FileNameCnt : public FileNameCntBase
{
public:
    FileNameCnt();
    virtual ~FileNameCnt() {}

    virtual void initialize();
	
	int element_index;   // 存储列表的第几个

	void add_list(const char* text_utf8,int list_index);  // 添加元素
		
	virtual void AluDeleteFile();   // 删除本元素,以及获取该元素的索引返回后台
		
	void setAction(GenericCallback< FileNameCnt& >& callback);
	GenericCallback< FileNameCnt& >* viewCallback;
protected:
};

#endif // FILENAMECNT_HPP
