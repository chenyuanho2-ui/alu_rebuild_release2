#ifndef MODEL_HPP
#define MODEL_HPP

#ifndef SIMULATOR
extern "C" 
{         
	#include "alu_file.h"
}
#else
	typedef struct {   // 自定义动态文件列表
		char** items;  // 文件对象
		int size;      // 共有多少个文件
		int capacity;  // 数组容量
	} AluDynList;  
#endif

class ModelListener;

class Model
{
public:
    Model();

    void bind(ModelListener* listener)
    {
        modelListener = listener;
    }

    void tick();
		
	void alu_do_back_test(bool state);
	
	void alu_do_back_delFile(int file_index,const char * file_name);
	
protected:
    ModelListener* modelListener;
};

#endif // MODEL_HPP
