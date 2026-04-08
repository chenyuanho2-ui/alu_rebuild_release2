#ifndef MODELLISTENER_HPP
#define MODELLISTENER_HPP

#include <gui/model/Model.hpp>

class ModelListener
{
public:
    ModelListener() : model(0) {}
    
    virtual ~ModelListener() {}
			
		virtual void alu_to_temperature(double temp,float pwm) {}
		virtual void alu_to_screen(int index_screen, AluDynList* list) {}  // AluDynList 引用于model.hpp
		virtual void alu_to_choose(int index_choose) {}
		virtual void alu_to_thres(int index_choose,float temp_thres,float power_thres, int num_file) {}  // 对于页面1更新阈值，页面2更新阈值
		
    void bind(Model* m)
    {
        model = m;
    }
protected:
    Model* model;
};

#endif // MODELLISTENER_HPP
