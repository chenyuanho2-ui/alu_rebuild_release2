#ifndef SCREEN1VIEW_HPP
#define SCREEN1VIEW_HPP

#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

class Screen1View : public Screen1ViewBase
{
public:
    Screen1View();
    virtual ~Screen1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

	virtual void alu_back_func_demo();  
	virtual void alu_show_temperature(double temp,float pwm);
	virtual void alu_show_thres(float temp_thres,float power_thres, int num_file);
	virtual void alu_change_screen(int index_screen);
protected:
};

#endif // SCREEN1VIEW_HPP
