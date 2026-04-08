#include <gui/screen1_screen/Screen1View.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

Screen1Presenter::Screen1Presenter(Screen1View& v)
    : view(v)
{

}

void Screen1Presenter::activate()
{

}

void Screen1Presenter::deactivate()
{

}
void Screen1Presenter::alu_to_screen(int index_screen, AluDynList* list)
{
	view.alu_change_screen(index_screen);
}

void Screen1Presenter::alu_back_test(bool state)
{
	model->alu_do_back_test(state);
}

void Screen1Presenter::alu_to_temperature(double temp,float pwm)
{
	view.alu_show_temperature(temp,pwm);
}
void Screen1Presenter::alu_to_thres(int index_choose,float temp_thres,float power_thres, int num_file)
{
	view.alu_show_thres(temp_thres, power_thres, num_file);
}
