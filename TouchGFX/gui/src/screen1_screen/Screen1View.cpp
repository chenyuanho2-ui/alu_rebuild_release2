#include <gui/screen1_screen/Screen1View.hpp>


Screen1View::Screen1View()
{

}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}



void Screen1View::alu_back_func_demo()
{
	presenter->alu_back_test(toggleButton1.getState());
}

void Screen1View::alu_show_temperature(double temp,float pwm)
{
	if (pwm>=1){
		Unicode::snprintf(pwmArea1Buffer,PWMAREA1_SIZE,"%d",100);
	}else{
		Unicode::snprintfFloat(pwmArea1Buffer,PWMAREA1_SIZE,"%2.1f",pwm*100);
	}
	pwmArea1.resizeToCurrentText();
	pwmArea1.invalidate();
	
	Unicode::snprintfFloat(tempArea1Buffer,TEMPAREA1_SIZE,"%.2f",temp);
	tempArea1.resizeToCurrentText();
	tempArea1.invalidate();
	tempGraph1.addDataPoint((float)temp);
}

void Screen1View::alu_change_screen(int index_screen)
{
	if (index_screen==0){
		application().gotoScrMainScreenNoTransition();
	}
}

void Screen1View::alu_show_thres(float temp_thres,float power_thres,int num_file)
{
	Unicode::snprintfFloat(thresTempArea1Buffer,THRESTEMPAREA1_SIZE,"%.0f",temp_thres);
	thresTempArea1.resizeToCurrentText();
	thresTempArea1.invalidate();
	
	Unicode::snprintfFloat(thresPowerArea1Buffer,THRESPOWERAREA1_SIZE,"%.1f",power_thres);
	thresPowerArea1.resizeToCurrentText();
	thresPowerArea1.invalidate();
	

	Unicode::snprintf(filenameArea1Buffer,FILENAMEAREA1_SIZE,"%d",num_file);
	filenameArea1.resizeToCurrentText();
	filenameArea1.invalidate();

	
	
	
}
