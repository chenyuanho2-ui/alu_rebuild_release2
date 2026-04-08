#include <gui/scrmain_screen/ScrMainView.hpp>
#include <gui/scrmain_screen/ScrMainPresenter.hpp>

ScrMainPresenter::ScrMainPresenter(ScrMainView& v)
    : view(v)
{

}

void ScrMainPresenter::activate()
{

}

void ScrMainPresenter::deactivate()
{

}

void ScrMainPresenter::alu_to_screen(int index_screen, AluDynList* list)
{
	view.alu_change_screen(index_screen, list);
}

void ScrMainPresenter::alu_to_choose(int index_choose)
{
	view.alu_change_choose(index_choose);
}

void ScrMainPresenter::alu_to_thres(int index_choose,float temp_thres,float power_thres, int num_file)
{
	view.alu_change_thres(index_choose,temp_thres,power_thres);
}

void ScrMainPresenter::alu_back_delFile(int file_index,const char * file_name)
{
	model->alu_do_back_delFile(file_index, file_name);
}
