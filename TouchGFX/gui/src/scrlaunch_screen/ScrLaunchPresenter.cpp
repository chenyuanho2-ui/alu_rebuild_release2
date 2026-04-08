#include <gui/scrlaunch_screen/ScrLaunchView.hpp>
#include <gui/scrlaunch_screen/ScrLaunchPresenter.hpp>

ScrLaunchPresenter::ScrLaunchPresenter(ScrLaunchView& v)
    : view(v)
{

}

void ScrLaunchPresenter::activate()
{

}

void ScrLaunchPresenter::deactivate()
{

}

void ScrLaunchPresenter::alu_to_screen(int index_screen, AluDynList* list)
{
	view.alu_change_screen(index_screen, list);
}
