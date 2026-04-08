#include <gui/scrlaunch_screen/ScrLaunchView.hpp>

ScrLaunchView::ScrLaunchView()
{

}

void ScrLaunchView::setupScreen()
{
    ScrLaunchViewBase::setupScreen();
}

void ScrLaunchView::tearDownScreen()
{
    ScrLaunchViewBase::tearDownScreen();
}

void ScrLaunchView::alu_change_screen(int index_screen, AluDynList* list)
{
	if (index_screen==0){  // Ìø×ªµ½main
		 application().gotoScrMainScreenNoTransition();
	}
}
