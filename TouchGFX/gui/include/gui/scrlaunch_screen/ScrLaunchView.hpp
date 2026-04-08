#ifndef SCRLAUNCHVIEW_HPP
#define SCRLAUNCHVIEW_HPP

#include <gui_generated/scrlaunch_screen/ScrLaunchViewBase.hpp>
#include <gui/scrlaunch_screen/ScrLaunchPresenter.hpp>

class ScrLaunchView : public ScrLaunchViewBase
{
public:
    ScrLaunchView();
    virtual ~ScrLaunchView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
		
	virtual void alu_change_screen(int index_screen, AluDynList* list);
protected:
};

#endif // SCRLAUNCHVIEW_HPP
