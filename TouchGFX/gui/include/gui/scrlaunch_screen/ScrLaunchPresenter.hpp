#ifndef SCRLAUNCHPRESENTER_HPP
#define SCRLAUNCHPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class ScrLaunchView;

class ScrLaunchPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    ScrLaunchPresenter(ScrLaunchView& v);

    /**
     * The activate function is called automatically when this screen is "switched in"
     * (ie. made active). Initialization logic can be placed here.
     */
    virtual void activate();

    /**
     * The deactivate function is called automatically when this screen is "switched out"
     * (ie. made inactive). Teardown functionality can be placed here.
     */
    virtual void deactivate();

    virtual ~ScrLaunchPresenter() {};
		
	virtual void alu_to_screen(int index_screen, AluDynList* list);
private:
    ScrLaunchPresenter();

    ScrLaunchView& view;
};

#endif // SCRLAUNCHPRESENTER_HPP
