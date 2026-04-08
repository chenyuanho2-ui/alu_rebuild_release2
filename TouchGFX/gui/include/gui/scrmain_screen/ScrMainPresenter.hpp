#ifndef SCRMAINPRESENTER_HPP
#define SCRMAINPRESENTER_HPP

#include <gui/model/ModelListener.hpp>
#include <mvp/Presenter.hpp>

using namespace touchgfx;

class ScrMainView;

class ScrMainPresenter : public touchgfx::Presenter, public ModelListener
{
public:
    ScrMainPresenter(ScrMainView& v);

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


    virtual ~ScrMainPresenter() {};
			
	virtual void alu_to_screen(int index_screen, AluDynList* list);
	virtual void alu_to_choose(int index_choose);
	virtual void alu_to_thres(int index_choose,float temp_thres,float power_thres, int num_file);
	
	virtual void alu_back_delFile(int file_index,const char * file_name);
private:
    ScrMainPresenter();

    ScrMainView& view;
};

#endif // SCRMAINPRESENTER_HPP
