#ifndef SCRMAINVIEW_HPP
#define SCRMAINVIEW_HPP

#include <gui_generated/scrmain_screen/ScrMainViewBase.hpp>
#include <gui/scrmain_screen/ScrMainPresenter.hpp>

#include <touchgfx/Color.hpp>              // ��scrmainviewbase.cpp�︴�Ƶ�,����Ԫ����ɫ
#include <gui/containers/FileNameCnt.hpp>  // ���뵥�ļ�����
	
class ScrMainView : public ScrMainViewBase
{
public:
    ScrMainView();
    virtual ~ScrMainView() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
	virtual void alu_change_screen(int index_screen, AluDynList* list);
	virtual void alu_change_choose(int index_choose);
	virtual void alu_change_thres(int index_choose,float temp_thres,float power_thres);
		
	FileNameCnt listElements[1024];                   // �ļ�����ʵ����
		

    void listCntClick(FileNameCnt& element);
	Callback<ScrMainView, FileNameCnt&> listCntClickCallback;

	virtual void btnAddClicked(const touchgfx::AbstractButton& src);
	virtual void btnDecClicked(const touchgfx::AbstractButton& src);
	Callback<ScrMainView, const touchgfx::AbstractButton&> btnAddThresCallback;
	Callback<ScrMainView, const touchgfx::AbstractButton&> btnDecThresCallback;
protected:
//	static const int numberOfFileNameCnt = 10;      // �����ļ�����
    
};

#endif // SCRMAINVIEW_HPP
