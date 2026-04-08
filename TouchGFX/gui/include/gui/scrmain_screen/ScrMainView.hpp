#ifndef SCRMAINVIEW_HPP
#define SCRMAINVIEW_HPP

#include <gui_generated/scrmain_screen/ScrMainViewBase.hpp>
#include <gui/scrmain_screen/ScrMainPresenter.hpp>

#include <touchgfx/Color.hpp>              // 在scrmainviewbase.cpp里复制的,控制元素颜色
#include <gui/containers/FileNameCnt.hpp>  // 引入单文件容器
	
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
		
	FileNameCnt listElements[1024];                   // 文件容器实例化
		

    void listCntClick(FileNameCnt& element);                   // 回调函数接手
	Callback<ScrMainView, FileNameCnt&> listCntClickCallback;  // 回调函数
protected:
//	static const int numberOfFileNameCnt = 10;      // 传入文件个数
    
};

#endif // SCRMAINVIEW_HPP
