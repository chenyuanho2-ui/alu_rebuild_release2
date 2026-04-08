#include <gui/scrmain_screen/ScrMainView.hpp>

#ifndef SIMULATOR

	extern AluDynList sd_file_list;   // 文件列表
	extern int    index_choose;       // 设置索引 功率还是温度
	extern float  temp_thres;	      // 温度阈值
	extern float  power_thres;        // 功率阈值
	
//	extern void Alu_list_init(AluDynList* list);
//	extern void Alu_sniff_files(AluDynList* list, const TCHAR *sniff_path);
#endif


ScrMainView::ScrMainView(): listCntClickCallback(this, &ScrMainView::listCntClick)
{

}

void ScrMainView::setupScreen()  
{
    ScrMainViewBase::setupScreen();
	
	#ifndef SIMULATOR
	Alu_list_init(&sd_file_list);
	Alu_sniff_files(&sd_file_list,"/");
	
	listLayout1.setHeight(0);
	for (int i = 0; i < sd_file_list.size; i++) {
		printf("File %d: %s\n", i+1, sd_file_list.items[i]);
		listElements[i].add_list(sd_file_list.items[i],i);
		listElements[i].setAction(listCntClickCallback);  // 回调相关
		listLayout1.add(listElements[i]);
		
	}
	if (index_choose==0)
	{  
		tempArea1.setColor(touchgfx::Color::getColorFromRGB(100, 255, 255));
		tempArea1.invalidate();
		powerArea1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
		powerArea1.invalidate();
	}
	else if(index_choose==1)
	{
		tempArea1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
		tempArea1.invalidate();
		powerArea1.setColor(touchgfx::Color::getColorFromRGB(100, 255, 255));
		powerArea1.invalidate();
	}
		
	Unicode::snprintfFloat(tempArea1Buffer,TEMPAREA1_SIZE,"%3.0f",temp_thres);
	tempArea1.resizeToCurrentText();
	tempArea1.invalidate();
		
	Unicode::snprintfFloat(powerArea1Buffer,POWERAREA1_SIZE,"%2.1f",power_thres);
	powerArea1.resizeToCurrentText();
	powerArea1.invalidate();
		
	#endif
		

//	listLayout1.setHeight(0);
//	
//	listElements[1].add_list("dat_21.csv");
//	listElements[2].add_list("dat222.csv");
//	listElements[3].add_list("dat111.csv");
//	listElements[4].add_list("dat333.csv");
//	listElements[5].add_list("dat111.csv");
//	listElements[6].add_list("dat111.csv");
//	listElements[7].add_list("dat111.csv");
//	listElements[8].add_list("dat111.csv");
//	listElements[9].add_list("dat111.csv");

//	for (uint8_t i = 0; i < 10; ++i)
//    {
//        listLayout1.add(listElements[i]);
//    }
}

void ScrMainView::tearDownScreen()
{
    ScrMainViewBase::tearDownScreen();
}

/* 通过屏幕按钮元素触发,更改页面 */
void ScrMainView::alu_change_screen(int index_screen, AluDynList* list)
{
	if (index_screen==1){
		application().gotoScreen1ScreenNoTransition();
	}
}

/* 通过硬件按键触发,选择阈值或者功率设置 */
void ScrMainView::alu_change_choose(int index_choose)
{
	if (index_choose==0)  //表示当前设置温度阈值
	{  
		tempArea1.setColor(touchgfx::Color::getColorFromRGB(100, 255, 255));
		tempArea1.invalidate();
		powerArea1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
		powerArea1.invalidate();
	}
	else if(index_choose==1)  //表示当前设置功率阈值
	{
		tempArea1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
		tempArea1.invalidate();
		powerArea1.setColor(touchgfx::Color::getColorFromRGB(100, 255, 255));
		powerArea1.invalidate();
	}
}

/* 通过硬件按键触发,更改阈值或温度的设置值 */
void ScrMainView::alu_change_thres(int index_choose,float temp_thres,float power_thres)
{
	if (index_choose==0)      // 修改温度阈值
	{  
		Unicode::snprintfFloat(tempArea1Buffer,TEMPAREA1_SIZE,"%3.0f",temp_thres);
		tempArea1.resizeToCurrentText();
		tempArea1.invalidate();
	}
	else if(index_choose==1)  // 修改功率阈值
	{
		Unicode::snprintfFloat(powerArea1Buffer,POWERAREA1_SIZE,"%2.1f",power_thres);
		powerArea1.resizeToCurrentText();
		powerArea1.invalidate();
	}
}

/* 回调函数，传给listCntClickCallback */
void ScrMainView::listCntClick(FileNameCnt& element)
{
	/*
	获得到该索引,
	寻找到sd_file_list的列表元素(不更新元素列表,每次setupscreen的时候再刷新列表)，
	删除该文件,总文件数量-1,
	感觉直接引用好像不太行,试着往回传?
	*/
	#ifndef SIMULATOR
		presenter->alu_back_delFile(element.element_index, sd_file_list.items[element.element_index]);
	#endif

    listLayout1.remove(element);
    scrollableContainer1.invalidate();
}



