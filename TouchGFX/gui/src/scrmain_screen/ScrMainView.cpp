#include <gui/scrmain_screen/ScrMainView.hpp>

#ifndef SIMULATOR

	extern AluDynList sd_file_list;   // �ļ��б�
	extern int    index_choose;       // �������� ���ʻ����¶�
	extern float  temp_thres;	      // �¶���ֵ
	extern float  power_thres;        // ������ֵ
	
//	extern void Alu_list_init(AluDynList* list);
//	extern void Alu_sniff_files(AluDynList* list, const TCHAR *sniff_path);
#endif


ScrMainView::ScrMainView(): listCntClickCallback(this, &ScrMainView::listCntClick),
                             btnAddThresCallback(this, &ScrMainView::btnAddClicked),
                             btnDecThresCallback(this, &ScrMainView::btnDecClicked)
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
		//printf("File %d: %s\n", i+1, sd_file_list.items[i]);
		listElements[i].add_list(sd_file_list.items[i],i);
		listElements[i].setAction(listCntClickCallback);  // �ص�����
		listLayout1.add(listElements[i]);
		
	}

	btnAddThres.setAction(btnAddThresCallback);
	btnDecThres.setAction(btnDecThresCallback);

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

/* ͨ����Ļ��ťԪ�ش���,����ҳ�� */
void ScrMainView::alu_change_screen(int index_screen, AluDynList* list)
{
	if (index_screen==1){
		application().gotoScreen1ScreenNoTransition();
	}
}

/* ͨ��Ӳ����������,ѡ����ֵ���߹������� */
void ScrMainView::alu_change_choose(int index_choose)
{
	if (index_choose==0)  //��ʾ��ǰ�����¶���ֵ
	{  
		tempArea1.setColor(touchgfx::Color::getColorFromRGB(100, 255, 255));
		tempArea1.invalidate();
		powerArea1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
		powerArea1.invalidate();
	}
	else if(index_choose==1)  //��ʾ��ǰ���ù�����ֵ
	{
		tempArea1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
		tempArea1.invalidate();
		powerArea1.setColor(touchgfx::Color::getColorFromRGB(100, 255, 255));
		powerArea1.invalidate();
	}
}

/* ͨ��Ӳ����������,������ֵ���¶ȵ�����ֵ */
void ScrMainView::alu_change_thres(int index_choose,float temp_thres,float power_thres)
{
	if (index_choose==0)      // �޸��¶���ֵ
	{  
		Unicode::snprintfFloat(tempArea1Buffer,TEMPAREA1_SIZE,"%3.0f",temp_thres);
		tempArea1.resizeToCurrentText();
		tempArea1.invalidate();
	}
	else if(index_choose==1)  // �޸Ĺ�����ֵ
	{
		Unicode::snprintfFloat(powerArea1Buffer,POWERAREA1_SIZE,"%2.1f",power_thres);
		powerArea1.resizeToCurrentText();
		powerArea1.invalidate();
	}
}

/* �ص�����������listCntClickCallback */
void ScrMainView::listCntClick(FileNameCnt& element)
{
	/*
	��õ�������,
	Ѱ�ҵ�sd_file_list���б�Ԫ��(������Ԫ���б�,ÿ��setupscreen��ʱ����ˢ���б�)��
	ɾ�����ļ�,���ļ�����-1,
	�о�ֱ�����ú���̫��,�������ش�?
	*/
	#ifndef SIMULATOR
		presenter->alu_back_delFile(element.element_index, sd_file_list.items[element.element_index]);
	#endif

    listLayout1.remove(element);
    scrollableContainer1.invalidate();
}

extern osSemaphoreId alu_thresholdHandle;

void ScrMainView::btnAddClicked(const touchgfx::AbstractButton& src)
{
    #ifndef SIMULATOR
    if (index_choose == 0)
    {
        if (temp_thres < 150)
        {
            temp_thres = temp_thres + 5;
        }
        else
        {
            temp_thres = 150;
        }
    }
    else if (index_choose == 1)
    {
        if (power_thres < 9.0f)
        {
            power_thres = power_thres + 0.1f;
        }
        else
        {
            power_thres = 9.0f;
        }
    }
    osSemaphoreRelease(alu_thresholdHandle);
    #endif
}

void ScrMainView::btnDecClicked(const touchgfx::AbstractButton& src)
{
    #ifndef SIMULATOR
    if (index_choose == 0)
    {
        if (temp_thres >= 5)
        {
            temp_thres = temp_thres - 5;
        }
        else
        {
            temp_thres = 0;
        }
    }
    else if (index_choose == 1)
    {
        if (power_thres >= 0.1f)
        {
            power_thres = power_thres - 0.1f;
        }
        else
        {
            power_thres = 0.0f;
        }
    }
    osSemaphoreRelease(alu_thresholdHandle);
    #endif
}



