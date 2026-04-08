#include <gui/containers/FileNameCnt.hpp>

FileNameCnt::FileNameCnt():viewCallback(0)
{

}

void FileNameCnt::initialize()
{
    FileNameCntBase::initialize();
}

/*
TouchGFX字符全是unicode字符，直接传char*会乱码，需要传wchat_t*
传入UTF-8编码的字符串(char*)      char*    text_utf8  =  "data123"; 
转换UTF-16 的宽字符串(wchar_t*)   wchar_t* text_utf16 = L"data123"; 
使用snprintf转换进入给textArea1Buffer,刷新
*/
void FileNameCnt::add_list(const char* text_utf8,int list_index)
{
    size_t len = mbstowcs(NULL, text_utf8, 0);
	wchar_t* text_utf16 = (wchar_t*)malloc((len + 1) * sizeof(wchar_t));
	mbstowcs(text_utf16, text_utf8, len + 1);
    touchgfx::Unicode::snprintf(textArea1Buffer,TEXTAREA1_SIZE,"%s",text_utf16);  
    textArea1.resizeToCurrentText();
    invalidate();
	
	element_index = list_index;

}


/* 自定义容器的回调函数 */
void FileNameCnt::AluDeleteFile()
{
    if (viewCallback->isValid())
    {
        viewCallback->execute(*this);
    }
}

/* 和view相关的回调 */
void FileNameCnt::setAction(GenericCallback< FileNameCnt& >& callback)
{	
    viewCallback = &callback;
}








