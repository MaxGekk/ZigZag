#include <windows.h>
#include <assert.h>

#include "_scrlib.h"

#define WM_INITSCRLIB   (WM_USER + 1)
#define SCREEN_SCALE    4
#define IDT_CARRIAGE	1
#define IDT_PUTCHAR		2



/**
 * In order to allocate BITMAPINFO plus necessary number of palette entries we create this
 * union
 */
static BYTE             _dummy[sizeof(BITMAPINFO) + 15 * sizeof(RGBQUAD)];
static BITMAPINFO* const g_BitmapInfo = (BITMAPINFO*) _dummy;

static HWND              g_hWindow;
/**
 * Screen back buffer. Bitmap bits 4bpp, maximum possible bitmap
 */
static BYTE              g_ScreenBuffer[SCREEN_WIDTH * SCREEN_HEIGHT >> 1];



LRESULT CALLBACK zzWndProc(HWND,  UINT,  WPARAM, LPARAM);




static void InitBitmapInfo();

int WINAPI WinMain
   (HINSTANCE hInstance, 
    HINSTANCE hPrevInstance, 
    LPSTR lpCmdLine, 
    int nCmdShow)
{
        
        MSG msg;
        WNDCLASS wndClass;
        
        ZeroMemory(&wndClass, sizeof(wndClass));
        
        wndClass.style          = CS_HREDRAW | CS_VREDRAW;
        wndClass.lpfnWndProc    = zzWndProc;
        wndClass.hInstance      = hInstance;
        wndClass.hCursor        = LoadCursor(0, IDC_ARROW);
        wndClass.hbrBackground  = (HBRUSH) COLOR_BACKGROUND;
        wndClass.lpszClassName  =  "ZigZag Window";

        if ( RegisterClass(&wndClass) == 0 ) {
            assert(0);
        }

        InitBitmapInfo();

        g_hWindow = CreateWindow
           (wndClass.lpszClassName,
            "ZigZag Emulator",
            WS_THICKFRAME | WS_CAPTION | WS_SYSMENU, 
            CW_USEDEFAULT, CW_USEDEFAULT,
            SCREEN_WIDTH * SCREEN_SCALE + 2 * GetSystemMetrics(SM_CXFRAME),
            SCREEN_HEIGHT * SCREEN_SCALE + 2 * GetSystemMetrics(SM_CYFRAME) + GetSystemMetrics(SM_CYCAPTION),
            NULL,
            NULL,
            hInstance,
            NULL);
        assert(g_hWindow != 0);    

	

        ShowWindow(g_hWindow, nCmdShow);
        UpdateWindow(g_hWindow);

        while(GetMessage(&msg, NULL, 0, 0))
        {
                TranslateMessage(&msg);
                DispatchMessage(&msg);
        }

        return (int) msg.wParam;
}


void OnKeyUp(HWND hwnd, WPARAM wParam, LPARAM lParam) {
	
    int msg = -1;
    switch(wParam)
    {
    case VK_UP:
        msg = KEY_UP;
        break;
    case VK_DOWN:
        msg = KEY_DOWN;
        break;
	case '0':
		msg = KEY_ZERO;
		break;
	case '1':
		msg = KEY_ONE;
		break;
	case '2':
		msg = KEY_TWO;
		break;
	case '3':
		msg = KEY_THREE;
		break;
	case '4':
		msg = KEY_FOUR;
		break;
	case '5':
		msg = KEY_FIVE;
		break;
	case '6':
		msg = KEY_SIX;
		break;
	case '7':
		msg = KEY_SEVEN;
		break;
	case '8':
		msg = KEY_EIGHT;
		break;
	case '9':
		msg = KEY_NINE;
		break;
	case VK_BACK:
		msg = KEY_BACKSPACE;
		break;
	case VK_RETURN:
		msg = KEY_ENTER;
		break;
	case VK_ESCAPE:
		msg = KEY_ESCAPE;
		break;
    }


    if ( msg != -1 ) {
       FireKeyMessage(msg);
    }

    if ( wParam == VK_TAB ) {
        RollFocus();
		SetTimer(g_hWindow, IDT_CARRIAGE, 500, (TIMERPROC) NULL);
    }
	
	if( wParam == VK_UP || wParam == VK_DOWN ) {
		KillTimer(g_hWindow, IDT_CARRIAGE);
		ChangeCarraigePositon((wParam == VK_UP) ? 1 : -1);
		SetTimer(g_hWindow, IDT_CARRIAGE, 500, (TIMERPROC) NULL); 
	}

	if( wParam >= '0' && wParam <= '9' ) {
		KillTimer(g_hWindow, IDT_CARRIAGE);
		SetTimer(g_hWindow, IDT_PUTCHAR, 2000, (TIMERPROC) NULL); 
		
	}

	if(wParam == VK_BACK) {
		SetTimer(g_hWindow, IDT_CARRIAGE, 500, (TIMERPROC) NULL); 
		
	}
}


void OnPaint(HWND hwnd) {
    PAINTSTRUCT ps;
    HDC hdc = BeginPaint(hwnd, &ps);
    RECT rcClient;
    assert(hdc != 0);

    GetClientRect(hwnd, &rcClient);

    g_BitmapInfo->bmiHeader.biWidth = SCREEN_WIDTH;
    g_BitmapInfo->bmiHeader.biHeight = -SCREEN_HEIGHT;
    /* Our bitmap stride must be multiple of DWORD and hence the screen
       width must be multiple of 8
    */   
    assert((SCREEN_WIDTH & 3) == 0);

	

    StretchDIBits
       (hdc,
        0, 0, rcClient.right, rcClient.bottom,
        0, 0, SCREEN_WIDTH, SCREEN_HEIGHT,
        g_ScreenBuffer,
        g_BitmapInfo,
        DIB_RGB_COLORS,
        SRCCOPY);
    EndPaint(hwnd, &ps);
}


LRESULT CALLBACK zzWndProc
   (HWND hwnd,
    UINT msg,
    WPARAM wParam,
    LPARAM lParam)
{
    switch(msg)
    {
    case WM_CREATE:
        /* We cannot draw ourselves here, so we postpone
          the initialization for a while
        */
        PostMessage(hwnd, WM_INITSCRLIB, 0, 0);
        return 0;
    
    case WM_INITSCRLIB:
        /* Now it is time to initialize ourselves and draw controls */
		SetTimer(g_hWindow, IDT_CARRIAGE, 500, (TIMERPROC) NULL); 
        InitializeScrLib();
        return 0;
	case WM_TIMER:
		switch (wParam) 
		{ 
		case IDT_CARRIAGE:
			CarriageState();
			return 0;
		case IDT_PUTCHAR:
			KillTimer(g_hWindow, IDT_PUTCHAR);
			SetTimer(g_hWindow, IDT_CARRIAGE, 500, (TIMERPROC) NULL); 
			CarriageState();
			return 0;
		}

		return 0;


    case WM_KEYUP:
        OnKeyUp(hwnd, wParam, lParam);
        return 0;
    
    case WM_PAINT:
        OnPaint(hwnd);
        return 0;

    case WM_ERASEBKGND:
        /* Do nothing here to avoid flickering during resizing */
        return 0;

    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }

    return DefWindowProc(hwnd, msg, wParam, lParam);
}

COLORREF ConvertColor(zz_coord_t color) {
    int c = 255 * color / (MAXCOLORS - 1);
    return RGB(c, c, c);
}

void OLED_putPixel(zz_coord_t x,  zz_coord_t y,  zz_color_t color)
{
    /* First draw the pixel in the back buffer */
    unsigned char* pDst;
    const int dst_stride = SCREEN_WIDTH >> 1;

    RECT rcClient;

    COLORREF rgbColor = ConvertColor(color);

    assert( (SCREEN_WIDTH & 1) == 0 );

    GetClientRect(g_hWindow, &rcClient);

    if ( x < 0 || x >= SCREEN_WIDTH || y < 0 || y >= SCREEN_HEIGHT )
        return;
        
    pDst = g_ScreenBuffer + y * dst_stride + (x >> 1);
    *pDst = (x & 1) ? (*pDst & 0xF0) | (color & 0x0F) : (*pDst & 0x0F) | (color << 4);
 
    /* Now draw it directly on the screen */
    InvalidateRect(g_hWindow, 0, FALSE);
}

void OLED_putBitmap(zz_coord_t x, zz_coord_t y, const struct OLED_bitmap * bmp)
{
    int width1, height1, dx, dy, line, column;
    const unsigned char* pSrc;
    unsigned char* pDst, val;
    const int src_stride = (bmp->bmpWidth >> 1) + (bmp->bmpWidth & 1);
    const int dst_stride = SCREEN_WIDTH >> 1;
    
    assert( (SCREEN_WIDTH & 1) == 0 );

    if ( x >= SCREEN_WIDTH || y >= SCREEN_HEIGHT )
        return; /* bitmap is not visible */
    
    /* coordinates within bitmap of the first visible top left point */
    dx = x < 0 ? -x : 0;
    dy = y < 0 ? -y : 0;

    /* dimensions of the visible part of the bitmap */
    width1 = bmp->bmpWidth - dx;
    height1 = bmp->bmpHeight - dy;

    for ( line = 0; line < height1; ++line ) { /* process scanlines */
        for ( column = 0; column < width1; ++column ) {
            /* process one pixel at a time */
            pSrc = bmp->BitmapData + line * src_stride + ((dx + column) >> 1);
            pDst = g_ScreenBuffer + (y + dy + line) * dst_stride
                + ((x + dx + column) >> 1);
        
            val = ((dx + column) & 1) ?  *pSrc & 0x0F : *pSrc >> 4;
            *pDst = ((x + dx + column) & 1) ?  (*pDst & 0xF0) | val : (*pDst & 0x0F) | (val << 4);
        }
    }

    InvalidateRect(g_hWindow, 0, FALSE);    
}




/*
    Инцилизирует глобальную структуру BITMAPINFO g_BitmapInfo для 4 битной цветовой палитры
    цвет: градации серого
*/
void InitBitmapInfo(void)
{
    int i;
    g_BitmapInfo->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    g_BitmapInfo->bmiHeader.biPlanes = 1;
    g_BitmapInfo->bmiHeader.biBitCount = 4;
    assert((1 << g_BitmapInfo->bmiHeader.biBitCount) == MAXCOLORS);
    g_BitmapInfo->bmiHeader.biCompression = BI_RGB;
    g_BitmapInfo->bmiHeader.biSizeImage = 0;
    g_BitmapInfo->bmiHeader.biXPelsPerMeter = 0;
    g_BitmapInfo->bmiHeader.biYPelsPerMeter = 0;
    g_BitmapInfo->bmiHeader.biClrUsed = MAXCOLORS;
    g_BitmapInfo->bmiHeader.biClrImportant = MAXCOLORS;
            
    for(i = 0; i < MAXCOLORS; i++ )
    {
            g_BitmapInfo->bmiColors[i].rgbBlue
                = g_BitmapInfo->bmiColors[i].rgbGreen
                = g_BitmapInfo->bmiColors[i].rgbRed
                = (int) (255 * i) / (MAXCOLORS - 1);
            g_BitmapInfo->bmiColors[i].rgbReserved = 0;
    }

    ZeroMemory(g_ScreenBuffer, SCREEN_WIDTH * SCREEN_HEIGHT >> 1);
}
