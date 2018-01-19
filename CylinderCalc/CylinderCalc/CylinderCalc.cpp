// CylinderCalc.cpp : 定义应用程序的入口点。
//

#include "stdafx.h"
#include "CylinderCalc.h"
#include <d2d1.h>
#include <Dwrite.h>
#include <cmath>
#include <RealSense\Session.h>
#include <RealSense\SenseManager.h>
#include <RealSense\SampleReader.h>
// the global macro define of max, min in windows.h with overwrite PCL's and causes error
#undef max
#undef min
#include <pcl\segmentation\sac_segmentation.h>
#include <pcl\filters\passthrough.h>
#include <pcl\filters\extract_indices.h>
#include <pcl\features\normal_3d.h>
#pragma comment(lib, "d2d1.lib")
#pragma comment(lib, "Dwrite.lib")

#define SAMPLE_NUM 1
#define WIDTH 640
#define HEIGHT 480

#define SQR(x) ((x) * (x))

#define MAX_LOADSTRING 100

using namespace Intel::RealSense;

// 全局变量: 
HINSTANCE hInst;                                // 当前实例
HWND hWnd;
WCHAR szTitle[MAX_LOADSTRING];                  // 标题栏文本
WCHAR szWindowClass[MAX_LOADSTRING];            // 主窗口类名

// 此代码模块中包含的函数的前向声明: 
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);


template <class T> void SafeRelease(T **ppT)
{
	if (*ppT)
	{
		(*ppT)->Release();
		*ppT = NULL;
	}
}

Point3DF32 g_vertices[WIDTH * HEIGHT];

UINT16 depthBuffer[HEIGHT][WIDTH];
Point3DF32 ar3D[2];
PointF32 ar2D[2];
TCHAR text[256];

inline void normalize(Point3DF32 *v)
{
	float l = sqrt(v->x * v->x + v->y * v->y + v->z * v->z);
	v->x /= l;
	v->y /= l;
	v->z /= l;
}

inline float dot(const Point3DF32 &u, const Point3DF32 &v)
{
	return u.x * v.x + u.y * v.y + u.z * v.z;
}

inline Point3DF32 add(const Point3DF32 &u, const Point3DF32 &v)
{
	return{ u.x + v.x, u.y + v.y, u.z + v.z };
}

inline Point3DF32 minus(const Point3DF32 &u, const Point3DF32 &v)
{
	return{ u.x - v.x, u.y - v.y, u.z - v.z };
}

inline Point3DF32 mul(float k, const Point3DF32 &v)
{
	return{ k * v.x, k * v.y, k * v.z };
}

Point3DF32 pointOnLine(const Point3DF32 &p, const Point3DF32 &n, const Point3DF32 &q)
{
	return add(p, mul(dot(n, minus(q, p)), n));
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    // TODO: 在此放置代码。

    // 初始化全局字符串
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_CYLINDERCALC, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

    // 执行应用程序初始化: 
    if (!InitInstance (hInstance, nCmdShow))
    {
        return FALSE;
    }

    HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_CYLINDERCALC));

	ID2D1Factory* pD2DFactory = NULL;
	HRESULT hr = D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &pD2DFactory);
	RECT rc;
	GetClientRect(hWnd, &rc);

	// Create a Direct2D render target			
	ID2D1HwndRenderTarget* pRT = NULL;
	hr = pD2DFactory->CreateHwndRenderTarget(
		D2D1::RenderTargetProperties(),
		D2D1::HwndRenderTargetProperties(
			hWnd,
			D2D1::SizeU(
				rc.right - rc.left,
				rc.bottom - rc.top)
		),
		&pRT
	);

	ID2D1SolidColorBrush* pGreenBrush = NULL;
	if (SUCCEEDED(hr))
	{

		pRT->CreateSolidColorBrush(
			D2D1::ColorF(D2D1::ColorF::Green),
			&pGreenBrush
		);
	}

	ID2D1Bitmap* pBitmap = NULL;
	D2D1_BITMAP_PROPERTIES props = { D2D1::PixelFormat(DXGI_FORMAT_R8G8B8A8_UNORM, D2D1_ALPHA_MODE_IGNORE), 0.0, 0.0 };
	pRT->CreateBitmap(D2D1::SizeU(640, 480), props, &pBitmap);

	IDWriteFactory* pDWriteFactory;
	IDWriteTextFormat* pTextFormat;

	DWriteCreateFactory(
		DWRITE_FACTORY_TYPE_SHARED,
		__uuidof(IDWriteFactory),
		reinterpret_cast<IUnknown**>(&pDWriteFactory)
	);

	pDWriteFactory->CreateTextFormat(
		TEXT("Microsoft YaHei UI"),                   // Font family name
		NULL,                          // Font collection(NULL sets it to the system font collection)
		DWRITE_FONT_WEIGHT_REGULAR,    // Weight
		DWRITE_FONT_STYLE_NORMAL,      // Style
		DWRITE_FONT_STRETCH_NORMAL,    // Stretch
		20.0f,                         // Size    
		TEXT("en-us"),                      // Local
		&pTextFormat                 // Pointer to recieve the created object
	);

	// Create a SenseManager instance
	SenseManager *sm = SenseManager::CreateInstance();

	// Select the color stream
	SampleReader *reader = SampleReader::Activate(sm);
	reader->EnableStream(StreamType::STREAM_TYPE_COLOR, 640, 480, 0, StreamOption::STREAM_OPTION_STRONG_STREAM_SYNC);
	reader->EnableStream(StreamType::STREAM_TYPE_DEPTH, 640, 480, 0, StreamOption::STREAM_OPTION_STRONG_STREAM_SYNC);

	ImageData data;

	// Initialize and Stream Samples
	sm->Init();
	CaptureManager *cm = sm->QueryCaptureManager();
	Device *device = cm->QueryDevice();
	Projection *projection = device->CreateProjection();

	int count = 0;
	if (SAMPLE_NUM > 1) ZeroMemory(depthBuffer, sizeof(depthBuffer));

	MSG msg;

	// 主消息循环: 
	while (1)
	{
		if (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE))
		{
			if (msg.message == WM_QUIT)
				break;
			if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
			{
				TranslateMessage(&msg);
				DispatchMessage(&msg);
			}
		}

		pRT->BeginDraw();

		/*
		pRT->DrawRectangle(
		D2D1::RectF(
		rc.left + 100.0f,
		rc.top + 100.0f,
		rc.right - 100.0f,
		rc.bottom - 100.0f),
		pGreenBrush);
		*/

		// This function blocks until a color sample is ready
		if (sm->AcquireFrame(true)<Status::STATUS_NO_ERROR) break;
		Sample *sample = reader->GetSample();
		Image *color = sample->color;
		Image *depth = sample->depth;

		color->AcquireAccess(ImageAccess::ACCESS_READ, PixelFormat::PIXEL_FORMAT_RGBA, &data);
		pBitmap->CopyFromMemory(NULL, data.planes[0], data.pitches[0]);
		color->ReleaseAccess(&data);

		depth->AcquireAccess(ImageAccess::ACCESS_READ_WRITE, PixelFormat::PIXEL_FORMAT_DEPTH, &data);
		if (SAMPLE_NUM > 1) {
			++count;
			UINT8 *ptr = (UINT8*)data.planes[0];
			for (int i = 0; i < HEIGHT; ++i)
				for (int j = 0; j < WIDTH; ++j)
				{
					UINT16* p = (UINT16*)(ptr + i * data.pitches[0] + j * sizeof(UINT16));
					if (*p == 0)
						*p = depthBuffer[i][j];
					depthBuffer[i][j] = *p;
				}
		}

		depth->ReleaseAccess(&data);
		float x, y, z, dx, dy, dz, r, h;
		if (SAMPLE_NUM == 1 || count == SAMPLE_NUM)
		{
			count = 0;
			projection->QueryVertices(depth, g_vertices);

			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
			const int gap = 2;
			int it = 0;
			cloud->width = WIDTH / gap;
			cloud->height = HEIGHT / gap;
			cloud->resize((WIDTH / gap) * (HEIGHT / gap));

			for (int i = gap / 2; i < HEIGHT / gap; i += gap)
				for (int j = gap / 2; j < WIDTH / gap; j += gap)
				{
					Point3DF32 p = g_vertices[i * WIDTH + j];
					cloud->points[it].x = p.x;
					cloud->points[it].y = p.y;
					cloud->points[it].z = p.z;
					++it;
				}
			pcl::PassThrough<pcl::PointXYZ> pass;
			pass.setInputCloud(cloud);
			// Filter out all points with Z values not in the [0-2] range.
			pass.setFilterFieldName("z");
			pass.setFilterLimits(10.0, 500.0);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
			pass.filter(*cloud_filtered);

			pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
			pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
			ne.setSearchMethod(tree);
			ne.setInputCloud(cloud_filtered);
			ne.setKSearch(50);
			ne.compute(*cloud_normals);

			// Object for storing the plane model coefficients.
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);
			
			// Create the segmentation object for the planar model and set all the parameters
			pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

			
			seg.setOptimizeCoefficients(true);
			seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
			seg.setNormalDistanceWeight(0.1);
			seg.setMethodType(pcl::SAC_RANSAC);
			//seg.setMaxIterations(100);
			seg.setDistanceThreshold(3);
			seg.setInputCloud(cloud_filtered);
			seg.setInputNormals(cloud_normals);
			// Obtain the plane inliers and coefficients
			seg.segment(*inliers_plane, *coefficients);

			pcl::ExtractIndices<pcl::PointXYZ> extract;

			// Extract the planar inliers from the input cloud
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
			pcl::ExtractIndices<pcl::Normal> extract_normals;

			extract.setInputCloud(cloud_filtered);
			extract.setIndices(inliers_plane);
			extract.setNegative(true);
			extract.filter(*cloud_filtered2);
			extract_normals.setNegative(true);
			extract_normals.setInputCloud(cloud_normals);
			extract_normals.setIndices(inliers_plane);
			extract_normals.filter(*cloud_normals2);
			
			// Create the segmentation object for cylinder segmentation and set all the parameters
			seg.setOptimizeCoefficients(true);
			seg.setModelType(pcl::SACMODEL_CYLINDER);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setNormalDistanceWeight(0.1);
			seg.setMaxIterations(10000);
			seg.setDistanceThreshold(5);
			//seg.setRadiusLimits(0, 1000);
			seg.setInputCloud(cloud_filtered2);
			seg.setInputNormals(cloud_normals2);
			//seg.setInputCloud(cloud_filtered);
			//seg.setInputNormals(cloud_normals);

			seg.segment(*inliers_cylinder, *coefficients);

			extract.setInputCloud(cloud_filtered2);
			extract.setIndices(inliers_cylinder);
			extract.setNegative(false);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder(new pcl::PointCloud<pcl::PointXYZ>());
			extract.filter(*cloud_cylinder);

			/*
			// Create the segmentation object.
			pcl::SACSegmentation<pcl::PointXYZ> segmentation;
			segmentation.setInputCloud(filteredCloud);
			segmentation.setMaxIterations(1000);
			// Configure the object to look for a plane.
			//segmentation.setModelType(pcl::SACMODEL_PLANE);
			segmentation.setModelType(pcl::SACMODEL_CYLINDER);
			// Use RANSAC method.
			segmentation.setMethodType(pcl::SAC_RANSAC);
			// Set the maximum allowed distance to the model.
			segmentation.setDistanceThreshold(10.0);
			//segmentation.setRadiusLimits(0.0, 100.0);
			// Enable model coefficient refinement (optional).
			segmentation.setOptimizeCoefficients(true);

			pcl::PointIndices inlierIndices;
			segmentation.segment(inlierIndices, *coefficients);
			*/

			x = y = z = dx = dy = dz = r = 1.0;

			if (inliers_cylinder->indices.size() == 0)
				std::cout << "Could not find any points that fitted the plane model." << std::endl;
			else
			{
				x = coefficients->values[0];
				y = coefficients->values[1];
				z = coefficients->values[2];
				dx = coefficients->values[3];
				dy = coefficients->values[4];
				dz = coefficients->values[5];
				r = coefficients->values[6];
			}

			Point3DF32 a, b, p = { x, y, z }, n = { dx, dy, dz };
			float max = -1e10;
			float min = 1e10;

			normalize(&n);

			for (int i = 0; i < cloud_cylinder->points.size(); ++i)
			{
				pcl::PointXYZ it = cloud_cylinder->points[i];
				if (dot({ it.x, it.y, it.z }, n) > max)
				{
					max = dot({ it.x, it.y, it.z }, n);
					a = { it.x, it.y, it.z };
				}
				if (dot({ it.x, it.y, it.z }, n) < min)
				{
					min = dot({ it.x, it.y, it.z }, n);
					b = { it.x, it.y, it.z };
				}
			}

			Point3DF32 c, d;
			c = pointOnLine(p, n, a);
			d = pointOnLine(p, n, b);

			h = sqrt(SQR(c.x - d.x) + SQR(c.y - d.y) + SQR(c.z - d.z));

			ar3D[0] = { c.x, c.y, c.z };
			ar3D[1] = { d.x, d.y, d.z };
			if (SAMPLE_NUM > 1) ZeroMemory(depthBuffer, sizeof(depthBuffer));
		}
		
		sm->ReleaseFrame();	
		

		projection->ProjectCameraToColor(2, ar3D, ar2D);

		pRT->DrawBitmap(pBitmap, D2D1::RectF(0, 0, WIDTH, HEIGHT));
		pRT->DrawLine({ ar2D[0].x, ar2D[0].y }, { ar2D[1].x, ar2D[1].y }, pGreenBrush, 5);
		//FLOAT radius = ar2D[1].x - ar2D[0].x;
		//pRT->DrawEllipse({ { ar2D[0].x, ar2D[0].y }, radius, radius }, pGreenBrush, 5);
		//_swprintf(text, L"r=%.2f mm", d);
		//pRT->DrawText(text, wcslen(text), pTextFormat,
		//	D2D1::RectF(ar2D[0].x, ar2D[0].y - 40, ar2D[1].x, ar2D[1].y), pGreenBrush);

		_swprintf(text, L"h=%.2f mm\nr=%.2f mm\nV=%.0f ml", h, r, h * r * r * M_PI / 1000.0);
		pRT->DrawText(text, wcslen(text), pTextFormat,
			D2D1::RectF(0, 0, WIDTH, HEIGHT), pGreenBrush);
		
		HRESULT hr = pRT->EndDraw();
	}

	SafeRelease(&pRT);
	SafeRelease(&pGreenBrush);
	SafeRelease(&pBitmap);
	SafeRelease(&pD2DFactory);

	return (int)msg.wParam;
}




//
//  函数: MyRegisterClass()
//
//  目的: 注册窗口类。
//
ATOM MyRegisterClass(HINSTANCE hInstance)
{
    WNDCLASSEXW wcex;

    wcex.cbSize = sizeof(WNDCLASSEX);

    wcex.style          = CS_HREDRAW | CS_VREDRAW;
    wcex.lpfnWndProc    = WndProc;
    wcex.cbClsExtra     = 0;
    wcex.cbWndExtra     = 0;
    wcex.hInstance      = hInstance;
    wcex.hIcon          = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_CYLINDERCALC));
    wcex.hCursor        = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground  = (HBRUSH)(COLOR_WINDOW+1);
    wcex.lpszMenuName   = MAKEINTRESOURCEW(IDC_CYLINDERCALC);
    wcex.lpszClassName  = szWindowClass;
    wcex.hIconSm        = LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));

    return RegisterClassExW(&wcex);
}

//
//   函数: InitInstance(HINSTANCE, int)
//
//   目的: 保存实例句柄并创建主窗口
//
//   注释: 
//
//        在此函数中，我们在全局变量中保存实例句柄并
//        创建和显示主程序窗口。
//
BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
   hInst = hInstance; // 将实例句柄存储在全局变量中

   hWnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
      CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);

   if (!hWnd)
   {
      return FALSE;
   }

   ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);

   return TRUE;
}

//
//  函数: WndProc(HWND, UINT, WPARAM, LPARAM)
//
//  目的:    处理主窗口的消息。
//
//  WM_COMMAND  - 处理应用程序菜单
//  WM_PAINT    - 绘制主窗口
//  WM_DESTROY  - 发送退出消息并返回
//
//
LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
    switch (message)
    {
    case WM_COMMAND:
        {
            int wmId = LOWORD(wParam);
            // 分析菜单选择: 
            switch (wmId)
            {
            case IDM_ABOUT:
                DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
                break;
            case IDM_EXIT:
                DestroyWindow(hWnd);
                break;
            default:
                return DefWindowProc(hWnd, message, wParam, lParam);
            }
        }
        break;
    case WM_PAINT:
        {
            PAINTSTRUCT ps;
            HDC hdc = BeginPaint(hWnd, &ps);
            // TODO: 在此处添加使用 hdc 的任何绘图代码...
            EndPaint(hWnd, &ps);
        }
        break;
    case WM_DESTROY:
        PostQuitMessage(0);
        break;
    default:
        return DefWindowProc(hWnd, message, wParam, lParam);
    }
    return 0;
}

// “关于”框的消息处理程序。
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
    UNREFERENCED_PARAMETER(lParam);
    switch (message)
    {
    case WM_INITDIALOG:
        return (INT_PTR)TRUE;

    case WM_COMMAND:
        if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
        {
            EndDialog(hDlg, LOWORD(wParam));
            return (INT_PTR)TRUE;
        }
        break;
    }
    return (INT_PTR)FALSE;
}
