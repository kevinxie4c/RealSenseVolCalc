// MeshCalc.cpp : 定义应用程序的入口点。
//

#include "stdafx.h"

#include "MeshCalc.h"
#include <d3d11.h>
#include <DirectXMath.h>
#include <D3Dcompiler.h>
#include <d2d1.h>
#include <Dwrite.h>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <RealSense\Session.h>
#include <RealSense\SenseManager.h>
#include <RealSense\SampleReader.h>
// the global macro define of max, min in windows.h with overwrite PCL's and causes error
#undef max
#undef min
#include <pcl\segmentation\sac_segmentation.h>
#include <pcl\filters\passthrough.h>
#include <iostream>

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "D3DCompiler.lib")
#pragma comment(lib, "d2d1.lib")
#pragma comment(lib, "Dwrite.lib")

#define MAX_LOADSTRING 100
#define EPS 0.000000001f
#define WIDTH 640
#define HEIGHT 480
#define RENDER_WIDTH 800.0f
#define RENDER_HEIGHT 600.0f
#define SAMPLE_NUM 1

#define MAX_LOADSTRING 100

using namespace DirectX;
using namespace Intel::RealSense;

// 全局变量: 
HINSTANCE hInst;                                // 当前实例
WCHAR szTitle[MAX_LOADSTRING];                  // 标题栏文本
WCHAR szWindowClass[MAX_LOADSTRING];            // 主窗口类名
WCHAR szChildClass[MAX_LOADSTRING] = TEXT("child");
HWND g_hwnd, g_hwndLeft, g_hwndRight;

// 此代码模块中包含的函数的前向声明: 
ATOM                MyRegisterClass(HINSTANCE hInstance);
BOOL                InitInstance(HINSTANCE, int);
LRESULT CALLBACK    WndProc(HWND, UINT, WPARAM, LPARAM);
LRESULT CALLBACK	ChildWndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam);
INT_PTR CALLBACK    About(HWND, UINT, WPARAM, LPARAM);


// global declarations
IDXGISwapChain *swapchain;             // the pointer to the swap chain interface
ID3D11Device *dev;                     // the pointer to our Direct3D device interface
ID3D11DeviceContext *devcon;           // the pointer to our Direct3D device context
ID3D11RenderTargetView *backbuffer;
ID3D11InputLayout *pLayout;            // the pointer to the input layout
ID3D11VertexShader *pVS;    // the vertex shader
ID3D11PixelShader *pPS;     // the pixel shader
ID3D11Buffer *pVBuffer;     // the vertex buffer
ID3D11Buffer *pIBuffer;		// the index buffer
ID3D11Buffer *pCBuffer;		// the constant buffer
ID3D11Buffer *pPlaneBuffer;

ID2D1Factory* pD2DFactory;
ID2D1HwndRenderTarget* pRT;
ID2D1SolidColorBrush* pBlackBrush;
IDWriteFactory* pDWriteFactory;
IDWriteTextFormat* pTextFormat;

Point3DF32 g_vertices[WIDTH * HEIGHT];
UINT g_indexNum = 0;
BOOL g_exit = FALSE;
XMVECTOR g_plane;
UINT8 itr = 0;
FLOAT g_volume = 0.0f;

namespace Colors
{
	const XMFLOAT4 Red(1.0f, 0.0f, 0.0f, 1.0f);
	const XMFLOAT4 Green(0.0f, 1.0f, 0.0f, 1.0f);
	const XMFLOAT4 Blue(0.0f, 0.0f, 1.0f, 1.0f);
	const XMFLOAT4 White(1.0f, 1.0f, 1.0f, 1.0f);
}

struct Vertex
{
	FLOAT x, y, z;
	XMFLOAT4 color;
	XMFLOAT3 normal;
};

struct MatrixBuffer
{
	//XMFLOAT4X4 viewMatrix;
	//XMFLOAT4X4 projectionMatrix;
	XMFLOAT4X4 transform;
};


void InitD3D(HWND hWnd);    // sets up and initializes Direct3D
void RenderFrame(void);     // renders a single frame
void CleanD3D(void);        // closes Direct3D and releases memory
void InitGraphics(void);    // creates the shape to render
void InitPipeline(void);    // loads and prepares the shaders
void InitBuffer();
const XMFLOAT3 Vector3Normalize(FLOAT, FLOAT, FLOAT);
bool isValid(const Point3DF32&);
XMVECTOR PlanePointProj(XMVECTOR plane, XMVECTOR point);

// this function initializes and prepares Direct3D for use
void InitD3D(HWND hWnd)
{
	// create a struct to hold information about the swap chain
	DXGI_SWAP_CHAIN_DESC scd;

	// clear out the struct for use
	ZeroMemory(&scd, sizeof(DXGI_SWAP_CHAIN_DESC));

	// fill the swap chain description struct
	scd.BufferCount = 1;                                    // one back buffer
	scd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;     // use 32-bit color
	scd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;      // how swap chain is to be used
	scd.OutputWindow = hWnd;                                // the window to be used
	scd.SampleDesc.Count = 4;                               // how many multisamples
	scd.Windowed = TRUE;                                    // windowed/full-screen mode

															// create a device, device context and swap chain using the information in the scd struct
	D3D11CreateDeviceAndSwapChain(NULL,
		D3D_DRIVER_TYPE_HARDWARE,
		NULL,
		NULL,
		NULL,
		NULL,
		D3D11_SDK_VERSION,
		&scd,
		&swapchain,
		&dev,
		NULL,
		&devcon);

	// get the address of the back buffer
	ID3D11Texture2D *pBackBuffer;
	swapchain->GetBuffer(0, __uuidof(ID3D11Texture2D), (LPVOID*)&pBackBuffer);

	// use the back buffer address to create the render target
	dev->CreateRenderTargetView(pBackBuffer, NULL, &backbuffer);
	pBackBuffer->Release();

	// set the render target as the back buffer
	devcon->OMSetRenderTargets(1, &backbuffer, NULL);

	// Set the viewport
	D3D11_VIEWPORT viewport;
	ZeroMemory(&viewport, sizeof(D3D11_VIEWPORT));

	viewport.TopLeftX = 0;
	viewport.TopLeftY = 0;
	viewport.Width = RENDER_WIDTH;
	viewport.Height = RENDER_HEIGHT;

	devcon->RSSetViewports(1, &viewport);
	InitPipeline();
	InitGraphics();
}

void InitPipeline()
{
	// load and compile the two shaders
	ID3D10Blob *VS, *PS;
	ID3D10Blob *errMsg;
	//D3DX11CompileFromFile(L"shaders.shader", 0, 0, "VShader", "vs_4_0", 0, 0, 0, &VS, 0, 0);
	//D3DX11CompileFromFile(L"shaders.shader", 0, 0, "PShader", "ps_4_0", 0, 0, 0, &PS, 0, 0);
	D3DCompileFromFile(L"shaders.hlsl", NULL, NULL, "VShader", "vs_5_0", 0, 0, &VS, &errMsg);
	if (errMsg)
	{
		MessageBoxA(0, (char*)errMsg->GetBufferPointer(), 0, 0);
		errMsg->Release();
	}
	D3DCompileFromFile(L"shaders.hlsl", NULL, NULL, "PShader", "ps_5_0", 0, 0, &PS, &errMsg);
	if (errMsg)
	{
		MessageBoxA(0, (char*)errMsg->GetBufferPointer(), 0, 0);
		errMsg->Release();
	}

	// encapsulate both shaders into shader objects
	dev->CreateVertexShader(VS->GetBufferPointer(), VS->GetBufferSize(), NULL, &pVS);
	dev->CreatePixelShader(PS->GetBufferPointer(), PS->GetBufferSize(), NULL, &pPS);

	// set the shader objects
	devcon->VSSetShader(pVS, 0, 0);
	devcon->PSSetShader(pPS, 0, 0);

	// create the input layout object
	D3D11_INPUT_ELEMENT_DESC ied[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "COLOR", 0, DXGI_FORMAT_R32G32B32A32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
		{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 28, D3D11_INPUT_PER_VERTEX_DATA, 0 }
	};

	HRESULT hr = dev->CreateInputLayout(ied, 3, VS->GetBufferPointer(), VS->GetBufferSize(), &pLayout);
	devcon->IASetInputLayout(pLayout);
}

void InitGraphics()
{
	D3D11_BUFFER_DESC cbd;
	// Setup the description of the dynamic matrix constant buffer that is in the vertex shader.
	ZeroMemory(&cbd, sizeof(cbd));
	cbd.Usage = D3D11_USAGE_DYNAMIC;
	cbd.ByteWidth = sizeof(MatrixBuffer);
	cbd.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	cbd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

	dev->CreateBuffer(&cbd, NULL, &pCBuffer);

	D3D11_MAPPED_SUBRESOURCE ms;

	devcon->Map(pCBuffer, NULL, D3D11_MAP_WRITE_DISCARD, NULL, &ms);    // map the buffer
	MatrixBuffer *pMB = (MatrixBuffer*)ms.pData;
	XMMATRIX view = XMMatrixLookAtLH(XMVectorSet(0.0f, 0.0f, 0.0f, 0.0f),
		XMVectorSet(0.0f, 0.0f, 1.0f, 0.0f),
		XMVectorSet(0.0f, 1.0f, 0.0f, 0.0f));
	//XMStoreFloat4x4(&pMB->viewMatrix, view);
	XMMATRIX projection = XMMatrixPerspectiveFovLH(XM_PI / 3.0f, RENDER_HEIGHT / RENDER_WIDTH, 0.5f, 1000.0f);
	//XMStoreFloat4x4(&pMB->projectionMatrix, projection);
	view = XMMatrixTranspose(view);
	projection = XMMatrixTranspose(projection);
	XMStoreFloat4x4(&pMB->transform, view * projection);
	//XMStoreFloat4x4(&pMB->transform, projection * view);
	devcon->Unmap(pCBuffer, NULL);                                      // unmap the buffer

																		// create the vertex buffer
	D3D11_BUFFER_DESC vbd;
	ZeroMemory(&vbd, sizeof(vbd));

	vbd.Usage = D3D11_USAGE_DYNAMIC;                // write access access by CPU and GPU
	vbd.ByteWidth = sizeof(Vertex) * HEIGHT * WIDTH;             // size is the VERTEX struct * 3
	vbd.BindFlags = D3D11_BIND_VERTEX_BUFFER;       // use as a vertex buffer
	vbd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;    // allow CPU to write in buffer

	dev->CreateBuffer(&vbd, NULL, &pVBuffer);       // create the buffer


													// create the index buffer
	D3D11_BUFFER_DESC ibd;
	ZeroMemory(&ibd, sizeof(ibd));
	ibd.Usage = D3D11_USAGE_DYNAMIC;
	ibd.ByteWidth = sizeof(UINT) * HEIGHT * WIDTH * 6;
	ibd.BindFlags = D3D11_BIND_INDEX_BUFFER;
	ibd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;

	dev->CreateBuffer(&ibd, NULL, &pIBuffer);


	D3D11_BUFFER_DESC pbd;
	ZeroMemory(&pbd, sizeof(pbd));

	pbd.Usage = D3D11_USAGE_DYNAMIC;                // write access access by CPU and GPU
	pbd.ByteWidth = sizeof(Vertex) * 6;             // size is the VERTEX struct * 6
	pbd.BindFlags = D3D11_BIND_VERTEX_BUFFER;       // use as a vertex buffer
	pbd.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;    // allow CPU to write in buffer

	dev->CreateBuffer(&pbd, NULL, &pPlaneBuffer);       // create the buffer


	devcon->VSSetConstantBuffers(0, 1, &pCBuffer);
}

bool isValid(const Point3DF32 &p)
{
	XMFLOAT4 d;
	XMStoreFloat4(&d, XMPlaneDotCoord(g_plane, XMVectorSet(p.x, p.y, p.z, 0.0f)));
	if (abs(p.x) > EPS && abs(p.y) > EPS && abs(p.z) > EPS && d.x > 10.0)
	//if (abs(p.x) > EPS && abs(p.y) > EPS && abs(p.z) > EPS)
		return true;
	else
		return false;
}

FLOAT getZByXY(FLOAT x, FLOAT y)
{
	XMFLOAT4 t;
	XMStoreFloat4(&t, g_plane);
	return -(t.x * x + t.y * y + t.w) / t.z;
}

Vertex vertices[HEIGHT * WIDTH];
UINT indices[HEIGHT * WIDTH * 6];
XMVECTOR arNormal[HEIGHT * WIDTH];
XMVECTOR arPos[HEIGHT * WIDTH];

void InitBuffer()
{
	// create a triangle using the Vertex struct
	UINT index = 0;
	for (int i = 0; i < HEIGHT * WIDTH; ++i)
	{
		arNormal[i] = XMVectorZero();
		Point3DF32 *p = g_vertices + i;
		arPos[i] = { p->x, p->y, p->z, 0.0f };
		vertices[i].x = p->x;
		vertices[i].y = p->y;
		vertices[i].z = p->z;
		vertices[i].color = Colors::Red;
	}
	for (int i = 0; i < HEIGHT - 1; ++i)
		for (int j = 0; j < WIDTH - 1; ++j)
		{
			if (!(isValid(g_vertices[i * WIDTH + j + 1]) && isValid(g_vertices[(i + 1) * WIDTH + j])))
				continue;
			if (isValid(g_vertices[i * WIDTH + j]))
			{
				const XMVECTOR &v0 = arPos[i * WIDTH + j];
				const XMVECTOR &v1 = arPos[i * WIDTH + j + 1];
				const XMVECTOR &v2 = arPos[(i + 1) * WIDTH + j];
				XMVECTOR e0 = v1 - v0;
				XMVECTOR e1 = v2 - v0;
				XMVECTOR faceNormal = XMVector3Cross(e0, e1);
				arNormal[i * WIDTH + j] += faceNormal;
				arNormal[i * WIDTH + j + 1] += faceNormal;
				arNormal[(i + 1) * WIDTH + j] += faceNormal;
				indices[index++] = i * WIDTH + j;
				indices[index++] = i * WIDTH + j + 1;
				indices[index++] = (i + 1) * WIDTH + j;

				const XMVECTOR p0 = PlanePointProj(g_plane, v0);
				const XMVECTOR p1 = PlanePointProj(g_plane, v1);
				const XMVECTOR p2 = PlanePointProj(g_plane, v2);
				const XMVECTOR mean = (1.0f / 3.0f) * (v0 + v1 + v2);
				XMFLOAT4 h, dv;
				XMStoreFloat4(&h, XMPlaneDotCoord(g_plane, mean));
				XMStoreFloat4(&dv, (h.x / 2.0 / 1000.0) * XMVector3Length(XMVector3Cross(p1 - p0, p2 - p0)));
				g_volume += dv.x;

			}
			if (isValid(g_vertices[(i + 1) * WIDTH + j + 1]))
			{
				const XMVECTOR &v0 = arPos[(i + 1) * WIDTH + j + 1];
				const XMVECTOR &v1 = arPos[(i + 1) * WIDTH + j];
				const XMVECTOR &v2 = arPos[i * WIDTH + j + 1];
				XMVECTOR e0 = v1 - v0;
				XMVECTOR e1 = v2 - v0;
				XMVECTOR faceNormal = XMVector3Cross(e0, e1);
				arNormal[(i + 1) * WIDTH + j + 1] += faceNormal;
				arNormal[i * WIDTH + j + 1] += faceNormal;
				arNormal[(i + 1) * WIDTH + j] += faceNormal;
				indices[index++] = (i + 1) * WIDTH + j + 1;
				indices[index++] = (i + 1) * WIDTH + j;
				indices[index++] = i * WIDTH + j + 1;

				const XMVECTOR p0 = PlanePointProj(g_plane, v0);
				const XMVECTOR p1 = PlanePointProj(g_plane, v1);
				const XMVECTOR p2 = PlanePointProj(g_plane, v2);
				const XMVECTOR mean = (1.0f / 3.0f) * (v0 + v1 + v2);
				XMFLOAT4 h, dv;
				XMStoreFloat4(&h, XMPlaneDotCoord(g_plane, mean));
				XMStoreFloat4(&dv, (h.x / 2.0 / 1000.0) * XMVector3Length(XMVector3Cross(p1 - p0, p2 - p0)));
				g_volume += dv.x;
			}
		}
	g_indexNum = index;
	for (int i = 0; i < HEIGHT * WIDTH; ++i)
	{
		XMFLOAT3 v;
		XMStoreFloat3(&v, XMVector3Length(arNormal[i]));
		if (v.x > EPS)
			XMStoreFloat3(&vertices[i].normal, XMVector3Normalize(arNormal[i]));
	}


	D3D11_MAPPED_SUBRESOURCE ms;
	// copy the vertices into the buffer
	devcon->Map(pVBuffer, NULL, D3D11_MAP_WRITE_DISCARD, NULL, &ms);    // map the buffer
	memcpy(ms.pData, vertices, sizeof(Vertex) * HEIGHT * WIDTH);                 // copy the data
	devcon->Unmap(pVBuffer, NULL);                                      // unmap the buffer

																		// copy the vertices into the buffer
	devcon->Map(pIBuffer, NULL, D3D11_MAP_WRITE_DISCARD, NULL, &ms);    // map the buffer
	memcpy(ms.pData, indices, sizeof(UINT) * g_indexNum);                 // copy the data
	devcon->Unmap(pIBuffer, NULL);
	
	XMFLOAT3 planeNormal;
	XMStoreFloat3(&planeNormal, g_plane);
	Vertex planeVertice[6] =
	{
		{ -500.0f, 500.0f, getZByXY(-500.0f, 500.0f), Colors::White, planeNormal },
		{ 500.0f, 500.0f, getZByXY(500.0f, 500.0f), Colors::White, planeNormal },
		{ -500.0f, -500.0f, getZByXY(-500.0f, -500.0f), Colors::White, planeNormal },
		{ 500.0f, -500.0f, getZByXY(500.0f, -500.0f), Colors::White, planeNormal },
		{ -500.0f, -500.0f, getZByXY(-500.0f, -500.0f), Colors::White, planeNormal },
		{ 500.0f, 500.0f, getZByXY(500.0f, 500.0f), Colors::White, planeNormal }
	};
	devcon->Map(pPlaneBuffer, NULL, D3D11_MAP_WRITE_DISCARD, NULL, &ms);    // map the buffer
	memcpy(ms.pData, planeVertice, sizeof(planeVertice));                 // copy the data
	devcon->Unmap(pPlaneBuffer, NULL);                                      // unmap the buffer
	
}

void RenderFrame(void)
{
	// clear the back buffer to a deep blue
	XMFLOAT4 deepBlue(0.0f, 0.2f, 0.4f, 1.0f);
	devcon->ClearRenderTargetView(backbuffer, (FLOAT*)&deepBlue);

	InitBuffer();

	// do 3D rendering on the back buffer here
	// select which vertex buffer to display
	UINT stride = sizeof(Vertex);
	UINT offset = 0;

	// select which primtive type we are using
	devcon->IASetPrimitiveTopology(D3D10_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	devcon->IASetVertexBuffers(0, 1, &pPlaneBuffer, &stride, &offset);
	devcon->Draw(6, 0);
	//devcon->Draw(6, WIDTH * HEIGHT);

	devcon->IASetVertexBuffers(0, 1, &pVBuffer, &stride, &offset);
	devcon->IASetIndexBuffer(pIBuffer, DXGI_FORMAT_R32_UINT, 0);


	// draw the vertex buffer to the back buffer
	devcon->DrawIndexed(g_indexNum, 0, 0);
	//devcon->Draw(3, 0);

	// switch the back buffer and the front buffer
	swapchain->Present(0, 0);
}

// this is the function that cleans up Direct3D and COM
void CleanD3D()
{
	// close and release all existing COM objects
	pLayout->Release();
	pVS->Release();
	pPS->Release();
	pCBuffer->Release();
	pVBuffer->Release();
	pIBuffer->Release();
	swapchain->Release();
	dev->Release();
	devcon->Release();
	backbuffer->Release();
}

XMVECTOR PlanePointProj(XMVECTOR plane, XMVECTOR point)
{
	plane = XMPlaneNormalize(plane);
	XMVECTOR k = XMPlaneDotCoord(plane, point);
	return point - XMVectorMultiply(k, plane);
}

void InitD2D(HWND hWnd)
{
	pD2DFactory = NULL;
	HRESULT hr = D2D1CreateFactory(D2D1_FACTORY_TYPE_SINGLE_THREADED, &pD2DFactory);
	RECT rc;
	GetClientRect(hWnd, &rc);

	// Create a Direct2D render target			
	pRT = NULL;
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

	pBlackBrush = NULL;
	if (SUCCEEDED(hr))
	{

		pRT->CreateSolidColorBrush(
			D2D1::ColorF(D2D1::ColorF::Red),
			&pBlackBrush
		);
	}
}

void CleanD2D()
{
	pBlackBrush->Release();
	pRT->Release();
	pD2DFactory->Release();
}

void InitDwrite()
{
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
}

void CleanDWrite()
{
	pTextFormat->Release();
	pDWriteFactory->Release();
}

UINT16 depthBuffer[HEIGHT][WIDTH];

DWORD WINAPI D3DThread(LPVOID p)
{
	// Create a SenseManager instance
	SenseManager *sm = SenseManager::CreateInstance();
	//Session *session = Session::CreateInstance();
	//CaptureManager *cm = session->CreateCaptureManager();
	//Capture *capture = cm->QueryCapture();
	//Device *device = capture->CreateDevice(0);

	// Select the color stream
	SampleReader *reader = SampleReader::Activate(sm);
	reader->EnableStream(StreamType::STREAM_TYPE_DEPTH, WIDTH, HEIGHT);
	ImageData data;

	// Initialize and Stream Samples
	sm->Init();
	CaptureManager *cm = sm->QueryCaptureManager();
	Device *device = cm->QueryDevice();
	Projection *projection = device->CreateProjection();

	InitD3D(g_hwndLeft);
	InitD2D(g_hwndRight);
	InitDwrite();

	int count = 0;
	if (SAMPLE_NUM > 1) ZeroMemory(depthBuffer, sizeof(depthBuffer));

	while (!g_exit)
	{
		if (sm->AcquireFrame(true)<Status::STATUS_NO_ERROR) break;
		Sample *sample = reader->GetSample();
		Image *depth = sample->depth;
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
			pcl::PassThrough<pcl::PointXYZ> filter;
			filter.setInputCloud(cloud);
			// Filter out all points with Z values not in the [0-2] range.
			filter.setFilterFieldName("z");
			filter.setFilterLimits(10.0, 1000.0);
			pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
			filter.filter(*filteredCloud);

			// Object for storing the plane model coefficients.
			pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
			// Create the segmentation object.
			pcl::SACSegmentation<pcl::PointXYZ> segmentation;
			segmentation.setInputCloud(filteredCloud);
			//segmentation.setMaxIterations(30);
			// Configure the object to look for a plane.
			segmentation.setModelType(pcl::SACMODEL_PLANE);
			// Use RANSAC method.
			segmentation.setMethodType(pcl::SAC_RANSAC);
			// Set the maximum allowed distance to the model.
			segmentation.setDistanceThreshold(2.0);
			// Enable model coefficient refinement (optional).
			segmentation.setOptimizeCoefficients(true);

			pcl::PointIndices inlierIndices;
			segmentation.segment(inlierIndices, *coefficients);

			float a, b, c, d;
			a = b = c = d = 1;

			if (inlierIndices.indices.size() == 0)
				std::cout << "Could not find any points that fitted the plane model." << std::endl;
			else
			{
				a = coefficients->values[0];
				b = coefficients->values[1];
				c = coefficients->values[2];
				d = coefficients->values[3];
				if (c > 0)
					g_plane = XMPlaneNormalize(XMVectorSet(-a, -b, -c, -d));
				else
					g_plane = XMPlaneNormalize(XMVectorSet(a, b, c, d));
			}

			g_volume = 0.0f;
			RenderFrame();

			pRT->BeginDraw();
			pRT->Clear();
			RECT rc;
			GetClientRect(g_hwndRight, &rc);
			TCHAR planeEqu[256];
			_swprintf(planeEqu, L"Plane Equation:\n%.4fx+%.4fy+%.4fz+%.4f=0\nVolume:\n%.0f ml", a, b, c, d, g_volume);
			pRT->DrawText(planeEqu, wcslen(planeEqu), pTextFormat,
				D2D1::RectF(rc.left, rc.top, rc.right, rc.bottom), pBlackBrush);
			pRT->EndDraw();

			if (SAMPLE_NUM > 1) ZeroMemory(depthBuffer, sizeof(depthBuffer));
		}
		sm->ReleaseFrame();
	}

	CleanDWrite();
	CleanD2D();
	CleanD3D();
	projection->Release();
	device->Release();
	//cm->Release();
	//sm->Release();

	return 0;
}

int APIENTRY wWinMain(_In_ HINSTANCE hInstance,
                     _In_opt_ HINSTANCE hPrevInstance,
                     _In_ LPWSTR    lpCmdLine,
                     _In_ int       nCmdShow)
{
	WNDCLASS wndclass;
    UNREFERENCED_PARAMETER(hPrevInstance);
    UNREFERENCED_PARAMETER(lpCmdLine);

    // TODO: 在此放置代码。

    // 初始化全局字符串
    LoadStringW(hInstance, IDS_APP_TITLE, szTitle, MAX_LOADSTRING);
    LoadStringW(hInstance, IDC_MESHCALC, szWindowClass, MAX_LOADSTRING);
    MyRegisterClass(hInstance);

	wndclass.style = CS_HREDRAW | CS_VREDRAW;
	wndclass.lpfnWndProc = ChildWndProc;
	wndclass.cbClsExtra = 0;
	wndclass.cbWndExtra = 0;
	wndclass.hInstance = hInst;
	wndclass.hIcon = NULL;
	wndclass.hCursor = LoadCursor(NULL, IDC_ARROW);
	wndclass.hbrBackground = (HBRUSH)GetStockObject(WHITE_BRUSH);
	wndclass.lpszMenuName = NULL;
	wndclass.lpszClassName = szChildClass;
	RegisterClass(&wndclass);

    // 执行应用程序初始化: 
    if (!InitInstance (hInstance, nCmdShow))
    {
        return FALSE;
    }

	g_exit = FALSE;
	g_hwndLeft = CreateWindow(szChildClass, NULL,
		WS_CHILDWINDOW | WS_VISIBLE,
		0, 0, WIDTH, HEIGHT,
		g_hwnd, 0,
		hInst,
		NULL);
	RECT rc;
	GetClientRect(g_hwnd, &rc);
	g_hwndRight = CreateWindow(szChildClass, NULL,
		WS_CHILDWINDOW | WS_VISIBLE,
		WIDTH, 0, rc.right - rc.left - WIDTH, HEIGHT,
		g_hwnd, 0,
		hInst,
		NULL);
	HANDLE handle = CreateThread(NULL, 0, D3DThread, NULL, 0, NULL);

    HACCEL hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_MESHCALC));


	MSG msg;

	// 主消息循环: 
	while (GetMessage(&msg, nullptr, 0, 0))
	{
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	g_exit = TRUE;
	WaitForSingleObject(handle, INFINITE);

    return (int) msg.wParam;
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
    wcex.hIcon          = LoadIcon(hInstance, MAKEINTRESOURCE(IDI_MESHCALC));
    wcex.hCursor        = LoadCursor(nullptr, IDC_ARROW);
    wcex.hbrBackground  = (HBRUSH)(COLOR_WINDOW+1);
    wcex.lpszMenuName   = MAKEINTRESOURCEW(IDC_MESHCALC);
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

   g_hwnd = CreateWindowW(szWindowClass, szTitle, WS_OVERLAPPEDWINDOW,
      CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, nullptr, nullptr, hInstance, nullptr);

   if (!g_hwnd)
   {
      return FALSE;
   }

   ShowWindow(g_hwnd, nCmdShow);
   UpdateWindow(g_hwnd);

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
	case WM_CREATE:
		break;
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

LRESULT CALLBACK ChildWndProc(HWND hwnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	return DefWindowProc(hwnd, message, wParam, lParam);
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

