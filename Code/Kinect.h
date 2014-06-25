#pragma once

#include <opencv2/opencv.hpp>
#include "stdafx.h"
#include "NuiApi.h"


// Current Kinect
INuiSensor*             m_pNuiSensor;

HANDLE                  m_pStreamColorHandle;
HANDLE                  m_pStreamDepthHandle;
HANDLE                  m_hColorEvent;
HANDLE					m_hDepthEvent;
HANDLE					m_hSkeletonEvent;

static const int        cWidth  = 640;
static const int        cHeight = 480;

static const int		COLOR = 0;
static const int		DEPTH = 1;

bool					m_bProcessColor;
bool					m_bProcessDepth;
bool					m_bProcessSkeleton;

bool					m_bUserDetection;

bool					m_bShow;
bool					m_bRecord;
int						m_bRecCounter;

FILE*					file;

void                    Run();
HRESULT					initColor();
HRESULT					initDepth();
HRESULT					initSkeleton();

void                    ProcessColor();
void					ProcessDepth();
void					ProcessSkeleton();
void                    Update();

HRESULT                 CreateFirstConnected();
HRESULT					GetScreenshotFileName(wchar_t *screenshotName, UINT screenshotNameSize, int frameType);
void					MapDepthToColor(const USHORT* depth, bool playerIndices, USHORT* alignedDepth, BYTE* alignedPlayerIndices, bool rmPlayerBits);