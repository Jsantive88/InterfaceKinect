#include <comdef.h>
#include <strsafe.h>
#include "Kinect.h"

/// <summary>
/// 
/// </summary>
/// <returns>HRESULT</returns>
HRESULT CreateFirstConnected() {
    INuiSensor * pNuiSensor;
    HRESULT hr;

    int iSensorCount = 0;
    hr = NuiGetSensorCount(&iSensorCount);
    if (FAILED(hr)) {
        return hr;
    }

    // Look at each Kinect sensor
    for (int i = 0; i < iSensorCount; ++i) {
        // Create the sensor so we can check status, if we can't create it, move on to the next
        hr = NuiCreateSensorByIndex(i, &pNuiSensor);
        if (FAILED(hr)) {
            continue;
        }

        // Get the status of the sensor, and if connected, then we can initialize it
        hr = pNuiSensor->NuiStatus();
        if (S_OK == hr) {
            m_pNuiSensor = pNuiSensor;
            break;
        }

        // This sensor wasn't OK, so release it since we're not using it
        pNuiSensor->Release();
    }

	// Initialize the Kinect and specify that we'll be using depth
	DWORD dwFlags = 0;
	if (m_bProcessColor) dwFlags |= NUI_INITIALIZE_FLAG_USES_COLOR;
	if (m_bProcessDepth) 
		m_bUserDetection ? (dwFlags |= NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX) : (dwFlags |= NUI_INITIALIZE_FLAG_USES_DEPTH);
	if (m_bProcessSkeleton)
		dwFlags |= NUI_INITIALIZE_FLAG_USES_SKELETON;

    hr = m_pNuiSensor->NuiInitialize(dwFlags);
    return hr;
}

/// <summary>
/// 
/// </summary>
/// <returns>HRESULT</returns>
HRESULT initColor() {
	HRESULT hr;

	// Initialize Color Sensor
    if (NULL != m_pNuiSensor){
		// Create an event that will be signaled when color data is available
        m_hColorEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

        // Open a depth image stream to receive color frames
        hr = m_pNuiSensor->NuiImageStreamOpen(
            NUI_IMAGE_TYPE_COLOR,
            NUI_IMAGE_RESOLUTION_640x480,
            0,
            2,
            m_hColorEvent,
            &m_pStreamColorHandle);
    }

    if (NULL == m_pNuiSensor || FAILED(hr)) {
        return E_FAIL;
    }
}

/// <summary>
/// 
/// </summary>
/// <returns>HRESULT</returns>
HRESULT initDepth() {
	HRESULT hr;

	// Initialize Depth Sensor
    if (NULL != m_pNuiSensor) {
		// Create an event that will be signaled when depth data is available
        m_hDepthEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

        // Open a depth image stream to receive depth frames
        hr = m_pNuiSensor->NuiImageStreamOpen(
			m_bUserDetection ? NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX : NUI_IMAGE_TYPE_DEPTH,
            NUI_IMAGE_RESOLUTION_640x480,
            0,
            2,
            m_hDepthEvent,
            &m_pStreamDepthHandle);

    }

    if (NULL == m_pNuiSensor || FAILED(hr)) {
        return E_FAIL;
    }
}

/// <summary>
/// 
/// </summary>
/// <returns>HRESULT</returns>
HRESULT initSkeleton() {
	HRESULT hr;
	// Initialize Skeleton Sensor
    if (NULL != m_pNuiSensor) {
		// Create an event that will be signaled when skeleton data is available
        m_hSkeletonEvent = CreateEventW(NULL, TRUE, FALSE, NULL);

        // Open a depth image stream to receive depth frames
        hr = m_pNuiSensor->NuiSkeletonTrackingEnable(m_hSkeletonEvent, 0);

		// Create file
		WCHAR name[MAX_PATH];
		StringCchPrintfW(name, _countof(name), L"skeleton.txt");
		file = fopen(_bstr_t(name), "w");
    }

    if (NULL == m_pNuiSensor || FAILED(hr)) {
        return E_FAIL;
    }
}

/// <summary>
/// 
/// </summary>
/// <param name="wchar_t">handle to the application instance</param>
/// <param name="UINT">always 0</param>
/// <param name="int">command line arguments</param>
/// <returns>HRESULT</returns>
HRESULT GetScreenshotFileName(wchar_t *screenshotName, UINT screenshotNameSize, int frameType) {
    wchar_t *knownPath = NULL;
    HRESULT hr = SHGetKnownFolderPath(FOLDERID_Pictures, 0, NULL, &knownPath);

    if (SUCCEEDED(hr)) {
		if (m_bRecord) {
			wchar_t counterString[6];
			swprintf_s(counterString, L"%05d", m_bRecCounter);

			if (frameType == COLOR) {
				StringCchPrintfW(screenshotName, screenshotNameSize, L"%s\\%sC.PNG", knownPath, counterString);
			} else {
				StringCchPrintfW(screenshotName, screenshotNameSize, L"%s\\%sD.PNG", knownPath, counterString);
			}
		}
    }

    CoTaskMemFree(knownPath);
    return hr;
}


/// <summary>
/// 
/// </summary>
void ProcessColor(){
	HRESULT hr;
	NUI_IMAGE_FRAME imageFrame;

	hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pStreamColorHandle, 0, &imageFrame);
	if (FAILED(hr))	{
		return;
	}

	INuiFrameTexture * pTexture = imageFrame.pFrameTexture;
    NUI_LOCKED_RECT LockedRect;

    // Lock the frame data so the Kinect knows not to modify it while we're reading it
    pTexture->LockRect(0, &LockedRect, NULL, 0);

    // Make sure we've received valid data
    if (LockedRect.Pitch != 0) {
		cv::Mat colorFrame(cHeight, cWidth, CV_8UC3);

		for(int i = 0; i < cHeight; i++) {
			uchar *ptr = colorFrame.ptr<uchar>(i);
			uchar *pBuffer = (uchar*)(LockedRect.pBits) + i * LockedRect.Pitch;

			for(int j = 0; j < cWidth; j++)	{
				ptr[3*j] = pBuffer[4*j];
				ptr[3*j+1] = pBuffer[4*j+1];
				ptr[3*j+2] = pBuffer[4*j+2];
			}
		}

		// Draw image
		if (m_bShow) {
			cv::imshow("Color", colorFrame);
			cv::waitKey(1);
		}

		// If m_bRecord
		if (m_bRecord) {
			// Retrieve the path to My Photos
            WCHAR screenshotPath[MAX_PATH];

            // Write out the bitmap to disk
			GetScreenshotFileName(screenshotPath, _countof(screenshotPath), COLOR);

			std::wstring screenshotPathWStr(screenshotPath);
			std::string screenshotPathStr(screenshotPathWStr.begin(), screenshotPathWStr.end());
			
			cv::imwrite(screenshotPathStr, colorFrame);
		}
	}

	pTexture->UnlockRect(0);
	m_pNuiSensor->NuiImageStreamReleaseFrame(m_pStreamColorHandle, &imageFrame);
}

void MapDepthToColor(const USHORT* depth, bool playerIndices, USHORT* alignedDepth, BYTE* alignedPlayerIndices, bool rmPlayerIndices) {
    // Initialize aligned depth map and the corresponding user player indices map
    memset(alignedDepth, 0, cHeight * cWidth);
	if (playerIndices)
		memset(alignedPlayerIndices, 0, cHeight * cWidth);
 
    // loop over each row and column of depth
    for (int y = 0; y < cHeight; y++)
    {
        USHORT* depthRawPtr = (USHORT*) (depth + cWidth * y); // use a pointer
		USHORT* alignedDepthPtr = (USHORT*) (alignedDepth + cWidth * y); // use a pointer
		BYTE* alignedPlayerIndicesPtr = (BYTE*) (alignedPlayerIndices + cWidth * y); // use a pointer
        for (int x = 0; x < cWidth; x++) {
            USHORT draw = *(depthRawPtr++); // depth value of a depth map's (x,y)-located pixel
            USHORT d;
			BYTE playerIdx;

			if (!playerIndices){
				d			= draw;
			} else {
				d			= draw >> 3;
				playerIdx	= draw & NUI_IMAGE_PLAYER_INDEX_MASK;
			}
			
			USHORT daux = (d << 3);

			LONG colorInDepthX, colorInDepthY; // corresponding coordinates of the depth pixel in the color image
            m_pNuiSensor->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
                NUI_IMAGE_RESOLUTION_640x480,
                NUI_IMAGE_RESOLUTION_640x480,
                NULL,
                x, y,
                d, //(d << 3) & ~NUI_IMAGE_PLAYER_INDEX_MASK,
                &colorInDepthX, &colorInDepthY);
                       
            // make sure the depth pixel maps to a valid point in color space
            if ( colorInDepthX >= 0 && colorInDepthX < cWidth && colorInDepthY >= 0 && colorInDepthY < cHeight ) {
				if (playerIndices && !rmPlayerIndices)
					d = (d << 3 & ~NUI_IMAGE_PLAYER_INDEX_MASK) + playerIdx; 
				*(alignedDepthPtr + colorInDepthY * (cWidth) + colorInDepthX) = d;

				if (playerIndices)
					*(alignedPlayerIndices + colorInDepthY * (cWidth) + colorInDepthX) = playerIdx;
            }
        }
    }
 
    return;
}

/// <summary>
/// 
/// </summary>
/// <param name="USHORT">handle to the application instance</param>
/// <param name="USHORT">always 0</param>
/// <param name="BYTE">handle to the application instance</param>
/// <param name="bool">always 0</param>
void MapDepthToColor(const USHORT* depth, USHORT* alignedDepth, BYTE* alignedPlayerIdx, bool rmPlayerBits){
    // Initialize aligned depth map and the corresponding user player indices map
    memset(alignedDepth, 0, 2 * cHeight * cWidth);
    memset(alignedPlayerIdx, 0, cHeight * cWidth);
 
    // loop over each row and column of depth
    for (int y = 0; y < cHeight; y++) {
        const USHORT* sp = depth + cWidth * y; // use a pointer
        for (int x = 0; x < cWidth; x++, sp++) {
            USHORT d = *sp; // depth value of a depth map's (x,y)-located pixel
                    
			LONG colorInDepthX, colorInDepthY; // corresponding coordinates of the depth pixel in the color image                 
            m_pNuiSensor->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
                NUI_IMAGE_RESOLUTION_640x480,
                NUI_IMAGE_RESOLUTION_640x480,
                NULL,
                x, y,
                (d & ~NUI_IMAGE_PLAYER_INDEX_MASK), // d >> 3,
                &colorInDepthX, &colorInDepthY);
                       
            // make sure the depth pixel maps to a valid point in color space
            if ( colorInDepthX >= 0 && colorInDepthX < cWidth && colorInDepthY >= 0 && colorInDepthY < cHeight ){
                *(alignedDepth + colorInDepthY * (cWidth) + colorInDepthX) = rmPlayerBits ? (d >> 3) : d;
                *(alignedPlayerIdx + colorInDepthY * (cWidth) + colorInDepthX) = d & NUI_IMAGE_PLAYER_INDEX_MASK;
            }
        }
    }
 
    return;
}

/// <summary>
/// 
/// </summary>
void ProcessDepth(){
    HRESULT hr;
    NUI_IMAGE_FRAME imageFrame;

	// Attempt to get frame
    hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pStreamDepthHandle, 0, &imageFrame);

    BOOL nearMode;
    INuiFrameTexture* pTexture;

    // Get the depth image pixel texture
    hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(m_pStreamDepthHandle, &imageFrame, &nearMode, &pTexture);
    if (FAILED(hr)) {
        goto ReleaseFrame;
    }

	NUI_LOCKED_RECT LockedRect;

    // Lock the frame data so the Kinect knows not to modify it while we're reading it
    pTexture->LockRect(0, &LockedRect, NULL, 0);

    // Make sure we've received valid data
    if ((LockedRect.Pitch != 0)) {
        const NUI_DEPTH_IMAGE_PIXEL * pBufferRun = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL *>(LockedRect.pBits);

        // end pixel is start + width*height - 1
        const NUI_DEPTH_IMAGE_PIXEL * pBufferEnd = pBufferRun + (cWidth * cHeight);
		USHORT* depthMap = new USHORT[cWidth * cHeight];
		USHORT* p_depthMap = depthMap;

        while ( pBufferRun < pBufferEnd ){
			*(p_depthMap++) = ((pBufferRun->depth << 3) & ~NUI_IMAGE_PLAYER_INDEX_MASK) + pBufferRun->playerIndex;

            // Increment our index into the Kinect's depth buffer
            ++pBufferRun;
        }

	    USHORT* alignedDepthMap = new USHORT[cWidth * cHeight];
		BYTE* alignedPlayerIndicesMap = new BYTE[cWidth * cHeight];
		MapDepthToColor(depthMap, alignedDepthMap, alignedPlayerIndicesMap, true);

		cv::Mat depthMat (cHeight, cWidth, cv::DataType<unsigned short>::type, alignedDepthMap);
	
		if (m_bShow) {
			double min;
			double max;
			cv::minMaxIdx(depthMat, &min, &max);

			cv::Mat depthMatUC1;
			depthMat.convertTo(depthMatUC1, CV_8UC1, 255/(max-min), -min);
			cv::namedWindow("Depth");
			cv::imshow("Depth", depthMat);// depthMatUC1);
			cv::waitKey(1);
		}

		// If m_bRecord
		if (m_bRecord){
			// Retrieve the path to My Photos
            WCHAR screenshotPath[MAX_PATH];

            // Write out the bitmap to disk
			GetScreenshotFileName(screenshotPath, _countof(screenshotPath), DEPTH);

			std::wstring screenshotPathWStr(screenshotPath);
			std::string screenshotPathStr(screenshotPathWStr.begin(), screenshotPathWStr.end());
			
			cv::imwrite(screenshotPathStr, depthMat);
		}
		
		delete depthMap;
		delete alignedDepthMap;
		delete alignedPlayerIndicesMap;
    }

    // We're done with the texture so unlock it
    pTexture->UnlockRect(0);

    pTexture->Release();

ReleaseFrame:
    // Release the frame
	m_pNuiSensor->NuiImageStreamReleaseFrame(m_pStreamDepthHandle, &imageFrame);
}

/// <summary>
/// 
/// </summary>
void ProcessSkeleton(){
	NUI_SKELETON_FRAME skeletonFrame = {0};
	
	HRESULT hr = m_pNuiSensor->NuiSkeletonGetNextFrame(0, &skeletonFrame);
    if ( FAILED(hr) ) {
        return;
    }

    // smooth out the skeleton data
    m_pNuiSensor->NuiTransformSmooth(&skeletonFrame, NULL);

	for (int i = 0 ; i < NUI_SKELETON_COUNT; ++i){
        NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

        if (NUI_SKELETON_TRACKED == trackingState) {
			if (m_bRecord){
				fprintf(file, "%05d\n", m_bRecCounter);
				for (int j = 0; j < 20; j++){
					fprintf(file, "(%f,%f,%f) ", 
						skeletonFrame.SkeletonData[i].SkeletonPositions[j].x,
						skeletonFrame.SkeletonData[i].SkeletonPositions[j].y,
						skeletonFrame.SkeletonData[i].SkeletonPositions[j].z);
				}
				fprintf(file, "\n");
			}

        } else if (NUI_SKELETON_POSITION_ONLY == trackingState) {

		}
	}
}

/// <summary>
/// 
/// </summary>
void Update(){
	if (NULL == m_pNuiSensor){
        return;
    }

	if ( (WAIT_OBJECT_0 == WaitForSingleObject(m_hColorEvent, 0)) && m_bProcessColor){
        ProcessColor();
    }

	if ( (WAIT_OBJECT_0 == WaitForSingleObject(m_hDepthEvent, 0)) && m_bProcessDepth ){
        ProcessDepth();
    }

	if ( (WAIT_OBJECT_0 == WaitForSingleObject(m_hSkeletonEvent, 0)) && m_bProcessSkeleton ){
        ProcessSkeleton();
    }
}

/// <summary>
/// 
/// </summary>
void Run(){
	HANDLE hEvents[] = {m_hColorEvent, m_hDepthEvent, m_hSkeletonEvent}; 

    // Main loop
	while (true){

        // Check to see if we have either a message (by passing in QS_ALLINPUT)
        // Or a Kinect event (hEvents)
        // Update() will check for Kinect events individually, in case more than one are signalled
        DWORD dwEvent = WaitForMultipleObjects(ARRAYSIZE(hEvents), hEvents, FALSE, INFINITE);

        // Check if this is an event we're waiting on and not a timeout or message
        if (WAIT_OBJECT_0 == dwEvent){
            Update();
			if (m_bRecord){
				m_bRecCounter++;
			}
        }
    }
}

///////
// MAIN
///////
/// <summary>
/// 
/// </summary>
/// <param name="HINSTANCE">handle to the application instance</param>
/// <param name="HINSTANCE">always 0</param>
/// <param name="LPWSTR">handle to the application instance</param>
/// <param name="int">always 0</param>
/// <returns>int</returns>
int APIENTRY wWinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPWSTR lpCmdLine, int nCmdShow){
	// Kinect's functionalities: color, depth (with user detection or not), and skeleton tracking
	m_bProcessColor = true;
	m_bProcessDepth = true;
	m_bUserDetection = true; // need to process depth
	m_bProcessSkeleton = false;

	// App's functionalities: show acquired data in window, record data to disk, etc
	m_bShow = true;
	m_bRecord = false;
	m_bRecCounter = 0;

	// Kinect's initialization
	CreateFirstConnected();
	initColor();
    initDepth();
	initSkeleton();

    Run(); // run loop
}