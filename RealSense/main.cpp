#include "main.h"
#pragma warning(disable: 26444)



int main(int argc, char* argv[]) try
{
    using namespace cv;
    using namespace rs2;
    using namespace std;

    rs2::colorizer color_map;
    g_rscam = new RsCamera();
    int enableFeatures = ColorStream | DepthStream;
    Features features = ColorStream;

    char clipdist_mode = 'n';
    float clipdist_near = 0.25f;
    float clipdist_far = 0.25f;
    RsCamera::rscam_clipper clipper(clipdist_near, clipdist_far);

    bool enableFrameAlign = true;

    // Setup rscamera.
    g_rscam->SetResolution(1280, 720);
    g_rscam->SetFeatures(enableFeatures);
    if (!g_rscam->Connect())
    {
        return 1;
    }
    //g_rscam->SetFrameAlign(enableFrameAlign);
	
    // Setup opencv.
    string window_name = g_rscam->GetWindowName();
	namedWindow(window_name, WINDOW_AUTOSIZE);
	setMouseCallback(window_name, WndMouseCallBack, NULL);

    // Continuous to get the frame.
	while (getWindowProperty(window_name, WND_PROP_AUTOSIZE) >= 0)
	{
        frameset fms = g_rscam->GetFrames();
		fms = g_rscam->AlignFrames(fms, RS2_STREAM_COLOR, clipper);
        if (!fms) continue;

		frame fm;
		int rendType;

        while (!(enableFeatures & features))
        {
            features++;
        }

        switch (features)
        {
        default:
        case ColorStream:
            fm = fms.get_color_frame();
            rendType = CV_8UC3;
            break;
        case DepthStream:
            fm = fms.get_depth_frame().apply_filter(color_map);
            rendType = CV_8UC3;
            break;
        case LInfraredStream:
            fm = fms.get_infrared_frame(1);
            rendType = CV_8UC1;
            break;
        case RInfraredStream:
            fm = fms.get_infrared_frame(2);
            rendType = CV_8UC1;
            break;
        }
        const int w = fm.as<video_frame>().get_width();
        const int h = fm.as<video_frame>().get_height();

        Mat img(Size(w, h), rendType, (void*)fm.get_data(), Mat::AUTO_STEP);
		
        depth_frame dptfm = fms.get_depth_frame();
		if (dptfm)
		{
			for (vector<RsCamera::rscam_point*>::iterator iter = g_rscam->m_vecPoint.begin(); iter != g_rscam->m_vecPoint.end(); ++iter)
			{
				DrawDstText(img, dptfm, **iter);
				//thread t(DrawDstText, img, dptfm, **iter);
				//t.join();
			}
		}
        for (std::vector<RsCamera::rscam_rectangle*>::iterator iter = g_rscam->m_vecRect.begin(); iter != g_rscam->m_vecRect.end(); ++iter)
        {
            DrawRectangle(img, **iter);
        }

		imshow(window_name, img);
		int key = waitKey(1);
        
        if (key >= 0)
        {
            //cout << "[Key press] No." << key << endl;
            switch (key)
            {
            case 32:
                features++; // Press "Space" Key to Change Displaying Stream Type.
                break;
            case 110: // 'n'
                clipdist_mode = 'n';
                cout << "[clipdist] Change mode to set near value\n";
                break;
            case 102: // 'f'
                clipdist_mode = 'f';
                cout << "[clipdist] Change mode to set far value\n";
                break;
            case 43:  // '+'
                if ('n' == clipdist_mode)
                {
                    clipdist_near += .001f;
                }
                else if ('f' == clipdist_mode)
                {
                    clipdist_far += .001f;
                }
                cout << "[clipdist] Change range to " << clipdist_near << " ~ " << clipdist_far << " m" << endl;
                clipper.set(clipdist_near, clipdist_far);
                break;
            case 45:  // '-'
                if ('n' == clipdist_mode)
                {
                    clipdist_near = (clipdist_near <= .001f) ? (clipdist_near <= 0) ? -0.001f : 0.f : clipdist_near - .001f;
                }
                else if ('f' == clipdist_mode)
                {
                    clipdist_far = (clipdist_far <= .001f) ? (clipdist_far <= 0) ? -0.001f : 0.f : clipdist_far - .001f;
                }
                cout << "[clipdist] Change range to " << clipdist_near << " ~ " << clipdist_far << " m" << endl;
                clipper.set(clipdist_near, clipdist_far);
                break;
            }
        }
	}

	SAFE_DELETE(g_rscam);

	system("pause");
	return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

RsCamera::RsCamera()
{
	m_sWindowName = "";

    m_eState = Status::Stopped;
    m_Features = 0;

    m_fDepthScale = 0;
    m_iRes_width = 640;
    m_iRes_height = 480;
	//m_dstAve = 0;
    //m_bAlign = false;
}

RsCamera::~RsCamera()
{
	Uninitialize();
}

bool RsCamera::SetResolution(int width, int height)
{
    if (m_eState != Status::Stopped)
    {
        return false;
    }
    // Todo: Check if w & h is over the range.

    m_iRes_width = width;
    m_iRes_height = height;
    return true;
}

bool RsCamera::SetFeatures(int enable_features)
{
    if (enable_features >= Features_Last ||
        m_eState != Status::Stopped)
    {
        return false;
    }

    m_Features = enable_features;
    return true;
}

bool RsCamera::Initialize()
{
    m_sWindowName = StringFormat("Intel RealSense Depth Camera D435i - Streaming (%dx%d, 30fps)", m_iRes_width, m_iRes_height);

    try {
        // Enable to support the features.
        if (m_Features & ColorStream)
        {
            m_rsConfig.enable_stream(RS2_STREAM_COLOR, m_iRes_width, m_iRes_height, RS2_FORMAT_BGR8, 30);
            std::cout << "[RsCamera::InitializeEnvironment] Color Stream Enabled.\n";
        }
        if (m_Features & DepthStream)
        {
            m_rsConfig.enable_stream(RS2_STREAM_DEPTH, m_iRes_width, m_iRes_height, RS2_FORMAT_Z16, 30);
            std::cout << "[RsCamera::InitializeEnvironment] Depth Stream Enabled.\n";
        }
        if (m_Features & LInfraredStream)
        {
            m_rsConfig.enable_stream(RS2_STREAM_INFRARED, 1, m_iRes_width, m_iRes_height, RS2_FORMAT_Y8, 30);
            std::cout << "[RsCamera::InitializeEnvironment] Left Infrared Stream Enabled.\n";
        }
        if (m_Features & RInfraredStream)
        {
            m_rsConfig.enable_stream(RS2_STREAM_INFRARED, 2, m_iRes_width, m_iRes_height, RS2_FORMAT_Y8, 30);
            std::cout << "[RsCamera::InitializeEnvironment] Right Infrared Stream Enabled.\n";
        }
    }
    catch (...)
    {
        return false;
    }

    return true;
}

bool RsCamera::Uninitialize()
{
	m_rsPipeline.stop();

	for (std::vector<rscam_point*>::iterator iter = m_vecPoint.begin(); iter != m_vecPoint.end(); ++iter)
	{
		SAFE_DELETE(*iter);
	}

	for (std::vector<rscam_rectangle*>::iterator iter = m_vecRect.begin(); iter != m_vecRect.end(); ++iter)
	{
		SAFE_DELETE(*iter);
	}

    return true;
}

bool RsCamera::Connect()
{
    rs2::depth_sensor* s;
    if (m_Features <= 0 ||
        !Initialize())
    {
        goto exit_error;
    }

    // Start to connect d435i depth camera, and get the depth scale data from device
    // to support to align the color and depth frame.
    m_rsProfile = m_rsPipeline.start(m_rsConfig);
    if (!m_rsProfile)
    {
        goto exit_error;
    }

    s = &m_rsProfile.get_device().first<rs2::depth_sensor>();
    m_fDepthScale = s->get_depth_scale();

    m_eState = Status::Ready;
    return true;

exit_error:
    m_eState = Status::Error;
    return false;
}

bool RsCamera::Disconnect()
{
    if (Uninitialize())
    {
        m_eState = Status::Stopped;
    }

    m_eState = Status::Error;
    return false;
}

bool RsCamera::Display()
{
    return false;
}

rs2::frameset RsCamera::GetFrames()
{
    return m_rsPipeline.wait_for_frames();
}

rs2::frameset RsCamera::AlignFrames(rs2::frameset& frameset, rs2_stream align_to, const rscam_clipper& clipper)
{
    if (align_to == RS2_STREAM_DEPTH)
        return frameset;

    // Start to align the frame.
    rs2::align align(align_to);
    rs2::frameset fms = align.process(frameset);
    KeepImageByDepth(fms, align_to, clipper);
    return fms;
}

void WndMouseCallBack(int event, int x, int y, int flags, void* userdata)
{
	if (g_rscam == NULL)
		return;

	if (event == cv::EVENT_LBUTTONDOWN)
	{
		printf("[Watchpoint] Add: (%d, %d)\n", x, y);
		g_rscam->m_vecPoint.push_back(new RsCamera::rscam_point(x, y));
	}
	else if (event == cv::EVENT_FLAG_RBUTTON)
	{
        printf("[Rectangle] ");
		if (NULL == firstPoint)
		{
			firstPoint = new RsCamera::rscam_point(x, y);
			std::cout << "First point: ";
		}
		else
		{
			RsCamera::rscam_point* secondPoint = &RsCamera::rscam_point(x, y);
			g_rscam->m_vecRect.push_back(new RsCamera::rscam_rectangle(*firstPoint, *secondPoint));
			SAFE_DELETE(firstPoint);
			std::cout << "Second point: ";
		}
        printf("(%d, %d)\n", x, y);
	}
}

float DrawDstText(cv::Mat img, rs2::depth_frame dptfm, RsCamera::rscam_point p)
{
	char text[16];
	float dst = dptfm.get_distance(p.x, p.y);
	sprintf_s(text, "D=%.3f", dst);
	cv::Scalar color(0, 255, 255);
	if (dst == 0)
	{
		color = cv::Scalar(0, 0, 255);
	}

	cv::circle(img, cv::Point(p.x, p.y), 2, color);
	cv::putText(img, text, cv::Point(p.x + 6, p.y + 2), cv::FONT_HERSHEY_SIMPLEX, 0.4, color);

	return dst;
}

void DrawRectangle(cv::Mat img, RsCamera::rscam_rectangle rect)
{
	char text[16];
	sprintf_s(text, "(%d, %d)", rect.p1.x, rect.p1.y);
	cv::putText(img, text, cv::Point(rect.p1.x - 50, rect.p1.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255));
	sprintf_s(text, "(%d, %d)", rect.p2.x, rect.p1.y);
	cv::putText(img, text, cv::Point(rect.p2.x - 50, rect.p1.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255));
	sprintf_s(text, "(%d, %d)", rect.p1.x, rect.p2.y);
	cv::putText(img, text, cv::Point(rect.p1.x - 50, rect.p2.y + 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255));
	sprintf_s(text, "(%d, %d)", rect.p2.x, rect.p2.y);
	cv::putText(img, text, cv::Point(rect.p2.x - 50, rect.p2.y + 25), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 255, 255));
	cv::line(img, cv::Point(rect.p1.x, rect.p1.y), cv::Point(rect.p2.x, rect.p1.y), cv::Scalar(255, 255, 255));
	cv::line(img, cv::Point(rect.p1.x, rect.p1.y), cv::Point(rect.p1.x, rect.p2.y), cv::Scalar(255, 255, 255));
	cv::line(img, cv::Point(rect.p2.x, rect.p2.y), cv::Point(rect.p2.x, rect.p1.y), cv::Scalar(255, 255, 255));
	cv::line(img, cv::Point(rect.p2.x, rect.p2.y), cv::Point(rect.p1.x, rect.p2.y), cv::Scalar(255, 255, 255));
}

void RsCamera::KeepImageByDepth(rs2::frameset& frameset, rs2_stream align_to, const rscam_clipper& clipper)
{
    // This function is copied from: 
    // https://dev.intelrealsense.com/docs/rs-align-advanced
    // ------------------------------------------------------

    rs2::video_frame otherFrame = frameset.first(align_to);
    const rs2::depth_frame& depthFrame = frameset.get_depth_frame();

    if (!otherFrame || !depthFrame || !clipper || 
        clipper.get_distance() == 0)
        return;

    const uint16_t* pDptfm = reinterpret_cast<const uint16_t*>(depthFrame.get_data());
    uint8_t* pOtherfm = reinterpret_cast<uint8_t*>(const_cast<void*>(otherFrame.get_data()));

    int width = otherFrame.get_width();
    int height = otherFrame.get_height();
    int other_bpp = otherFrame.get_bytes_per_pixel();

    for (int y = 0; y < height; y++)
    {
        auto depth_pixel_index = y * width;
        for (int x = 0; x < width; x++, ++depth_pixel_index)
        {
            // Get the depth value of the current pixel
            auto pixels_distance = m_fDepthScale * pDptfm[depth_pixel_index];

            // Check if the depth value is in the range.
            if (pixels_distance <= clipper.get_start() || pixels_distance > clipper.get_end())
            {
                // Calculate the offset in other frame's buffer to current pixel
                auto offset = depth_pixel_index * other_bpp;

                // Set pixel to "background" color (0x999999)
                std::memset(&pOtherfm[offset], 0x99, other_bpp);
            }
        }
    }
}

template<typename ... Args>
std::string StringFormat(const char* format, Args ... args)
{
    size_t size = snprintf(nullptr, 0, format, args ...) + 1;
    char* buf = new char[size];
    snprintf(buf, size, format, args ...);
    std::string str(buf, size - 1);
    delete[] buf;
    return str;
}



















//---------------Scara---------------

//int main()
//{
//	// Create robot setting
//	short sessionIdx = 0;
//	int makeID = 23594510;
//	char encStr[] = "0B9287F3AE9D949A7751D8C8E51A50BE46FBA406D7E9CE0B";
//	char connIP[] = "10.0.0.31";
//	DLL_USE_SETTING* setting = new DLL_USE_SETTING;
//
//	memset(setting, 0, sizeof(DLL_USE_SETTING));
//	setting->SoftwareType = 5;
//	setting->TalkInfoNum = 10;
//	setting->MemSizeI = I_NUM;
//	setting->MemSizeO = O_NUM;
//	setting->MemSizeC = C_NUM;
//	setting->MemSizeS = S_NUM;
//	setting->MemSizeA = A_NUM;
//	setting->MemSizeR = R_NUM;
//	setting->MemSizeF = F_NUM;
//	setting->MemSizeTT = 0;
//	setting->MemSizeCT = 0;
//	setting->MemSizeTS = 0;
//	setting->MemSizeTV = 0;
//	setting->MemSizeCS = 0;
//	setting->MemSizeCV = 0;
//	
//	if (100 != scif_Init(setting, makeID, encStr))
//	{
//		cout << "DLL initialization failed!" << endl;
//		return 0;
//	}
//	if (1 != scif_LocalConnectIP(sessionIdx, connIP))
//	{
//		cout << "Connection failed!" << endl;
//		return 0;
//	}
//
//	
//	// SC_DEFAULT_CMD: Will run with SC_POLLING_CMD and be used to monitor the multiple controllers simultaneously.
//	// SC_POLLING_CMD: Used to synchronize the data from controller and PC.
//	// SC_DIRECT_CMD:  It will process the one-time command with the highest priority.
//	// 
//	// Example.
//	// unsigned int addr[] = { 3001,3003,3005 };
//	// scif_cb_ReadR(SC_POLLING_CMD, 0, 3000, addr);	// Synchronize non-consecutive data addresses.
//
//	scif_StartCombineSet(sessionIdx);
//	scif_cmd_ReadR(SC_POLLING_CMD, sessionIdx, 8503, 3);		// Synchronize consecutive data addresses.
//	scif_FinishCombineSet(sessionIdx);
//
//	// Wait for the controller accept connection.
//	while (1)
//	{
//		if (SC_CONN_STATE_OK == scif_GetTalkMsg(sessionIdx, SCIF_CONNECT_STATE))
//		{
//			cout << "Data setting successed." << endl;
//			break;
//		}
//		Sleep(100);
//	}
//
//	// Set point value of each axis for NC-Prog.
//	scif_cmd_WriteR(sessionIdx, 8503, 100);	//X
//	scif_cmd_WriteR(sessionIdx, 8503, 100);	//Y
//	scif_cmd_WriteR(sessionIdx, 8503, 50);	//C
//
//	// Open NC-prog file
//	scif_cmd_WriteR(sessionIdx, 17004, 300);
//
//	// Run NC-prog
//	scif_cmd_WriteC(sessionIdx, 22, 1);
//
//	// Cleanup API
//	scif_Disconnect(sessionIdx);
//	scif_Destroy();
//	delete setting;
//
//	cout << "Cleanup successed!" << endl;
//	return 0;
//}


/*---------------Sample for cv::imshow---------------*/

//int main()
//{
//	cout << "OpenCV - Transform Image to Gray Level" << endl;
//	Homework1* h1 = new Homework1("D:\\sky88\\Pictures\\IMG_20151230_070901.jpg");
//	h1->LoadPicture();
//	h1->TransPictureToGray();
//	h1->SavePicture();
//
//	delete h1;
//	system("pause");
//	return 0;
//}
//
//bool Homework1::LoadPicture()
//{
//	Image = imread(Path, 1);
//	return true;
//}
//
//bool Homework1::SavePicture()
//{
//	int dot = Path.rfind('.');
//	string fileName = Path.substr(0, dot);
//	string extName = Path.substr(dot, Path.length() - dot);
//	string grayImgPath = fileName + "_gray" + extName;
//	cout << "Save path: " << grayImgPath << endl;
//	imwrite(grayImgPath, Image);
//	return true;
//}
//
//bool Homework1::TransPictureToGray()
//{
//	cvtColor(Image, Image, COLOR_BGR2GRAY, 0);
//	namedWindow(Path, WINDOW_NORMAL);
//	imshow(Path, Image);
//	resizeWindow(Path, 1366, 768);
//	waitKey(0);
//	return true;
//}
//
//Homework1::Homework1(string path)
//{
//	Path = path;
//	Image = NULL;
//	return;
//}
//
//Homework1::~Homework1()
//{
//	return;
//}