#ifndef _MAIN_H
#define _MAIN_H
#pragma once
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <vector>
#include <scif.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <librealsense2/rs.hpp>

#define SAFE_DELETE(pPtr) { delete pPtr; pPtr = NULL; }

//using namespace cv;
//using namespace std;

template<typename ... Args>
std::string StringFormat(const char* format, Args ... args);

class RsCamera
{
public:
	typedef struct RsCamera_Point
	{
	public:
		int x;
		int y;

		RsCamera_Point() : x(0), y(0) {}
		RsCamera_Point(int _x, int _y) { x = _x; y = _y; }
	} rscam_point;

	typedef struct RsCamera_Rectangle
	{
	public:
		rscam_point p1;
		rscam_point p2;

        RsCamera_Rectangle() : p1(rscam_point()), p2(rscam_point()) {};
		RsCamera_Rectangle(int p1_x, int p1_y, int p2_x, int p2_y) : p1(rscam_point(p1_x, p1_y)), p2(rscam_point(p2_x, p2_y)) {}
        RsCamera_Rectangle(rscam_point _p1, rscam_point _p2) : p1(_p1), p2(_p2) {}
	} rscam_rectangle;

    typedef struct RsCamera_Clipper
    {
    private:
        float _start;
        float _end;

    public:
        RsCamera_Clipper() : _start(0.f), _end(0.f) {}
        RsCamera_Clipper(float start, float end) : _start(start), _end(end) {}

        void set(float start, float end) { _start = start; _end = end; }
        void set_start(float start)      { _start = start; }
        void set_end(float end)          { _end = end; }

        const float get_start() const { return _start; }
        const float get_end() const { return _end; }
        const float get_distance() const { return _start - _end; }

        operator bool() const { return this != nullptr; }
    } rscam_clipper;

    enum class Status
    {
        Stopped = 0,
        Ready = 1,
        Streaming = 2,
        Error = 3
    };

	RsCamera();
	~RsCamera();

    bool SetFeatures(int enable_features);
    bool SetResolution(int width, int height);
    void Process();
    bool Connect();
    bool Disconnect();
    bool Display();

	std::string   GetWindowName() { return m_sWindowName; }
    rs2::frameset GetFrames();
    rs2::frameset AlignFrames(rs2::frameset& frameset, rs2_stream align_to, const rscam_clipper& clipper = rscam_clipper());

	// Memory point
	std::vector<rscam_point*>     m_vecPoint;
	std::vector<rscam_rectangle*> m_vecRect;

	//float m_dstAve;

private:
    bool Initialize();
    bool Uninitialize();

    static void ProcStreamByCV(RsCamera* rscam);
    void KeepImageByDepth(rs2::frameset& frameset, rs2_stream align_to, const rscam_clipper& clipper);

    std::vector<std::thread> m_wndProc;

	rs2::config	          m_rsConfig;
	rs2::pipeline         m_rsPipeline;
    rs2::pipeline_profile m_rsProfile;

    int m_Features;
    Status m_eState;

	std::string   m_sWindowName;
    
    int   m_iRes_height;
    int   m_iRes_width;
    float m_fDepthScale;
    float m_fClipDist_near = 0.f;
    float m_fClipDist_far = 0.f;
    //bool m_bAlign;
};

	enum Features
	{
        ColorStream     = 1,
        DepthStream     = 2,
        LInfraredStream = 4,
        RInfraredStream = 8,
        Features_Last   = 16,
	};

std::mutex mtx;
RsCamera* g_rscam;
RsCamera::rscam_point* firstPoint;
void WndMouseCallBack(int event, int x, int y, int flags, void* userdata);
float DrawDstText(cv::Mat img, rs2::depth_frame dptfm, RsCamera::rscam_point p);
void DrawRectangle(cv::Mat img, RsCamera::rscam_rectangle rect);

inline Features& operator++(Features& state, int) 
{
    const int i = static_cast<int>(state) << 1;
    state = static_cast<Features>((i < 1 || i >= Features_Last) ? 1 : i);
    return state;
}
inline Features& operator--(Features& state, int)
{
    const int i = static_cast<int>(state) >> 1;
    state = static_cast<Features>((i < 1 || i >= Features_Last) ? Features_Last >> 1 : i);
    return state;
}

//Features operator++(Features& rs) { 
//    return static_cast<Features>((rs << 1 >= Features_Last) ? 1 : rs << 1);
//}
//Features operator++(Features& rs, int) { 
//    Features r(rs); 
//    ++rs;
//    return r; }

//class Homework1
//{
//public:
//	Homework1(string path);
//	~Homework1();
//	bool LoadPicture();
//	bool SavePicture();
//	bool TransPictureToGray();
//
//	string Path;
//	Mat Image;
//};
#endif