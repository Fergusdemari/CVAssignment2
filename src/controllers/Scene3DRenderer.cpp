/*
 * Scene3DRenderer.cpp
 *
 *  Created on: Nov 15, 2013
 *      Author: coert
 */

#include "Scene3DRenderer.h"

#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/features2d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/types_c.h>
#include <stddef.h>
#include <string>
#include <iostream>

#include "../Settings.h"
#include "../utilities/General.h"

using namespace std;
using namespace cv;

namespace nl_uu_science_gmt
{

	/**
	 * Constructor
	 * Scene properties class (mostly called by Glut)
	 */
	Scene3DRenderer::Scene3DRenderer(
		Reconstructor &r, const vector<Camera*> &cs) :
		m_reconstructor(r),
		m_cameras(cs),
		m_num(4),
		m_sphere_radius(1850)
	{
		m_width = 640;
		m_height = 480;
		m_quit = false;
		m_paused = false;
		m_rotate = false;
		m_camera_view = true;
		m_show_volume = true;
		m_show_grd_flr = true;
		m_show_cam = true;
		m_show_org = true;
		m_show_arcball = false;
		m_show_info = true;
		m_fullscreen = false;

		// Read the checkerboard properties (XML)
		FileStorage fs;
		fs.open(m_cameras.front()->getDataPath() + ".." + string(PATH_SEP) + General::CBConfigFile, FileStorage::READ);
		if (fs.isOpened())
		{
			fs["CheckerBoardWidth"] >> m_board_size.width;
			fs["CheckerBoardHeight"] >> m_board_size.height;
			fs["CheckerBoardSquareSize"] >> m_square_side_len;
		}
		fs.release();


		m_current_camera = 0;
		m_previous_camera = 0;

		m_number_of_frames = m_cameras.front()->getFramesAmount();
		m_current_frame = 0;
		m_previous_frame = -1;

		setBestHSV();

		createTrackbar("Frame", VIDEO_WINDOW, &m_current_frame, m_number_of_frames - 2);
		createTrackbar("H", VIDEO_WINDOW, &m_h_threshold, 255);
		createTrackbar("S", VIDEO_WINDOW, &m_s_threshold, 255);
		createTrackbar("V", VIDEO_WINDOW, &m_v_threshold, 255);
		createTrackbar("ErodeSize", VIDEO_WINDOW, &erode_threshold, 10);
		createTrackbar("DilateSize", VIDEO_WINDOW, &dilate_threshold, 10);

		createFloorGrid();
		setTopView();
	}

	/**
	 * Deconstructor
	 * Free the memory of the floor_grid pointer vector
	 */
	Scene3DRenderer::~Scene3DRenderer()
	{
		for (size_t f = 0; f < m_floor_grid.size(); ++f)
			for (size_t g = 0; g < m_floor_grid[f].size(); ++g)
				delete m_floor_grid[f][g];
	}

	/**
	 * Process the current frame on each camera
	 */
	bool Scene3DRenderer::processFrame()
	{
		for (size_t c = 0; c < m_cameras.size(); ++c)
		{
			if (m_current_frame == m_previous_frame + 1)
			{
				m_cameras[c]->advanceVideoFrame();
			}
			else if (m_current_frame != m_previous_frame)
			{
				m_cameras[c]->getVideoFrame(m_current_frame);
			}
			assert(m_cameras[c] != NULL);
			processForeground(m_cameras[c]);
		}
		return true;
	}

	/**
	 * Separate the background from the foreground
	 * ie.: Create an 8 bit image where only the foreground of the scene is white (255)
	 */
	void Scene3DRenderer::processForeground(
		Camera* camera)
	{

		assert(!camera->getFrame().empty());
		Mat hsv_image;
		cvtColor(camera->getFrame(), hsv_image, CV_BGR2HSV);  // from BGR to HSV color space

		vector<Mat> channels;
		split(hsv_image, channels);  // Split the HSV-channels for further analysis

		// Background subtraction H
		Mat tmp, foreground, background;
		absdiff(channels[0], camera->getBgHsvChannels().at(0), tmp);
		threshold(tmp, foreground, m_h_threshold, 255, CV_THRESH_BINARY);

		// Background subtraction S
		absdiff(channels[1], camera->getBgHsvChannels().at(1), tmp);
		threshold(tmp, background, m_s_threshold, 255, CV_THRESH_BINARY);
		bitwise_and(foreground, background, foreground);

		// Background subtraction V
		absdiff(channels[2], camera->getBgHsvChannels().at(2), tmp);
		threshold(tmp, background, m_v_threshold, 255, CV_THRESH_BINARY);
		bitwise_or(foreground, background, foreground);

		// Improve the foreground image
		Mat structuringElementErode = getStructuringElement(MORPH_RECT, Size(erode_threshold * 2 + 1, erode_threshold * 2 + 1), Point(erode_threshold, erode_threshold));
		Mat structuringElementDilate = getStructuringElement(MORPH_RECT, Size(dilate_threshold * 2 + 1, dilate_threshold * 2 + 1), Point(dilate_threshold, dilate_threshold));

		// dilate / erode to get rid of noise
		dilate(foreground, foreground, structuringElementDilate);
		erode(foreground, foreground, structuringElementErode);

		//Fill holes in the person, kinda useful but might mess up with the hole between legs and chair
		//Mat floodFilledForeground = foreground.clone();
		//floodFill(floodFilledForeground, Point(0, 0), Scalar(255));
		//bitwise_not(floodFilledForeground, floodFilledForeground);
		//foreground = (floodFilledForeground | foreground);

		camera->setForegroundImage(foreground);
	}

	/**
	 * Set currently visible camera to the given camera id
	 */
	void Scene3DRenderer::setCamera(
		int camera)
	{
		m_camera_view = true;

		if (m_current_camera != camera)
		{
			m_previous_camera = m_current_camera;
			m_current_camera = camera;
			m_arcball_eye.x = m_cameras[camera]->getCameraPlane()[0].x;
			m_arcball_eye.y = m_cameras[camera]->getCameraPlane()[0].y;
			m_arcball_eye.z = m_cameras[camera]->getCameraPlane()[0].z;
			m_arcball_up.x = 0.0f;
			m_arcball_up.y = 0.0f;
			m_arcball_up.z = 1.0f;
		}
	}

	/**
	 * Set the 3D scene to bird's eye view
	 */
	void Scene3DRenderer::setTopView()
	{
		m_camera_view = false;
		if (m_current_camera != -1)
			m_previous_camera = m_current_camera;
		m_current_camera = -1;

		m_arcball_eye = vec(0.0f, 0.0f, 10000.0f);
		m_arcball_centre = vec(0.0f, 0.0f, 0.0f);
		m_arcball_up = vec(0.0f, 1.0f, 0.0f);
	}

	/**
	 * Create a LUT for the floor grid
	 */
	void Scene3DRenderer::createFloorGrid()
	{
		const int size = m_reconstructor.getSize() / m_num;
		const int z_offset = 3;

		// edge 1
		vector<Point3i*> edge1;
		for (int y = -size * m_num; y <= size * m_num; y += size)
			edge1.push_back(new Point3i(-size * m_num, y, z_offset));

		// edge 2
		vector<Point3i*> edge2;
		for (int x = -size * m_num; x <= size * m_num; x += size)
			edge2.push_back(new Point3i(x, size * m_num, z_offset));

		// edge 3
		vector<Point3i*> edge3;
		for (int y = -size * m_num; y <= size * m_num; y += size)
			edge3.push_back(new Point3i(size * m_num, y, z_offset));

		// edge 4
		vector<Point3i*> edge4;
		for (int x = -size * m_num; x <= size * m_num; x += size)
			edge4.push_back(new Point3i(x, -size * m_num, z_offset));

		m_floor_grid.push_back(edge1);
		m_floor_grid.push_back(edge2);
		m_floor_grid.push_back(edge3);
		m_floor_grid.push_back(edge4);
	}

	vector<Point> findMiddle(Mat mask) {

		// 
		Mat drawing = Mat::zeros(mask.size(), CV_8UC3);
		vector<vector<Point>> contoursFound;
		findContours(mask, contoursFound, RETR_EXTERNAL, CHAIN_APPROX_NONE);

		// loop over contours, find the biggest one.
		int biggestSize = 0;
		int bestI = 0;
		for (int i = 0; i < contoursFound.size(); i++)
		{
			if (contoursFound[i].size() > biggestSize) {
				biggestSize = contoursFound[i].size();
				bestI = i;
			}
		}
		drawContours(drawing, contoursFound, bestI, Scalar(0, 255, 0), 5);

		Rect res = boundingRect(contoursFound[bestI]);

		//Find middlepoint of biggest contour
		//Moments m = moments(contoursFound[bestI], true);
		//Point p(m.m10 / m.m00, m.m01 / m.m00);

		//Draw circle
		//circle(drawing, p, 5, Scalar(0, 0, 255));
		//imshow("Contours", drawing);
		//imshow("mask", mask);
		//waitKey(0);
		return contoursFound[bestI];
	}


	double clip(double n, double lower, double upper) {
		return max(lower, min(n, upper));
	}

	void Scene3DRenderer::setBestHSV() {
		int hsvMax = 255;
		int edMax = 5;

		double difference = 10000000000;
		int bestH = 0;
		int bestS = 0;
		int bestV = 0;
		int bestD = 0;
		int bestE = 0;

		Mat mask = imread("data/cam2/customMask.png");
		cvtColor(mask, mask, CV_BGR2GRAY);

		//Contour around main mask object
		vector<Point> contour = findMiddle(mask);

		//Calculating middle point
		Moments m = moments(contour, true);
		Point middlePoint(m.m10 / m.m00, m.m01 / m.m00);

		//Calculating AABB
		Rect AABB = boundingRect(contour);
		Mat distanceMap = Mat::zeros(Size(644, 486), CV_8UC3);
		double maxDistance = max(
			max(norm(middlePoint - Point(0, 0)),
				norm(middlePoint - Point(644, 486))),
			max(norm(middlePoint - Point(0, 486)),
				norm(middlePoint - Point(644, 0))));


		Mat frame;
		VideoCapture inputVideo;
		inputVideo.open("data/cam2/video.avi");
		inputVideo >> frame;
		inputVideo.release();

		vector<Mat> background1;
		Mat background2 = imread("data/cam2/background.png");
		Mat background3;
		cvtColor(background2, background3, CV_BGR2HSV);  // from BGR to HSV color space
		split(background3, background1);


		// Value bruteforcing / comparison to manual masks to get HSVED values.
		for (int h = 0; h < 1; h += 10)
		{
			for (int s = 0; s < hsvMax; s += 10)
			{
				cout << "HSV picking: " << to_string(s * 100 / 255) << "%" << endl;

				for (int v = 0; v < hsvMax; v += 10)
				{

					for (int e = 2; e < 3; e++)
					{
						for (int d = 3; d < 4; d++)
						{
							Mat localDistanceMap = Mat::zeros(Size(644, 486), CV_8UC3);
							/// Get HSV image for this pic
							Mat hsv_image;
							cvtColor(frame, hsv_image, CV_BGR2HSV);  // from BGR to HSV color space

							vector<Mat> channels;
							split(hsv_image, channels);  // Split the HSV-channels for further analysis

							// Background subtraction H
							Mat tmp, foreground, background;
							absdiff(channels[0], background1[0], tmp);
							threshold(tmp, foreground, h, 255, CV_THRESH_BINARY);

							// Background subtraction S
							absdiff(channels[1], background1[1], tmp);
							threshold(tmp, background, s, 255, CV_THRESH_BINARY);
							bitwise_and(foreground, background, foreground);

							// Background subtraction V
							absdiff(channels[2], background1[2], tmp);
							threshold(tmp, background, v, 255, CV_THRESH_BINARY);
							bitwise_or(foreground, background, foreground);

							// Improve the foreground image
							Mat structuringElementErode = getStructuringElement(MORPH_RECT, Size(e * 2 + 1, e * 2 + 1), Point(e, e));
							Mat structuringElementDilate = getStructuringElement(MORPH_RECT, Size(d * 2 + 1, d * 2 + 1), Point(d, d));

							// dilate / erode to get rid of noise
							dilate(foreground, foreground, structuringElementDilate);
							erode(foreground, foreground, structuringElementErode);
							///


							double localDifference = 0;
							for (int x = 0; x < 644; x++)
							{
								for (int y = 0; y < 486; y++)
								{
									float val = (float)mask.at<unsigned char>(x, y);
									float val2 = (float)foreground.at<unsigned char>(x, y);
									if (fabs(val - val2) > 0) {
										//double zeroToOne = clip(-log10((relDist-0.02)/2.5)-0.7, 0, 1);
										double relDist = norm(middlePoint - Point(x, y)) / maxDistance;
										double zeroToOne = clip(-log10((relDist - 0.02) / 2.5) - 0.7, 0, 1);
										// If outside of AABB, dont do the expensive polygon test
										if (AABB.contains(Point(x, y))) {
											if (pointPolygonTest(contour, Point(x, y), false) == 1) {
												zeroToOne += 0.4;
											}
										}
										localDifference += zeroToOne;

										circle(localDistanceMap, Point(y, x), 1, Scalar(zeroToOne * 255, zeroToOne * 255, zeroToOne * 255));
									}
								}
							}

							if (localDifference < difference) {
								distanceMap = localDistanceMap;
								difference = localDifference;
								bestH = h;
								bestS = s;
								bestV = v;
								bestD = d;
								bestE = e;
							}

						}
					}
				}
			}
		}
		for (int x = 0; x < 644; x++)
		{
			for (int y = 0; y < 486; y++)
			{
				//double zeroToOne = clip(-log10((relDist-0.02)/2.5)-0.7, 0, 1);
				double relDist = norm(middlePoint - Point(x, y)) / maxDistance;
				double zeroToOne = clip(-log10((relDist - 0.02) / 2.5) - 0.7, 0, 1);
				// If outside of AABB, dont do the expensive polygon test
				if (AABB.contains(Point(x, y))) {
					zeroToOne += 0.05;
					if (pointPolygonTest(contour, Point(x, y), false) == 1) {
						zeroToOne = 2;
					}
				}
				circle(distanceMap, Point(x, y), 1, Scalar(zeroToOne * 255, zeroToOne * 255, zeroToOne * 255));
			}
		}
		imshow("distanceMap", distanceMap);
		m_h_threshold = bestH;
		m_ph_threshold = bestH;
		m_s_threshold = bestS;
		m_ps_threshold = bestS;
		m_v_threshold = bestV;
		m_pv_threshold = bestV;
		erode_threshold = bestE;
		dilate_threshold = bestD;
	}
}
/* namespace nl_uu_science_gmt */
