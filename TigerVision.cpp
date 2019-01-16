#include "TigerVision.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <chrono>
#include <wpi/raw_ostream.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

TigerVision::TigerVision(int imageSizeX, int imageSizeY) {
	system("v4l2-ctl --set-ctrl=saturation=200");
	system("v4l2-ctl --set-ctrl=brightness=10");
	system("v4l2-ctl --set-ctrl=exposure_auto=1");
	system("v4l2-ctl --set-ctrl=exposure_absolute=5");
	//start NetworkTables
  	ntinst = nt::NetworkTableInstance::GetDefault();
	ntinst.SetUpdateRate(1);
    if (server) {
    	wpi::outs() << "Setting up NetworkTables server\n";
    	TigerVision::ntinst.StartServer();
  	} else {
    	wpi::outs() << "Setting up NetworkTables client for team " << team << '\n';
    	TigerVision::ntinst.StartClientTeam(team);
  	}
	imageSize = cv::Size(imageSizeX, imageSizeY);
	mjpegServer.SetSource(cvsource);
}

void TigerVision::InitCamera(int camId) {
	vidCap.open(camId);
}
void TigerVision::InitCamera2(int camId) {
	vidCap2.open(camId);
}

void TigerVision::FindTarget() {
	while(true) {
		std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
		cv::Mat imgOriginal;
		vidCap >> imgOriginal;

		cv::Mat imgOriginal2;
		//vidCap2 >> imgOriginal2;

		//resize to reduce processing time
		cv::Mat imgResize;
		cv::resize(imgOriginal, imgResize, imageSize);

		cv::Mat imgResize2;
		//cv::resize(imgOriginal2, imgResize2, imageSize);

		//change color space so we have consistent color ranges
		cv::Mat imgHSVImage;
		cv::cvtColor(imgResize, imgHSVImage, cv::COLOR_BGR2HSV);

		cv::Mat imgHSVImage2;
		//cv::cvtColor(imgResize2, imgHSVImage2, cv::COLOR_BGR2HSV);

		cv::Mat imgGray;
		cv::Mat imgGray2;

		cv::Mat hsv_channels[3];
		//cv::Mat hsv_channels2[3];

		cv::split(imgHSVImage, hsv_channels);
		//cv::split(imgHSVImage2, hsv_channels2);

		imgGray = hsv_channels[2];
		//imgGray2 = hsv_channels2[2];

		cv::Mat disparity;
		
		//checks for HSV values in range
		cv::Mat imgThreshold;
		cv::inRange(imgHSVImage, LOWER_BOUNDS, UPPER_BOUNDS, imgThreshold);

		//clones image so we have a copy of the threshold matrix
		cv::Mat imgContours = imgThreshold.clone();

		//finds closed shapes within threshold image
		std::vector<std::vector<cv::Point>> contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours(imgContours, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

		std::vector<std::vector<cv::Point>> filteredContours = TigerVision::FilterContours(contours);

		std::vector<cv::RotatedRect> leftRects;
		std::vector<cv::RotatedRect> rightRects;
		for (int j = 0; j < filteredContours.size(); j++) {
			//creates rectangle around one target side
			cv::RotatedRect targetRectangle = cv::minAreaRect(filteredContours[j]);
			cv::Point2f vertices[4];
			TargetInfo info = TargetInfo(targetRectangle);

			//draws info on image
			TigerVision::DrawInfo(imgResize, info);

			//this is for combining left and right sides into one target
			if (info.GetType() == "Left") {
				leftRects.push_back(targetRectangle);
			}
			if (info.GetType() == "Right") {
				rightRects.push_back(targetRectangle);
			}

			double angleToTarget = TigerVision::CalculateAngleBetweenCameraAndPixel(info);
		}

		//combining left and right rectangles for center target
		for (int j = 0; j < leftRects.size() + rightRects.size(); j++) {
			cv::RotatedRect leftMostRect;
			int minLeftX = 2147483647;
			int idxLeft = 0;
			for (cv::RotatedRect leftRect : leftRects) {
				if (minLeftX > leftRect.center.x) {
					minLeftX = leftRect.center.x;
					leftMostRect = leftRect;
				}
				idxLeft++;
			}
			if (leftRects.size() > 0) {
				leftRects.erase(leftRects.begin() + (idxLeft - 1));
			}

			cv::RotatedRect rightMostRect;
			int minRightX = 2147483647;
			int idxRight = 0;
			for (cv::RotatedRect rightRect : rightRects) {
				if (minRightX > rightRect.center.x) {
					minRightX = rightRect.center.x;
					rightMostRect = rightRect;
				}
				idxRight++;
			}
			if (rightRects.size() > 0) {
				rightRects.erase(rightRects.begin() + (idxRight - 1));
			}

			std::vector<cv::Point2f> combinedTape;
			cv::Point2f p1[4];
			cv::Point2f p2[4];
			leftMostRect.points(p1);
			rightMostRect.points(p2);
			combinedTape.push_back(p1[0]);
			combinedTape.push_back(p1[1]);
			combinedTape.push_back(p1[2]);
			combinedTape.push_back(p1[3]);

			combinedTape.push_back(p2[0]);
			combinedTape.push_back(p2[1]);
			combinedTape.push_back(p2[2]);
			combinedTape.push_back(p2[3]);

			cv::Rect targetRect = cv::minAreaRect(combinedTape).boundingRect();
			cv::rectangle(imgResize, targetRect, PURPLE);
			cv::Point targetToSendCenter =  (targetRect.br() + targetRect.tl()) * 0.5;
			ntinst.GetTable("vision")->PutNumber("X" + std::to_string(j), targetToSendCenter.x);
			ntinst.GetTable("vision")->PutNumber("Y" + std::to_string(j), targetToSendCenter.y);
			ntinst.Flush();
			DrawInfo(imgResize, targetRect);
		}
		cv::putText(imgResize, "TESTING", cv::Point(50, 50), cv::FONT_HERSHEY_PLAIN, 1, RED);
		//cvsource.PutFrame(imgResize);
		cvsource.PutFrame(imgGray);
		std::chrono::high_resolution_clock::time_point t2 = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
		std::cout << "Duration: " << duration << "\n";
	}
}

std::vector<std::vector<cv::Point>> TigerVision::FilterContours(const std::vector<std::vector<cv::Point>>& contours) {
	//filters contours on aspect ratio and min area and solidity

	std::vector<std::vector<cv::Point>> selectedContours;

	for (int i = 0; i < contours.size(); i++) {
		cv::Rect rect = boundingRect(contours[i]);
		double aspect = rect.width / rect.height;
		std::vector<cv::Point> hull;

		//does solidity calculations
		convexHull(contours[i], hull);
		double area = contourArea(contours[i]);
		double hull_area = contourArea(hull);
		double solidity = (float)area / hull_area;

		if (rect.area() > RECTANCLE_AREA_SIZE && (solidity >= SOLIDITY_MIN && solidity <= SOLIDITY_MAX)) {
			selectedContours.push_back(contours[i]);
		}
	}

	return selectedContours;
}

void TigerVision::DrawInfo(const cv::Mat& imageToDrawTo, const TargetInfo& info) {
	//Draws rectangle
	for (int i = 0; i < 4; i++) {
		cv::line(imageToDrawTo, info.vertices[i], info.vertices[(i + 1) % 4], RED);
	}

	//bounding rectangle around rotated rect
	cv::Rect rect = info.rectangle.boundingRect();

	//sets up coordinates of where to place text
	cv::Point targetTextX = cv::Point(rect.br().x, rect.br().y - 20);
	cv::Point targetTextY = cv::Point(rect.br().x, rect.br().y);
	cv::Point targetTextAngle = cv::Point(rect.br().x, rect.br().y - 40);
	cv::Point leftOrRightText = cv::Point(rect.br().x, rect.br().y - 60);

	//put text
	putText(imageToDrawTo, "x:" + std::to_string(info.centerX), targetTextX, cv::FONT_HERSHEY_PLAIN, 1, RED);
	putText(imageToDrawTo, "y:" + std::to_string(info.centerY), targetTextY, cv::FONT_HERSHEY_PLAIN, 1, RED);
	putText(imageToDrawTo, "angle:" + std::to_string(info.angle), targetTextAngle, cv::FONT_HERSHEY_PLAIN, 1, RED);
	putText(imageToDrawTo, info.GetType(), leftOrRightText, cv::FONT_HERSHEY_PLAIN, 1, RED);
}

void TigerVision::DrawInfo(const cv::Mat& imageToDrawTo, const cv::Rect& rect) {
	cv::Point targetTextX = cv::Point(rect.br().x, rect.br().y - 20);
	cv::Point targetTextY = cv::Point(rect.br().x, rect.br().y);
	cv::Point center = (rect.br() + rect.tl()) * 0.5;
	putText(imageToDrawTo, "x:" + std::to_string(center.x), targetTextX, cv::FONT_HERSHEY_PLAIN, 1, PURPLE);
	putText(imageToDrawTo, "y:" + std::to_string(center.y), targetTextY, cv::FONT_HERSHEY_PLAIN, 1, PURPLE);
}

double TigerVision::CalculateAngleBetweenCameraAndPixel(const TargetInfo& info) {
	double focalLengthPixels = .5 * imageSize.width / std::tan((CAMERA_FOV * (PI / 180)) / 2);
	double angle = std::atan((info.centerX - imageSize.width / 2) / focalLengthPixels);
	double angleDegrees = angle * (180 / PI);
	return angleDegrees;
}
