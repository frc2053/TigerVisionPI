#pragma once

#include <opencv2/core/core.hpp>
#include <string>

class TargetInfo
{
public:
	TargetInfo(const cv::RotatedRect& rect);
	~TargetInfo();
	enum class targetType { LEFT, RIGHT, UNKNOWN };
	int centerX;
	int centerY;
	int angle;
	std::string GetType() const;
	cv::RotatedRect rectangle;
	cv::Point2f vertices[4];
private:
	targetType type;
	bool inRange(int val, int low, int high);
	const static int LEFT_LOW_VAL = -80;
	const static int LEFT_HIGH_VAL = -70;
	const static int RIGHT_LOW_VAL = -25;
	const static int RIGHT_HIGH_VAL = -5;
};

