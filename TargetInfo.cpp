#include "TargetInfo.h"

TargetInfo::TargetInfo(const cv::RotatedRect& rect)
{
	rectangle = rect;
	rectangle.points(vertices);
	centerX = rect.boundingRect().br().x - rect.boundingRect().width / 2;
	centerY = rect.boundingRect().br().y - rect.boundingRect().height / 2;
	angle = (int)std::round(rect.angle);
	if (inRange(angle, LEFT_LOW_VAL, LEFT_HIGH_VAL)) {
		type = targetType::LEFT;
	}
	else if (inRange(angle, RIGHT_LOW_VAL, RIGHT_HIGH_VAL)) {
		type = targetType::RIGHT;
	}
	else {
		type = targetType::UNKNOWN;
	}
}

TargetInfo::~TargetInfo()
{
}

bool TargetInfo::inRange(int val, int low, int high)
{
	return (low <= val && val <= high);
}

std::string TargetInfo::GetType() const {
	if (type == targetType::LEFT) {
		return "Left";
	}
	if (type == targetType::RIGHT) {
		return "Right";
	}
	if (type == targetType::UNKNOWN) {
		return "Unknown";
	}
}
