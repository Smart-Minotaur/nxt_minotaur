#ifndef MOUSEMONITORCALIBRATIONDATA_HPP
#define MOUSEMONITORCALIBRATIONDATA_HPP

namespace minotaur
{

class MouseMonitorCalibrationData
{
private:
	int distance;

	double s1XDisplacement;
	double s1YDisplacement;
	double s2XDisplacement;
	double s2YDisplacement;

	double s1AngleOffset;
	double s2AngleOffset;
	double s1RealDistance;
	double s2RealDistance;

public:
	MouseMonitorCalibrationData();
	~MouseMonitorCalibrationData();

	void setS1AngleOffset(double angleOffset);
	void setS2AngleOffset(double angleOffset);
	void setDistance(int distance);
	void setS1RealDistance(double realDistance);
	void setS2RealDistance(double realDistance);
	void setS1XDisplacement(double s1XDisplacement);
	void setS1YDisplacement(double s1YDisplacement);
	void setS2XDisplacement(double s2XDisplacement);
	void setS2YDisplacement(double s2YDisplacement);
	
	double getS1AngleOffset() const;
	double getS2AngleOffset() const;
	int getDistance() const;
	double getS1RealDistance()const;
	double getS2RealDistance()const;
	double getS1XDisplacement() const;
	double getS1YDisplacement() const;
	double getS2XDisplacement() const;
	double getS2YDisplacement() const;
};

}

#endif // MOUSEMONITORCALIBRATIONDATA_HPP
